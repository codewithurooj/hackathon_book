"""
Translation Service

Handles all business logic for translation operations including:
- Fetching translations
- Submitting new translations
- Awarding points to users
- Bulk upload operations
"""

import asyncpg
from typing import Optional, List, Dict, Any
from datetime import datetime
from ..utils.config import settings
from ..utils.urdu_validator import is_valid_urdu_text
from ..models.translation import (
    TranslationResponse,
    TranslationSubmitResult,
    UserPointsResponse
)


class TranslationService:
    """
    Service class for translation operations

    This service manages all database operations related to translations
    and user translation points.
    """

    def __init__(self):
        """Initialize translation service"""
        self._pool: Optional[asyncpg.Pool] = None

    async def get_connection_pool(self) -> asyncpg.Pool:
        """
        Get or create database connection pool

        Returns:
            asyncpg.Pool: Database connection pool

        Raises:
            ValueError: If DATABASE_URL is not configured
        """
        if self._pool is None:
            if not settings.DATABASE_URL:
                raise ValueError("DATABASE_URL not configured in settings")

            self._pool = await asyncpg.create_pool(
                settings.DATABASE_URL,
                min_size=1,
                max_size=10,
                command_timeout=60
            )

        return self._pool

    async def close(self):
        """Close database connection pool"""
        if self._pool:
            await self._pool.close()
            self._pool = None

    async def get_translation(self, chapter_id: str) -> Optional[TranslationResponse]:
        """
        Fetch Urdu translation for a chapter

        Args:
            chapter_id: Unique chapter identifier (e.g., "chapter-1-intro")

        Returns:
            TranslationResponse if translation exists, None otherwise

        Example:
            >>> translation = await service.get_translation("chapter-1-intro")
            >>> if translation:
            ...     print(translation.urdu_content)
        """
        pool = await self.get_connection_pool()

        async with pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT chapter_id, urdu_content, contributor_id, created_at
                FROM translations
                WHERE chapter_id = $1
                """,
                chapter_id
            )

            if row:
                return TranslationResponse(
                    chapter_id=row['chapter_id'],
                    urdu_content=row['urdu_content'],
                    contributor_id=row['contributor_id'],
                    created_at=row['created_at']
                )

            return None

    async def check_translation_exists(self, chapter_id: str) -> bool:
        """
        Check if translation exists for a chapter

        Args:
            chapter_id: Unique chapter identifier

        Returns:
            True if translation exists, False otherwise
        """
        pool = await self.get_connection_pool()

        async with pool.acquire() as conn:
            exists = await conn.fetchval(
                "SELECT EXISTS(SELECT 1 FROM translations WHERE chapter_id = $1)",
                chapter_id
            )

            return bool(exists)

    async def submit_translation(
        self,
        chapter_id: str,
        urdu_content: str,
        user_id: int
    ) -> TranslationSubmitResult:
        """
        Submit a new Urdu translation for a chapter

        This implements the "first submission wins" policy using database
        UNIQUE constraint on chapter_id.

        Args:
            chapter_id: Unique chapter identifier
            urdu_content: Urdu translation content
            user_id: ID of the user submitting the translation

        Returns:
            TranslationSubmitResult with success status and details

        Example:
            >>> result = await service.submit_translation(
            ...     "chapter-1-intro",
            ...     "یہ پہلا باب ہے",
            ...     user_id=123
            ... )
            >>> if result.success:
            ...     print(f"Points awarded: {result.points_awarded}")
        """
        # Validate Urdu content
        is_valid, error_message = is_valid_urdu_text(urdu_content)
        if not is_valid:
            return TranslationSubmitResult(
                success=False,
                message=f"Invalid Urdu content: {error_message}",
                translation_id=None,
                points_awarded=0
            )

        pool = await self.get_connection_pool()

        async with pool.acquire() as conn:
            async with conn.transaction():
                try:
                    # Insert translation (will fail if chapter_id already exists due to UNIQUE constraint)
                    translation_id = await conn.fetchval(
                        """
                        INSERT INTO translations (chapter_id, urdu_content, contributor_id)
                        VALUES ($1, $2, $3)
                        RETURNING id
                        """,
                        chapter_id,
                        urdu_content,
                        user_id
                    )

                    # Award points to user
                    points_awarded = await self._award_points(user_id, conn)

                    return TranslationSubmitResult(
                        success=True,
                        message="Translation submitted successfully",
                        translation_id=translation_id,
                        points_awarded=points_awarded
                    )

                except asyncpg.UniqueViolationError:
                    # Translation already exists for this chapter
                    return TranslationSubmitResult(
                        success=False,
                        message="Translation already exists for this chapter",
                        translation_id=None,
                        points_awarded=0
                    )

                except Exception as e:
                    # Unexpected error
                    return TranslationSubmitResult(
                        success=False,
                        message=f"Error submitting translation: {str(e)}",
                        translation_id=None,
                        points_awarded=0
                    )

    async def _award_points(self, user_id: int, conn: asyncpg.Connection) -> int:
        """
        Award points to user for translation contribution

        Points are awarded as follows:
        - 1 point per chapter translated
        - Maximum 50 points total

        Args:
            user_id: User ID
            conn: Database connection (for transaction support)

        Returns:
            Points awarded (1 if under cap, 0 if already at cap)
        """
        # Get or create user points record
        user_points = await conn.fetchrow(
            """
            INSERT INTO user_translation_points (user_id, chapters_translated, total_points)
            VALUES ($1, 0, 0)
            ON CONFLICT (user_id) DO UPDATE SET user_id = $1
            RETURNING chapters_translated, total_points
            """,
            user_id
        )

        current_chapters = user_points['chapters_translated']
        current_points = user_points['total_points']

        # Calculate new points (1 point per chapter, max 50)
        new_chapters = current_chapters + 1
        new_points = min(new_chapters, 50)
        points_awarded = new_points - current_points

        # Update user points
        await conn.execute(
            """
            UPDATE user_translation_points
            SET chapters_translated = $1, total_points = $2, updated_at = CURRENT_TIMESTAMP
            WHERE user_id = $3
            """,
            new_chapters,
            new_points,
            user_id
        )

        return points_awarded

    async def get_user_points(self, user_id: int) -> Optional[UserPointsResponse]:
        """
        Get translation points for a user

        Args:
            user_id: User ID

        Returns:
            UserPointsResponse if user has translation points, None otherwise
        """
        pool = await self.get_connection_pool()

        async with pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT user_id, chapters_translated, total_points
                FROM user_translation_points
                WHERE user_id = $1
                """,
                user_id
            )

            if row:
                return UserPointsResponse(
                    user_id=row['user_id'],
                    chapters_translated=row['chapters_translated'],
                    total_points=row['total_points']
                )

            return None

    async def bulk_upload_translations(
        self,
        translations: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Bulk upload translations (admin only)

        Args:
            translations: List of dicts with chapter_id and urdu_content

        Returns:
            Dict with upload results

        Example:
            >>> results = await service.bulk_upload_translations([
            ...     {"chapter_id": "chapter-1-intro", "urdu_content": "یہ..."},
            ...     {"chapter_id": "chapter-2-ros2", "urdu_content": "یہ..."}
            ... ])
            >>> print(f"Uploaded: {results['uploaded']}/{results['total']}")
        """
        results = {
            "total": len(translations),
            "uploaded": 0,
            "failed": 0,
            "details": []
        }

        for translation in translations:
            chapter_id = translation.get("chapter_id")
            urdu_content = translation.get("urdu_content")

            if not chapter_id or not urdu_content:
                results["failed"] += 1
                results["details"].append({
                    "chapter_id": chapter_id,
                    "success": False,
                    "error": "Missing chapter_id or urdu_content"
                })
                continue

            # Submit with system user (user_id = 0 for bulk uploads)
            result = await self.submit_translation(chapter_id, urdu_content, user_id=0)

            if result.success:
                results["uploaded"] += 1
                results["details"].append({
                    "chapter_id": chapter_id,
                    "success": True,
                    "translation_id": result.translation_id
                })
            else:
                results["failed"] += 1
                results["details"].append({
                    "chapter_id": chapter_id,
                    "success": False,
                    "error": result.message
                })

        return results


# Global service instance
translation_service = TranslationService()
