"""
Translation API Endpoints

Provides REST API for Urdu translation feature including:
- Fetching translations
- Checking translation existence
- Submitting new translations
- Getting user points
- Bulk upload (admin only)
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import List
from app.models.translation import (
    TranslationResponse,
    TranslationSubmit,
    TranslationSubmitResult,
    UserPointsResponse,
    BulkTranslationUpload,
    TranslationCheckResponse
)
from app.services.translation_service import translation_service
from app.middleware.auth import get_current_user_id, verify_api_key
import structlog


router = APIRouter(prefix="/translations", tags=["translations"])
logger = structlog.get_logger()


@router.get("/{chapter_id}", response_model=TranslationResponse)
async def get_translation(chapter_id: str):
    """
    Get Urdu translation for a chapter

    Args:
        chapter_id: Unique chapter identifier (e.g., "chapter-1-intro")

    Returns:
        TranslationResponse with Urdu content

    Raises:
        HTTPException 404: If translation not found

    Example:
        GET /api/v1/translations/chapter-1-intro
    """
    try:
        logger.info("get_translation", chapter_id=chapter_id)

        translation = await translation_service.get_translation(chapter_id)

        if not translation:
            logger.warning("translation_not_found", chapter_id=chapter_id)
            raise HTTPException(
                status_code=404,
                detail=f"Translation not found for chapter: {chapter_id}"
            )

        logger.info("translation_retrieved", chapter_id=chapter_id)
        return translation

    except HTTPException:
        raise
    except Exception as e:
        logger.error("get_translation_error", chapter_id=chapter_id, error=str(e))
        raise HTTPException(status_code=500, detail=f"Error fetching translation: {str(e)}")


@router.get("/check/{chapter_id}", response_model=TranslationCheckResponse)
async def check_translation_exists(chapter_id: str):
    """
    Check if Urdu translation exists for a chapter

    Args:
        chapter_id: Unique chapter identifier

    Returns:
        TranslationCheckResponse with exists boolean

    Example:
        GET /api/v1/translations/check/chapter-1-intro
        Response: {"chapter_id": "chapter-1-intro", "exists": true}
    """
    try:
        logger.info("check_translation_exists", chapter_id=chapter_id)

        exists = await translation_service.check_translation_exists(chapter_id)

        return TranslationCheckResponse(
            chapter_id=chapter_id,
            exists=exists
        )

    except Exception as e:
        logger.error("check_translation_error", chapter_id=chapter_id, error=str(e))
        raise HTTPException(status_code=500, detail=f"Error checking translation: {str(e)}")


@router.post("", response_model=TranslationSubmitResult)
async def submit_translation(
    request: TranslationSubmit,
    user_id: int = Depends(get_current_user_id)
):
    """
    Submit a new Urdu translation for a chapter

    Requires authentication (X-User-Id header).
    First submission wins - subsequent submissions rejected.

    Args:
        request: TranslationSubmit with chapter_id and urdu_content
        user_id: Current user ID (from authentication)

    Returns:
        TranslationSubmitResult with success status and points awarded

    Raises:
        HTTPException 401: If not authenticated
        HTTPException 409: If translation already exists (first wins policy)

    Example:
        POST /api/v1/translations
        Headers: X-User-Id: 123
        Body: {
            "chapter_id": "chapter-1-intro",
            "urdu_content": "یہ پہلا باب ہے..."
        }
    """
    try:
        logger.info(
            "submit_translation",
            chapter_id=request.chapter_id,
            user_id=user_id,
            content_length=len(request.urdu_content)
        )

        result = await translation_service.submit_translation(
            chapter_id=request.chapter_id,
            urdu_content=request.urdu_content,
            user_id=user_id
        )

        if not result.success:
            # Check if duplicate submission
            if "already exists" in result.message.lower():
                logger.warning(
                    "duplicate_translation",
                    chapter_id=request.chapter_id,
                    user_id=user_id
                )
                raise HTTPException(status_code=409, detail=result.message)
            else:
                # Validation or other error
                logger.warning(
                    "translation_submission_failed",
                    chapter_id=request.chapter_id,
                    user_id=user_id,
                    reason=result.message
                )
                raise HTTPException(status_code=400, detail=result.message)

        logger.info(
            "translation_submitted",
            chapter_id=request.chapter_id,
            user_id=user_id,
            points_awarded=result.points_awarded,
            translation_id=result.translation_id
        )

        return result

    except HTTPException:
        raise
    except Exception as e:
        logger.error(
            "submit_translation_error",
            chapter_id=request.chapter_id,
            user_id=user_id,
            error=str(e)
        )
        raise HTTPException(status_code=500, detail=f"Error submitting translation: {str(e)}")


@router.get("/users/{user_id}/points", response_model=UserPointsResponse)
async def get_user_translation_points(user_id: int):
    """
    Get translation points for a user

    Args:
        user_id: User ID to get points for

    Returns:
        UserPointsResponse with points breakdown

    Raises:
        HTTPException 404: If user has no translation points

    Example:
        GET /api/v1/translations/users/123/points
        Response: {
            "user_id": 123,
            "chapters_translated": 5,
            "total_points": 5
        }
    """
    try:
        logger.info("get_user_points", user_id=user_id)

        points = await translation_service.get_user_points(user_id)

        if not points:
            logger.info("user_points_not_found", user_id=user_id)
            # Return zero points if user hasn't translated anything yet
            return UserPointsResponse(
                user_id=user_id,
                chapters_translated=0,
                total_points=0
            )

        logger.info(
            "user_points_retrieved",
            user_id=user_id,
            points=points.total_points
        )
        return points

    except Exception as e:
        logger.error("get_user_points_error", user_id=user_id, error=str(e))
        raise HTTPException(status_code=500, detail=f"Error fetching user points: {str(e)}")


@router.post("/admin/bulk", response_model=dict)
async def bulk_upload_translations(
    translations: List[BulkTranslationUpload],
    _: bool = Depends(verify_api_key)
):
    """
    Bulk upload translations (admin only)

    Requires API key authentication (X-API-Key header).

    Args:
        translations: List of translations to upload
        _: API key verification (dependency)

    Returns:
        Dict with upload results

    Raises:
        HTTPException 401: If API key missing
        HTTPException 403: If API key invalid

    Example:
        POST /api/v1/translations/admin/bulk
        Headers: X-API-Key: your-api-key
        Body: [
            {
                "chapter_id": "chapter-1-intro",
                "urdu_content": "یہ پہلا باب ہے..."
            },
            ...
        ]
    """
    try:
        logger.info("bulk_upload_start", count=len(translations))

        # Convert Pydantic models to dicts
        translations_data = [t.model_dump() for t in translations]

        result = await translation_service.bulk_upload_translations(translations_data)

        logger.info(
            "bulk_upload_complete",
            total=result["total"],
            uploaded=result["uploaded"],
            failed=result["failed"]
        )

        return result

    except Exception as e:
        logger.error("bulk_upload_error", error=str(e))
        raise HTTPException(status_code=500, detail=f"Error uploading translations: {str(e)}")
