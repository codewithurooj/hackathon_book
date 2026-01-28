"""
Pydantic models for translation feature

These models define the request/response schemas for translation API endpoints.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional
from datetime import datetime


class TranslationSubmit(BaseModel):
    """
    Request model for submitting a new Urdu translation

    Example:
        {
            "chapter_id": "chapter-1-intro",
            "urdu_content": "یہ پہلا باب ہے..."
        }
    """
    chapter_id: str = Field(..., min_length=1, max_length=100, description="Unique chapter identifier")
    urdu_content: str = Field(..., min_length=1, description="Urdu translation content in markdown format")

    @field_validator('chapter_id')
    @classmethod
    def validate_chapter_id(cls, v: str) -> str:
        """Ensure chapter_id is not empty and trimmed"""
        v = v.strip()
        if not v:
            raise ValueError("chapter_id cannot be empty or whitespace")
        return v

    @field_validator('urdu_content')
    @classmethod
    def validate_urdu_content(cls, v: str) -> str:
        """Ensure urdu_content is not empty"""
        v = v.strip()
        if not v:
            raise ValueError("urdu_content cannot be empty or whitespace")
        return v


class TranslationResponse(BaseModel):
    """
    Response model for fetching a translation

    Example:
        {
            "chapter_id": "chapter-1-intro",
            "urdu_content": "یہ پہلا باب ہے...",
            "contributor_id": 123,
            "created_at": "2025-12-07T10:30:00Z"
        }
    """
    chapter_id: str
    urdu_content: str
    contributor_id: int
    created_at: datetime

    class Config:
        from_attributes = True  # Enable ORM mode for SQLAlchemy models


class TranslationSubmitResult(BaseModel):
    """
    Response model for translation submission result

    Example (success):
        {
            "success": true,
            "message": "Translation submitted successfully",
            "translation_id": 1,
            "points_awarded": 1
        }

    Example (failure - duplicate):
        {
            "success": false,
            "message": "Translation already exists for this chapter",
            "translation_id": null,
            "points_awarded": 0
        }
    """
    success: bool
    message: str
    translation_id: Optional[int] = None
    points_awarded: int = 0


class UserPointsResponse(BaseModel):
    """
    Response model for user translation points

    Example:
        {
            "user_id": 123,
            "chapters_translated": 5,
            "total_points": 5
        }
    """
    user_id: int
    chapters_translated: int
    total_points: int

    class Config:
        from_attributes = True


class BulkTranslationUpload(BaseModel):
    """
    Request model for bulk translation upload (admin only)

    Example:
        {
            "chapter_id": "chapter-1-intro",
            "urdu_content": "یہ پہلا باب ہے..."
        }
    """
    chapter_id: str = Field(..., min_length=1, max_length=100)
    urdu_content: str = Field(..., min_length=1)

    @field_validator('chapter_id')
    @classmethod
    def validate_chapter_id(cls, v: str) -> str:
        """Ensure chapter_id is not empty and trimmed"""
        return v.strip()

    @field_validator('urdu_content')
    @classmethod
    def validate_urdu_content(cls, v: str) -> str:
        """Ensure urdu_content is not empty"""
        return v.strip()


class TranslationCheckResponse(BaseModel):
    """
    Response model for checking if translation exists

    Example:
        {
            "chapter_id": "chapter-1-intro",
            "exists": true
        }
    """
    chapter_id: str
    exists: bool
