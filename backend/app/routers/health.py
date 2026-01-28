"""
Health check endpoints
"""
from fastapi import APIRouter
from datetime import datetime
from app.models.chat import HealthResponse
from app.utils.config import settings

router = APIRouter(tags=["health"])


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint
    Returns service status and configuration
    """
    return HealthResponse(
        status="healthy",
        environment=settings.ENVIRONMENT,
        timestamp=datetime.now(),
        services={
            "openai": "configured" if settings.OPENAI_API_KEY else "missing",
            "qdrant": "configured" if settings.QDRANT_URL else "missing",
        }
    )
