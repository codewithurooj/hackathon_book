"""
Authentication Middleware

Provides authentication dependencies for protected endpoints.

NOTE: This is a placeholder implementation. In production, this should be
replaced with a proper authentication system (JWT, OAuth2, etc.)
"""

from fastapi import Header, HTTPException, Depends
from typing import Optional
from app.utils.config import settings


async def get_current_user_id(
    x_user_id: Optional[str] = Header(None, description="User ID from authentication system")
) -> int:
    """
    Extract user ID from request headers

    In production, this should:
    1. Validate JWT token or session cookie
    2. Extract user ID from validated token
    3. Check user exists and is active

    For now, this is a placeholder that accepts X-User-Id header.

    Args:
        x_user_id: User ID from X-User-Id header

    Returns:
        User ID as integer

    Raises:
        HTTPException: If user is not authenticated

    Example:
        @router.post("/protected")
        async def protected_endpoint(user_id: int = Depends(get_current_user_id)):
            return {"user_id": user_id, "message": "Authenticated!"}
    """
    if not x_user_id:
        raise HTTPException(
            status_code=401,
            detail="Authentication required. Please provide X-User-Id header."
        )

    try:
        user_id = int(x_user_id)
    except ValueError:
        raise HTTPException(
            status_code=400,
            detail="Invalid X-User-Id header. Must be an integer."
        )

    if user_id <= 0:
        raise HTTPException(
            status_code=400,
            detail="Invalid user ID. Must be positive integer."
        )

    return user_id


async def verify_api_key(x_api_key: Optional[str] = Header(None)) -> bool:
    """
    Verify API key for admin endpoints

    Args:
        x_api_key: API key from X-API-Key header

    Returns:
        True if API key is valid

    Raises:
        HTTPException: If API key is missing or invalid
    """
    if not x_api_key:
        raise HTTPException(
            status_code=401,
            detail="API key required. Please provide X-API-Key header."
        )

    if x_api_key != settings.BACKEND_API_KEY:
        raise HTTPException(
            status_code=403,
            detail="Invalid API key"
        )

    return True


# Optional authentication (returns None if not authenticated)
async def get_optional_user_id(
    x_user_id: Optional[str] = Header(None)
) -> Optional[int]:
    """
    Extract user ID if available, otherwise return None

    Use this for endpoints that can work both authenticated and unauthenticated.

    Args:
        x_user_id: User ID from X-User-Id header

    Returns:
        User ID as integer if authenticated, None otherwise
    """
    if not x_user_id:
        return None

    try:
        return int(x_user_id)
    except ValueError:
        return None
