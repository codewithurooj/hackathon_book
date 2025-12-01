"""
Configuration management using Pydantic Settings
"""
from pydantic_settings import BaseSettings
from typing import List
import json


class Settings(BaseSettings):
    """Application settings"""

    # OpenAI Configuration
    OPENAI_API_KEY: str
    EMBEDDING_MODEL: str = "text-embedding-3-small"
    CHAT_MODEL: str = "gpt-4o"

    # Qdrant Configuration
    QDRANT_URL: str
    QDRANT_API_KEY: str
    QDRANT_COLLECTION_NAME: str = "physical-ai-book"

    # API Configuration
    API_BASE_URL: str = "http://localhost:8000"
    CORS_ORIGINS: str = '["http://localhost:3000"]'

    # Environment
    ENVIRONMENT: str = "development"
    LOG_LEVEL: str = "info"

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from JSON string"""
        return json.loads(self.CORS_ORIGINS)

    class Config:
        env_file = ".env"
        case_sensitive = True


# Global settings instance
settings = Settings()
