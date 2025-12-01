import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()
load_dotenv(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '.env'))  # Load from project root

class Settings:
    # Backend API Key for authentication
    BACKEND_API_KEY: Optional[str] = os.getenv("BACKEND_API_KEY")

    # OpenAI API Key
    OPENAI_API_KEY: Optional[str] = os.getenv("OPENAI_API_KEY")
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"

    # Qdrant Cloud Configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = "physical-ai-book"

    # Neon Postgres Database Configuration
    DATABASE_URL: Optional[str] = os.getenv("DATABASE_URL")

settings = Settings()
