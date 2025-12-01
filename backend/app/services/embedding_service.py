"""
Embedding service using OpenAI API
"""
from openai import OpenAI
from app.utils.config import settings
from typing import List
import structlog

logger = structlog.get_logger()


class EmbeddingService:
    """Service for generating text embeddings"""

    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.EMBEDDING_MODEL

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error("embedding_error", error=str(e), text_length=len(text))
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts
        Process in smaller batches to avoid API limits

        Args:
            texts: List of texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            # Filter out empty texts to avoid API errors
            filtered_texts = [text for text in texts if text.strip()]

            if not filtered_texts:
                # Return empty list if no valid texts to embed
                return []

            # OpenAI has limits on batch size and text length
            # Maximum batch size is usually around 2048 tokens
            # Let's break into smaller chunks of 10-20 texts
            batch_size = 10
            all_embeddings = []

            for i in range(0, len(filtered_texts), batch_size):
                batch = filtered_texts[i:i + batch_size]

                # Truncate very long texts to avoid API limits
                truncated_batch = []
                for text in batch:
                    # OpenAI has input limits, so we truncate to a safe length
                    truncated_text = text[:8192]  # Truncate to 8192 chars max
                    truncated_batch.append(truncated_text)

                response = self.client.embeddings.create(
                    model=self.model,
                    input=truncated_batch
                )
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

            return all_embeddings
        except Exception as e:
            logger.error("batch_embedding_error", error=str(e), batch_size=len(texts))
            raise
