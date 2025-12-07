"""
Qdrant vector database service
"""
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from app.utils.config import settings
from typing import List, Dict, Any
import structlog
import uuid

logger = structlog.get_logger()


class QdrantService:
    """Service for interacting with Qdrant vector database"""

    def __init__(self):
        from app.utils.config import settings
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )
        self.collection_name = settings.QDRANT_COLLECTION_NAME

    async def create_collection(self, vector_size: int = 1536):
        """
        Create collection if it doesn't exist

        Args:
            vector_size: Dimension of embedding vectors (1536 for text-embedding-3-small)
        """
        try:
            # Force reload settings to ensure we're using the correct collection name
            from app.utils.config import settings
            self.collection_name = settings.QDRANT_COLLECTION_NAME

            collections = self.client.get_collections().collections
            exists = any(c.name == self.collection_name for c in collections)

            if not exists:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info("collection_created", collection=self.collection_name)
            else:
                logger.info("collection_exists", collection=self.collection_name)

        except Exception as e:
            logger.error("create_collection_error", error=str(e))
            raise

    async def upsert_points(
        self,
        embeddings: List[List[float]],
        payloads: List[Dict[str, Any]]
    ):
        """
        Insert or update points in the collection

        Args:
            embeddings: List of embedding vectors
            payloads: List of metadata dictionaries
        """
        try:
            # Force reload settings to ensure we're using the correct collection name
            from app.utils.config import settings
            self.collection_name = settings.QDRANT_COLLECTION_NAME

            points = [
                PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload=payload
                )
                for embedding, payload in zip(embeddings, payloads)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info("points_upserted", count=len(points))

        except Exception as e:
            logger.error("upsert_error", error=str(e))
            raise

    async def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.3  # Lowered from 0.7 to make search less strict
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors

        Args:
            query_embedding: Query vector
            limit: Maximum number of results
            score_threshold: Minimum similarity score

        Returns:
            List of search results with payload and score
        """
        try:
            # Force reload settings to ensure we're using the correct collection name
            from app.utils.config import settings
            self.collection_name = settings.QDRANT_COLLECTION_NAME

            # Use query_points for the new Qdrant client API
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=limit,
                with_payload=True
            ).points

            # Filter by score threshold manually
            # Lower the default threshold to return more results
            actual_threshold = score_threshold if score_threshold is not None else 0.0
            filtered_results = [
                {
                    "payload": result.payload,
                    "score": result.score
                }
                for result in results
                if result.score >= actual_threshold
            ]

            logger.info("search_completed", found=len(filtered_results), total=len(results))
            return filtered_results

        except Exception as e:
            logger.error("search_error", error=str(e))
            raise

    async def delete_collection(self):
        """Delete the collection"""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info("collection_deleted", collection=self.collection_name)
        except Exception as e:
            logger.error("delete_collection_error", error=str(e))
            raise
