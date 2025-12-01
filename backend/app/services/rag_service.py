"""
RAG (Retrieval Augmented Generation) service
Orchestrates embedding, vector search, and answer generation
"""
from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
from app.services.agent_service import AgentService
from app.models.chat import Source
from typing import List, Tuple
import structlog

logger = structlog.get_logger()


class RAGService:
    """Main RAG service orchestrating the pipeline"""

    def __init__(self):
        self.embedding_service = EmbeddingService()
        self.qdrant_service = QdrantService()
        self.agent_service = AgentService()

    async def answer_general_question(
        self,
        question: str,
        top_k: int = 5
    ) -> Tuple[str, List[Source], float]:
        """
        Answer general question using RAG

        Args:
            question: User's question
            top_k: Number of context chunks to retrieve

        Returns:
            Tuple of (answer, sources, confidence)
        """
        try:
            # 1. Generate embedding for question
            logger.info("rag_step_embedding", question_length=len(question))
            question_embedding = await self.embedding_service.generate_embedding(question)

            # 2. Search for relevant context
            logger.info("rag_step_search", top_k=top_k)
            search_results = await self.qdrant_service.search(
                query_embedding=question_embedding,
                limit=top_k,
                score_threshold=0.3  # Lower threshold to find more results
            )

            # 3. Generate answer using context
            logger.info("rag_step_generate", chunks_found=len(search_results))
            answer = await self.agent_service.generate_answer(
                question=question,
                context_chunks=search_results,
                mode="general"
            )

            # 4. Format sources
            sources = [
                Source(
                    title=result["payload"].get("title", "Unknown"),
                    url=result["payload"].get("url", "#"),
                    relevance_score=result["score"],
                    snippet=result["payload"].get("content", "")[:200] + "..."
                )
                for result in search_results
            ]

            # 5. Calculate confidence (average of retrieval scores)
            confidence = sum(r["score"] for r in search_results) / len(search_results) if search_results else 0.5

            logger.info(
                "rag_completed",
                sources_count=len(sources),
                confidence=confidence
            )

            return answer, sources, confidence

        except Exception as e:
            logger.error("rag_general_error", error=str(e))
            raise

    async def answer_context_question(
        self,
        question: str,
        context: str,
        top_k: int = 3
    ) -> Tuple[str, List[Source], float]:
        """
        Answer question using provided context + RAG

        Args:
            question: User's question
            context: Selected text from the book
            top_k: Number of additional context chunks to retrieve

        Returns:
            Tuple of (answer, sources, confidence)
        """
        try:
            # 1. Generate embedding for context
            logger.info("rag_context_embedding", context_length=len(context))
            context_embedding = await self.embedding_service.generate_embedding(context)

            # 2. Search for similar/related content
            logger.info("rag_context_search", top_k=top_k)
            search_results = await self.qdrant_service.search(
                query_embedding=context_embedding,
                limit=top_k,
                score_threshold=0.3  # Lower threshold to find more results
            )

            # 3. Add selected context as primary source
            context_chunk = {
                "payload": {
                    "title": "Selected Context",
                    "content": context,
                    "url": "#selected-text"
                },
                "score": 1.0
            }
            all_chunks = [context_chunk] + search_results

            # 4. Generate answer using all context
            logger.info("rag_context_generate", total_chunks=len(all_chunks))
            answer = await self.agent_service.generate_answer(
                question=question,
                context_chunks=all_chunks,
                mode="context"
            )

            # 5. Format sources
            sources = [
                Source(
                    title=result["payload"].get("title", "Unknown"),
                    url=result["payload"].get("url", "#"),
                    relevance_score=result["score"],
                    snippet=result["payload"].get("content", "")[:200] + "..."
                )
                for result in all_chunks
            ]

            # 6. Higher confidence for context-based (we have exact context)
            confidence = 0.90

            logger.info(
                "rag_context_completed",
                sources_count=len(sources),
                confidence=confidence
            )

            return answer, sources, confidence

        except Exception as e:
            logger.error("rag_context_error", error=str(e))
            raise
