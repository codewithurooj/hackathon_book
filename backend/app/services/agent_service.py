"""
Agent service using OpenAI API
"""
from openai import OpenAI
from app.utils.config import settings
from typing import List, Dict, Any
import structlog

logger = structlog.get_logger()


class AgentService:
    """Service for AI agent interactions"""

    def __init__(self):
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = settings.CHAT_MODEL

    async def generate_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]],
        mode: str = "general"
    ) -> str:
        """
        Generate answer using retrieved context

        Args:
            question: User's question
            context_chunks: Retrieved context from vector search
            mode: "general" or "context" mode

        Returns:
            Generated answer
        """
        try:
            # Build context from chunks - don't include source titles to prevent AI from citing them
            context_text = "\n\n".join([
                chunk['payload'].get('content', '')
                for chunk in context_chunks
            ])

            # Create system prompt
            if mode == "context":
                system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
Based on the provided context, answer the user's question thoroughly but concisely.
Explain concepts clearly using the information from the context.

IMPORTANT: Do NOT include source citations or chapter references in your answer.
Do NOT say things like "Source: Chapter X" or "According to the textbook...".
The sources will be shown separately to the user. Just provide a clear, direct answer."""
            else:
                system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
Use the retrieved context to provide accurate and helpful answers to the user's question.
Explain concepts clearly and thoroughly.

IMPORTANT: Do NOT include source citations or chapter references in your answer.
Do NOT say things like "Source: Chapter X" or "According to the textbook...".
The sources will be shown separately to the user. Just provide a clear, direct answer."""

            # Create messages
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {question}"}
            ]

            # Generate response
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=500  # Allow for more comprehensive answers
            )

            answer = response.choices[0].message.content
            logger.info("answer_generated", question_length=len(question), answer_length=len(answer))

            return answer

        except Exception as e:
            logger.error("generate_answer_error", error=str(e))
            raise

    async def generate_streaming_answer(
        self,
        question: str,
        context_chunks: List[Dict[str, Any]],
        mode: str = "general"
    ):
        """
        Generate streaming answer (for future implementation)

        Args:
            question: User's question
            context_chunks: Retrieved context from vector search
            mode: "general" or "context" mode

        Yields:
            Answer chunks
        """
        try:
            # Build context from chunks - don't include source titles to prevent AI from citing them
            context_text = "\n\n".join([
                chunk['payload'].get('content', '')
                for chunk in context_chunks
            ])

            # Create system prompt
            if mode == "context":
                system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
Based on the provided context, answer the user's question thoroughly but concisely.
Explain concepts clearly using the information from the context.

IMPORTANT: Do NOT include source citations or chapter references in your answer.
Do NOT say things like "Source: Chapter X" or "According to the textbook...".
The sources will be shown separately to the user. Just provide a clear, direct answer."""
            else:
                system_prompt = """You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
Use the retrieved context to provide accurate and helpful answers to the user's question.
Explain concepts clearly and thoroughly.

IMPORTANT: Do NOT include source citations or chapter references in your answer.
Do NOT say things like "Source: Chapter X" or "According to the textbook...".
The sources will be shown separately to the user. Just provide a clear, direct answer."""

            # Create messages
            messages = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"Context:\n{context_text}\n\nQuestion: {question}"}
            ]

            # Generate streaming response
            stream = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=500,  # Allow for comprehensive answers
                stream=True
            )

            for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except Exception as e:
            logger.error("streaming_answer_error", error=str(e))
            raise
