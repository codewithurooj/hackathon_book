"""
Chat endpoints for general and context-based Q&A
"""
from fastapi import APIRouter, HTTPException, Request, Depends
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from typing import List, Dict, Optional
from app.models.chat import (
    GeneralChatRequest,
    ContextChatRequest,
    ChatResponse,
    Source
)
from app.services.rag_service import RAGService
import time
import structlog
import json
import uuid


router = APIRouter(tags=["chat"])
logger = structlog.get_logger()

# Initialize RAG service
rag_service = RAGService()


@router.post("/chat/general", response_model=ChatResponse)
async def general_chat(request: GeneralChatRequest):
    """
    General Q&A endpoint
    Uses RAG to search entire book and answer questions
    """
    start_time = time.time()

    try:
        logger.info(
            "general_chat_request",
            question=request.question[:100],
            conversation_id=request.conversation_id
        )

        # Use RAG service to answer question
        answer, sources, confidence = await rag_service.answer_general_question(
            question=request.question,
            top_k=5
        )
        sources = [] # Remove sources as per user request

        processing_time = time.time() - start_time

        logger.info(
            "general_chat_response",
            processing_time=processing_time,
            sources_count=len(sources),
            confidence=confidence
        )

        return ChatResponse(
            answer=answer,
            sources=sources,
            confidence=confidence,
            processing_time=processing_time
        )

    except Exception as e:
        logger.error("general_chat_error", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/chat/context", response_model=ChatResponse)
async def context_chat(request: ContextChatRequest):
    """
    Context-based Q&A endpoint
    Uses selected text as primary context for answering
    """
    start_time = time.time()

    try:
        logger.info(
            "context_chat_request",
            question=request.question[:100],
            context_length=len(request.context),
            conversation_id=request.conversation_id
        )

        # Use RAG service to answer with context
        answer, sources, confidence = await rag_service.answer_context_question(
            question=request.question,
            context=request.context,
            top_k=3
        )
        sources = [] # Remove sources as per user request

        processing_time = time.time() - start_time

        logger.info(
            "context_chat_response",
            processing_time=processing_time,
            sources_count=len(sources),
            confidence=confidence
        )

        return ChatResponse(
            answer=answer,
            sources=sources,
            confidence=confidence,
            processing_time=processing_time
        )

    except Exception as e:
        logger.error("context_chat_error", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


# ChatKit-compatible endpoint
class ChatKitMessage(BaseModel):
    role: str = Field(..., description="Message role: 'user', 'assistant', or 'system'")
    content: str = Field(..., description="Message content")


class ChatKitRequest(BaseModel):
    messages: List[ChatKitMessage] = Field(..., description="List of conversation messages")
    stream: Optional[bool] = Field(default=True, description="Whether to stream the response")
    session_id: Optional[str] = Field(default=None, description="Session ID for chat history")


@router.post("/chatkit/chat")
async def chatkit_chat(
    request: ChatKitRequest,
    api_key: str = Depends(lambda: __import__('app.api.auth', fromlist=['get_api_key']).get_api_key)
):
    """
    ChatKit-compatible endpoint that follows OpenAI's Chat Completion API format.

    This endpoint:
    1. Accepts messages in ChatKit/OpenAI format
    2. Extracts the user's question
    3. Performs RAG retrieval from Qdrant
    4. Generates an answer using OpenAI Agent
    5. Streams the response in Server-Sent Events (SSE) format
    """
    logger.info(f"ChatKit endpoint called with {len(request.messages)} messages")

    # Get the last user message
    user_message = next(
        (msg.content for msg in reversed(request.messages) if msg.role == "user"),
        None,
    )

    if not user_message:
        raise HTTPException(status_code=400, detail="No user message found in request")

    try:
        # Use RAG service to answer the question
        # For simplicity, using general question approach here
        answer, sources, confidence = await rag_service.answer_general_question(
            question=user_message,
            top_k=5
        )
        sources = [] # Remove sources as per user request

        # Stream response in ChatKit/OpenAI format
        async def generate_stream():
            """
            Generate Server-Sent Events (SSE) stream compatible with ChatKit.
            Format follows OpenAI's Chat Completion API streaming format.
            """
            chunk_id = f"chatcmpl-{uuid.uuid4().hex[:8]}"
            timestamp = int(time.time())

            # Split answer into chunks for streaming
            words = answer.split()
            chunk_size = 5  # Send 5 words at a time

            for i in range(0, len(words), chunk_size):
                chunk_text = " ".join(words[i : i + chunk_size]) + " "

                chunk_data = {
                    "id": chunk_id,
                    "object": "chat.completion.chunk",
                    "created": timestamp,
                    "model": "gpt-4o",
                    "choices": [
                        {
                            "index": 0,
                            "delta": {"role": "assistant", "content": chunk_text},
                            "finish_reason": None,
                        }
                    ],
                }

                yield f"data: {json.dumps(chunk_data)}\n\n"

                # Small delay for streaming effect
                import asyncio
                await asyncio.sleep(0.05)

            # Send sources as metadata in a final chunk
            if sources:
                # Format sources properly for frontend consumption
                formatted_sources = []
                for source in sources:
                    source_dict = source.dict()
                    # Map the fields to what the frontend expects
                    formatted_sources.append({
                        "title": source_dict.get("title", "Unknown"),
                        "url": source_dict.get("url", "#"),
                        "content": source_dict.get("snippet", source_dict.get("content", "")),
                        "snippet": source_dict.get("snippet", source_dict.get("content", ""))
                    })

                metadata_chunk = {
                    "id": chunk_id,
                    "object": "chat.completion.chunk",
                    "created": timestamp,
                    "model": "gpt-4o",
                    "choices": [
                        {
                            "index": 0,
                            "delta": {
                                "role": "assistant",
                                "content": "",
                                "metadata": {"sources": formatted_sources},
                            },
                            "finish_reason": None,
                        }
                    ],
                }
                yield f"data: {json.dumps(metadata_chunk)}\n\n"

            # Send finish event
            finish_data = {
                "id": chunk_id,
                "object": "chat.completion.chunk",
                "created": timestamp,
                "model": "gpt-4o",
                "choices": [
                    {"index": 0, "delta": {}, "finish_reason": "stop"}
                ],
            }
            yield f"data: {json.dumps(finish_data)}\n\n"
            yield "data: [DONE]\n\n"

        # Return streaming response
        return StreamingResponse(
            generate_stream(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "Connection": "keep-alive",
                "X-Accel-Buffering": "no",  # Disable proxy buffering
            },
        )

    except Exception as e:
        logger.exception(f"Error in ChatKit endpoint: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"An error occurred while processing your request: {str(e)}",
        )
