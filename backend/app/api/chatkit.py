from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel, Field
from typing import List, Dict, Optional
import json
import time
import uuid

from backend.app.api.auth import get_api_key
from backend.app.services.openai_service import generate_embedding
from backend.app.services.qdrant_service import search_qdrant
from backend.app.services.openai_agent import generate_answer_from_context
from backend.app.database import get_db
from backend.app.services.chat_history_service import save_chat_history
from sqlalchemy.orm import Session
import logging

logger = logging.getLogger(__name__)

router = APIRouter()


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
    api_key: str = Depends(get_api_key),
    db: Session = Depends(get_db),
):
    """
    ChatKit-compatible endpoint that follows OpenAI's Chat Completion API format.

    This endpoint:
    1. Accepts messages in ChatKit/OpenAI format
    2. Extracts the user's question
    3. Performs RAG retrieval from Qdrant
    4. Generates an answer using OpenAI Agent
    5. Streams the response in Server-Sent Events (SSE) format
    6. Saves to chat history
    """
    logger.info(f"ChatKit endpoint called with {len(request.messages)} messages")

    # Get the last user message
    user_message = next(
        (msg.content for msg in reversed(request.messages) if msg.role == "user"),
        None,
    )

    if not user_message:
        raise HTTPException(status_code=400, detail="No user message found in request")

    # Generate session_id if not provided
    session_id = request.session_id or str(uuid.uuid4())

    # Check if the message contains selected text context
    # Format: "Please explain this text from the textbook: \"<selected_text>\""
    selected_text = None
    if 'Please explain this text from the textbook:' in user_message:
        # Extract text between quotes
        import re
        match = re.search(r'"([^"]*)"', user_message)
        if match:
            selected_text = match.group(1)
            logger.info(f"Detected selected text query: {selected_text[:50]}...")

    try:
        # Perform RAG retrieval
        context_chunks = []

        if selected_text:
            # Use selected text as context
            context_chunks = [{"text_chunk": selected_text}]
            logger.info("Using selected text as context")
        else:
            # Generate embedding for the question
            query_embedding = await generate_embedding(user_message)

            # Retrieve relevant chunks from Qdrant
            context_chunks = await search_qdrant(query_embedding)
            logger.info(f"Retrieved {len(context_chunks)} context chunks from Qdrant")

        # Generate answer using OpenAI Agent
        agent_response = await generate_answer_from_context(user_message, context_chunks)

        # Save to chat history
        save_chat_history(
            db=db,
            session_id=session_id,
            question=user_message,
            answer=agent_response.answer,
            sources=agent_response.sources,
        )
        logger.info(f"Saved chat history for session {session_id}")

        # Stream response in ChatKit/OpenAI format
        async def generate_stream():
            """
            Generate Server-Sent Events (SSE) stream compatible with ChatKit.
            Format follows OpenAI's Chat Completion API streaming format.
            """
            chunk_id = f"chatcmpl-{uuid.uuid4().hex[:8]}"
            timestamp = int(time.time())

            # Split answer into words for streaming effect
            words = agent_response.answer.split()
            chunk_size = 5  # Send 5 words at a time

            for i in range(0, len(words), chunk_size):
                chunk_text = " ".join(words[i : i + chunk_size]) + " "

                chunk_data = {
                    "id": chunk_id,
                    "object": "chat.completion.chunk",
                    "created": timestamp,
                    "model": "gpt-4o-mini",
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
            if agent_response.sources:
                metadata_chunk = {
                    "id": chunk_id,
                    "object": "chat.completion.chunk",
                    "created": timestamp,
                    "model": "gpt-4o-mini",
                    "choices": [
                        {
                            "index": 0,
                            "delta": {
                                "role": "assistant",
                                "content": "",
                                "metadata": {"sources": agent_response.sources},
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
                "model": "gpt-4o-mini",
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


@router.get("/chatkit/health")
async def chatkit_health():
    """Health check endpoint for ChatKit integration."""
    return {
        "status": "healthy",
        "service": "chatkit",
        "message": "ChatKit backend is operational",
    }
