from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session
from typing import AsyncGenerator, List
import json
import uuid

from app.api.auth import get_api_key
from app.database import get_db
from app.models.chat import ChatRequest, ChatResponseChunk, Source
from app.services.rag_service import RAGService
from app.services.chat_history_service import save_chat_history, get_chat_history, ChatHistory

router = APIRouter()

@router.post("/chat", response_model=ChatResponseChunk)
async def chat(request: ChatRequest, api_key: str = Depends(get_api_key), db: Session = Depends(get_db)):
    """
    Handles general chat questions using RAG.
    """
    session_id = request.session_id if request.session_id else str(uuid.uuid4())

    # Initialize RAG service
    rag_service = RAGService()

    # Generate answer based on whether selected text is provided
    if request.selected_text:
        # Use context-based question answering with selected text
        answer, sources, confidence = await rag_service.answer_context_question(
            question=request.question,
            context=request.selected_text,
            top_k=3
        )
    else:
        # Use general question answering
        answer, sources, confidence = await rag_service.answer_general_question(
            question=request.question,
            top_k=5
        )

    # Format sources to match expected structure
    formatted_sources = [
        {
            "title": source.title,
            "url": source.url,
            "snippet": source.snippet
        }
        for source in sources
    ]

    # Save interaction to chat history
    saved_chat = save_chat_history(
        db=db,
        session_id=session_id,
        question=request.question,
        answer=answer,
        sources=formatted_sources  # Save sources to DB
    )

    # Retrieve full chat history for the session
    history_records = get_chat_history(db=db, session_id=session_id)
    full_history = [hr.to_dict() for hr in history_records]

    # For now, simulate streaming by yielding the full answer in one chunk
    async def generate_chunks():
        sources_list = [s.model_dump() for s in sources] if sources else []
        yield json.dumps(ChatResponseChunk(
            answer_chunk=answer,
            sources=sources_list,
            session_id=session_id,
            history=full_history  # Include full history in the response
        ).model_dump(mode='json')) + "\n"  # Use model_dump for Pydantic v2

    return StreamingResponse(generate_chunks(), media_type="application/json")
