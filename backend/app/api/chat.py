from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.responses import StreamingResponse
from sqlalchemy.orm import Session
from typing import AsyncGenerator, List
import json
import uuid

from backend.app.api.auth import get_api_key
from backend.app.database import get_db
from backend.app.models.chat import ChatRequest, ChatResponseChunk, Source
from backend.app.services.openai_service import generate_embedding
from backend.app.services.qdrant_service import search_qdrant
from backend.app.services.openai_agent import generate_answer_from_context, AgentResponse
from backend.app.services.chat_history_service import save_chat_history, get_chat_history, ChatHistory

router = APIRouter()

@router.post("/chat", response_model=ChatResponseChunk)
async def chat(request: ChatRequest, api_key: str = Depends(get_api_key), db: Session = Depends(get_db)):
    """
    Handles general chat questions using RAG.
    """
    session_id = request.session_id if request.session_id else str(uuid.uuid4())

    context_chunks = []
    if request.selected_text:
        # If selected_text is provided, bypass Qdrant and use it directly for context
        context_chunks = [{"text_chunk": request.selected_text}]
    else:
        # Generate embedding for the user's question
        query_embedding = await generate_embedding(request.question)

        # Retrieve top-k relevant text chunks from Qdrant
        context_chunks = await search_qdrant(query_embedding)
        if not context_chunks:
            # If no context found and no selected text, still try to answer
            # Or raise HTTP exception depending on desired behavior
            pass # Agent will handle no context if passed an empty list

    # Generate answer from context using OpenAI Agent
    agent_response: AgentResponse = await generate_answer_from_context(request.question, context_chunks)

    # Save interaction to chat history
    saved_chat = save_chat_history(
        db=db,
        session_id=session_id,
        question=request.question,
        answer=agent_response.answer,
        sources=agent_response.sources # Save sources to DB
    )

    # Retrieve full chat history for the session
    history_records = get_chat_history(db=db, session_id=session_id)
    full_history = [hr.to_dict() for hr in history_records]

    # For now, simulate streaming by yielding the full answer in one chunk
    async def generate_chunks():
        sources_list = [Source(**s) for s in agent_response.sources] if agent_response.sources else None
        yield json.dumps(ChatResponseChunk(
            answer_chunk=agent_response.answer,
            sources=sources_list,
            session_id=session_id,
            history=full_history # Include full history in the response
        ).model_dump(mode='json')) + "\n" # Use model_dump for Pydantic v2

    return StreamingResponse(generate_chunks(), media_type="application/json")
