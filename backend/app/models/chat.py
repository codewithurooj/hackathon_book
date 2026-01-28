"""
Pydantic models for chat API requests and responses
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime


class Source(BaseModel):
    """Model for source information"""
    title: str = Field(..., description="Title of the source document")
    url: str = Field(..., description="URL or reference to the source")
    relevance_score: Optional[float] = Field(None, description="Relevance score of the source")
    snippet: Optional[str] = Field(None, description="Snippet or excerpt from the source")


class GeneralChatRequest(BaseModel):
    """Request model for general chat questions"""
    question: str = Field(..., description="User's question")
    conversation_id: Optional[str] = Field(None, description="Conversation ID for tracking")


class ContextChatRequest(BaseModel):
    """Request model for context-based Q&A"""
    question: str = Field(..., description="User's question")
    context: str = Field(..., description="Selected text context from the book")
    conversation_id: Optional[str] = Field(None, description="Conversation ID for tracking")


class ChatResponse(BaseModel):
    """Response model for chat answers"""
    answer: str = Field(..., description="Generated answer to the question")
    sources: List[Source] = Field(default_factory=list, description="List of sources used")
    confidence: float = Field(..., description="Confidence score of the answer")
    processing_time: float = Field(..., description="Time taken to process the request in seconds")


class ChatRequest(BaseModel):
    """Request model for chat API"""
    question: str = Field(..., description="User's question")
    session_id: Optional[str] = Field(None, description="Session ID for conversation history")
    selected_text: Optional[str] = Field(
        None, 
        description="Optional selected text for context-based questions"
    )


class ChatResponseChunk(BaseModel):
    """Response model for chat response chunks"""
    answer_chunk: str = Field(..., description="Chunk of the generated answer")
    sources: Optional[List[Source]] = Field(None, description="Sources for the answer")
    session_id: str = Field(..., description="Session ID for the conversation")
    history: List[Dict[str, Any]] = Field(default_factory=list, description="Full chat history for the session")


class HealthResponse(BaseModel):
    """Response model for health check"""
    status: str = Field(..., description="Overall status of the service")
    environment: str = Field(..., description="Environment (dev, staging, prod)")
    timestamp: datetime = Field(..., description="Timestamp of the health check")
    services: Dict[str, str] = Field(..., description="Status of dependent services")