from sqlalchemy.orm import Session
from backend.app.database import get_db, Base
from sqlalchemy import Column, Integer, String, Text, DateTime, JSON
from datetime import datetime
from typing import List, Dict, Optional

# Define the SQLAlchemy model for chat_history table
class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, index=True, nullable=False)
    question = Column(Text, nullable=False)
    answer = Column(Text, nullable=False)
    sources = Column(JSON, nullable=True) # Storing list of dicts as JSONB
    created_at = Column(DateTime, default=datetime.utcnow)

    def to_dict(self):
        return {
            "id": self.id,
            "session_id": self.session_id,
            "question": self.question,
            "answer": self.answer,
            "sources": self.sources,
            "created_at": self.created_at.isoformat() if self.created_at else None,
        }

def save_chat_history(
    db: Session,
    session_id: str,
    question: str,
    answer: str,
    sources: Optional[List[Dict]] = None
) -> ChatHistory:
    """
    Saves a chat interaction to the chat_history table.
    """
    db_chat = ChatHistory(
        session_id=session_id,
        question=question,
        answer=answer,
        sources=sources,
        created_at=datetime.utcnow()
    )
    db.add(db_chat)
    db.commit()
    db.refresh(db_chat)
    return db_chat

def get_chat_history(db: Session, session_id: str) -> List[ChatHistory]:
    """
    Retrieves all chat interactions for a given session ID.
    """
    return db.query(ChatHistory).filter(ChatHistory.session_id == session_id).order_by(ChatHistory.created_at).all()
