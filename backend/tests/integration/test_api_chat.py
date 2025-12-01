import pytest
from fastapi.testclient import TestClient
from backend.app.main import app

def test_api_chat_root():
    client = TestClient(app)
    response = client.get("/")
    assert response.status_code == 200
    assert response.json() == {"message": "RAG Chatbot API is running!"}
