# Physical AI & Humanoid Robotics - Backend API

FastAPI backend with RAG (Retrieval Augmented Generation) for the Physical AI and Humanoid Robotics textbook chatbot.

## Features

- **General Q&A**: Ask questions about any Physical AI and Humanoid Robotics topic covered in the book
- **Context-Based Q&A**: Select text and ask questions about it specifically
- **RAG Pipeline**: Uses OpenAI embeddings, Qdrant vector search, and GPT-4 for accurate answers
- **Source Citations**: Every answer includes relevant sources from the book
- **Health Monitoring**: Health check endpoint for service status

## Tech Stack

- **FastAPI**: Modern Python web framework
- **OpenAI API**: Embeddings (text-embedding-3-small) and Chat (gpt-4o)
- **Qdrant Cloud**: Vector database for semantic search
- **Pydantic**: Data validation and settings management
- **Structlog**: Structured logging

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Edit `.env` with your credentials:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-your-actual-api-key

# Qdrant Configuration (free tier at https://cloud.qdrant.io)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=physical-ai-book

# API Configuration
API_BASE_URL=http://localhost:8000
CORS_ORIGINS=["http://localhost:3000"]
BACKEND_API_KEY=your-secure-api-key-here
```

### 3. Ingest Book Content

Run the ingestion script to populate Qdrant with book content:

```bash
python scripts/ingest_book.py
```

This will:
- Create the Qdrant collection
- Generate embeddings for all book chapters
- Upload vectors and metadata to Qdrant

### 4. Start the Server

```bash
# Development mode with auto-reload
python -m app.main

# Or using uvicorn directly
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Server will be running at:
- API: http://localhost:8000
- Docs: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## API Endpoints

### Health Check

```bash
GET /api/health
```

Returns service status and configuration.

### General Q&A

```bash
POST /api/chat/general
Content-Type: application/json

{
  "question": "What is embodied AI?",
  "conversation_id": "optional-uuid"
}
```

Response:
```json
{
  "answer": "Embodied AI refers to artificial intelligence systems that interact with the physical world through sensors and actuators...",
  "sources": [
    {
      "title": "Chapter 1: Foundations of Physical AI",
      "url": "/docs/chapter-01-physical-ai",
      "relevance_score": 0.95,
      "snippet": "Physical AI systems must integrate perception, reasoning, and action..."
    }
  ],
  "confidence": 0.92,
  "processing_time": 1.23
}
```

### Context-Based Q&A

```bash
POST /api/chat/context
Content-Type: application/json

{
  "question": "What are the main components?",
  "context": "Humanoid robots are complex systems that integrate multiple technologies...",
  "conversation_id": "optional-uuid"
}
```

### ChatKit Streaming Endpoint (Used by Frontend)

```bash
POST /api/v1/chatkit/chat
Content-Type: application/json
X-API-Key: your-backend-api-key

{
  "messages": [
    {"role": "user", "content": "What is deep reinforcement learning?"}
  ],
  "selected_text": "Optional selected text from book for context",
  "stream": true,
  "session_id": "session-123"
}
```

Response (Server-Sent Events):
```
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"content": "Deep..."}}]}
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"metadata": {"sources": [...]}}}]}
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {}, "finish_reason": "stop"}]}
data: [DONE]
```

## Project Structure

```
backend/
├── app/
│   ├── main.py              # FastAPI application
│   ├── models/
│   │   └── chat.py          # Pydantic models
│   ├── routers/
│   │   ├── health.py        # Health check endpoint
│   │   └── chat.py          # Chat endpoints
│   ├── services/
│   │   ├── embedding_service.py   # OpenAI embeddings
│   │   ├── qdrant_service.py      # Vector database
│   │   ├── agent_service.py       # AI agent (GPT-4)
│   │   └── rag_service.py         # RAG orchestration
│   └── utils/
│       └── config.py        # Configuration management
├── scripts/
│   └── ingest_book.py       # Content ingestion script
├── tests/                   # Test files
├── requirements.txt         # Python dependencies
├── .env.example            # Environment template
└── README.md               # This file
```

## Development

### Running Tests

```bash
pytest tests/
```

### Code Formatting

```bash
black app/ scripts/
```

### Type Checking

```bash
mypy app/
```

## Deployment

### Railway

1. Create new project on Railway
2. Add environment variables from `.env`
3. Deploy from GitHub repository
4. Run ingestion script in Railway console:
   ```bash
   python scripts/ingest_book.py
   ```

### Render

1. Create new Web Service
2. Set environment variables
3. Build command: `pip install -r requirements.txt`
4. Start command: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Run ingestion via Render Shell

## Troubleshooting

### "Collection not found" error
- Run the ingestion script: `python scripts/ingest_book.py`

### CORS errors
- Check `CORS_ORIGINS` in `.env` matches your frontend URL
- For development: `["http://localhost:3000"]`

### OpenAI API errors
- Verify your API key in `.env`
- Check API key has sufficient credits
- Ensure correct model names (gpt-4o, text-embedding-3-small)

### Qdrant connection errors
- Verify Qdrant URL and API key
- Check cluster is running in Qdrant Cloud dashboard
- Free tier has rate limits - consider upgrade for production

## Contributing

1. Create a feature branch
2. Make changes with tests
3. Run tests and linting
4. Submit pull request

## License

MIT
