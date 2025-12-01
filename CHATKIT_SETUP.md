# ChatKit Integration - Complete Setup Guide

✅ **ChatKit SDK is now integrated** - Your RAG chatbot now uses OpenAI's official ChatKit UI library!

## What Was Implemented

### Frontend (ChatKit SDK)
- ✅ Installed `@openai/chatkit-react` package
- ✅ Created ChatKit wrapper component (`website/src/components/ChatbotChatKit/`)
- ✅ Integrated text selection "Ask about this" feature
- ✅ Added custom citation rendering for sources
- ✅ Beautiful floating chat button with animations
- ✅ Responsive design for mobile and desktop
- ✅ Dark mode support

### Backend (ChatKit-Compatible API)
- ✅ Created `/api/v1/chatkit/chat` endpoint
- ✅ Server-Sent Events (SSE) streaming support
- ✅ OpenAI Chat Completion API format compatibility
- ✅ RAG integration with Qdrant and OpenAI Agent
- ✅ Chat history persistence to Neon Postgres
- ✅ CORS configuration for frontend
- ✅ Health check endpoint

## Quick Start

### Step 1: Set Up Environment Variables

```bash
# Copy the example file
cp .env.example .env

# Edit .env with your actual credentials
```

Required variables:
```bash
# Backend
BACKEND_API_KEY=generate-random-key-here  # Use: openssl rand -hex 32
OPENAI_API_KEY=sk-...                     # From platform.openai.com
QDRANT_URL=https://xxx.qdrant.io         # From qdrant.tech
QDRANT_API_KEY=...                        # From qdrant.tech
DATABASE_URL=postgresql://...             # From neon.tech

# Frontend (Docusaurus)
REACT_APP_BACKEND_URL=http://localhost:8000/api/v1
REACT_APP_CHATBOT_API_KEY=same-as-backend-api-key
```

### Step 2: Initialize Database

```bash
# Connect to your Neon database
psql $DATABASE_URL

# Run the initialization script
\i backend/scripts/init_db.sql

# Or copy/paste the contents into Neon SQL Editor
```

Verify:
```sql
SELECT * FROM chat_history LIMIT 1;
```

### Step 3: Set Up Qdrant Collection

The collection will be created automatically on first ingestion, but you can verify:

```bash
cd backend
source venv/bin/activate  # Windows: venv\Scripts\activate

python -c "from qdrant_client import QdrantClient; from backend.app.config import settings; client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY); print('Collections:', client.get_collections())"
```

### Step 4: Ingest Documents

```bash
# Make sure you're in the project root and venv is activated
python -m backend.scripts.ingest
```

Expected output:
```
INFO - Ingesting: ./docs/chapter-1-intro/01-what-is-physical-ai.md
INFO - Successfully ingested 15 chunks from ./docs/chapter-1-intro/01-what-is-physical-ai.md
...
```

This takes 5-10 minutes. It will:
- Read all `.md` files from `docs/`
- Generate embeddings using OpenAI
- Upload to Qdrant

### Step 5: Start Backend Server

```bash
cd backend
uvicorn backend.app.main:app --reload --host 0.0.0.0 --port 8000
```

Test the endpoints:
```bash
# Health check
curl http://localhost:8000/

# ChatKit health check
curl http://localhost:8000/api/v1/chatkit/health

# Test chat (replace YOUR_API_KEY)
curl -X POST http://localhost:8000/api/v1/chatkit/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: YOUR_API_KEY" \
  -d '{
    "messages": [{"role": "user", "content": "What is physical AI?"}],
    "stream": false
  }'
```

### Step 6: Start Frontend

```bash
cd website
npm start
```

The site will open at http://localhost:3000

### Step 7: Test ChatKit Integration

1. **Open the website** at http://localhost:3000
2. **Look for the floating chat button** (💬) in the bottom-right corner
3. **Click it** to open the ChatKit interface
4. **Ask a question**: "What is physical AI?"
5. **Verify**:
   - Response appears with streaming effect
   - Citations show up below the answer
   - Citations are clickable and link to book sections
6. **Test text selection**:
   - Navigate to any chapter page
   - Select some text (at least 10 characters)
   - Click the "💬 Ask about this" button that appears
   - Verify it answers based on selected text

## ChatKit-Specific Features

### 1. Floating Chat Button
- Located bottom-right corner
- Smooth animations on hover/click
- Changes color when open (blue → red)
- Mobile responsive

### 2. Text Selection Integration
- Select any text on the page
- Floating "Ask about this" button appears
- Automatically opens chat and sends query
- Auto-removes after 5 seconds if not clicked

### 3. Citation Rendering
- Custom styled citation bubbles
- Clickable links to source sections
- Shows chapter and section names
- Hover effects for better UX

### 4. Streaming Responses
- Real-time word-by-word streaming
- Server-Sent Events (SSE) format
- Smooth animation effect
- 50ms delay between chunks for readability

### 5. Dark Mode Support
- Automatically adapts to Docusaurus theme
- Styled for both light and dark modes
- Proper contrast ratios

## Important: OpenAI Organization Setup

⚠️ **CRITICAL STEP**: Before ChatKit will work, you must whitelist your domain in OpenAI settings.

### How to Whitelist Your Domain

1. Go to https://platform.openai.com/
2. Navigate to **Settings** → **Organization** → **ChatKit Settings**
3. Add these domains:
   - Development: `http://localhost:3000`
   - Development: `http://127.0.0.1:3000`
   - Production: `https://yourdomain.com`

Without this step, ChatKit will not render (security feature).

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     User's Browser                          │
│  ┌────────────────────────────────────────────────────┐    │
│  │  Docusaurus Site (React)                           │    │
│  │  ┌──────────────────────────────────────────┐      │    │
│  │  │  ChatbotChatKit Component                │      │    │
│  │  │  - ChatKit SDK (@openai/chatkit-react)   │      │    │
│  │  │  - Text selection handler                │      │    │
│  │  │  - Custom citation renderer              │      │    │
│  │  └──────────────────────────────────────────┘      │    │
│  └────────────────────────────────────────────────────┘    │
└──────────────────┬──────────────────────────────────────────┘
                   │ HTTP/SSE
                   ▼
┌─────────────────────────────────────────────────────────────┐
│              FastAPI Backend (Python)                       │
│  ┌────────────────────────────────────────────────────┐    │
│  │  /api/v1/chatkit/chat (SSE Streaming)             │    │
│  │  ┌──────────────────────────────────────────┐      │    │
│  │  │  1. Generate embedding (OpenAI)          │      │    │
│  │  │  2. Retrieve chunks (Qdrant)             │      │    │
│  │  │  3. Generate answer (OpenAI Agent)       │      │    │
│  │  │  4. Stream response (SSE format)         │      │    │
│  │  │  5. Save to history (Neon Postgres)      │      │    │
│  │  └──────────────────────────────────────────┘      │    │
│  └────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                   │
                   ├─────► Qdrant Cloud (Vector Search)
                   ├─────► OpenAI API (Embeddings + Chat)
                   └─────► Neon Postgres (Chat History)
```

## Troubleshooting

### ChatKit Not Rendering

**Symptom**: Chat button appears but ChatKit doesn't render

**Solution**:
1. Check browser console for errors
2. Verify domain is whitelisted in OpenAI organization settings
3. Check that `REACT_APP_BACKEND_URL` is correct

### No Responses from Backend

**Symptom**: Messages send but no responses appear

**Solution**:
1. Check backend logs: Look for errors in the terminal running `uvicorn`
2. Verify API key: Ensure `REACT_APP_CHATBOT_API_KEY` matches `BACKEND_API_KEY`
3. Test endpoint directly:
   ```bash
   curl -X POST http://localhost:8000/api/v1/chatkit/chat \
     -H "X-API-Key: YOUR_KEY" \
     -H "Content-Type: application/json" \
     -d '{"messages":[{"role":"user","content":"test"}]}'
   ```
4. Check CORS: Backend logs should show OPTIONS preflight requests

### Empty or Irrelevant Answers

**Symptom**: Bot responds but answers are generic or "I cannot answer"

**Solution**:
1. Verify ingestion completed: Check Qdrant collection count
   ```bash
   python -c "from qdrant_client import QdrantClient; from backend.app.config import settings; client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY); print(client.count(collection_name='physical-ai-book'))"
   ```
2. Expected: 200-500 points (depending on book size)
3. Re-run ingestion if count is low

### Text Selection Not Working

**Symptom**: Selecting text doesn't show "Ask about this" button

**Solution**:
1. Select more than 10 characters (minimum threshold)
2. Check browser console for JavaScript errors
3. Verify component is properly mounted in Root.js

### Citations Not Showing

**Symptom**: Answers appear but no source citations

**Solution**:
1. Check backend response includes `metadata.sources`
2. Verify `renderMessage` function in ChatbotChatKit component
3. Look for sources in browser DevTools Network tab (SSE events)

## File Structure

```
physical-ai-book/
├── backend/
│   └── app/
│       ├── api/
│       │   └── chatkit.py           # ✨ NEW: ChatKit endpoint
│       └── main.py                  # ✅ UPDATED: Added chatkit router
│
├── website/
│   └── src/
│       ├── components/
│       │   ├── Chatbot/             # ❌ OLD: Custom component (keep for reference)
│       │   └── ChatbotChatKit/      # ✨ NEW: ChatKit integration
│       │       ├── index.js         # Main ChatKit wrapper
│       │       └── index.module.css # Styles
│       └── theme/
│           └── Root.js              # ✅ UPDATED: Uses ChatbotChatKit
│
├── .env.example                     # ✅ UPDATED: ChatKit config
├── CHATKIT_INTEGRATION.md          # Documentation
└── CHATKIT_SETUP.md                # This file
```

## Next Steps

1. **Get API Keys**: Sign up for OpenAI, Qdrant, and Neon if you haven't
2. **Configure Environment**: Fill in `.env` with real credentials
3. **Initialize Services**: Run database script and ingestion
4. **Test Locally**: Start both backend and frontend
5. **Whitelist Domain**: Add to OpenAI organization settings
6. **Deploy**: Follow deployment guides for production

## Production Deployment

### Environment Variables in Production

Update `.env` for production:
```bash
# Production Backend URL
REACT_APP_BACKEND_URL=https://api.yourdomain.com/api/v1

# Same API key as backend
REACT_APP_CHATBOT_API_KEY=your-production-api-key
```

### CORS Configuration

Update `backend/app/main.py`:
```python
allow_origins=[
    "http://localhost:3000",
    "https://yourdomain.com",  # Add your production domain
],
```

### OpenAI Domain Whitelist

Add production domain to OpenAI organization settings:
- `https://yourdomain.com`

## Cost Monitoring

Track usage in your dashboards:
- **OpenAI**: https://platform.openai.com/usage
- **Qdrant**: Check cluster metrics
- **Neon**: Check database storage

Expected costs:
- Ingestion: $0.50-2.00 (one-time)
- Per conversation: $0.01-0.05
- Monthly: $5-20 (depending on traffic)

## Support & Resources

- **ChatKit Docs**: https://openai.github.io/chatkit-js/
- **ChatKit GitHub**: https://github.com/openai/chatkit-js
- **FastAPI Docs**: https://fastapi.tiangolo.com/
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Neon Docs**: https://neon.tech/docs/

---

**You're all set!** 🎉 Your RAG chatbot now uses OpenAI's official ChatKit SDK and fully meets your requirements.
