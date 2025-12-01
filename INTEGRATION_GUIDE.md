# RAG Chatbot Integration Guide

This guide will help you complete the integration of the RAG chatbot with your Physical AI textbook.

## Prerequisites

You need accounts and API keys for:
1. **OpenAI** - For embeddings and chat completions
2. **Qdrant Cloud** - Free tier for vector storage
3. **Neon** - Serverless Postgres database

## Step-by-Step Integration

### 1. Environment Setup

#### 1.1 Create your `.env` file

```bash
cp .env.example .env
```

#### 1.2 Fill in your API keys and connection strings

Edit `.env` with your actual credentials:

- `BACKEND_API_KEY`: Create a strong random key (e.g., using `openssl rand -hex 32`)
- `OPENAI_API_KEY`: Get from https://platform.openai.com/api-keys
- `QDRANT_URL` and `QDRANT_API_KEY`: Get from https://qdrant.tech/
- `DATABASE_URL`: Get from https://neon.tech/

### 2. Database Initialization

#### 2.1 Initialize Neon Postgres

1. Log in to your Neon dashboard
2. Create a new project (if you haven't already)
3. Copy the connection string to `DATABASE_URL` in `.env`
4. Run the SQL initialization script:

```bash
# Connect to your Neon database
psql $DATABASE_URL

# Or if you have the Neon CLI:
psql -h <your-host>.neon.tech -U <your-user> -d <your-db>
```

5. Execute the schema from `backend/scripts/init_db.sql`:

```sql
\i backend/scripts/init_db.sql
```

Alternatively, copy and paste the contents of `init_db.sql` into the Neon SQL Editor in your dashboard.

#### 2.2 Verify Database Setup

```bash
# Test connection
python -c "from backend.app.database import engine; print('Database connected successfully!')"
```

### 3. Qdrant Setup

#### 3.1 Create Qdrant Cluster

1. Sign up at https://qdrant.tech/
2. Create a free cluster
3. Get your cluster URL and API key
4. Add them to `.env`

#### 3.2 Test Qdrant Connection

```bash
cd backend
python -c "from qdrant_client import QdrantClient; from backend.app.config import settings; client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY); print('Qdrant connected:', client.get_collections())"
```

### 4. Document Ingestion

This step populates your vector database with the book content.

#### 4.1 Install Backend Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

#### 4.2 Run the Ingestion Script

```bash
# Make sure you're in the project root directory
python -m backend.scripts.ingest
```

This will:
- Read all `.md` files from the `docs/` directory
- Generate embeddings for each chunk
- Upload vectors to Qdrant
- Take approximately 5-10 minutes depending on content size

#### 4.3 Verify Ingestion

```bash
python -c "from qdrant_client import QdrantClient; from backend.app.config import settings; client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY); print('Total points:', client.count(collection_name='physical-ai-book').count)"
```

### 5. Backend Testing

#### 5.1 Start the Backend Server

```bash
cd backend
uvicorn backend.app.main:app --reload --host 0.0.0.0 --port 8000
```

#### 5.2 Test the API

Open another terminal and test the endpoint:

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-backend-api-key" \
  -d '{"question": "What is physical AI?", "session_id": "test-session"}'
```

Expected response should include:
- `answer_chunk`: The generated answer
- `sources`: Citations from the book
- `session_id`: Your session ID
- `history`: Chat history

### 6. Frontend Integration

#### 6.1 Update Frontend Configuration

Edit `website/src/components/Chatbot/index.js` and replace the mock `sendMessageToBackend` function with the real implementation:

```javascript
const sendMessageToBackend = async (message, textContext = null) => {
  const endpoint = textContext
    ? `${process.env.REACT_APP_BACKEND_URL}/chat/selected`
    : `${process.env.REACT_APP_BACKEND_URL}/chat`;

  const response = await fetch(endpoint, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY,
    },
    body: JSON.stringify({
      question: message,
      selected_text: textContext,
      session_id: sessionId,
    }),
  });

  if (!response.ok) {
    throw new Error(`API error: ${response.status}`);
  }

  const data = await response.json();
  return data;
};
```

#### 6.2 Update Docusaurus Config

Edit `website/docusaurus.config.ts` to include environment variables:

```typescript
customFields: {
  backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/api/v1',
  chatbotApiKey: process.env.REACT_APP_CHATBOT_API_KEY,
},
```

#### 6.3 Start the Frontend

```bash
cd website
npm install  # If not already done
npm start
```

### 7. End-to-End Testing

1. **Open the book** at http://localhost:3000
2. **Click the chat button** (should appear in the bottom right)
3. **Ask a question** like "What is physical AI?"
4. **Verify**:
   - You get a relevant answer
   - Sources are displayed with clickable links
   - Chat history persists when you close/reopen
5. **Test text selection**:
   - Select some text on any page
   - Click "Ask about this"
   - Verify it answers based on selected text

### 8. Deployment

#### 8.1 Backend Deployment Options

**Option A: Render.com** (Recommended for Free Tier)
1. Connect your GitHub repository
2. Create a new Web Service
3. Set build command: `pip install -r backend/requirements.txt`
4. Set start command: `uvicorn backend.app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables from `.env`

**Option B: Railway.app**
Similar process to Render.com

**Option C: Vercel** (Serverless)
Use the Vercel Python runtime

#### 8.2 Frontend Deployment

The Docusaurus site can be deployed to:
- **Vercel** (Recommended)
- **Netlify**
- **GitHub Pages**
- **Cloudflare Pages**

Update environment variables in your deployment platform.

#### 8.3 Production Environment Variables

Make sure to set these in your deployment platform:
- All variables from `.env.example`
- Update `REACT_APP_BACKEND_URL` to your production backend URL
- Ensure CORS is configured in `backend/app/main.py` if frontend and backend are on different domains

## Troubleshooting

### Common Issues

**1. "Database connection failed"**
- Verify `DATABASE_URL` is correct
- Check Neon dashboard for connection status
- Ensure SSL mode is included: `?sslmode=require`

**2. "Qdrant connection failed"**
- Verify cluster URL and API key
- Check if cluster is active in Qdrant dashboard
- Test with the verification command above

**3. "No embeddings generated"**
- Check OpenAI API key validity
- Verify billing is set up on OpenAI account
- Check API usage limits

**4. "Frontend can't connect to backend"**
- Verify `REACT_APP_BACKEND_URL` is correct
- Check CORS configuration
- Verify API key is being sent correctly
- Check browser console for errors

**5. "Empty or irrelevant responses"**
- Ensure ingestion completed successfully
- Verify Qdrant has data (use count command)
- Try asking questions closer to actual book content

### Debugging Commands

```bash
# Check backend logs
uvicorn backend.app.main:app --log-level debug

# Test OpenAI connection
python -c "from openai import OpenAI; from backend.app.config import settings; client = OpenAI(api_key=settings.OPENAI_API_KEY); print(client.models.list())"

# Check database tables
psql $DATABASE_URL -c "SELECT table_name FROM information_schema.tables WHERE table_schema='public';"

# Query chat history
psql $DATABASE_URL -c "SELECT * FROM chat_history LIMIT 5;"
```

## Cost Estimates

Based on your textbook size (~5 chapters):

- **OpenAI Embeddings**: ~$0.50-2.00 for initial ingestion
- **OpenAI Chat**: ~$0.01-0.05 per conversation (10-20 messages)
- **Qdrant Free Tier**: Sufficient for this project (1GB)
- **Neon Free Tier**: Sufficient for this project (0.5GB)

**Monthly recurring costs**: ~$5-20 depending on usage

## Next Steps

After successful integration:

1. **Monitor usage** in OpenAI, Qdrant, and Neon dashboards
2. **Gather feedback** from users
3. **Iterate on prompts** in `backend/app/services/openai_agent.py` to improve answers
4. **Add analytics** to track popular questions
5. **Implement caching** for common questions to reduce costs

## Need Help?

- Check the spec: `specs/006-rag-chatbot/spec.md`
- Review the plan: `specs/006-rag-chatbot/plan.md`
- See tasks: `specs/006-rag-chatbot/tasks.md`
- Research notes: `specs/006-rag-chatbot/research.md`
