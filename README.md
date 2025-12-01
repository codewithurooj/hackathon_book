# Physical AI & Humanoid Robotics Textbook

An interactive textbook with an embedded RAG chatbot powered by OpenAI's ChatKit SDK.

## 🎯 Project Overview

This project combines a comprehensive Physical AI textbook (Docusaurus) with an intelligent RAG (Retrieval-Augmented Generation) chatbot that can answer questions about the book's content.

### Tech Stack

**Frontend:**
- Docusaurus (React-based documentation site)
- **OpenAI ChatKit SDK** (`@openai/chatkit-react`) - Official chat UI
- React 19

**Backend:**
- FastAPI (Python)
- OpenAI API (Embeddings + Chat Completion)
- Qdrant Cloud (Vector Database)
- Neon Serverless Postgres (Chat History)

## 🚀 Quick Start

### Prerequisites

1. **API Keys** (sign up for free tiers):
   - [OpenAI API](https://platform.openai.com/api-keys)
   - [Qdrant Cloud](https://qdrant.tech/)
   - [Neon Postgres](https://neon.tech/)

2. **Software**:
   - Python 3.11+ with `venv`
   - Node.js 20+
   - Git

### Setup (10 minutes)

```bash
# 1. Clone and navigate
cd physical-ai-book

# 2. Set up environment
cp .env.example .env
# Edit .env with your API keys

# 3. Install backend dependencies
cd backend
python -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate
pip install -r requirements.txt

# 4. Initialize database
psql $DATABASE_URL -f scripts/init_db.sql

# 5. Ingest documents (takes 5-10 minutes)
cd ..
python -m backend.scripts.ingest

# 6. Install frontend dependencies
cd website
npm install
```

### Run (2 commands)

**Terminal 1 - Backend:**
```bash
cd backend
uvicorn backend.app.main:app --reload
```

**Terminal 2 - Frontend:**
```bash
cd website
npm start
```

Visit: http://localhost:3000

### ⚠️ Important: OpenAI Domain Whitelist

Before the chatbot will work:
1. Go to https://platform.openai.com/
2. Settings → Organization → ChatKit Settings
3. Add domain: `http://localhost:3000`

Without this, ChatKit won't render (security feature).

## 📚 Documentation

| File | Description |
|------|-------------|
| **[CHATKIT_SETUP.md](CHATKIT_SETUP.md)** | **START HERE** - Complete setup guide |
| [CHATKIT_SUMMARY.md](CHATKIT_SUMMARY.md) | Implementation summary |
| [CHATKIT_INTEGRATION.md](CHATKIT_INTEGRATION.md) | Technical details |
| [INTEGRATION_GUIDE.md](INTEGRATION_GUIDE.md) | General RAG setup |
| [backend/README.md](backend/README.md) | Backend documentation |

## ✅ Verify Setup

Run the verification script:

```bash
cd backend
python verify_chatkit.py
```

This checks:
- ✅ Environment variables
- ✅ Python dependencies
- ✅ Database connection
- ✅ Qdrant connection
- ✅ OpenAI connection
- ✅ File structure

## 🎨 Features

### 1. **ChatKit SDK Integration**
- Official OpenAI chat component
- Professional, streaming UI
- Meets all project requirements

### 2. **RAG Pipeline**
- Questions → Embeddings → Vector search → Context retrieval → AI answer
- Only uses book content (no hallucinations)
- Citations link back to source sections

### 3. **Text Selection**
- Select any text on the page
- Click "Ask about this" button
- Get answers based on selected context

### 4. **Smart Citations**
- Every answer includes sources
- Clickable links to book sections
- Shows chapter and section names

### 5. **Responsive Design**
- Mobile and desktop support
- Dark mode compatible
- Smooth animations

## 🗂 Project Structure

```
physical-ai-book/
├── backend/               # FastAPI backend
│   ├── app/
│   │   ├── api/
│   │   │   ├── chat.py          # Original chat endpoint
│   │   │   └── chatkit.py       # ChatKit SSE endpoint
│   │   ├── services/
│   │   │   ├── openai_service.py    # Embeddings
│   │   │   ├── openai_agent.py      # Chat completion
│   │   │   ├── qdrant_service.py    # Vector search
│   │   │   └── chat_history_service.py
│   │   ├── config.py        # Configuration
│   │   └── main.py          # FastAPI app
│   ├── scripts/
│   │   └── ingest.py        # Document ingestion
│   └── verify_chatkit.py    # Verification script
│
├── website/               # Docusaurus frontend
│   ├── src/
│   │   ├── components/
│   │   │   └── ChatbotChatKit/   # ChatKit integration
│   │   └── theme/
│   │       └── Root.js           # Global wrapper
│   └── package.json
│
├── docs/                  # Book content (markdown)
│   ├── chapter-1-intro/
│   ├── chapter-2-ros2/
│   └── ...
│
└── specs/                 # Planning documents
    └── 006-rag-chatbot/
```

## 🔧 Development

### Backend Development

```bash
# Run backend with auto-reload
cd backend
uvicorn backend.app.main:app --reload --log-level debug

# Run tests
pytest

# Add new dependency
echo "new-package" >> requirements.in
pip-compile --output-file=requirements.txt requirements.in
pip install -r requirements.txt
```

### Frontend Development

```bash
# Run with hot reload
cd website
npm start

# Build for production
npm run build

# Serve production build
npm run serve
```

## 🚢 Deployment

### Backend

**Recommended: Render.com**
1. Connect GitHub repository
2. Create Web Service
3. Build: `pip install -r backend/requirements.txt`
4. Start: `uvicorn backend.app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables

**Alternatives:**
- Railway.app
- Vercel (serverless)
- AWS/GCP/Azure

### Frontend

**Recommended: Vercel**
1. Connect GitHub repository
2. Framework: Docusaurus
3. Build: `cd website && npm run build`
4. Output: `website/build`
5. Add environment variables

**Alternatives:**
- Netlify
- Cloudflare Pages
- GitHub Pages

### Environment Variables

Production `.env`:
```bash
REACT_APP_BACKEND_URL=https://your-api.render.com/api/v1
REACT_APP_CHATBOT_API_KEY=your-production-api-key
```

Update CORS in `backend/app/main.py`:
```python
allow_origins=[
    "https://your-site.vercel.app",
]
```

Add production domain to OpenAI ChatKit settings.

## 📊 Cost Estimate

**Development:**
- Free (using free tiers)

**Production (100 conversations/month):**
- OpenAI API: ~$1-5
- Qdrant Free Tier: $0
- Neon Free Tier: $0
- **Total: ~$1-5/month**

**Per conversation:**
- ~$0.01-0.05 (10-20 messages)

## 🐛 Troubleshooting

**ChatKit not rendering?**
→ Check domain is whitelisted in OpenAI settings

**No responses?**
→ Check API keys in `.env`
→ Verify backend is running
→ Check browser console for errors

**Empty answers?**
→ Run ingestion: `python -m backend.scripts.ingest`
→ Verify Qdrant has data: `python verify_chatkit.py`

**More help:**
→ See [CHATKIT_SETUP.md](CHATKIT_SETUP.md)
→ Run `python verify_chatkit.py`

## 📖 Book Content

The textbook covers:
1. **Chapter 1**: Introduction to Physical AI
2. **Chapter 2**: ROS 2 Fundamentals
3. **Chapter 3**: Simulation Environments
4. **Chapter 4**: NVIDIA Isaac Platform
5. **Chapter 5**: Vision-Language-Action Models

Each chapter includes:
- Conceptual explanations
- Code examples
- Practical exercises
- Self-assessment questions

## 🤝 Contributing

This is a textbook project. For chatbot improvements:
1. Backend: Modify `backend/app/api/chatkit.py`
2. Frontend: Modify `website/src/components/ChatbotChatKit/`
3. Test locally before deploying
4. Follow existing code style

## 📄 License

[Add your license here]

## 🙏 Acknowledgments

- **OpenAI** - ChatKit SDK, GPT models, Embeddings API
- **Qdrant** - Vector database
- **Neon** - Serverless Postgres
- **Docusaurus** - Documentation framework

---

**Ready to start?** Follow the [CHATKIT_SETUP.md](CHATKIT_SETUP.md) guide!
