# Text Selection Feature Fixes

## Problem Summary
The text selection feature for the RAG chatbot was not working properly. Users should be able to select text from the book and see a popup asking "Ask about this", which would then query the RAG chatbot with the selected text as context.

## Root Cause
The `TextSelectionButton` component was listening for a custom `textSelected` event, but there was no code dispatching this event when users selected text on the page.

## Fixes Applied

### 1. Created Text Selection Event Dispatcher
**File**: `website/src/clientModules/textSelection.js` (NEW)

This Docusaurus client module:
- Listens for `mouseup` and `touchend` events globally
- Detects when text is selected (minimum 10 characters)
- Dispatches a custom `textSelected` event with:
  - Selected text
  - X and Y coordinates for popup positioning
- Re-attaches listeners on route changes

### 2. Registered Client Module
**File**: `website/docusaurus.config.ts`

Added the text selection client module to the `clientModules` array:
```typescript
clientModules: [
  require.resolve('./src/clientModules/env.js'),
  require.resolve('./src/clientModules/textSelection.js'),  // NEW
],
```

Removed duplicate script reference that was no longer needed.

### 3. Environment Variables Setup

**Backend** (`backend/.env`):
- Added `BACKEND_API_KEY` for API authentication

**Frontend** (`website/.env`): (NEW)
- `REACT_APP_BACKEND_URL`: Backend API URL
- `REACT_APP_CHATBOT_API_KEY`: API key matching the backend

**Frontend** (`website/.env.example`): (NEW)
- Template for environment variables

### 4. Updated .gitignore
**File**: `website/.gitignore`

Added `.env` to prevent committing sensitive credentials.

## Backend Implementation (Already in Place)

The backend was already properly configured with:

1. **ChatKit Endpoint** (`backend/app/routers/chat.py:127`):
   - Accepts `selected_text` parameter
   - Routes to context-based or general question answering

2. **RAG Service** (`backend/app/services/rag_service.py:85`):
   - `answer_context_question()` method that:
     - Uses selected text as primary context (score 1.0)
     - Optionally retrieves related content from Qdrant
     - Generates comprehensive answers using OpenAI

3. **Agent Service** (`backend/app/services/agent_service.py:19`):
   - Proper system prompts for context-based answering
   - Increased max_tokens to 500 for comprehensive responses

## How It Works Now

1. **User selects text** on any page of the book
2. **Text selection listener** (in `textSelection.js`) detects the selection
3. **Custom event** is dispatched with text and position
4. **TextSelectionButton** receives the event and shows popup
5. **User clicks** "Ask about this"
6. **Chatbot opens** with the question: "Explain this text: \"[selected text]\""
7. **Frontend** sends request to `/api/v1/chatkit/chat` with `selected_text` parameter
8. **Backend** uses RAG service with context-based answering
9. **Selected text** is added as primary source (score 1.0)
10. **Additional context** is retrieved from Qdrant (optional, top_k=3)
11. **OpenAI generates** comprehensive answer using all context
12. **Response streams** back to frontend with sources
13. **User sees** answer with source references

## Testing the Feature

### Start the Backend:
```bash
cd backend
python -m uvicorn app.main:app --reload
```

### Start the Frontend:
```bash
cd website
npm start
```

### Test Steps:
1. Navigate to any chapter in the book
2. Select at least 10 characters of text
3. Wait for the "Ask about this" popup to appear
4. Click the popup button
5. The chatbot should open with the question pre-filled
6. Watch the streaming response with sources

## API Flow

### Request Format:
```json
{
  "messages": [
    {"role": "user", "content": "What is deep reinforcement learning?"}
  ],
  "selected_text": "Deep reinforcement learning combines...",
  "stream": true,
  "session_id": "session-123"
}
```

### Response Format (SSE Stream):
```
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"content": "Deep..."}}]}
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {"metadata": {"sources": [...]}}}]}
data: {"id": "chatcmpl-xxx", "choices": [{"delta": {}, "finish_reason": "stop"}]}
data: [DONE]
```

## Key Components

### Frontend:
- `website/src/clientModules/textSelection.js` - Event dispatcher
- `website/src/components/Chatbot/index.js` - Main chatbot component
- `website/src/components/Chatbot/TextSelectionButton.js` - Popup button
- `website/src/theme/Root.js` - Chatbot integration

### Backend:
- `backend/app/routers/chat.py` - ChatKit endpoint
- `backend/app/services/rag_service.py` - RAG orchestration
- `backend/app/services/agent_service.py` - OpenAI interaction
- `backend/app/services/qdrant_service.py` - Vector search
- `backend/app/api/auth.py` - API key validation

## Hackathon Requirements Met

✅ **RAG Chatbot**: Integrated with FastAPI, OpenAI, Qdrant
✅ **Book Embedding**: Published with Docusaurus
✅ **Text Selection**: Users can select text and ask questions
✅ **Context-Aware**: Selected text is prioritized in answers
✅ **Streaming**: Real-time response streaming
✅ **Sources**: Citations from the book content
✅ **Session Management**: Conversation history tracking

## Next Steps (Optional Enhancements)

1. Add visual highlighting of selected text
2. Implement conversation history UI
3. Add "copy answer" functionality
4. Improve mobile responsiveness
5. Add keyboard shortcuts (e.g., Ctrl+Enter to ask)
6. Implement answer feedback system
7. Add bookmark/save functionality for Q&A pairs
