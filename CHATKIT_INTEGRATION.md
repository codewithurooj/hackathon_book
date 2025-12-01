# ChatKit SDK Integration Guide

This guide explains how to replace the custom chatbot component with OpenAI's official ChatKit SDK to meet the project requirements.

## Why ChatKit SDK?

Your requirements explicitly state:
> "This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI..."

**Spec Section 7**: "**UI Library**: **ChatKit SDK** will be used to build the chatbot interface."

## Current Status

❌ **Gap**: The current implementation uses a custom React component instead of ChatKit SDK.

✅ **Backend**: Fully compatible with ChatKit (FastAPI + OpenAI Agent pattern)

## ChatKit Integration Steps

### Step 1: Install ChatKit SDK

```bash
cd website
npm install @openai/chatkit-react
```

### Step 2: Configure OpenAI Organization Settings

**CRITICAL**: Before ChatKit will render, you must whitelist your domain in OpenAI:

1. Go to [OpenAI Platform](https://platform.openai.com/)
2. Navigate to **Settings** → **Organization** → **ChatKit Settings**
3. Add your domains:
   - Development: `http://localhost:3000`
   - Production: `https://yourdomain.com`

Without this step, ChatKit will not render (security feature).

### Step 3: Create ChatKit Wrapper Component

Create `website/src/components/ChatbotChatKit/index.js`:

```javascript
import React from 'react';
import { ChatKit } from '@openai/chatkit-react';
import '@openai/chatkit-react/styles.css';

function ChatbotChatKit() {
  // ChatKit configuration
  const config = {
    // Your OpenAI API key (client-side - use with caution or proxy through backend)
    apiKey: process.env.REACT_APP_OPENAI_API_KEY,

    // Agent configuration
    agent: {
      // Use your custom backend endpoint
      endpoint: `${process.env.REACT_APP_BACKEND_URL}/chat`,
      headers: {
        'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY,
      },
    },

    // UI customization
    theme: {
      primaryColor: '#0066cc',
      borderRadius: '8px',
    },

    // Initial message
    welcomeMessage: 'Hi! I\'m your AI assistant for the Physical AI textbook. Ask me anything about the content!',

    // Placeholder text
    placeholder: 'Ask me about the textbook...',
  };

  return (
    <div style={{ position: 'fixed', bottom: '20px', right: '20px', zIndex: 1000 }}>
      <ChatKit config={config} />
    </div>
  );
}

export default ChatbotChatKit;
```

### Step 4: Backend Adapter for ChatKit

ChatKit expects a specific API format. Update your FastAPI endpoint to be ChatKit-compatible:

**Option A: Modify existing endpoint** to support both formats

**Option B: Create new ChatKit-specific endpoint**

Create `backend/app/api/chatkit.py`:

```python
from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import List, Dict, Optional
import json

from backend.app.api.auth import get_api_key
from backend.app.services.openai_service import generate_embedding
from backend.app.services.qdrant_service import search_qdrant
from backend.app.services.openai_agent import generate_answer_from_context

router = APIRouter()

class ChatKitMessage(BaseModel):
    role: str  # "user" or "assistant"
    content: str

class ChatKitRequest(BaseModel):
    messages: List[ChatKitMessage]
    stream: Optional[bool] = True

@router.post("/chatkit/chat")
async def chatkit_chat(request: ChatKitRequest, api_key: str = Depends(get_api_key)):
    """
    ChatKit-compatible endpoint that follows OpenAI's Chat Completion API format.
    """
    # Get the last user message
    user_message = next((msg.content for msg in reversed(request.messages) if msg.role == "user"), None)

    if not user_message:
        raise HTTPException(status_code=400, detail="No user message found")

    # Generate embedding
    query_embedding = await generate_embedding(user_message)

    # Retrieve context from Qdrant
    context_chunks = await search_qdrant(query_embedding)

    # Generate answer
    agent_response = await generate_answer_from_context(user_message, context_chunks)

    # Format response for ChatKit (streaming)
    async def generate_stream():
        # ChatKit expects Server-Sent Events format
        response_data = {
            "id": "chatcmpl-" + str(hash(user_message)),
            "object": "chat.completion.chunk",
            "created": int(time.time()),
            "model": "gpt-4o-mini",
            "choices": [{
                "index": 0,
                "delta": {"role": "assistant", "content": agent_response.answer},
                "finish_reason": None
            }]
        }
        yield f"data: {json.dumps(response_data)}\n\n"

        # Send finish event
        finish_data = {
            "id": "chatcmpl-" + str(hash(user_message)),
            "object": "chat.completion.chunk",
            "created": int(time.time()),
            "model": "gpt-4o-mini",
            "choices": [{
                "index": 0,
                "delta": {},
                "finish_reason": "stop"
            }]
        }
        yield f"data: {json.dumps(finish_data)}\n\n"
        yield "data: [DONE]\n\n"

    return StreamingResponse(generate_stream(), media_type="text/event-stream")
```

Register the router in `backend/app/main.py`:

```python
from backend.app.api import chat, chatkit

app.include_router(chat.router, prefix="/api/v1")
app.include_router(chatkit.router, prefix="/api/v1")  # Add this line
```

### Step 5: Update Root Component

Replace the custom chatbot in `website/src/theme/Root.js`:

```javascript
import React from 'react';
import ChatbotChatKit from '@site/src/components/ChatbotChatKit';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotChatKit />
    </>
  );
}
```

### Step 6: Text Selection Integration

For the "Ask about this" feature with selected text, you'll need to enhance ChatKit:

Create `website/src/components/ChatbotChatKit/TextSelectionHandler.js`:

```javascript
import React, { useEffect, useState } from 'react';
import { ChatKit } from '@openai/chatkit-react';

function ChatbotWithTextSelection() {
  const [selectedText, setSelectedText] = useState('');
  const [chatKitRef, setChatKitRef] = useState(null);

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 10) { // Minimum selection length
        setSelectedText(text);
        showSelectionButton(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const showSelectionButton = (text) => {
    // Create floating button for "Ask about this"
    const button = document.createElement('button');
    button.innerText = 'Ask about this';
    button.style.cssText = 'position: absolute; z-index: 9999; padding: 8px 12px; background: #0066cc; color: white; border: none; border-radius: 4px; cursor: pointer;';

    const selection = window.getSelection();
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    button.style.top = `${rect.bottom + window.scrollY + 5}px`;
    button.style.left = `${rect.left + window.scrollX}px`;

    button.onclick = () => {
      // Send selected text to ChatKit
      if (chatKitRef) {
        chatKitRef.sendMessage(`Please explain this text from the book: "${text}"`);
      }
      button.remove();
    };

    document.body.appendChild(button);

    // Remove button after 5 seconds or on click outside
    setTimeout(() => button.remove(), 5000);
  };

  const config = {
    apiKey: process.env.REACT_APP_OPENAI_API_KEY,
    agent: {
      endpoint: `${process.env.REACT_APP_BACKEND_URL}/chatkit/chat`,
      headers: {
        'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY,
      },
    },
    theme: {
      primaryColor: '#0066cc',
    },
    welcomeMessage: 'Hi! Ask me anything about the Physical AI textbook, or select text to ask specific questions!',
  };

  return <ChatKit ref={setChatKitRef} config={config} />;
}

export default ChatbotWithTextSelection;
```

### Step 7: Environment Variables

Update `.env`:

```bash
# Add OpenAI API key for ChatKit (if needed client-side)
REACT_APP_OPENAI_API_KEY=your-openai-api-key

# Or better: Use backend proxy (recommended for security)
REACT_APP_BACKEND_URL=http://localhost:8000/api/v1
REACT_APP_CHATBOT_API_KEY=your-backend-api-key
```

### Step 8: Citation Integration

ChatKit supports custom message rendering. To show citations:

```javascript
const config = {
  // ... other config

  // Custom message renderer for citations
  renderMessage: (message) => {
    if (message.metadata?.sources) {
      return (
        <div>
          <p>{message.content}</p>
          <div className="sources">
            {message.metadata.sources.map((source, idx) => (
              <a key={idx} href={source.page_url} className="source-link">
                {source.chapter} - {source.section}
              </a>
            ))}
          </div>
        </div>
      );
    }
    return <p>{message.content}</p>;
  },
};
```

## Alternative: Hybrid Approach

If you want to keep some custom functionality:

1. **Use ChatKit for main chat UI** (meets requirement)
2. **Keep custom text selection button** (enhanced UX)
3. **Integrate both** via shared backend

## Migration Checklist

- [ ] Install `@openai/chatkit-react`
- [ ] Whitelist domain in OpenAI organization settings
- [ ] Create ChatKit wrapper component
- [ ] Create ChatKit-compatible backend endpoint
- [ ] Update Root.js to use ChatKit
- [ ] Test basic chat functionality
- [ ] Implement text selection integration
- [ ] Add citation rendering
- [ ] Style ChatKit to match Docusaurus theme
- [ ] Test end-to-end with real data

## Benefits of Using ChatKit

✅ **Meets project requirements** (explicitly specified in spec)
✅ **Official OpenAI component** (better support and updates)
✅ **Built-in streaming** (better UX)
✅ **Professional UI** (designed by OpenAI UX team)
✅ **Agent-ready** (works seamlessly with OpenAI Agent SDK)
✅ **Less maintenance** (offload UI to OpenAI)

## Comparison

| Feature | Custom Component | ChatKit SDK |
|---------|------------------|-------------|
| Meets Requirements | ❌ | ✅ |
| Development Time | High | Low |
| Maintenance | You maintain | OpenAI maintains |
| OpenAI Integration | Manual | Built-in |
| Streaming Support | Custom | Built-in |
| Professional UI | DIY | Professional |
| Updates | Manual | Automatic |

## Documentation

- [ChatKit GitHub](https://github.com/openai/chatkit-js)
- [ChatKit Documentation](https://openai.github.io/chatkit-js/)
- [ChatKit React NPM](https://www.npmjs.com/package/@openai/chatkit-react)
- [Getting Started Guide](https://medium.com/@mcraddock/getting-started-with-openai-chatkit-the-one-setup-step-you-cant-skip-7d4c0110404a)

## Next Steps

1. Follow this guide to integrate ChatKit
2. Keep the existing backend (it's compatible)
3. Test with the existing ingestion and RAG pipeline
4. Deploy with proper environment variables

---

**Sources:**
- [GitHub - openai/chatkit-js](https://github.com/openai/chatkit-js)
- [OpenAI Agent Embeds Documentation](https://openai.github.io/chatkit-js/)
- [@openai/chatkit-react NPM Package](https://www.npmjs.com/package/@openai/chatkit-react)
- [ChatKit Setup Guide by Mark Craddock](https://medium.com/@mcraddock/getting-started-with-openai-chatkit-the-one-setup-step-you-cant-skip-7d4c0110404a)
