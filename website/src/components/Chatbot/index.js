import React, { useState, useEffect } from 'react';
import styles from './index.module.css';
import TextSelectionButton from './TextSelectionButton'; // Import the new component
import SourceBubble from './SourceBubble'; // Import the new SourceBubble component
import { customToast, Toaster } from './ErrorToast'; // Import customToast and Toaster

function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [selectedTextContext, setSelectedTextContext] = useState(''); // State to hold selected text
  const [sessionId, setSessionId] = useState(() => {
    // Initialize session ID from localStorage or generate a new one
    const savedSessionId = localStorage.getItem('chatbotSessionId');
    return savedSessionId || `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  });

  useEffect(() => {
    // Save session ID to localStorage whenever it changes
    localStorage.setItem('chatbotSessionId', sessionId);
  }, [sessionId]);


  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessageToBackend = async (message, textContext = null) => {
    try {
      // Determine the backend URL based on environment
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/api/v1';

      // Prepare the request body
      const requestBody = {
        question: message,
        session_id: sessionId,
        selected_text: textContext || null
      };

      // Make the API call to the backend
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY || process.env.BACKEND_API_KEY || 'your-backend-api-key', // You'll need to set this
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      // Handle the streaming response
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      let answer = '';
      let sources = [];

      while (true) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop(); // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = JSON.parse(line.slice(6)); // Remove 'data: ' prefix
              if (data.answer_chunk) {
                answer += data.answer_chunk;
              }
              if (data.sources) {
                sources = data.sources;
              }
              if (data.session_id) {
                setSessionId(data.session_id); // Update session ID if changed by backend
              }
            } catch (e) {
              console.warn("Could not parse SSE data:", line);
            }
          }
        }
      }

      // Return the collected response
      return {
        answer_chunk: answer,
        sources: sources,
        session_id: sessionId,
        history: messages
      };
    } catch (error) {
      console.error("Backend communication error:", error);
      customToast(error.message || "An unexpected error occurred.", "error");
      throw error; // Re-throw to handle in handleSendMessage
    }
  };

  const handleSendMessage = async (textContext = null) => {
    const messageToSend = textContext || input;

    if (messageToSend.trim() === '') return;

    const userMessage = { id: messages.length + 1, text: messageToSend, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);

    if (!textContext) { // Clear input only if it's not from selected text
      setInput('');
    } else {
      setSelectedTextContext(''); // Clear selected text context
    }

    try {
        const backendResponse = await sendMessageToBackend(messageToSend, textContext);

        const botMessage = {
          id: messages.length + 2,
          text: backendResponse.answer_chunk,
          sender: 'bot',
          sources: backendResponse.sources,
        };
        setMessages((prevMessages) => [...prevMessages, botMessage]);

        // TODO: Integrate with actual backend API (T016, T019)
    } catch (error) {
        // Error already displayed by customToast in sendMessageToBackend
        // We might want to add a 'failed' message to history here if needed
    }
  };

  const handleAskAboutThis = (text) => {
    setSelectedTextContext(text); // Store selected text for sending
    setIsOpen(true); // Open chat window if not already open
    handleSendMessage(text); // Immediately send the selected text as a question
  };


  return (
    <>
      <TextSelectionButton onAskAboutThis={handleAskAboutThis} /> {/* Integrate the button */}

      <div className={styles.chatButton} onClick={toggleChat}>
        Chat
      </div>

      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
            <button onClick={toggleChat}>X</button>
          </div>
          <div className={styles.chatBody}>
            {messages.map((msg) => (
              <div key={msg.id} className={`${styles.message} ${styles[msg.sender]}`}>
                {msg.text}
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sourcesContainer}>
                    {msg.sources.map((source, idx) => (
                      <SourceBubble key={idx} source={source} />
                    ))}
                  </div>
                )}
              </div>
            ))}
          </div>
          <div className={styles.chatInput}>
            <input
              type="text"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={(e) => e.key === 'Enter' && handleSendMessage()}
              placeholder="Ask me about the textbook..."
            />
            <button onClick={handleSendMessage}>Send</button>
          </div>
        </div>
      )}
      <Toaster /> {/* Place Toaster component at the root of your app */}
    </>
  );
}

export default Chatbot;
