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
    // Initialize session ID - will be loaded from localStorage in useEffect
    return `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
  });

  useEffect(() => {
    // Only access localStorage on client side
    if (typeof window !== 'undefined') {
      const savedSessionId = localStorage.getItem('chatbotSessionId');
      if (savedSessionId) {
        setSessionId(savedSessionId);
      } else {
        localStorage.setItem('chatbotSessionId', sessionId);
      }
    }
  }, []); // Run once on mount

  useEffect(() => {
    // Save session ID to localStorage whenever it changes (client-side only)
    if (typeof window !== 'undefined') {
      localStorage.setItem('chatbotSessionId', sessionId);
    }
  }, [sessionId]);


  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessageToBackend = async (message, textContext = null, botMessageId) => {
    try {
      // Determine the backend URL based on environment
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/api/v1';

      // Prepare the request body in ChatKit format
      const requestBody = {
        messages: [
          { role: 'user', content: message }
        ],
        session_id: sessionId,
        selected_text: textContext || null,
        stream: true
      };

      // Make the API call to the backend - using ChatKit-compatible endpoint
      const response = await fetch(`${backendUrl}/chatkit/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY || process.env.BACKEND_API_KEY || 'temp-key-placeholder',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      // Handle the streaming response from ChatKit endpoint
      const reader = response.body.getReader();
      const decoder = new TextDecoder();
      let buffer = '';
      let answer = '';
      let sources = [];
      let finishReceived = false;

      while (!finishReceived) {
        const { done, value } = await reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });
        const lines = buffer.split('\n');
        buffer = lines.pop(); // Keep incomplete line in buffer

        for (const line of lines) {
          if (line.startsWith('data: ')) {
            try {
              const data = line.slice(6); // Remove 'data: ' prefix
              if (data === '[DONE]') {
                finishReceived = true;
                break;
              }

              const parsedData = JSON.parse(data);
              if (parsedData.choices && parsedData.choices[0]) {
                const choice = parsedData.choices[0];
                if (choice.delta && choice.delta.content) {
                  answer += choice.delta.content;

                  // Update the bot message in real-time as chunks arrive
                  setMessages((prevMessages) =>
                    prevMessages.map((msg) =>
                      msg.id === botMessageId
                        ? { ...msg, text: answer }
                        : msg
                    )
                  );
                }
                // Check for metadata in the response
                if (choice.delta && choice.delta.metadata) {
                  if (choice.delta.metadata.sources) {
                    sources = choice.delta.metadata.sources;
                  }
                }
              }
            } catch (e) {
              console.warn("Could not parse SSE data:", line);
            }
          }
        }
      }

      // Return the final collected response with sources
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

    if (!messageToSend || String(messageToSend).trim() === '') return;

    const userMessage = { id: Date.now(), text: messageToSend, sender: 'user' };
    setMessages((prevMessages) => [...prevMessages, userMessage]);

    if (!textContext) { // Clear input only if it's not from selected text
      setInput('');
    } else {
      setSelectedTextContext(''); // Clear selected text context
    }

    // Create an empty bot message that will be updated with streaming content
    const botMessageId = Date.now() + 1;
    const botMessage = {
      id: botMessageId,
      text: '',
      sender: 'bot',
      sources: [],
    };
    setMessages((prevMessages) => [...prevMessages, botMessage]);

    try {
        const backendResponse = await sendMessageToBackend(messageToSend, textContext, botMessageId);

        // Update the bot message with final sources
        setMessages((prevMessages) =>
          prevMessages.map((msg) =>
            msg.id === botMessageId
              ? { ...msg, sources: backendResponse.sources || [] }
              : msg
          )
        );
    } catch (error) {
        // Error already displayed by customToast in sendMessageToBackend
        // Update the bot message to show error
        setMessages((prevMessages) =>
          prevMessages.map((msg) =>
            msg.id === botMessageId
              ? { ...msg, text: 'Sorry, I encountered an error while processing your request.' }
              : msg
          )
        );
    }
  }  // Closing brace for handleSendMessage function

  const handleAskAboutThis = (text) => {
    // Ensure we're working with a proper string and not an event object
    let selectedText = text;

    // If text is an event object or has event properties, extract the actual text
    if (text && typeof text === 'object' && text.text) {
      selectedText = text.text; // If it's already an object with text property
    } else if (text && typeof text === 'object' && text.detail) {
      selectedText = text.detail.text; // If it's a custom event
    } else if (text && typeof text === 'object') {
      // If it's a general object, try to get text from selection
      selectedText = window.getSelection ? window.getSelection().toString().trim() : '';
    }

    if (!selectedText || typeof selectedText !== 'string' || selectedText.trim() === '') {
      return;
    }

    setSelectedTextContext(selectedText); // Store selected text for sending
    setIsOpen(true); // Open chat window if not already open

    // Create user message with the question
    const userMessage = {
      id: Date.now(),
      text: `Explain this text: "${selectedText}"`,
      sender: 'user'
    };
    setMessages((prevMessages) => [...prevMessages, userMessage]);

    // Send the message with selected text as context
    sendMessageWithContext(selectedText);
  };

  const sendMessageWithContext = async (selectedText) => {
    // Create an empty bot message that will be updated with streaming content
    const botMessageId = Date.now() + 1;
    const botMessage = {
      id: botMessageId,
      text: '',
      sender: 'bot',
      sources: [],
    };
    setMessages((prevMessages) => [...prevMessages, botMessage]);

    try {
      const questionText = `Explain this text: "${selectedText}"`;
      const backendResponse = await sendMessageToBackend(questionText, selectedText, botMessageId);

      // Update the bot message with final sources
      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botMessageId
            ? { ...msg, sources: backendResponse.sources || [] }
            : msg
        )
      );
    } catch (error) {
      console.error('Error sending message with context:', error);
      // Update the bot message to show error
      setMessages((prevMessages) =>
        prevMessages.map((msg) =>
          msg.id === botMessageId
            ? { ...msg, text: 'Sorry, I encountered an error while processing your request.' }
            : msg
        )
      );
    }
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
                {typeof msg.text === 'string' && msg.text ? msg.text : (msg.sender === 'bot' && !msg.text ? '...' : String(msg.text || ''))}
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
