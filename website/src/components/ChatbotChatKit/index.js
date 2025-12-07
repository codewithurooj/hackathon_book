import React, { useState, useEffect, useRef } from 'react';
import './chatkit-styles.css'; // Import custom styles
import styles from './index.module.css';

function ChatbotChatKit() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);
  const textareaRef = useRef(null);
  
  // Handle text selection for "Ask about this" feature
  useEffect(() => {
    if (!isOpen) return;

    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      // Remove any existing selection button
      const existingButton = document.getElementById('ask-about-selection-btn');
      if (existingButton) {
        existingButton.remove();
      }

      if (text.length > 10) {
        // Create floating button
        const button = document.createElement('button');
        button.id = 'ask-about-selection-btn';
        button.className = styles.askAboutButton;
        button.innerHTML = '<span style="display:flex;align-items:center;gap:6px;"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path></svg> Ask about this</span>';

        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        button.style.position = 'absolute';
        button.style.top = `${rect.bottom + window.scrollY + 5}px`;
        button.style.left = `${rect.left + window.scrollX}px`;
        button.style.zIndex = '10000';

        button.onclick = () => {
          setInput(`Explain this text: "${text}"`);
          setIsOpen(true);
          button.remove();

          // Focus the input area after a short delay
          setTimeout(() => {
            textareaRef.current?.focus();
          }, 300);
        };

        document.body.appendChild(button);

        // Auto-remove after 5 seconds
        setTimeout(() => {
          if (button.parentNode) {
            button.remove();
          }
        }, 5000);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);

      // Cleanup any remaining button
      const existingButton = document.getElementById('ask-about-selection-btn');
      if (existingButton) {
        existingButton.remove();
      }
    };
  }, [isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const sendMessageToBackend = async (message) => {
    try {
      const backendUrl = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/api/v1';
      const response = await fetch(`${backendUrl}/chatkit/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.REACT_APP_CHATBOT_API_KEY || 'NdqKa4dKGIaZRy0W7qLmpKsFxj903xPcLrJIvP44Hqa4xack5FZO82xYpejZaEZe',
        },
        body: JSON.stringify({
          messages: [
            { role: 'user', content: message }
          ],
          stream: true
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
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
              if (data.choices && data.choices[0] && data.choices[0].delta && data.choices[0].delta.content) {
                const content = data.choices[0].delta.content;
                if (content) {
                  answer += content;
                }
              }

              // Check for sources in metadata
              if (data.choices && data.choices[0] && data.choices[0].delta && data.choices[0].delta.metadata) {
                sources = data.choices[0].delta.metadata.sources || [];
              }

              if (data.choices && data.choices[0] && data.choices[0].finish_reason === 'stop') {
                break;
              }
            } catch (e) {
              // Skip non-JSON lines
              if (line.trim() !== '[DONE]') {
                console.warn("Could not parse SSE data:", line);
              }
            }
          }
        }
      }

      return {
        content: answer,
        sources: sources
      };
    } catch (error) {
      console.error("Error sending message to backend:", error);
      return {
        content: `Error: ${error.message}`,
        sources: []
      };
    }
  };

  const handleSendMessage = async (messageText = null) => {
    const messageToSend = messageText || input;
    
    if (!messageToSend.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage = { 
      id: Date.now(), 
      text: messageToSend, 
      sender: 'user',
      timestamp: new Date()
    };
    setMessages(prev => [...prev, userMessage]);
    setInput(''); // Clear input after message is added to UI

    setIsLoading(true);

    try {
      // Add a temporary "typing" message
      const typingMessage = { 
        id: Date.now() + 1, 
        text: '', 
        sender: 'bot', 
        isTyping: true,
        timestamp: new Date()
      };
      setMessages(prev => [...prev, typingMessage]);

      // Get response from backend
      const response = await sendMessageToBackend(messageToSend);

      // Remove the "typing" message and add the actual response
      setMessages(prev => {
        const updated = prev.filter(msg => !msg.isTyping);
        return [...updated, {
          id: Date.now() + 2,
          text: response.content,
          sender: 'bot',
          sources: response.sources,
          timestamp: new Date()
        }];
      });
    } catch (error) {
      setMessages(prev => {
        const updated = prev.filter(msg => !msg.isTyping);
        return [...updated, {
          id: Date.now() + 2,
          text: `Error: ${error.message}`,
          sender: 'bot',
          timestamp: new Date()
        }];
      });
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleInputHeight = (e) => {
    const target = e.target;
    target.style.height = 'auto';
    target.style.height = Math.min(target.scrollHeight, 150) + 'px';
  };

  return (
    <>
      {/* Floating Chat Button */}
      <button
        className={`${styles.chatButton} ${isOpen ? styles.chatButtonOpen : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? '✕' : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat Container */}
      {isOpen && (
        <div className={styles.chatContainer}>
          <div className={styles.chatHeader}>
            <h3>AI Assistant</h3>
            <button onClick={toggleChat} className={styles.closeButton} aria-label="Close chat">
              ✕
            </button>
          </div>
          
          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hi! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
                <p>Ask me anything about the content, or select text on any page to ask specific questions!</p>
              </div>
            ) : (
              messages.map((msg) => (
                <div 
                  key={msg.id} 
                  className={`${styles.message} ${styles[msg.sender]} ${msg.isTyping ? styles.typing : ''}`}
                >
                  <div className={styles.messageContent}>
                    {msg.isTyping ? (
                      <div className={styles.typingIndicator}>
                        <div></div><div></div><div></div>
                      </div>
                    ) : (
                      <>
                        {msg.text}
                        {msg.sources && msg.sources.length > 0 && (
                          <div className={styles.sourcesContainer}>
                            <div className={styles.sourcesLabel}>Sources:</div>
                            {msg.sources.map((source, idx) => (
                              <a
                                key={idx}
                                href={source.url || '#'}
                                className={styles.sourceLink}
                                target="_blank"
                                rel="noopener noreferrer"
                              >
                                <span className={styles.sourceChapter}>{source.title || 'Unknown'}</span>
                                <span className={styles.sourceSection}>{source.snippet?.substring(0, 60) || source.content?.substring(0, 60) || '...'}</span>
                              </a>
                            ))}
                          </div>
                        )}
                      </>
                    )}
                  </div>
                </div>
              ))
            )}
            <div ref={messagesEndRef} />
          </div>
          
          <div className={styles.chatInputArea}>
            <textarea
              ref={textareaRef}
              value={input}
              onChange={(e) => {
                setInput(e.target.value);
                handleInputHeight(e);
              }}
              onKeyDown={handleKeyDown}
              placeholder="Ask me about the textbook..."
              className={styles.chatInput}
              disabled={isLoading}
              rows="1"
            />
            <button 
              onClick={() => handleSendMessage()} 
              className={styles.sendButton}
              disabled={isLoading || !input.trim()}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
}

export default ChatbotChatKit;