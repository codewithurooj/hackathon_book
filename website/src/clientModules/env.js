// Expose environment variables to the client
// This file helps Docusaurus properly expose environment variables to client-side React components

// Ensure the environment variables are properly exposed to the client
if (typeof process === 'undefined') {
  global.process = { env: {} };
}

// Specifically set the required environment variables with fallbacks
process.env.REACT_APP_BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000/api/v1';
process.env.REACT_APP_CHATBOT_API_KEY = process.env.REACT_APP_CHATBOT_API_KEY || 'NdqKa4dKGIaZRy0W7qLmpKsFxj903xPcLrJIvP44Hqa4xack5FZO82xYpejZaEZe'; // Using the key from your .env