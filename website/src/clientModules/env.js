// Expose environment variables to the client
// This file helps Docusaurus properly expose environment variables to client-side React components

// Ensure the environment variables are properly exposed to the client
if (typeof process === 'undefined') {
  global.process = { env: {} };
}

// Detect production environment (Vercel or other hosting)
const isProduction = typeof window !== 'undefined' &&
  window.location.hostname !== 'localhost' &&
  window.location.hostname !== '127.0.0.1';

// Production backend URL
const PRODUCTION_BACKEND_URL = 'https://physical-ai-book-backend.onrender.com/api/v1';
const LOCAL_BACKEND_URL = 'http://localhost:8000/api/v1';

// Specifically set the required environment variables with fallbacks
process.env.REACT_APP_BACKEND_URL = isProduction ? PRODUCTION_BACKEND_URL : LOCAL_BACKEND_URL;
process.env.REACT_APP_CHATBOT_API_KEY = process.env.REACT_APP_CHATBOT_API_KEY || 'NdqKa4dKGIaZRy0W7qLmpKsFxj903xPcLrJIvP44Hqa4xack5FZO82xYpejZaEZe';