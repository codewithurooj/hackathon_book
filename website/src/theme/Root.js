import React from 'react';
import Chatbot from '@site/src/components/Chatbot'; // Original Chatbot with text selection functionality that I updated to use RAG service

// Default Docusaurus Root component (passed as children)
function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />  {/* Original Chatbot with proper text selection functionality */}
    </>
  );
}

export default Root;
