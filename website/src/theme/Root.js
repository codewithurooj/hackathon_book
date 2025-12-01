import React from 'react';
import ChatbotChatKit from '@site/src/components/ChatbotChatKit'; // ChatKit component that connects to backend

// Default Docusaurus Root component (passed as children)
function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotChatKit />  {/* Using the ChatKit component that connects to your backend */}
    </>
  );
}

export default Root;
