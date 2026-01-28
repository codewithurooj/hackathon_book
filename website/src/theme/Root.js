import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// Default Docusaurus Root component (passed as children)
function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}

export default Root;
