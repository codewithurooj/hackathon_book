import React, { useState, useEffect } from 'react';
import styles from './TextSelectionButton.module.css';

function TextSelectionButton({ onAskAboutThis }) {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleTextSelected = (event) => {
      const { text, x, y } = event.detail;
      setSelectedText(text);
      setPosition({ x, y });
      setIsVisible(true);
    };

    const handleMouseUp = () => {
      // Hide button if selection is empty or outside relevant area
      if (window.getSelection().toString().trim().length === 0 && isVisible) {
        setIsVisible(false);
      }
    };

    document.addEventListener('textSelected', handleTextSelected);
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('touchend', handleMouseUp); // For mobile

    return () => {
      document.removeEventListener('textSelected', handleTextSelected);
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('touchend', handleMouseUp);
    };
  }, [isVisible]);

  const handleClick = (e) => {
    e.preventDefault();
    e.stopPropagation();
    const textToSend = selectedText; // Capture text before any state changes
    setIsVisible(false); // Hide button after clicking
    onAskAboutThis(textToSend);
  };

  if (!isVisible) {
    return null;
  }

  // Position the button slightly above the mouse-up coordinates
  const buttonStyle = {
    position: 'absolute',
    left: `${position.x}px`,
    top: `${position.y - 40}px`, // Adjust as needed
    zIndex: 1001, // Above chat window
  };

  return (
    <div
      className={styles.askButton}
      style={buttonStyle}
      onMouseDown={handleClick}
      onTouchStart={handleClick}
    >
      Ask about this
    </div>
  );
}

export default TextSelectionButton;
