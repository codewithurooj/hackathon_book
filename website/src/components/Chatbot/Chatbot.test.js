import React from 'react';
import { render, screen } from '@testing-library/react';
import Chatbot from './index'; // Assuming index.js exports default Chatbot

test('renders chatbot component', () => {
  render(<Chatbot />);
  const chatButton = screen.getByText(/Chat/i);
  expect(chatButton).toBeInTheDocument();
});