import React from 'react';
import { render, screen } from '@testing-library/react';
import TextSelectionButton from './TextSelectionButton';

test('renders TextSelectionButton component', () => {
  // Mock the prop function
  const mockOnAskAboutThis = jest.fn();
  render(<TextSelectionButton onAskAboutThis={mockOnAskAboutThis} />);
  // Since it's hidden by default, we expect it not to be in the document initially
  expect(screen.queryByText(/Ask about this/i)).not.toBeInTheDocument();
});
