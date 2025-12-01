import React from 'react';
import { render, screen } from '@testing-library/react';
import SourceBubble from './SourceBubble';

test('renders SourceBubble component', () => {
  const mockSource = { chapter: "Test Chapter", section: "Test Section", page_url: "/docs/test-chapter/test-section" };
  render(<SourceBubble source={mockSource} />);
  const sourceLink = screen.getByRole('link', { name: /Test Chapter » Test Section/i });
  expect(sourceLink).toBeInTheDocument();
  expect(sourceLink).toHaveAttribute('href', '/docs/test-chapter/test-section');
});
