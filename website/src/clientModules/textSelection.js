/**
 * Client module to handle text selection events across the site
 * This dispatches a custom 'textSelected' event when users select text
 */

export function onRouteDidUpdate() {
  // Re-attach listeners on route changes
  attachTextSelectionListeners();
}

function attachTextSelectionListeners() {
  // Remove existing listeners to prevent duplicates
  document.removeEventListener('mouseup', handleTextSelection);
  document.removeEventListener('touchend', handleTextSelection);

  // Attach new listeners
  document.addEventListener('mouseup', handleTextSelection);
  document.addEventListener('touchend', handleTextSelection);
}

function handleTextSelection(event) {
  // Small delay to ensure selection is complete
  setTimeout(() => {
    const selection = window.getSelection();
    const selectedText = selection.toString().trim();

    // Only trigger if text is selected and it's at least 10 characters
    // (to avoid triggering on accidental clicks)
    if (selectedText && selectedText.length >= 10) {
      const range = selection.getRangeAt(0);
      const rect = range.getBoundingClientRect();

      // Dispatch custom event with selection details
      const textSelectedEvent = new CustomEvent('textSelected', {
        detail: {
          text: selectedText,
          x: rect.left + (rect.width / 2), // Center of selection
          y: rect.bottom + window.scrollY, // Bottom of selection
        },
        bubbles: true,
      });

      document.dispatchEvent(textSelectedEvent);
    }
  }, 100);
}

// Initial setup when module loads
if (typeof window !== 'undefined') {
  attachTextSelectionListeners();
}
