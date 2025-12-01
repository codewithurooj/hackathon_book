// website/static/scripts/text-selection.js

document.addEventListener('mouseup', function(event) {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText.length > 0) {
        // Here, we would typically make a UI element visible
        // For now, log to console and simulate an event
        console.log('Selected text:', selectedText);

        // Example: Dispatch a custom event that the React component can listen to
        const customEvent = new CustomEvent('textSelected', {
            detail: { text: selectedText, x: event.pageX, y: event.pageY }
        });
        document.dispatchEvent(customEvent);
    }
});

document.addEventListener('touchend', function(event) {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText.length > 0) {
        console.log('Selected text (mobile):', selectedText);
        const customEvent = new CustomEvent('textSelected', {
            detail: { text: selectedText, x: event.pageX, y: event.pageY }
        });
        document.dispatchEvent(customEvent);
    }
});
