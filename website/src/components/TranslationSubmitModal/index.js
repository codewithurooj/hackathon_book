/**
 * TranslationSubmitModal Component
 *
 * Modal for submitting Urdu translations for chapters.
 *
 * Features:
 * - Textarea for Urdu input with character count
 * - Client-side Urdu validation
 * - Submit to API with error handling
 * - Success/error feedback
 * - First-wins policy enforcement
 */

import React, { useState } from 'react';
import styles from './styles.module.css';

// API configuration
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

const TranslationSubmitModal = ({ chapterId, onClose, onSuccess }) => {
  const [urduContent, setUrduContent] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [error, setError] = useState(null);
  const [validationError, setValidationError] = useState(null);

  /**
   * Client-side validation for Urdu content
   */
  const validateUrduContent = (content) => {
    if (!content || content.trim().length === 0) {
      return 'Translation cannot be empty';
    }

    // Check for Urdu characters (Unicode range U+0600-U+06FF)
    const urduPattern = /[\u0600-\u06FF]/g;
    const urduChars = content.match(urduPattern) || [];
    const totalChars = content.replace(/\s/g, '').length;

    if (totalChars === 0) {
      return 'Translation contains only whitespace';
    }

    const urduPercentage = urduChars.length / totalChars;

    if (urduPercentage < 0.5) {
      return `Translation must contain at least 50% Urdu characters. Current: ${Math.round(urduPercentage * 100)}%`;
    }

    return null;
  };

  /**
   * Handle content change with real-time validation
   */
  const handleContentChange = (e) => {
    const content = e.target.value;
    setUrduContent(content);

    // Clear previous errors
    setValidationError(null);
    setError(null);

    // Validate if content is not empty
    if (content.trim().length > 0) {
      const validationResult = validateUrduContent(content);
      setValidationError(validationResult);
    }
  };

  /**
   * Submit translation to API
   */
  const handleSubmit = async (e) => {
    e.preventDefault();

    // Validate before submission
    const validationResult = validateUrduContent(urduContent);
    if (validationResult) {
      setValidationError(validationResult);
      return;
    }

    setIsSubmitting(true);
    setError(null);

    try {
      // Get user ID from localStorage
      const userId = localStorage.getItem('user_id') || sessionStorage.getItem('user_id');

      if (!userId) {
        throw new Error('User ID not found. Please log in.');
      }

      const response = await fetch(`${API_BASE_URL}/translations`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-User-Id': userId
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          urdu_content: urduContent.trim()
        })
      });

      const data = await response.json();

      if (!response.ok) {
        // Handle specific errors
        if (response.status === 409) {
          throw new Error('Translation already exists for this chapter. First submission wins!');
        } else if (response.status === 401) {
          throw new Error('Authentication required. Please log in.');
        } else {
          throw new Error(data.detail || 'Failed to submit translation');
        }
      }

      // Success!
      if (onSuccess) {
        onSuccess(data);
      }

      // Close modal
      if (onClose) {
        onClose();
      }
    } catch (err) {
      console.error('Error submitting translation:', err);
      setError(err.message);
    } finally {
      setIsSubmitting(false);
    }
  };

  /**
   * Handle modal backdrop click to close
   */
  const handleBackdropClick = (e) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  const characterCount = urduContent.length;
  const isValid = !validationError && urduContent.trim().length > 0;

  return (
    <div className={styles.backdrop} onClick={handleBackdropClick}>
      <div className={styles.modal}>
        <div className={styles.header}>
          <h2 className={styles.title}>Submit Urdu Translation</h2>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close modal"
          >
            ✕
          </button>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          <div className={styles.field}>
            <label htmlFor="urdu-content" className={styles.label}>
              Chapter: <strong>{chapterId}</strong>
            </label>

            <textarea
              id="urdu-content"
              className={styles.textarea}
              value={urduContent}
              onChange={handleContentChange}
              placeholder="یہاں اردو ترجمہ درج کریں..."
              rows={15}
              dir="rtl"
              lang="ur"
              disabled={isSubmitting}
            />

            <div className={styles.charCount}>
              {characterCount} characters
              {characterCount > 0 && (
                <span className={isValid ? styles.valid : styles.invalid}>
                  {isValid ? ' ✓ Valid' : ' ✗ Invalid'}
                </span>
              )}
            </div>
          </div>

          {validationError && (
            <div className={styles.validationError}>
              {validationError}
            </div>
          )}

          {error && (
            <div className={styles.error}>
              <strong>Error:</strong> {error}
            </div>
          )}

          <div className={styles.infoBox}>
            <strong>Important:</strong> First submission wins! Once a translation is
            submitted for a chapter, it cannot be changed. Please review carefully
            before submitting.
          </div>

          <div className={styles.actions}>
            <button
              type="button"
              className={styles.cancelButton}
              onClick={onClose}
              disabled={isSubmitting}
            >
              Cancel
            </button>

            <button
              type="submit"
              className={styles.submitButton}
              disabled={isSubmitting || !isValid}
            >
              {isSubmitting ? 'Submitting...' : 'Submit Translation'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default TranslationSubmitModal;
