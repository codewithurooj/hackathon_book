/**
 * UrduTranslationButton Component
 *
 * Toggle button to switch between English and Urdu content for each chapter.
 * Also allows authenticated users to contribute translations.
 *
 * Features:
 * - Fetches Urdu translation from API
 * - Caches translations in memory for performance
 * - Persists language preference in localStorage
 * - Handles loading and error states
 * - Allows translation submission via modal
 * - Only visible for authenticated users
 */

import React, { useState, useEffect, useCallback } from 'react';
import TranslationSubmitModal from '../TranslationSubmitModal';
import styles from './styles.module.css';

// API configuration
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

// In-memory cache for translations (survives component re-renders but not page reloads)
const translationCache = new Map();

const UrduTranslationButton = ({ chapterId, isAuthenticated = false }) => {
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [urduTranslation, setUrduTranslation] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [translationExists, setTranslationExists] = useState(false);
  const [showSubmitModal, setShowSubmitModal] = useState(false);

  // Load language preference from localStorage on mount
  useEffect(() => {
    const savedLanguage = localStorage.getItem(`chapter-${chapterId}-language`);
    if (savedLanguage === 'ur') {
      setCurrentLanguage('ur');
      // Load translation if switching to Urdu
      loadTranslation();
    }

    // Check if translation exists
    checkTranslationExists();
  }, [chapterId]);

  /**
   * Check if Urdu translation exists for this chapter
   */
  const checkTranslationExists = useCallback(async () => {
    try {
      const response = await fetch(
        `${API_BASE_URL}/translations/check/${chapterId}`
      );

      if (response.ok) {
        const data = await response.json();
        setTranslationExists(data.exists);
      }
    } catch (err) {
      console.error('Error checking translation existence:', err);
      // Default to false to show "Add Translation" button
      setTranslationExists(false);
    }
  }, [chapterId]);

  /**
   * Load Urdu translation from API (with caching)
   */
  const loadTranslation = useCallback(async () => {
    // Check cache first
    if (translationCache.has(chapterId)) {
      setUrduTranslation(translationCache.get(chapterId));
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(
        `${API_BASE_URL}/translations/${chapterId}`
      );

      if (!response.ok) {
        if (response.status === 404) {
          throw new Error('Translation not available for this chapter yet.');
        }
        throw new Error('Failed to load translation');
      }

      const data = await response.json();

      // Cache the translation
      translationCache.set(chapterId, data);
      setUrduTranslation(data);
    } catch (err) {
      console.error('Error loading translation:', err);
      setError(err.message);
      // Revert to English on error
      setCurrentLanguage('en');
      localStorage.setItem(`chapter-${chapterId}-language`, 'en');
    } finally {
      setIsLoading(false);
    }
  }, [chapterId]);

  /**
   * Toggle between English and Urdu
   */
  const toggleLanguage = useCallback(async () => {
    const newLanguage = currentLanguage === 'en' ? 'ur' : 'en';

    // If switching to Urdu and translation not loaded yet, fetch it
    if (newLanguage === 'ur' && !urduTranslation) {
      await loadTranslation();
    }

    // Update state and persist preference
    setCurrentLanguage(newLanguage);
    localStorage.setItem(`chapter-${chapterId}-language`, newLanguage);

    // Update DOM content
    updateContentDisplay(newLanguage);
  }, [currentLanguage, urduTranslation, loadTranslation, chapterId]);

  /**
   * Update chapter content based on selected language
   */
  const updateContentDisplay = useCallback((language) => {
    const contentElement = document.querySelector('article.markdown');

    if (!contentElement) {
      console.warn('Content element not found');
      return;
    }

    if (language === 'ur' && urduTranslation) {
      // Store original English content if not already stored
      if (!contentElement.dataset.originalContent) {
        contentElement.dataset.originalContent = contentElement.innerHTML;
      }

      // Replace with Urdu content (rendered as markdown)
      contentElement.innerHTML = renderMarkdown(urduTranslation.urdu_content);
      contentElement.style.direction = 'rtl';
      contentElement.style.textAlign = 'right';
    } else {
      // Restore original English content
      if (contentElement.dataset.originalContent) {
        contentElement.innerHTML = contentElement.dataset.originalContent;
        contentElement.style.direction = 'ltr';
        contentElement.style.textAlign = 'left';
      }
    }
  }, [urduTranslation]);

  /**
   * Simple markdown to HTML renderer (basic implementation)
   */
  const renderMarkdown = (markdown) => {
    return markdown
      .replace(/\n\n/g, '</p><p>')
      .replace(/\n/g, '<br>')
      .replace(/^/, '<p>')
      .replace(/$/, '</p>');
  };

  /**
   * Handle successful translation submission
   */
  const handleSubmissionSuccess = useCallback((result) => {
    console.log('Translation submitted successfully:', result);

    // Clear cache and reload
    translationCache.delete(chapterId);

    // Update state
    setTranslationExists(true);
    setShowSubmitModal(false);

    // Reload translation if in Urdu mode
    if (currentLanguage === 'ur') {
      loadTranslation();
    }

    // Show success message
    alert(`Translation submitted successfully! You earned ${result.points_awarded} point(s).`);
  }, [chapterId, currentLanguage, loadTranslation]);

  // Update content display when urduTranslation or currentLanguage changes
  useEffect(() => {
    if (currentLanguage === 'ur' && urduTranslation) {
      updateContentDisplay('ur');
    }
  }, [urduTranslation, currentLanguage, updateContentDisplay]);

  // Don't show anything if not authenticated
  if (!isAuthenticated) {
    return null;
  }

  return (
    <>
      <div className={styles.container}>
        {translationExists ? (
          // Show toggle button if translation exists
          <button
            className={styles.button}
            onClick={toggleLanguage}
            disabled={isLoading}
          >
            {isLoading ? (
              <span>Loading...</span>
            ) : (
              <>
                <span className={styles.icon}>
                  {currentLanguage === 'en' ? '🔄' : '↩️'}
                </span>
                <span className={styles.text}>
                  {currentLanguage === 'en' ? 'اردو میں پڑھیں' : 'Read in English'}
                </span>
              </>
            )}
          </button>
        ) : (
          // Show "Add Translation" button if translation doesn't exist
          <button
            className={`${styles.button} ${styles.addButton}`}
            onClick={() => setShowSubmitModal(true)}
          >
            <span className={styles.icon}>➕</span>
            <span className={styles.text}>Add Urdu Translation</span>
          </button>
        )}

        {error && (
          <div className={styles.error}>
            {error}
          </div>
        )}
      </div>

      {/* Translation submission modal */}
      {showSubmitModal && (
        <TranslationSubmitModal
          chapterId={chapterId}
          onClose={() => setShowSubmitModal(false)}
          onSuccess={handleSubmissionSuccess}
        />
      )}
    </>
  );
};

export default UrduTranslationButton;
