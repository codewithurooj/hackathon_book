/**
 * UrduTranslationButton Component
 *
 * Button that translates page content to Urdu using free translation API
 */

import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const UrduTranslationButton = ({ chapterId }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [originalContent, setOriginalContent] = useState('');
  const [originalTocContent, setOriginalTocContent] = useState('');

  useEffect(() => {
    // Load saved preference
    const saved = localStorage.getItem(`translate-${chapterId}`);
    if (saved === 'ur' && originalContent) {
      setIsUrdu(true);
    }
  }, [chapterId, originalContent]);

  const translateText = async (text) => {
    try {
      // Using LibreTranslate free API
      const response = await fetch('https://libretranslate.de/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          q: text,
          source: 'en',
          target: 'ur',
          format: 'text'
        })
      });

      if (!response.ok) {
        throw new Error('Translation service unavailable');
      }

      const data = await response.json();
      return data.translatedText;
    } catch (error) {
      console.error('Translation error:', error);
      // Fallback: Try MyMemory API
      try {
        const fallbackResponse = await fetch(
          `https://api.mymemory.translated.net/get?q=${encodeURIComponent(text)}&langpair=en|ur`
        );
        const fallbackData = await fallbackResponse.json();
        return fallbackData.responseData.translatedText;
      } catch (fallbackError) {
        throw new Error('All translation services failed');
      }
    }
  };

  const translateToUrdu = async () => {
    setIsTranslating(true);

    try {
      const article = document.querySelector('article');
      if (!article) {
        alert('Content not found');
        return;
      }

      // Store original HTML if not stored
      if (!originalContent) {
        setOriginalContent(article.innerHTML);
      }

      // Get all text nodes from article
      const textNodes = [];
      const walker = document.createTreeWalker(
        article,
        NodeFilter.SHOW_TEXT,
        {
          acceptNode: (node) => {
            // Skip empty text nodes and script/style content
            if (node.textContent.trim().length === 0) return NodeFilter.FILTER_REJECT;
            if (node.parentElement.tagName === 'SCRIPT') return NodeFilter.FILTER_REJECT;
            if (node.parentElement.tagName === 'STYLE') return NodeFilter.FILTER_REJECT;
            if (node.parentElement.tagName === 'CODE') return NodeFilter.FILTER_REJECT;
            if (node.parentElement.tagName === 'PRE') return NodeFilter.FILTER_REJECT;
            return NodeFilter.FILTER_ACCEPT;
          }
        }
      );

      while (walker.nextNode()) {
        textNodes.push(walker.currentNode);
      }

      // Also get text nodes from TOC (table of contents on right side)
      const toc = document.querySelector('.table-of-contents');
      if (toc) {
        // Store original TOC HTML if not stored
        if (!originalTocContent) {
          setOriginalTocContent(toc.innerHTML);
        }

        const tocWalker = document.createTreeWalker(
          toc,
          NodeFilter.SHOW_TEXT,
          {
            acceptNode: (node) => {
              if (node.textContent.trim().length === 0) return NodeFilter.FILTER_REJECT;
              return NodeFilter.FILTER_ACCEPT;
            }
          }
        );

        while (tocWalker.nextNode()) {
          textNodes.push(tocWalker.currentNode);
        }

        // Apply RTL to TOC
        toc.style.direction = 'rtl';
        toc.style.textAlign = 'right';
      }

      // Collect all texts to translate
      const textsToTranslate = textNodes
        .map(node => node.textContent.trim())
        .filter(text => text.length > 0);

      // Translate all texts in parallel
      const translationPromises = textsToTranslate.map(text =>
        translateText(text).catch(error => {
          console.error('Failed to translate:', text, error);
          return text; // Return original text on failure
        })
      );

      const translatedTexts = await Promise.all(translationPromises);

      // Apply all translations at once
      let translationIndex = 0;
      for (const node of textNodes) {
        const originalText = node.textContent.trim();
        if (originalText.length > 0) {
          node.textContent = translatedTexts[translationIndex];
          translationIndex++;
        }
      }

      // Apply RTL styling to article
      article.style.direction = 'rtl';
      article.style.textAlign = 'right';
      article.setAttribute('lang', 'ur');

      setIsUrdu(true);
      localStorage.setItem(`translate-${chapterId}`, 'ur');
      setIsTranslating(false);

    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation failed. Please try again.');
      setIsTranslating(false);
    }
  };

  const restoreEnglish = () => {
    const article = document.querySelector('article');
    if (article && originalContent) {
      article.innerHTML = originalContent;
      article.style.direction = 'ltr';
      article.style.textAlign = 'left';
      article.removeAttribute('lang');
    }

    // Restore TOC content
    const toc = document.querySelector('.table-of-contents');
    if (toc && originalTocContent) {
      toc.innerHTML = originalTocContent;
      toc.style.direction = 'ltr';
      toc.style.textAlign = 'left';
    }

    setIsUrdu(false);
    localStorage.setItem(`translate-${chapterId}`, 'en');
  };

  const toggleTranslation = () => {
    if (!isUrdu) {
      translateToUrdu();
    } else {
      restoreEnglish();
    }
  };

  return (
    <div className={styles.container}>
      <button
        className={styles.button}
        onClick={toggleTranslation}
        disabled={isTranslating}
        title={isUrdu ? 'Read in English' : 'Translate to Urdu'}
      >
        {isTranslating ? (
          <>
            <span className={styles.icon}>⏳</span>
            <span className={styles.text}>Translating...</span>
          </>
        ) : (
          <>
            <span className={styles.icon}>
              {isUrdu ? '🔙' : '🔄'}
            </span>
            <span className={styles.text}>
              {isUrdu ? 'Read in English' : 'اردو میں پڑھیں'}
            </span>
          </>
        )}
      </button>
    </div>
  );
};

export default UrduTranslationButton;
