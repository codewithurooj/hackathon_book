/**
 * Swizzled DocItem/Layout component
 *
 * This wraps the original Docusaurus DocItem/Layout to inject the
 * UrduTranslationButton at the start of each chapter.
 */

import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

export default function LayoutWrapper(props) {
  // Extract chapter ID from the document metadata
  const { content } = props;
  const { metadata } = content;
  const chapterId = metadata?.id || metadata?.permalink?.replace(/^\//, '').replace(/\//g, '-');

  // Check if user is authenticated
  // In production, this should check actual auth state from context/Redux
  // For now, we'll check for a user ID in localStorage or sessionStorage
  const isAuthenticated = typeof window !== 'undefined' && (
    localStorage.getItem('user_id') !== null ||
    sessionStorage.getItem('user_id') !== null
  );

  return (
    <>
      {chapterId && (
        <UrduTranslationButton
          chapterId={chapterId}
          isAuthenticated={isAuthenticated}
        />
      )}
      <Layout {...props} />
    </>
  );
}
