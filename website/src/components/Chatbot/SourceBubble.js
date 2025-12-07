import React from 'react';
import styles from './SourceBubble.module.css';

function SourceBubble({ source }) {
  // Handle different source formats from backend
  let title = source.title || "Unknown Source";
  let url = source.url || source.page_url || "#";
  let snippet = source.snippet || source.content || "";

  if (source.metadata) {
    // If source has metadata field, get the actual source from there
    const actualSource = source.metadata.sources?.[0] || source.metadata;
    title = actualSource.title || title;
    url = actualSource.url || url;
    snippet = actualSource.snippet || snippet;
  }

  const displayUrl = url.replace('/docs/', '').replace(/\.md$/, '').replace(/\//g, ' > ');

  return (
    <a href={url} target="_blank" rel="noopener noreferrer" className={styles.sourceBubble}>
      {title}
    </a>
  );
}

export default SourceBubble;
