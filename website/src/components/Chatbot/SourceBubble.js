import React from 'react';
import styles from './SourceBubble.module.css';

function SourceBubble({ source }) {
  const { chapter, section, page_url } = source;
  const displayUrl = page_url.replace('/docs/', '').replace(/\.md$/, '').replace(/\//g, ' > ');

  return (
    <a href={page_url} target="_blank" rel="noopener noreferrer" className={styles.sourceBubble}>
      {chapter} &raquo; {section}
    </a>
  );
}

export default SourceBubble;
