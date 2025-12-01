// website/src/components/Chatbot/ErrorToast.js
import React from 'react';
import toast, { Toaster } from 'react-hot-toast';
import styles from './ErrorToast.module.css';

// Custom toast component for consistent styling
const customToast = (message, type) => {
  const icon = type === 'error' ? '❌' : 'ℹ️';
  const className = type === 'error' ? styles.errorToast : styles.infoToast;

  return toast(
    <div className={styles.toastContent}>
      <span>{icon}</span>
      <span>{message}</span>
    </div>,
    {
      className: className,
      duration: type === 'error' ? 5000 : 3000,
    }
  );
};

export { customToast, Toaster };
