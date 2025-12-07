/**
 * User Profile Page
 *
 * Displays user information and translation bonus points.
 * Access at: /profile
 */

import React from 'react';
import Layout from '@theme/Layout';
import UserTranslationPoints from '@site/src/components/UserTranslationPoints';

export default function Profile() {
  // In production, get user ID from authentication context
  // For now, read from localStorage
  const userId = typeof window !== 'undefined'
    ? localStorage.getItem('user_id') || sessionStorage.getItem('user_id')
    : null;

  const isAuthenticated = userId !== null;

  return (
    <Layout
      title="User Profile"
      description="View your translation contributions and bonus points"
    >
      <main style={{ padding: '2rem', maxWidth: '800px', margin: '0 auto' }}>
        <h1>User Profile</h1>

        {!isAuthenticated ? (
          <div
            style={{
              padding: '2rem',
              backgroundColor: 'var(--ifm-color-warning-lightest)',
              borderLeft: '4px solid var(--ifm-color-warning)',
              borderRadius: '0.5rem',
              marginTop: '2rem'
            }}
          >
            <h3>Authentication Required</h3>
            <p>
              Please log in to view your profile and translation points.
            </p>
            <p style={{ fontSize: '0.9rem', marginTop: '1rem', opacity: 0.8 }}>
              <strong>For testing:</strong> Set a user ID in localStorage:
              <br />
              <code>localStorage.setItem('user_id', '123')</code>
            </p>
          </div>
        ) : (
          <>
            <div style={{ marginBottom: '2rem' }}>
              <p>
                <strong>User ID:</strong> {userId}
              </p>
            </div>

            <UserTranslationPoints userId={parseInt(userId)} autoRefresh={true} />

            <div
              style={{
                marginTop: '2rem',
                padding: '1rem',
                backgroundColor: 'var(--ifm-background-surface-color)',
                border: '1px solid var(--ifm-color-emphasis-300)',
                borderRadius: '0.5rem'
              }}
            >
              <h3>How to Earn More Points</h3>
              <ul>
                <li>Translate chapters that don't have Urdu translations yet</li>
                <li>Each chapter translation earns you 1 point</li>
                <li>Maximum bonus points: 50 points</li>
                <li>First submission wins - be quick!</li>
              </ul>
            </div>
          </>
        )}
      </main>
    </Layout>
  );
}
