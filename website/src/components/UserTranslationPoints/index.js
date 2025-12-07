/**
 * UserTranslationPoints Component
 *
 * Displays user's translation bonus points and breakdown.
 *
 * Features:
 * - Fetches user points from API
 * - Shows chapters translated and total points
 * - Updates in real-time when user submits translations
 * - Visual progress indicator (0-50 points)
 */

import React, { useState, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

// API configuration
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

const UserTranslationPoints = ({ userId, autoRefresh = false }) => {
  const [points, setPoints] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  /**
   * Fetch user translation points from API
   */
  const fetchPoints = useCallback(async () => {
    if (!userId) {
      setIsLoading(false);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(
        `${API_BASE_URL}/translations/users/${userId}/points`
      );

      if (!response.ok) {
        throw new Error('Failed to fetch translation points');
      }

      const data = await response.json();
      setPoints(data);
    } catch (err) {
      console.error('Error fetching translation points:', err);
      setError(err.message);
    } finally {
      setIsLoading(false);
    }
  }, [userId]);

  // Fetch points on mount
  useEffect(() => {
    fetchPoints();
  }, [fetchPoints]);

  // Auto-refresh if enabled (e.g., after submitting translation)
  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      fetchPoints();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [autoRefresh, fetchPoints]);

  if (!userId) {
    return null;
  }

  if (isLoading && !points) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading points...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>
          Error loading points: {error}
        </div>
      </div>
    );
  }

  const totalPoints = points?.total_points || 0;
  const chaptersTranslated = points?.chapters_translated || 0;
  const progressPercentage = (totalPoints / 50) * 100;

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <div className={styles.header}>
          <h3 className={styles.title}>Translation Bonus Points</h3>
          <button
            className={styles.refreshButton}
            onClick={fetchPoints}
            title="Refresh points"
          >
            🔄
          </button>
        </div>

        <div className={styles.pointsDisplay}>
          <div className={styles.mainPoints}>
            <span className={styles.pointsNumber}>{totalPoints}</span>
            <span className={styles.pointsLabel}>/ 50 points</span>
          </div>

          <div className={styles.progressBar}>
            <div
              className={styles.progressFill}
              style={{ width: `${progressPercentage}%` }}
            />
          </div>
        </div>

        <div className={styles.breakdown}>
          <div className={styles.stat}>
            <span className={styles.statLabel}>Chapters Translated:</span>
            <span className={styles.statValue}>{chaptersTranslated}</span>
          </div>

          <div className={styles.stat}>
            <span className={styles.statLabel}>Points per Chapter:</span>
            <span className={styles.statValue}>1 point</span>
          </div>

          {totalPoints >= 50 && (
            <div className={styles.maxPointsNotice}>
              🎉 Maximum bonus points reached!
            </div>
          )}

          {totalPoints < 50 && chaptersTranslated > 0 && (
            <div className={styles.encouragement}>
              Keep translating to earn up to {50 - totalPoints} more points!
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default UserTranslationPoints;
