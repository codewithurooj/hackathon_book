/**
 * TypeScript types for translation feature
 *
 * These types match the backend Pydantic models for type safety
 * across the frontend-backend API boundary.
 */

/**
 * Translation response from GET /api/v1/translations/{chapter_id}
 */
export interface Translation {
  chapter_id: string;
  urdu_content: string;
  contributor_id: number;
  created_at: string; // ISO8601 format
}

/**
 * Translation submission request for POST /api/v1/translations
 */
export interface TranslationSubmitRequest {
  chapter_id: string;
  urdu_content: string;
}

/**
 * Translation submission result from POST /api/v1/translations
 */
export interface TranslationSubmitResult {
  success: boolean;
  message: string;
  translation_id?: number | null;
  points_awarded: number;
}

/**
 * User translation points from GET /api/v1/users/{user_id}/translation-points
 */
export interface UserTranslationPoints {
  user_id: number;
  chapters_translated: number;
  total_points: number;
}

/**
 * Translation existence check from GET /api/v1/translations/check/{chapter_id}
 */
export interface TranslationCheckResponse {
  chapter_id: string;
  exists: boolean;
}

/**
 * Language codes
 */
export type Language = 'en' | 'ur';

/**
 * Chapter language state (stored in localStorage and component state)
 */
export interface ChapterLanguageState {
  chapterId: string;
  currentLanguage: Language;
  translation?: Translation | null;
  isLoading: boolean;
  error?: string | null;
}

/**
 * Bulk translation upload item (admin only)
 */
export interface BulkTranslationUpload {
  chapter_id: string;
  urdu_content: string;
}

/**
 * API Error Response
 */
export interface APIError {
  detail: string;
  status_code: number;
}
