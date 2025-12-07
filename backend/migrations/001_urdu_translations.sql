-- Migration: 001_urdu_translations.sql
-- Description: Create tables for Urdu translation feature
-- Date: 2025-12-07

-- Table: translations
-- Purpose: Store Urdu translations for each chapter
CREATE TABLE IF NOT EXISTS translations (
    id SERIAL PRIMARY KEY,
    chapter_id VARCHAR(100) NOT NULL UNIQUE,  -- e.g., "chapter-1-intro", "chapter-2-ros2-basics"
    urdu_content TEXT NOT NULL,               -- Urdu markdown content
    contributor_id INTEGER NOT NULL,          -- User ID who submitted translation
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT chk_urdu_content_not_empty CHECK (LENGTH(urdu_content) > 0)
);

-- Indexes for performance
CREATE INDEX idx_translations_chapter ON translations(chapter_id);
CREATE INDEX idx_translations_contributor ON translations(contributor_id);

-- Table: user_translation_points
-- Purpose: Track bonus points earned by users for translation contributions
CREATE TABLE IF NOT EXISTS user_translation_points (
    id SERIAL PRIMARY KEY,
    user_id INTEGER NOT NULL UNIQUE,          -- User ID (maps to existing users table)
    chapters_translated INTEGER DEFAULT 0,    -- Number of chapters translated
    total_points INTEGER DEFAULT 0,           -- Total bonus points (min(chapters_translated, 50))
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Constraints
    CONSTRAINT chk_points_range CHECK (total_points >= 0 AND total_points <= 50),
    CONSTRAINT chk_chapters_positive CHECK (chapters_translated >= 0)
);

-- Index for user lookup
CREATE INDEX idx_user_points_user ON user_translation_points(user_id);

-- Function to update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

-- Triggers to automatically update updated_at
CREATE TRIGGER update_translations_updated_at
    BEFORE UPDATE ON translations
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_translation_points_updated_at
    BEFORE UPDATE ON user_translation_points
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

-- Comments for documentation
COMMENT ON TABLE translations IS 'Stores Urdu translations for each chapter (first submission wins)';
COMMENT ON TABLE user_translation_points IS 'Tracks bonus points earned by users for translation contributions (max 50 points)';
COMMENT ON COLUMN translations.chapter_id IS 'Unique identifier for chapter (e.g., chapter-1-intro)';
COMMENT ON COLUMN translations.urdu_content IS 'Urdu markdown content for the chapter';
COMMENT ON COLUMN translations.contributor_id IS 'User ID who first submitted this translation';
COMMENT ON COLUMN user_translation_points.total_points IS 'Total bonus points (capped at 50)';
