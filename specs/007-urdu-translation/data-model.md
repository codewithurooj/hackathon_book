# Data Model: Urdu Translation Feature

**Feature**: 001-urdu-translation
**Date**: 2025-12-03
**Status**: Design

## Entity Relationship Overview

```
User (existing)
  ↓ 1:1
UserTranslationPoints
  ↓ 1:N
Translation
  ↓ N:1
Chapter (conceptual - stored as chapter_id string)
```

## Entities

### 1. Translation

**Purpose**: Store Urdu translations for textbook chapters

**Attributes**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY | Unique translation identifier |
| chapter_id | String | NOT NULL, UNIQUE | Chapter identifier (e.g., "chapter-1-intro/what-is-physical-ai") |
| urdu_content | Text | NOT NULL | Full Urdu translation of chapter content |
| contributor_user_id | String | NOT NULL, FOREIGN KEY → User.id | User who submitted the translation |
| created_at | Timestamp | NOT NULL, DEFAULT now() | When translation was submitted |
| updated_at | Timestamp | NOT NULL, DEFAULT now() | Last modification timestamp |

**Indexes**:
- PRIMARY KEY on `id`
- UNIQUE INDEX on `chapter_id` (ensures "first submission wins")
- INDEX on `contributor_user_id` (for user queries)

**Validation Rules**:
- `urdu_content` must contain at least 50% Urdu Unicode characters (U+0600-U+06FF)
- `chapter_id` must match valid chapter format
- `contributor_user_id` must reference existing user

**State Transitions**: None (immutable after creation - first wins policy)

---

### 2. UserTranslationPoints

**Purpose**: Track bonus points earned from translation contributions

**Attributes**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| user_id | String | PRIMARY KEY, FOREIGN KEY → User.id | User identifier |
| translation_bonus_points | Integer | NOT NULL, DEFAULT 0, CHECK >= 0 AND <= 50 | Total bonus points from translations |
| chapters_translated | Integer | NOT NULL, DEFAULT 0, CHECK >= 0 | Count of chapters user has translated |
| last_translation_at | Timestamp | NULLABLE | Timestamp of most recent translation |
| updated_at | Timestamp | NOT NULL, DEFAULT now() | Last points update |

**Indexes**:
- PRIMARY KEY on `user_id`

**Validation Rules**:
- `translation_bonus_points` capped at 50 (database constraint)
- `chapters_translated` must be non-negative
- `translation_bonus_points` = min(chapters_translated * 1, 50)

**State Transitions**:
```
Initial State: { translation_bonus_points: 0, chapters_translated: 0 }
                      ↓ (User submits translation)
Updated State: { translation_bonus_points: +1, chapters_translated: +1 }
                      ↓ (User submits more translations)
Final State: { translation_bonus_points: 50, chapters_translated: ≥ 37 } (capped)
```

---

### 3. TranslationContribution (View/Derived Entity)

**Purpose**: Aggregate view joining translations with user points for display

**Not a physical table** - derived from joins for dashboard/profile views

**Attributes**:
| Field | Type | Source | Description |
|-------|------|--------|-------------|
| user_id | String | UserTranslationPoints.user_id | User identifier |
| username | String | User.username | User display name |
| total_points | Integer | UserTranslationPoints.translation_bonus_points | Total bonus points |
| chapters_translated | Integer | UserTranslationPoints.chapters_translated | Number of chapters |
| translations | Array | Translation[] | List of translations by user |

**Query**:
```sql
SELECT
  utp.user_id,
  u.username,
  utp.translation_bonus_points AS total_points,
  utp.chapters_translated,
  JSON_AGG(
    JSON_BUILD_OBJECT(
      'chapter_id', t.chapter_id,
      'created_at', t.created_at
    )
  ) AS translations
FROM user_translation_points utp
JOIN users u ON u.id = utp.user_id
LEFT JOIN translations t ON t.contributor_user_id = utp.user_id
GROUP BY utp.user_id, u.username, utp.translation_bonus_points, utp.chapters_translated;
```

---

## Database Schema (SQL)

### PostgreSQL Schema

```sql
-- Note: Assumes 'users' table already exists with 'id' column

-- Translations table
CREATE TABLE translations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(255) NOT NULL UNIQUE,
    urdu_content TEXT NOT NULL,
    contributor_user_id VARCHAR(255) NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Foreign key constraint (assuming users table exists)
    CONSTRAINT fk_contributor FOREIGN KEY (contributor_user_id)
        REFERENCES users(id) ON DELETE CASCADE,

    -- Validation: Ensure urdu_content is not empty
    CONSTRAINT check_urdu_content_not_empty CHECK (LENGTH(TRIM(urdu_content)) > 0)
);

-- Index for fast lookups by chapter
CREATE INDEX idx_translations_chapter ON translations(chapter_id);

-- Index for user queries (which chapters did user translate?)
CREATE INDEX idx_translations_contributor ON translations(contributor_user_id);

-- User translation points table
CREATE TABLE user_translation_points (
    user_id VARCHAR(255) PRIMARY KEY,
    translation_bonus_points INTEGER NOT NULL DEFAULT 0
        CHECK (translation_bonus_points >= 0 AND translation_bonus_points <= 50),
    chapters_translated INTEGER NOT NULL DEFAULT 0
        CHECK (chapters_translated >= 0),
    last_translation_at TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,

    -- Foreign key constraint
    CONSTRAINT fk_user FOREIGN KEY (user_id)
        REFERENCES users(id) ON DELETE CASCADE
);

-- Trigger to update 'updated_at' on row modification
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_translations_updated_at
    BEFORE UPDATE ON translations
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_translation_points_updated_at
    BEFORE UPDATE ON user_translation_points
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();
```

---

## Backend Models (Pydantic)

### Request/Response Models

```python
from pydantic import BaseModel, Field, validator
from datetime import datetime
from typing import Optional, List
import re

class TranslationSubmit(BaseModel):
    """Request model for submitting a new translation"""
    chapter_id: str = Field(..., min_length=1, max_length=255)
    urdu_content: str = Field(..., min_length=10)

    @validator('urdu_content')
    def validate_urdu_content(cls, v):
        """Ensure content contains Urdu characters"""
        if not v or len(v.strip()) < 10:
            raise ValueError("Urdu content must be at least 10 characters")

        # Check for Urdu Unicode characters (U+0600 - U+06FF)
        urdu_pattern = re.compile(r'[\u0600-\u06FF]')
        urdu_chars = len(urdu_pattern.findall(v))
        total_chars = len(re.findall(r'\S', v))  # Non-whitespace

        if total_chars == 0 or (urdu_chars / total_chars) < 0.5:
            raise ValueError("Content must contain at least 50% Urdu characters")

        return v

class TranslationResponse(BaseModel):
    """Response model for translation data"""
    id: str
    chapter_id: str
    urdu_content: str
    contributor: dict  # { user_id: str, username: Optional[str] }
    created_at: datetime

    class Config:
        from_attributes = True

class TranslationSubmitResult(BaseModel):
    """Response for translation submission"""
    success: bool
    translation_id: Optional[str] = None
    points_awarded: Optional[int] = None
    error: Optional[str] = None

class UserPointsResponse(BaseModel):
    """Response for user translation points"""
    user_id: str
    username: Optional[str]
    translation_bonus_points: int = Field(..., ge=0, le=50)
    chapters_translated: int = Field(..., ge=0)
    breakdown: List[dict]  # [{ chapter_id: str, points: int, translated_at: datetime }]

    class Config:
        from_attributes = True

class BulkTranslationUpload(BaseModel):
    """Admin bulk upload request"""
    translations: List[dict]  # [{ chapter_id: str, urdu_content: str }]

    @validator('translations')
    def validate_translations_list(cls, v):
        if not v or len(v) == 0:
            raise ValueError("Must provide at least one translation")
        return v
```

---

## Frontend Models (TypeScript)

```typescript
// types/translation.ts

export interface Translation {
  id: string;
  chapter_id: string;
  urdu_content: string;
  contributor: {
    user_id: string;
    username?: string;
  };
  created_at: string; // ISO8601
}

export interface TranslationSubmitRequest {
  chapter_id: string;
  urdu_content: string;
}

export interface TranslationSubmitResult {
  success: boolean;
  translation_id?: string;
  points_awarded?: number;
  error?: string;
}

export interface UserTranslationPoints {
  user_id: string;
  username?: string;
  translation_bonus_points: number; // 0-50
  chapters_translated: number;
  breakdown: Array<{
    chapter_id: string;
    points: number;
    translated_at: string; // ISO8601
  }>;
}

export type Language = 'en' | 'ur';

export interface ChapterLanguageState {
  chapterId: string;
  currentLanguage: Language;
  urduTranslation: Translation | null;
  loading: boolean;
  error: string | null;
}
```

---

## Validation Rules Summary

### Translation Entity
1. **chapter_id uniqueness**: Enforced by database UNIQUE constraint
2. **Urdu content validation**: At least 50% Urdu Unicode characters
3. **Content length**: Minimum 10 characters (reasonable minimum for a chapter)
4. **Immutability**: Once created, translations cannot be updated (first wins)

### UserTranslationPoints Entity
1. **Points cap**: Cannot exceed 50 points (database CHECK constraint)
2. **Points calculation**: 1 point per chapter, min(chapters_translated * 1, 50)
3. **Chapters count**: Must be non-negative
4. **Consistency**: chapters_translated count must match actual translation records

### Business Rules
1. **First submission wins**: UNIQUE constraint on chapter_id prevents duplicate translations
2. **Atomic operations**: Translation submission + points award in single transaction
3. **No deletions**: Translations are permanent (no DELETE API)
4. **No updates**: Translations cannot be modified after submission

---

## Migration Strategy

### Step 1: Create Tables
```bash
# Run SQL schema creation script
psql -d physical_ai_book -f specs/001-urdu-translation/migrations/001_create_translation_tables.sql
```

### Step 2: Seed Initial Data (Optional)
```bash
# If seeding translations, run bulk upload via admin API
curl -X POST /api/v1/admin/translations/bulk \
  -H "Authorization: Bearer $ADMIN_TOKEN" \
  -H "Content-Type: application/json" \
  -d @translations_seed.json
```

### Step 3: Verify
```sql
-- Check tables created
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
AND table_name IN ('translations', 'user_translation_points');

-- Check constraints
SELECT conname, contype FROM pg_constraint
WHERE conrelid = 'translations'::regclass;
```

---

## Data Access Patterns

### Common Queries

**1. Get translation for chapter**
```sql
SELECT id, chapter_id, urdu_content, contributor_user_id, created_at
FROM translations
WHERE chapter_id = :chapter_id;
```

**2. Check if chapter has translation**
```sql
SELECT EXISTS(SELECT 1 FROM translations WHERE chapter_id = :chapter_id);
```

**3. Get user's translation points**
```sql
SELECT user_id, translation_bonus_points, chapters_translated
FROM user_translation_points
WHERE user_id = :user_id;
```

**4. Get user's contributed translations**
```sql
SELECT t.chapter_id, t.created_at
FROM translations t
WHERE t.contributor_user_id = :user_id
ORDER BY t.created_at DESC;
```

**5. Get top contributors (leaderboard)**
```sql
SELECT
  utp.user_id,
  u.username,
  utp.translation_bonus_points,
  utp.chapters_translated
FROM user_translation_points utp
JOIN users u ON u.id = utp.user_id
ORDER BY utp.translation_bonus_points DESC, utp.chapters_translated DESC
LIMIT 10;
```

---

## Status

✅ **Data model complete**
✅ **Entities defined with relationships**
✅ **Validation rules specified**
✅ **Database schema provided**
✅ **Backend and frontend models documented**

**Next**: API Contracts (contracts/)
