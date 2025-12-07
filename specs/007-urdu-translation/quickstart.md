# Quickstart: Urdu Translation Feature

**Feature**: 001-urdu-translation
**Date**: 2025-12-03
**For**: Developers implementing the Urdu translation feature

## Overview

This feature adds Urdu translation support to the Physical AI textbook with:
- ✅ Button to toggle between English and Urdu at the start of each chapter
- ✅ User contribution system for crowdsourcing translations
- ✅ Bonus points (up to 50) awarded for translation contributions
- ✅ "First submission wins" policy - one translation per chapter

## Prerequisites

### Backend
- Python 3.11+
- PostgreSQL database (existing Neon setup)
- FastAPI (already installed)
- Existing authentication middleware

### Frontend
- Node.js 20+
- Docusaurus 3.9.2 (already installed)
- React 19 (already installed)

## Implementation Steps

### Phase 1: Database Setup

**1. Create migration file**

Create `backend/migrations/001_urdu_translations.sql`:

```sql
-- Translations table
CREATE TABLE translations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(255) NOT NULL UNIQUE,
    urdu_content TEXT NOT NULL,
    contributor_user_id VARCHAR(255) NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_contributor FOREIGN KEY (contributor_user_id)
        REFERENCES users(id) ON DELETE CASCADE,
    CONSTRAINT check_urdu_content_not_empty CHECK (LENGTH(TRIM(urdu_content)) > 0)
);

CREATE INDEX idx_translations_chapter ON translations(chapter_id);
CREATE INDEX idx_translations_contributor ON translations(contributor_user_id);

-- User points table
CREATE TABLE user_translation_points (
    user_id VARCHAR(255) PRIMARY KEY,
    translation_bonus_points INTEGER NOT NULL DEFAULT 0
        CHECK (translation_bonus_points >= 0 AND translation_bonus_points <= 50),
    chapters_translated INTEGER NOT NULL DEFAULT 0,
    last_translation_at TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_user FOREIGN KEY (user_id)
        REFERENCES users(id) ON DELETE CASCADE
);

-- Triggers for updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_translations_updated_at
    BEFORE UPDATE ON translations FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_translation_points_updated_at
    BEFORE UPDATE ON user_translation_points FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();
```

**2. Run migration**

```bash
cd backend
psql $DATABASE_URL -f migrations/001_urdu_translations.sql
```

---

### Phase 2: Backend API

**1. Create models** (`backend/app/models/translation.py`)

```python
from pydantic import BaseModel, Field, validator
from datetime import datetime
from typing import Optional, List
import re

class TranslationSubmit(BaseModel):
    chapter_id: str = Field(..., min_length=1, max_length=255)
    urdu_content: str = Field(..., min_length=10)

    @validator('urdu_content')
    def validate_urdu_content(cls, v):
        if not v or len(v.strip()) < 10:
            raise ValueError("Urdu content must be at least 10 characters")

        urdu_pattern = re.compile(r'[\u0600-\u06FF]')
        urdu_chars = len(urdu_pattern.findall(v))
        total_chars = len(re.findall(r'\S', v))

        if total_chars == 0 or (urdu_chars / total_chars) < 0.5:
            raise ValueError("Content must contain at least 50% Urdu characters")

        return v

class TranslationResponse(BaseModel):
    id: str
    chapter_id: str
    urdu_content: str
    contributor: dict
    created_at: datetime

class UserPointsResponse(BaseModel):
    user_id: str
    username: Optional[str]
    translation_bonus_points: int
    chapters_translated: int
    breakdown: List[dict]
```

**2. Create service** (`backend/app/services/translation_service.py`)

```python
from typing import Optional
import uuid
from datetime import datetime

class TranslationService:
    def __init__(self, db):
        self.db = db

    async def get_translation(self, chapter_id: str) -> Optional[dict]:
        """Get Urdu translation for a chapter"""
        return await self.db.translations.find_one({"chapter_id": chapter_id})

    async def submit_translation(self, chapter_id: str, urdu_content: str, user_id: str):
        """Submit new translation (first wins)"""
        try:
            translation = {
                "id": str(uuid.uuid4()),
                "chapter_id": chapter_id,
                "urdu_content": urdu_content,
                "contributor_user_id": user_id,
                "created_at": datetime.utcnow()
            }

            await self.db.translations.insert_one(translation)
            await self._award_points(user_id)

            return {"success": True, "translation_id": translation["id"], "points_awarded": 1}

        except UniqueViolationError:
            return {"success": False, "error": "Translation already exists for this chapter"}

    async def _award_points(self, user_id: str):
        """Award 1 point to user, cap at 50"""
        await self.db.user_translation_points.upsert(
            {"user_id": user_id},
            {
                "$inc": {"chapters_translated": 1},
                "$min": {"translation_bonus_points": 50},
                "$set": {"last_translation_at": datetime.utcnow()}
            }
        )
        # Recalculate points: min(chapters_translated, 50)
        points_doc = await self.db.user_translation_points.find_one({"user_id": user_id})
        new_points = min(points_doc["chapters_translated"], 50)
        await self.db.user_translation_points.update_one(
            {"user_id": user_id},
            {"$set": {"translation_bonus_points": new_points}}
        )

    async def get_user_points(self, user_id: str):
        """Get user's translation points"""
        points = await self.db.user_translation_points.find_one({"user_id": user_id})
        if not points:
            return {"user_id": user_id, "translation_bonus_points": 0, "chapters_translated": 0, "breakdown": []}

        # Get breakdown
        translations = await self.db.translations.find({"contributor_user_id": user_id}).to_list(None)
        breakdown = [
            {"chapter_id": t["chapter_id"], "points": 1, "translated_at": t["created_at"]}
            for t in translations
        ]

        return {**points, "breakdown": breakdown}
```

**3. Create router** (`backend/app/routers/translation.py`)

```python
from fastapi import APIRouter, Depends, HTTPException, status
from app.models.translation import *
from app.services.translation_service import TranslationService
from app.utils.auth import get_current_user

router = APIRouter(prefix="/api/v1", tags=["translations"])

@router.get("/translations/{chapter_id}", response_model=TranslationResponse)
async def get_translation(chapter_id: str, service: TranslationService = Depends()):
    translation = await service.get_translation(chapter_id)
    if not translation:
        raise HTTPException(status_code=404, detail="Translation not found")
    return translation

@router.post("/translations", response_model=TranslationSubmitResult, status_code=201)
async def submit_translation(
    data: TranslationSubmit,
    current_user: User = Depends(get_current_user),
    service: TranslationService = Depends()
):
    result = await service.submit_translation(
        data.chapter_id,
        data.urdu_content,
        current_user.id
    )

    if not result["success"]:
        raise HTTPException(status_code=409, detail=result["error"])

    return result

@router.get("/users/{user_id}/translation-points", response_model=UserPointsResponse)
async def get_user_points(
    user_id: str,
    current_user: User = Depends(get_current_user),
    service: TranslationService = Depends()
):
    points = await service.get_user_points(user_id)
    return points
```

**4. Register router** in `backend/app/main.py`:

```python
from app.routers import translation

app.include_router(translation.router)
```

---

### Phase 3: Frontend Components

**1. Create translation button component**

Create `website/src/components/UrduTranslationButton/index.js`:

```jsx
import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

function UrduTranslationButton({ chapterId, onLanguageChange }) {
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [loading, setLoading] = useState(false);
  const [hasTranslation, setHasTranslation] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    // Check authentication (adapt to your auth system)
    const checkAuth = () => {
      // TODO: Replace with actual auth check
      const authToken = localStorage.getItem('authToken');
      setIsAuthenticated(!!authToken);
    };

    // Check if translation exists
    const checkTranslation = async () => {
      try {
        const response = await fetch(`http://localhost:8000/api/v1/translations/check/${chapterId}`);
        const data = await response.json();
        setHasTranslation(data.has_translation);
      } catch (error) {
        console.error('Error checking translation:', error);
      }
    };

    checkAuth();
    checkTranslation();

    // Load saved language preference
    const savedLang = localStorage.getItem(`chapter-${chapterId}-lang`);
    if (savedLang) {
      setCurrentLanguage(savedLang);
    }
  }, [chapterId]);

  const toggleLanguage = async () => {
    if (!hasTranslation) {
      alert('Translation not yet available for this chapter');
      return;
    }

    setLoading(true);
    const newLang = currentLanguage === 'en' ? 'ur' : 'en';

    try {
      if (newLang === 'ur') {
        // Fetch translation
        const response = await fetch(`http://localhost:8000/api/v1/translations/${chapterId}`);
        if (!response.ok) throw new Error('Translation not found');
        const translation = await response.json();
        onLanguageChange(newLang, translation.urdu_content);
      } else {
        onLanguageChange(newLang, null);
      }

      setCurrentLanguage(newLang);
      localStorage.setItem(`chapter-${chapterId}-lang`, newLang);
    } catch (error) {
      console.error('Error toggling language:', error);
      alert('Failed to load translation');
    } finally {
      setLoading(false);
    }
  };

  if (!isAuthenticated || !hasTranslation) {
    return null; // Hide button if not authenticated or no translation
  }

  return (
    <button
      className={styles.translationButton}
      onClick={toggleLanguage}
      disabled={loading}
    >
      {loading ? 'Loading...' : currentLanguage === 'en' ? '🌐 اردو میں پڑھیں' : '🌐 Read in English'}
    </button>
  );
}

export default UrduTranslationButton;
```

**2. Create styles** (`website/src/components/UrduTranslationButton/styles.module.css`):

```css
.translationButton {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border: none;
  padding: 10px 20px;
  border-radius: 8px;
  font-size: 16px;
  font-weight: 600;
  cursor: pointer;
  margin-bottom: 20px;
  transition: transform 0.2s, box-shadow 0.2s;
}

.translationButton:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(102, 126, 234, 0.4);
}

.translationButton:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}
```

**3. Swizzle Docusaurus to inject button**

```bash
cd website
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

Edit `website/src/theme/DocItem/Layout/index.js`:

```jsx
import React, { useState } from 'react';
import Layout from '@theme-original/DocItem/Layout';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

export default function LayoutWrapper(props) {
  const [language, setLanguage] = useState('en');
  const [urduContent, setUrduContent] = useState(null);

  const handleLanguageChange = (newLang, content) => {
    setLanguage(newLang);
    setUrduContent(content);
  };

  const chapterId = props.content.metadata.id;

  return (
    <>
      <UrduTranslationButton
        chapterId={chapterId}
        onLanguageChange={handleLanguageChange}
      />
      {language === 'ur' && urduContent ? (
        <div style={{ direction: 'rtl', fontFamily: 'Noto Nastaliq Urdu, serif' }}>
          <div dangerouslySetInnerHTML={{ __html: urduContent }} />
        </div>
      ) : (
        <Layout {...props} />
      )}
    </>
  );
}
```

---

### Phase 4: Testing

**Backend tests** (`backend/tests/test_translation.py`):

```python
import pytest
from app.services.translation_service import TranslationService

@pytest.mark.asyncio
async def test_submit_translation_success():
    service = TranslationService(mock_db)
    result = await service.submit_translation(
        "chapter-1", "اردو میں متن", "user-123"
    )
    assert result["success"] is True
    assert result["points_awarded"] == 1

@pytest.mark.asyncio
async def test_submit_translation_duplicate():
    service = TranslationService(mock_db)
    # First submission
    await service.submit_translation("chapter-1", "اردو میں متن", "user-123")
    # Second submission (should fail)
    result = await service.submit_translation("chapter-1", "دوسرا متن", "user-456")
    assert result["success"] is False
    assert "already exists" in result["error"]
```

**Run tests**:

```bash
cd backend
pytest tests/test_translation.py -v
```

---

## Quick Verification

### 1. Backend health check
```bash
curl http://localhost:8000/api/v1/translations/check/chapter-1-intro
# Expected: {"chapter_id":"chapter-1-intro","has_translation":false}
```

### 2. Submit test translation (with auth token)
```bash
curl -X POST http://localhost:8000/api/v1/translations \
  -H "Authorization: Bearer YOUR_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"chapter-1-intro","urdu_content":"یہ ایک ٹیسٹ ترجمہ ہے"}'
```

### 3. Frontend check
- Open http://localhost:3000/docs/chapter-1-intro/what-is-physical-ai
- Log in (if not already)
- You should see the Urdu translation button at the top

---

## Troubleshooting

### Button not showing
- ✅ Check user is logged in
- ✅ Check translation exists in database (`SELECT * FROM translations WHERE chapter_id = 'xxx'`)
- ✅ Check browser console for errors

### Translation not loading
- ✅ Verify backend API is running (http://localhost:8000)
- ✅ Check network tab for failed requests
- ✅ Verify CORS settings in FastAPI

### Points not updating
- ✅ Check `user_translation_points` table
- ✅ Verify transaction completed successfully
- ✅ Check logs for database errors

---

## Next Steps

After basic implementation:
1. Add contribution UI for users to submit translations (User Story 3)
2. Add points display on user profile/dashboard
3. Seed initial translations using bulk upload API
4. Add analytics tracking for translation usage

**Need help?** See `data-model.md` for database schema and `contracts/openapi.yaml` for full API specification.
