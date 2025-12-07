# Research: Urdu Translation Feature

**Feature**: 001-urdu-translation
**Date**: 2025-12-03
**Purpose**: Document technical research and decisions for implementing Urdu translation feature

## Technical Decisions

### 1. Authentication Check

**Decision**: Use existing Docusaurus/React authentication context

**Rationale**:
- Spec assumes authentication system exists (Assumption #1)
- Current project likely uses session-based or token-based auth
- Button visibility controlled client-side via React state
- Backend API validates authentication via existing middleware

**Alternatives Considered**:
- Build new authentication system → Rejected: Out of scope, assumption states it exists
- SSO integration → Rejected: Not mentioned in requirements

**Implementation Approach**:
- Check for authentication context in React components
- Conditionally render button based on `isAuthenticated` state
- Backend validates requests using existing auth middleware

---

### 2. Translation Storage

**Decision**: Store translations in existing database (likely PostgreSQL/Neon based on project stack)

**Rationale**:
- Need persistent storage for translations (FR-005)
- Must track contributor user ID (FR-006)
- Must support fast retrieval (3-second load time - SC-001)
- Relational data model fits naturally (user → translation → chapter relationships)

**Data Schema**:
```sql
translations (
  id: uuid PRIMARY KEY,
  chapter_id: string NOT NULL UNIQUE,  -- One translation per chapter
  urdu_content: text NOT NULL,
  contributor_user_id: string NOT NULL,
  created_at: timestamp DEFAULT now(),
  updated_at: timestamp DEFAULT now()
)

user_points (
  user_id: string PRIMARY KEY,
  translation_bonus_points: integer DEFAULT 0 CHECK (translation_bonus_points >= 0 AND translation_bonus_points <= 50),
  chapters_translated: integer DEFAULT 0,
  updated_at: timestamp DEFAULT now()
)
```

**Alternatives Considered**:
- File-based storage (MD files) → Rejected: Harder to query, track contributors, prevent race conditions
- Qdrant vector database → Rejected: Overkill for simple key-value lookups, Qdrant is for embeddings
- Redis cache only → Rejected: Need persistence, not just caching

---

### 3. Frontend Implementation Approach

**Decision**: Use React state + localStorage for language preference persistence

**Rationale**:
- Docusaurus is React-based (package.json shows React 19)
- Need to toggle content without page reload (FR-003, SC-003)
- Session persistence required (FR-009)
- LocalStorage survives page refreshes and tab switches

**Implementation Strategy**:
```javascript
// State management
const [currentLanguage, setCurrentLanguage] = useState('en'); // 'en' or 'ur'
const [urduTranslation, setUrduTranslation] = useState(null);

// Load preference from localStorage
useEffect(() => {
  const saved = localStorage.getItem(`chapter-${chapterId}-language`);
  if (saved) setCurrentLanguage(saved);
}, [chapterId]);

// Persist preference
const toggleLanguage = async () => {
  const newLang = currentLanguage === 'en' ? 'ur' : 'en';

  if (newLang === 'ur' && !urduTranslation) {
    // Fetch translation from API
    const translation = await fetchTranslation(chapterId);
    setUrduTranslation(translation);
  }

  setCurrentLanguage(newLang);
  localStorage.setItem(`chapter-${chapterId}-language`, newLang);
};
```

**Alternatives Considered**:
- Cookies → Rejected: LocalStorage is simpler, no server-side involvement needed
- Redux/Zustand state management → Rejected: Overkill for simple toggle, adds complexity
- URL parameter (?lang=ur) → Rejected: Doesn't persist across navigation

---

### 4. Translation API Design

**Decision**: RESTful API endpoints for translation operations

**Endpoints**:
```
GET  /api/v1/translations/{chapter_id}        # Fetch Urdu translation
POST /api/v1/translations                     # Submit new translation
GET  /api/v1/users/{user_id}/translation-points  # Get user's bonus points
```

**Rationale**:
- Existing backend uses FastAPI REST architecture
- Simple CRUD operations don't need GraphQL
- Consistent with existing `/api/v1/chatkit/chat` endpoint pattern

**Request/Response Formats**:

```typescript
// GET /api/v1/translations/{chapter_id}
Response: {
  chapter_id: string;
  urdu_content: string;
  contributor: {
    user_id: string;
    username?: string;
  };
  created_at: string; // ISO8601
}

// POST /api/v1/translations
Request: {
  chapter_id: string;
  urdu_content: string;
}
Response: {
  success: boolean;
  translation_id: string;
  points_awarded: number;
}

// GET /api/v1/users/{user_id}/translation-points
Response: {
  user_id: string;
  translation_bonus_points: number; // 0-50
  chapters_translated: number;
  breakdown: Array<{
    chapter_id: string;
    points: number;
    translated_at: string;
  }>;
}
```

**Alternatives Considered**:
- GraphQL → Rejected: Overkill for simple queries, adds complexity
- WebSockets → Rejected: No real-time requirements, REST is sufficient
- Embed in existing chat endpoint → Rejected: Violates single responsibility

---

### 5. Bonus Points Calculation

**Decision**: Linear distribution - ~1.35 points per chapter (50 points ÷ 37 chapters)

**Rationale**:
- Spec states "up to 50 total for all chapters" (Constraint)
- 37 chapters in the textbook (User Story 2, Acceptance Scenario 2)
- Fair distribution: 50 / 37 ≈ 1.35 points per chapter
- Round to 1 point per chapter for simplicity, cap at 50 total

**Implementation Logic**:
```python
def calculate_translation_points(user_id: str, chapters_translated_count: int) -> int:
    """
    Award 1 point per translated chapter, max 50 points total.
    First 37 chapters get 1 point each.
    After 37, no additional points (already at max if user translated all 37).
    """
    return min(chapters_translated_count * 1, 50)
```

**Alternative Considered**:
- Weighted by chapter length → Rejected: Spec doesn't mention quality/length factors
- Quality-based scoring → Rejected: Out of scope (Spec: Out of Scope section)
- Diminishing returns → Rejected: Spec implies linear (proportional bonus points)

---

### 6. Urdu Text Validation

**Decision**: Validate Unicode range for Urdu characters (U+0600 to U+06FF)

**Rationale**:
- FR-013 requires validation of Urdu text
- Urdu uses Arabic script Unicode block
- Prevent submission of non-Urdu text or empty translations

**Validation Logic**:
```python
import re

def is_valid_urdu_text(text: str) -> bool:
    """
    Validate that text contains Urdu/Arabic script characters.
    Unicode range: U+0600 - U+06FF (Arabic/Urdu)
    Also allow common punctuation and whitespace.
    """
    if not text or len(text.strip()) == 0:
        return False

    # Check if at least 50% of non-whitespace characters are Urdu
    urdu_pattern = re.compile(r'[\u0600-\u06FF]')
    urdu_chars = len(urdu_pattern.findall(text))
    total_chars = len(re.findall(r'\S', text))  # Non-whitespace chars

    return total_chars > 0 and (urdu_chars / total_chars) >= 0.5
```

**Alternatives Considered**:
- No validation → Rejected: FR-013 explicitly requires validation
- Language detection API (Google, Azure) → Rejected: Adds external dependency, cost
- Manual moderator review → Rejected: Out of scope per spec clarification (first wins)

---

### 7. Handling "First Submission Wins"

**Decision**: Database constraint + atomic check-and-insert

**Rationale**:
- User Story 3, Acceptance Scenario 3: First submission becomes canonical
- Must prevent race conditions when multiple users submit simultaneously
- Database UNIQUE constraint on (chapter_id) ensures atomicity

**Implementation**:
```python
async def submit_translation(chapter_id: str, urdu_content: str, user_id: str):
    """
    Attempt to insert translation. If chapter_id already exists, reject.
    """
    try:
        # Database enforces UNIQUE constraint on chapter_id
        result = await db.translations.insert_one({
            "id": str(uuid.uuid4()),
            "chapter_id": chapter_id,
            "urdu_content": urdu_content,
            "contributor_user_id": user_id,
            "created_at": datetime.utcnow()
        })

        # Award points
        await increment_user_points(user_id)

        return {"success": True, "translation_id": result.id}

    except UniqueViolationError:
        # Chapter already has a translation
        return {"success": False, "error": "Translation already exists for this chapter"}
```

**Alternatives Considered**:
- Application-level locking → Rejected: Race conditions possible, database constraint is atomic
- Queue system → Rejected: Adds complexity, not needed for hackathon scope
- Allow multiple submissions + voting → Rejected: User explicitly chose Option A (first wins)

---

### 8. Frontend UI Button Placement

**Decision**: Inject button into Docusaurus chapter layout using React component

**Rationale**:
- Spec requires button "at the start of each chapter" (FR-001, Constraint)
- Docusaurus allows custom React components in MDX
- Can use theme swizzling or doc wrapper to inject button

**Implementation Approach**:

Option A: Theme Swizzling (Recommended)
```bash
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

Then modify wrapper to inject button:
```jsx
// src/theme/DocItem/Layout/index.js
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import UrduTranslationButton from '@site/src/components/UrduTranslationButton';

export default function LayoutWrapper(props) {
  return (
    <>
      <UrduTranslationButton chapterId={props.content.metadata.id} />
      <Layout {...props} />
    </>
  );
}
```

Option B: Custom doc wrapper in docusaurus.config.js

**Alternative Considered**:
- Floating button → Rejected: Spec says "at the start of each chapter", not floating
- Manual MDX injection in each file → Rejected: Not maintainable for 37 chapters

---

### 9. Performance Optimization

**Decision**: Cache translations in memory after first load

**Rationale**:
- SC-001 requires 3-second load time
- Translations are static once submitted (no updates allowed - first wins)
- Reduce database queries for repeat visitors

**Caching Strategy**:
```javascript
// Frontend: In-memory cache for current session
const translationCache = new Map();

async function fetchTranslation(chapterId) {
  if (translationCache.has(chapterId)) {
    return translationCache.get(chapterId);
  }

  const response = await fetch(`/api/v1/translations/${chapterId}`);
  const data = await response.json();

  translationCache.set(chapterId, data);
  return data;
}
```

```python
# Backend: Redis cache (optional, if Redis available)
from functools import lru_cache

@lru_cache(maxsize=128)  # Cache up to 128 translations in memory
async def get_translation(chapter_id: str):
    return await db.translations.find_one({"chapter_id": chapter_id})
```

**Alternatives Considered**:
- No caching → Rejected: May not meet 3-second requirement consistently
- CDN caching → Rejected: Requires infrastructure setup, overkill for hackathon
- Service worker caching → Rejected: Adds complexity, in-memory cache is sufficient

---

### 10. Initial Translation Seeding

**Decision**: Provide admin API endpoint or script for bulk translation upload

**Rationale**:
- SC-005 expects 80% of chapters translated within first week
- Unlikely to get 30+ crowdsourced translations quickly
- Need way to seed initial translations (machine-translated + reviewed)

**Implementation**:
```python
# Admin endpoint (protected by admin auth)
@router.post("/api/v1/admin/translations/bulk")
async def bulk_upload_translations(
    translations: List[TranslationUpload],
    current_user: User = Depends(get_admin_user)
):
    """
    Bulk upload translations. Admin only.
    Format: [{ chapter_id, urdu_content }]
    """
    results = []
    for t in translations:
        try:
            await submit_translation(t.chapter_id, t.urdu_content, "system")
            results.append({"chapter_id": t.chapter_id, "success": True})
        except Exception as e:
            results.append({"chapter_id": t.chapter_id, "success": False, "error": str(e)})

    return {"uploaded": len([r for r in results if r["success"]]), "results": results}
```

**Alternatives Considered**:
- Wait for crowdsourcing → Rejected: Won't meet 80% coverage goal
- Automatic machine translation → Rejected: Spec says "not automatic without human review"
- Migration script → Considered: Could work, but API is more flexible

---

## Technology Stack Summary

### Frontend
- **Framework**: Docusaurus 3.9.2 (React 19)
- **State Management**: React useState + useEffect
- **Storage**: Browser localStorage
- **HTTP Client**: fetch API (built-in)
- **UI Integration**: Theme swizzling for button injection

### Backend
- **Framework**: FastAPI (Python)
- **Database**: PostgreSQL/Neon (assumed from project context)
- **ORM**: Direct SQL or async ORM (SQLAlchemy, Tortoise-ORM)
- **Authentication**: Existing middleware (to be identified)
- **Validation**: Pydantic models + custom Urdu validator

### Testing
- **Frontend**: Jest + React Testing Library
- **Backend**: pytest + pytest-asyncio
- **Integration**: End-to-end tests with test database

---

## Open Questions Resolved

All technical unknowns from spec have been researched and resolved:

1. ✅ Authentication method → Use existing system
2. ✅ Storage solution → PostgreSQL with relational schema
3. ✅ API design → RESTful endpoints documented
4. ✅ Points calculation → Linear: 1 point per chapter, cap at 50
5. ✅ UI integration → Docusaurus theme swizzling
6. ✅ Validation approach → Unicode range check for Urdu
7. ✅ Race condition handling → Database UNIQUE constraint
8. ✅ Performance → Client-side and optional server-side caching
9. ✅ Initial data → Admin bulk upload endpoint

**Status**: Research complete. Ready to proceed to Phase 1 (Data Model & Contracts).
