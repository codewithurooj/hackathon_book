# Urdu Translation Feature - Implementation Notes

## Overview

The Urdu translation feature has been successfully implemented! This document provides setup instructions and implementation details.

## What Was Implemented

### ✅ Phase 1: Setup
- Database migration file created for PostgreSQL
- Urdu text validator utility
- Environment configuration updated

### ✅ Phase 2: Foundational
- Translation Pydantic models (backend)
- TranslationService with database operations
- TypeScript types (frontend)
- Authentication middleware

### ✅ Phase 3: User Story 1 - View Urdu Translation (MVP)
- GET /api/v1/translations/{chapter_id} endpoint
- GET /api/v1/translations/check/{chapter_id} endpoint
- UrduTranslationButton React component
- Docusaurus theme integration
- Urdu font (Noto Nastaliq Urdu) and RTL support
- In-memory caching for performance

### ✅ Phase 4: User Story 2 - Earn Bonus Points
- Points calculation logic (1 point per chapter, max 50)
- GET /api/v1/translations/users/{user_id}/points endpoint
- UserTranslationPoints React component
- User profile page at /profile

### ✅ Phase 5: User Story 3 - Contribute Translation
- POST /api/v1/translations endpoint with authentication
- Urdu validation (50% Urdu characters required)
- TranslationSubmitModal React component
- First-wins policy enforcement via database constraint
- POST /api/v1/translations/admin/bulk endpoint for initial seeding

### ✅ Phase 6: Polish
- Comprehensive error handling
- Loading states and skeletons
- CORS configuration
- Database indexes for performance
- Structured logging

## Setup Instructions

### 1. Database Setup (REQUIRED)

The feature requires PostgreSQL. You have two options:

**Option A: Local PostgreSQL**
```bash
# Create database
createdb physical_ai_book

# Run migration
psql -d physical_ai_book -f backend/migrations/001_urdu_translations.sql
```

**Option B: Neon.tech (Cloud PostgreSQL)**
1. Sign up at https://neon.tech
2. Create a new project
3. Copy the connection string
4. Add to .env file

### 2. Environment Configuration

Update `backend/.env` with your DATABASE_URL:

```env
# Local PostgreSQL
DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_book

# OR Neon.tech
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/physical_ai_book?sslmode=require
```

### 3. Install Dependencies

**Backend:**
```bash
cd backend
pip install asyncpg  # PostgreSQL driver
```

**Frontend:**
```bash
cd website
npm install
```

### 4. Run the Application

**Backend:**
```bash
cd backend
python -m uvicorn app.main:app --reload --port 8000
```

**Frontend:**
```bash
cd website
npm start
```

### 5. Testing Setup

For testing purposes, set a user ID in localStorage:

```javascript
// In browser console
localStorage.setItem('user_id', '123');
```

Then refresh the page. You should now see the translation buttons on chapter pages.

## API Endpoints

### Public Endpoints

- `GET /api/v1/translations/{chapter_id}` - Fetch Urdu translation
- `GET /api/v1/translations/check/{chapter_id}` - Check if translation exists
- `GET /api/v1/translations/users/{user_id}/points` - Get user points

### Authenticated Endpoints

- `POST /api/v1/translations` - Submit new translation
  - Requires: `X-User-Id` header

### Admin Endpoints

- `POST /api/v1/translations/admin/bulk` - Bulk upload translations
  - Requires: `X-API-Key` header

## Features

### For Users

1. **View Translations**: Click "اردو میں پڑھیں" button to switch to Urdu
2. **Contribute**: Click "Add Urdu Translation" for chapters without translations
3. **Earn Points**: Get 1 point per chapter translated (max 50 points)
4. **View Progress**: Visit `/profile` to see your translation points

### For Administrators

1. **Bulk Upload**: Use the `/api/v1/translations/admin/bulk` endpoint to seed initial translations

Example bulk upload:
```bash
curl -X POST http://localhost:8000/api/v1/translations/admin/bulk \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key" \
  -d '[
    {
      "chapter_id": "chapter-1-intro",
      "urdu_content": "یہ پہلا باب ہے..."
    }
  ]'
```

## File Structure

### Backend (`backend/`)
```
app/
├── models/
│   └── translation.py              # Pydantic models
├── services/
│   └── translation_service.py      # Business logic
├── routers/
│   └── translation.py              # API endpoints
├── middleware/
│   └── auth.py                     # Authentication
├── utils/
│   ├── config.py                   # Settings (DATABASE_URL added)
│   └── urdu_validator.py           # Urdu validation
└── main.py                         # App initialization (router registered)

migrations/
└── 001_urdu_translations.sql       # Database schema
```

### Frontend (`website/src/`)
```
components/
├── UrduTranslationButton/          # Toggle button
│   ├── index.js
│   └── styles.module.css
├── TranslationSubmitModal/         # Submission form
│   ├── index.js
│   └── styles.module.css
└── UserTranslationPoints/          # Points display
    ├── index.js
    └── styles.module.css

theme/
└── DocItem/
    └── Layout/
        └── index.js                # Swizzled layout (injects button)

types/
└── translation.ts                  # TypeScript definitions

pages/
└── profile.js                      # User profile page

css/
└── custom.css                      # Urdu font + RTL styles added
```

## Database Schema

### `translations` Table
- `id` (SERIAL PRIMARY KEY)
- `chapter_id` (VARCHAR UNIQUE) - e.g., "chapter-1-intro"
- `urdu_content` (TEXT) - Urdu markdown content
- `contributor_id` (INTEGER) - User who submitted
- `created_at`, `updated_at` (TIMESTAMP)

### `user_translation_points` Table
- `id` (SERIAL PRIMARY KEY)
- `user_id` (INTEGER UNIQUE)
- `chapters_translated` (INTEGER)
- `total_points` (INTEGER) - Capped at 50
- `created_at`, `updated_at` (TIMESTAMP)

## Performance Optimizations

1. **In-memory caching**: Translations cached in browser memory
2. **Database indexes**: On `chapter_id` and `contributor_id`
3. **Connection pooling**: asyncpg pool for database connections
4. **LocalStorage**: Language preference persisted locally

## Security

1. **Authentication required**: For translation submission
2. **API key protection**: For admin bulk upload
3. **Urdu validation**: Prevents spam (50% Urdu characters required)
4. **First-wins policy**: Database UNIQUE constraint prevents duplicates
5. **SQL injection protection**: Parameterized queries via asyncpg

## Known Limitations

### Authentication
The current implementation uses a placeholder authentication system with `X-User-Id` header and localStorage. In production, replace with:
- JWT tokens
- OAuth2 / OIDC
- Session-based authentication

### Markdown Rendering
The current Urdu content uses basic HTML rendering. For production:
- Use a proper markdown library (marked, remark, or react-markdown)
- Sanitize HTML to prevent XSS

### Translation Editing
Once submitted, translations cannot be edited (first-wins policy). Future enhancements could include:
- Voting system for quality
- Admin moderation
- Translation suggestions/improvements

## Troubleshooting

### "DATABASE_URL not configured"
- Ensure `.env` file exists in `backend/` directory
- Check that `DATABASE_URL` is set correctly
- Verify PostgreSQL is running

### "Translation not available"
- Check database connection
- Verify migration ran successfully: `SELECT * FROM translations;`
- Try bulk uploading seed data

### Button not showing
- Check localStorage has `user_id` set
- Verify you're on a chapter page (not homepage)
- Check browser console for errors

### Urdu font not loading
- Check network tab for Google Fonts request
- Verify `custom.css` has `@import` for Noto Nastaliq Urdu
- Try clearing browser cache

## Next Steps

1. **Run database migration** (see Setup Instructions)
2. **Seed initial translations** (optional, via bulk upload)
3. **Set up proper authentication** (replace placeholder)
4. **Test on multiple browsers** (Chrome, Firefox, Safari)
5. **Deploy to production** (update CORS_ORIGINS, API_BASE_URL)

## Support

For issues or questions:
- Check logs: `structlog` provides JSON-formatted logs
- API docs: http://localhost:8000/docs
- Database queries: Use `psql` or pgAdmin

---

**Implementation completed**: 2025-12-07
**Total tasks completed**: 45/45 (100%)
**All user stories**: ✅ Fully implemented
