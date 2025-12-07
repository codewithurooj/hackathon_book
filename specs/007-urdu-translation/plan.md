# Implementation Plan: Urdu Translation Feature

**Branch**: `001-urdu-translation` | **Date**: 2025-12-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-urdu-translation/spec.md`

## Summary

Add Urdu translation support to all 37 chapters of the Physical AI textbook, allowing logged-in users to toggle between English and Urdu content via a button at the start of each chapter. Users can contribute translations (first submission wins), earning up to 50 bonus points for their hackathon participation. Technical approach uses PostgreSQL for storage, FastAPI REST endpoints for backend API, and React components integrated via Docusaurus theme swizzling for frontend.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React 19 (frontend)
**Primary Dependencies**: FastAPI, Pydantic v2, PostgreSQL/Neon, Docusaurus 3.9.2, React 19
**Storage**: PostgreSQL (Neon) - two new tables: `translations`, `user_translation_points`
**Testing**: pytest + pytest-asyncio (backend), Jest + React Testing Library (frontend)
**Target Platform**: Web application (desktop + mobile browsers)
**Project Type**: Web (existing backend + frontend structure)
**Performance Goals**: Translation load time < 3 seconds (SC-001), API response < 500ms
**Constraints**: Max 50 bonus points per user, one translation per chapter (first wins), login required
**Scale/Scope**: 37 chapters, expected 100-1000 users, ~50KB average translation size

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Status**: ✅ PASS (No constitution file configured - using best practices)

**Note**: The `.specify/memory/constitution.md` file is a template. Applying general software engineering best practices:

1. **Simplicity**: Solution uses existing infrastructure (FastAPI, PostgreSQL, React) without adding new frameworks
2. **Testability**: Clear separation of concerns - services, routers, components are independently testable
3. **Performance**: Caching strategy for translations, database indexes for fast queries
4. **Security**: Authentication required for all write operations, Urdu text validation prevents spam
5. **Maintainability**: RESTful API design, standard Pydantic models, documented contracts in OpenAPI

**No violations to report**

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── models/
│   │   └── translation.py              # NEW: Pydantic models for translation
│   ├── services/
│   │   └── translation_service.py      # NEW: Translation business logic
│   ├── routers/
│   │   └── translation.py              # NEW: Translation API endpoints
│   ├── utils/
│   │   └── urdu_validator.py           # NEW: Urdu text validation
│   └── main.py                          # MODIFIED: Register translation router
├── migrations/
│   └── 001_urdu_translations.sql       # NEW: Database schema
└── tests/
    └── test_translation.py              # NEW: Translation tests

website/
├── src/
│   ├── components/
│   │   └── UrduTranslationButton/      # NEW: Translation toggle button
│   │       ├── index.js
│   │       └── styles.module.css
│   ├── theme/
│   │   └── DocItem/
│   │       └── Layout/
│   │           └── index.js             # MODIFIED: Inject translation button
│   └── types/
│       └── translation.ts               # NEW: TypeScript types
└── tests/
    └── UrduTranslationButton.test.js    # NEW: Component tests
```

**Structure Decision**: Web application structure with separate backend (FastAPI/Python) and frontend (Docusaurus/React). New files are isolated to translation-specific modules, minimizing impact on existing codebase. Backend follows existing patterns (models → services → routers), frontend uses Docusaurus theme swizzling for non-invasive integration.

## Complexity Tracking

**Status**: N/A - No constitution violations detected

All design decisions follow simplicity principles:
- Uses existing FastAPI patterns (no new frameworks)
- Standard REST API (no GraphQL/WebSocket complexity)
- Direct database access via existing ORM patterns
- React components follow Docusaurus conventions
- First-wins policy avoids complex conflict resolution

**No complexity justification required**

---

## Artifacts Generated

### Phase 0: Research (✅ Complete)
- **research.md**: 10 technical decisions documented with rationale and alternatives
  - Authentication, storage, API design, points calculation, validation, caching, etc.

### Phase 1: Design & Contracts (✅ Complete)
- **data-model.md**: Database schema, Pydantic models, TypeScript types, validation rules
  - 2 new tables: `translations`, `user_translation_points`
  - Relationships, indexes, constraints documented
- **contracts/openapi.yaml**: Full OpenAPI 3.0 specification
  - 5 endpoints: GET/POST translations, check translation, get user points, bulk upload
  - Request/response schemas, error codes, security schemes
- **quickstart.md**: Developer implementation guide
  - Step-by-step setup instructions
  - Code examples for backend services, routers, frontend components
  - Testing and troubleshooting sections

---

## Implementation Strategy

### Priority Order (from spec.md)

**P1: View Urdu Translation** (MVP - Must have)
- Database tables created
- GET /api/v1/translations/{chapter_id} endpoint
- UrduTranslationButton component
- Theme swizzling to inject button
- **Success Metric**: Users can toggle between English/Urdu < 3 seconds

**P2: Earn Bonus Points** (Important)
- Points calculation logic in TranslationService
- GET /api/v1/users/{user_id}/translation-points endpoint
- Points display in user profile
- **Success Metric**: Accurate tracking of 0-50 points range

**P3: Contribute Translation** (Nice to have)
- POST /api/v1/translations endpoint with Urdu validation
- Translation submission UI (modal/form)
- First-wins enforcement via DB constraint
- **Success Metric**: 10+ translations submitted in first 48 hours

### Risk Mitigation

| Risk | Impact | Mitigation |
|------|--------|-----------|
| No existing auth system | HIGH - Feature requires login | Identify auth middleware early; adapt to existing system |
| Database migration fails | HIGH - No feature without tables | Test migration on staging first; provide rollback script |
| Translation fetch > 3 seconds | MEDIUM - Violates SC-001 | Implement caching; add DB indexes; test with realistic data |
| Urdu font not rendering | MEDIUM - Poor UX | Include Noto Nastaliq Urdu font; test on multiple browsers |
| Race condition on submit | LOW - First-wins policy | DB UNIQUE constraint handles atomically |

---

## Testing Strategy

### Backend Tests (`backend/tests/test_translation.py`)
1. **Unit Tests**:
   - Urdu validation logic (50% character threshold)
   - Points calculation (capped at 50)
   - Service methods (get, submit, award points)

2. **Integration Tests**:
   - API endpoints with test database
   - First-wins enforcement (duplicate submission fails)
   - Transaction rollback on errors

3. **Performance Tests**:
   - Translation fetch < 500ms
   - Concurrent submissions handled correctly

### Frontend Tests (`website/tests/UrduTranslationButton.test.js`)
1. **Component Tests**:
   - Button visibility based on auth status
   - Button disabled when no translation exists
   - Language toggle updates UI correctly

2. **Integration Tests**:
   - API calls return expected data
   - localStorage persists language preference
   - Error handling (network failures)

---

## Deployment Checklist

**Backend**:
- [ ] Run database migration (`psql -f migrations/001_urdu_translations.sql`)
- [ ] Add translation router to main.py
- [ ] Deploy backend to production
- [ ] Verify API endpoints with curl/Postman
- [ ] Seed initial translations via bulk upload API

**Frontend**:
- [ ] Swizzle DocItem/Layout theme component
- [ ] Build frontend (`npm run build`)
- [ ] Deploy to GitHub Pages
- [ ] Test on multiple browsers (Chrome, Firefox, Safari)
- [ ] Verify Urdu font rendering

**Post-Deployment**:
- [ ] Monitor error logs for API failures
- [ ] Track translation fetch times (should be < 3s)
- [ ] Monitor user engagement (how many clicks on button?)
- [ ] Collect feedback on translation quality

---

## Next Steps

1. **Run `/sp.tasks`** to generate detailed implementation tasks with test cases
2. **Implement P1 (View Translation)** as MVP
3. **Test with real Urdu content** before moving to P2/P3
4. **Collect user feedback** after MVP deployment
5. **Iterate** on P2 (points) and P3 (contribution) based on usage

**Status**: ✅ Planning complete. Ready for task generation (`/sp.tasks`)
