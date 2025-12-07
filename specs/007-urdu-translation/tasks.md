# Tasks: Urdu Translation Feature

**Input**: Design documents from `/specs/007-urdu-translation/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Tests are NOT explicitly requested in the feature specification, so test tasks are excluded per SDD guidelines.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a web application with:
- **Backend**: `backend/` (FastAPI/Python)
- **Frontend**: `website/` (Docusaurus/React)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and database schema

- [X] T001 Create database migration file `backend/migrations/001_urdu_translations.sql` with translations and user_translation_points tables
- [X] T002 Run database migration to create tables in PostgreSQL/Neon
- [X] T003 [P] Create Urdu text validator utility in `backend/app/utils/urdu_validator.py`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create Translation Pydantic models in `backend/app/models/translation.py` (TranslationSubmit, TranslationResponse, TranslationSubmitResult, UserPointsResponse, BulkTranslationUpload)
- [X] T005 [P] Create TranslationService base class in `backend/app/services/translation_service.py` with database connection setup
- [X] T006 [P] Create TypeScript types in `website/src/types/translation.ts` (Translation, TranslationSubmitRequest, TranslationSubmitResult, UserTranslationPoints, Language, ChapterLanguageState)
- [X] T007 Verify existing authentication middleware in backend is compatible with translation endpoints

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - View Urdu Translation (Priority: P1) 🎯 MVP

**Goal**: Logged-in users can click a button at the start of each chapter to toggle between English and Urdu content

**Independent Test**: Log in, navigate to any chapter, click "Translate to Urdu" button, verify content displays in Urdu within 3 seconds, then click button again to toggle back to English

**Acceptance Criteria**:
- Chapter content displays in Urdu when button is clicked
- Content switches back to English when button is clicked again
- Translation loads within 3 seconds
- Original formatting, code blocks, and images are preserved

### Implementation for User Story 1

**Backend (API Endpoints)**:

- [X] T008 [P] [US1] Implement `get_translation(chapter_id)` method in `backend/app/services/translation_service.py`
- [X] T009 [P] [US1] Implement `check_translation_exists(chapter_id)` method in `backend/app/services/translation_service.py`
- [X] T010 [US1] Create translation router in `backend/app/routers/translation.py` with GET /api/v1/translations/{chapter_id} endpoint (depends on T008)
- [X] T011 [US1] Add GET /api/v1/translations/check/{chapter_id} endpoint to `backend/app/routers/translation.py` (depends on T009)
- [X] T012 [US1] Register translation router in `backend/app/main.py`

**Frontend (UI Components)**:

- [X] T013 [P] [US1] Create UrduTranslationButton component in `website/src/components/UrduTranslationButton/index.js` with toggle logic, localStorage persistence, and loading states
- [X] T014 [P] [US1] Create styles for UrduTranslationButton in `website/src/components/UrduTranslationButton/styles.module.css`
- [X] T015 [US1] Swizzle Docusaurus DocItem/Layout theme component: `npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap` in website directory
- [X] T016 [US1] Modify `website/src/theme/DocItem/Layout/index.js` to inject UrduTranslationButton at the start of each chapter and handle language toggle state
- [X] T017 [US1] Add Urdu font (Noto Nastaliq Urdu) to `website/src/css/custom.css` and configure RTL text direction for Urdu content
- [X] T018 [US1] Implement client-side caching in UrduTranslationButton to cache translations in Map for performance

**Checkpoint**: At this point, User Story 1 should be fully functional - users can toggle between English and Urdu translations

---

## Phase 4: User Story 2 - Earn Bonus Points (Priority: P2)

**Goal**: Users who contribute Urdu translations earn bonus points (up to 50 total) towards their hackathon score

**Independent Test**: Contribute a translation, check that user's points increase by 1, verify points are capped at 50 after 50+ contributions, view points in user profile/dashboard

**Acceptance Criteria**:
- Users receive proportional bonus points for each translation (1 point per chapter)
- Maximum 50 bonus points total
- Users can view their current bonus points

### Implementation for User Story 2

**Backend (Points Calculation)**:

- [X] T019 [P] [US2] Implement `_award_points(user_id)` private method in `backend/app/services/translation_service.py` with logic to increment chapters_translated, calculate points (min of chapters_translated and 50), and update user_translation_points table
- [X] T020 [P] [US2] Implement `get_user_points(user_id)` method in `backend/app/services/translation_service.py` to retrieve points and breakdown
- [X] T021 [US2] Add GET /api/v1/users/{user_id}/translation-points endpoint to `backend/app/routers/translation.py` (depends on T020)

**Frontend (Points Display)**:

- [X] T022 [P] [US2] Create UserTranslationPoints component in `website/src/components/UserTranslationPoints/index.js` to display total points and breakdown
- [X] T023 [P] [US2] Create styles for UserTranslationPoints in `website/src/components/UserTranslationPoints/styles.module.css`
- [X] T024 [US2] Integrate UserTranslationPoints component into user profile or dashboard page (identify existing profile location first)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - users can view translations and see their earned points

---

## Phase 5: User Story 3 - Contribute Translation (Priority: P3)

**Goal**: Logged-in users can contribute Urdu translations for chapters that don't have one yet through a submission interface

**Independent Test**: Navigate to a chapter without Urdu translation, click "Add Urdu Translation", enter Urdu text in the interface, submit, verify it becomes immediately available to other users, confirm first-submission-wins policy

**Acceptance Criteria**:
- Translation input interface is accessible for chapters without translations
- Submitted translations are saved and immediately available to other users
- First submission wins - subsequent submissions for the same chapter are rejected
- "Add Urdu Translation" button is hidden/disabled for chapters that already have translations
- System validates that submitted text contains actual Urdu characters

### Implementation for User Story 3

**Backend (Submission API)**:

- [X] T025 [P] [US3] Implement Urdu validation logic in `backend/app/utils/urdu_validator.py` to check for 50% Urdu Unicode characters (U+0600-U+06FF)
- [X] T026 [US3] Implement `submit_translation(chapter_id, urdu_content, user_id)` method in `backend/app/services/translation_service.py` with database insert, unique constraint handling, and points award integration (depends on T019, T025)
- [X] T027 [US3] Add POST /api/v1/translations endpoint to `backend/app/routers/translation.py` with authentication, validation, and first-wins enforcement (depends on T026)
- [X] T028 [P] [US3] Implement `bulk_upload_translations()` admin method in `backend/app/services/translation_service.py` for seeding initial translations
- [X] T029 [US3] Add POST /api/v1/admin/translations/bulk endpoint to `backend/app/routers/translation.py` with admin-only authentication (depends on T028)

**Frontend (Contribution UI)**:

- [X] T030 [P] [US3] Create TranslationSubmitModal component in `website/src/components/TranslationSubmitModal/index.js` with textarea for Urdu input, validation, and submit handling
- [X] T031 [P] [US3] Create styles for TranslationSubmitModal in `website/src/components/TranslationSubmitModal/styles.module.css`
- [X] T032 [US3] Add "Add Urdu Translation" button to UrduTranslationButton component in `website/src/components/UrduTranslationButton/index.js` (shown only when no translation exists and user is authenticated)
- [X] T033 [US3] Integrate TranslationSubmitModal with UrduTranslationButton to show modal on click
- [X] T034 [US3] Add error handling for duplicate submission (409 Conflict) in TranslationSubmitModal with user-friendly message
- [X] T035 [US3] Update UrduTranslationButton to hide "Add Translation" option after chapter already has translation

**Checkpoint**: All user stories should now be independently functional - view translations, earn points, and contribute new translations

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T036 [P] Add API error handling middleware in `backend/app/routers/translation.py` for consistent error responses (500, 404, 401, 409)
- [X] T037 [P] Add loading skeletons to UrduTranslationButton and TranslationSubmitModal for better UX
- [X] T038 [P] Configure CORS settings in `backend/app/main.py` to allow frontend requests from website domain
- [X] T039 Add client-side validation in TranslationSubmitModal to check for Urdu characters before API submission
- [X] T040 [P] Add database indexes verification: check idx_translations_chapter and idx_translations_contributor exist
- [X] T041 Add logging for translation operations in `backend/app/services/translation_service.py` (submit, fetch, points award)
- [X] T042 [P] Test translation feature on multiple browsers (Chrome, Firefox, Safari) for Urdu font rendering
- [X] T043 [P] Update `backend/.env.example` with any new environment variables needed for translation feature
- [X] T044 Create seed data script or use bulk upload API to populate initial translations for testing
- [X] T045 Validate quickstart.md instructions by following them step-by-step

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3, 4, 5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 but independently testable (points work without translation viewing)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Integrates with US1 and US2 but independently testable (submission works standalone)

### Within Each User Story

**User Story 1**:
- Backend endpoints (T008-T012) can complete before frontend
- Frontend UI components (T013-T018) can start once backend is ready
- Within backend: T008 and T009 can run in parallel
- Within frontend: T013 and T014 can run in parallel

**User Story 2**:
- Backend methods (T019-T021) must complete before frontend
- T019 and T020 can run in parallel
- Frontend components (T022-T024) can start once backend is ready
- T022 and T023 can run in parallel

**User Story 3**:
- Backend (T025-T029): T025 can run in parallel with T028, then T026 depends on both T019 and T025
- Frontend (T030-T035): T030 and T031 can run in parallel, then sequential integration (T032-T035)

### Parallel Opportunities

**Setup Phase**:
- T003 can run in parallel with T001-T002

**Foundational Phase**:
- T004, T005, T006, T007 can all run in parallel

**User Story 1**:
- T008 and T009 can run in parallel
- T013 and T014 can run in parallel
- Backend (T008-T012) and Frontend (T013-T018) can run in parallel if separate developers

**User Story 2**:
- T019 and T020 can run in parallel
- T022 and T023 can run in parallel

**User Story 3**:
- T025 and T028 can run in parallel
- T030 and T031 can run in parallel

**Polish Phase**:
- T036, T037, T038, T039, T040, T042, T043 can all run in parallel

---

## Parallel Example: User Story 1 (Backend + Frontend Teams)

```bash
# Backend Team - Launch in parallel:
Task T008: "Implement get_translation method in backend/app/services/translation_service.py"
Task T009: "Implement check_translation_exists method in backend/app/services/translation_service.py"

# Frontend Team - Launch in parallel:
Task T013: "Create UrduTranslationButton component in website/src/components/UrduTranslationButton/index.js"
Task T014: "Create UrduTranslationButton styles in website/src/components/UrduTranslationButton/styles.module.css"

# After both complete, integrate:
Task T010-T012: Backend router setup
Task T015-T018: Frontend theme swizzling and integration
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T007) - CRITICAL
3. Complete Phase 3: User Story 1 (T008-T018)
4. **STOP and VALIDATE**:
   - Test login → navigate to chapter → click Urdu button → verify translation loads < 3 seconds
   - Test toggle back to English
   - Test localStorage persistence across page refreshes
5. Deploy/demo if ready

**Suggested MVP scope**: Phase 1 + Phase 2 + Phase 3 = 18 tasks

### Incremental Delivery

1. **Foundation** (Phases 1-2): Setup database and models → 7 tasks
2. **MVP Release** (Phase 3): User Story 1 → Deploy/Demo (users can view translations) → 11 tasks
3. **V2 Release** (Phase 4): User Story 2 → Deploy/Demo (users see points earned) → 6 tasks
4. **V3 Release** (Phase 5): User Story 3 → Deploy/Demo (users can contribute translations) → 11 tasks
5. **Production Release** (Phase 6): Polish and optimization → 10 tasks

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1. **Team completes Setup + Foundational together** (Phases 1-2)
2. **Once Foundational is done**:
   - Developer A: User Story 1 Backend (T008-T012)
   - Developer B: User Story 1 Frontend (T013-T018)
   - After US1 complete, Dev A → US2, Dev B → US3
3. Stories complete and integrate independently

---

## Summary

**Total Tasks**: 45 tasks

**Task Count by User Story**:
- Setup: 3 tasks
- Foundational: 4 tasks
- User Story 1 (P1 - View Translation): 11 tasks
- User Story 2 (P2 - Earn Points): 6 tasks
- User Story 3 (P3 - Contribute Translation): 11 tasks
- Polish: 10 tasks

**Parallel Opportunities**: 21 tasks marked [P] can run in parallel with other tasks in same phase

**Independent Test Criteria**:
- ✅ US1: Log in → click Urdu button → content shows in Urdu < 3s → toggle back works
- ✅ US2: Submit translation → check points increased → verify cap at 50 → view in profile
- ✅ US3: Click "Add Translation" → enter Urdu text → submit → verify available to others → test first-wins

**Suggested MVP Scope**: Phases 1-3 (Setup + Foundational + User Story 1) = 18 tasks
- Delivers core value: logged-in users can view Urdu translations
- All other features build on this foundation

**Format Validation**: ✅ All tasks follow checklist format with checkbox, task ID, [P] markers, [Story] labels, and file paths

---

## Notes

- Tasks are organized by user story for independent implementation
- Each user story can be deployed and tested independently
- [P] markers indicate parallelizable tasks (different files, no blocking dependencies)
- [Story] labels (US1, US2, US3) enable traceability to spec.md user stories
- Database migration (T002) is critical path - must complete before any API work
- Foundational phase (T004-T007) blocks all user story work
- Tests are excluded per spec (no TDD requirement stated)
- Commit after each task or logical group for incremental progress
