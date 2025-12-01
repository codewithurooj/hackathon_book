# Implementation Tasks: RAG Chatbot

**Branch**: `006-rag-chatbot` | **Date**: 2025-11-29 | **Spec**: [./spec.md](./spec.md)
**Plan**: [./plan.md](./plan.md) | **Research**: [./research.md](./research.md)
**Data Model**: [./data-model.md](./data-model.md) | **API Contracts**: [./contracts/openapi.yaml](./contracts/openapi.yaml)

This document outlines the actionable, dependency-ordered tasks required to implement the RAG Chatbot feature. Tasks are organized into phases, with User Story tasks grouped for independent development and testing.

## Summary

Total Tasks: 40
Tasks per User Story: US1: 7, US2: 3, US3: 4, US4: 3, US5: 2, US6: 2
Parallel Opportunities: 23
Suggested MVP Scope: User Story 1 (Answering questions based on book content)

## Phase 1: Setup

- [X] T001 Create backend directory structure (`backend/app`, `backend/scripts`, `backend/tests`)
- [X] T002 Initialize Python virtual environment in `backend/`
- [X] T003 Create `backend/requirements.in` with initial dependencies (fastapi, openai, qdrant-client, psycopg2, pip-tools)
- [X] T004 Install `pip-tools` and compile `backend/requirements.txt`
- [X] T005 Configure `website/` (Docusaurus) for React component integration and external script loading (e.g., in `website/docusaurus.config.ts`)

## Phase 2: Foundational

- [X] T006 [P] Implement API key authentication middleware (`backend/app/api/auth.py`)
- [X] T007 [P] Implement environment variable loading utility (`backend/app/config.py`)
- [X] T008 [P] Implement Qdrant client initialization (`backend/app/services/qdrant_service.py`)
- [X] T009 [P] Implement OpenAI client initialization (`backend/app/services/openai_service.py`)
- [X] T010 [P] Implement database connection and session management for Neon Postgres (`backend/app/database.py`)
- [X] T011 [P] Create SQL script for `chat_history` table (`backend/scripts/init_db.sql`) based on `data-model.md`

## Phase 3: User Story 1 (FR-1: The chatbot must answer questions based on the book's content)

**Goal**: Enable the chatbot to answer general questions using the textbook content.
**Independent Test Criteria**: A user can ask a general question via the chatbot UI and receive a relevant answer derived from the book's content.

- [X] T012 [P] [US1] Create Pydantic models for `/chat` request/response (`backend/app/models/chat.py`)
- [X] T013 [P] [US1] Implement embedding generation for questions using OpenAI (`backend/app/services/openai_service.py`)
- [X] T014 [P] [US1] Implement Qdrant retrieval logic for top-k relevant chunks (`backend/app/services/qdrant_service.py`)
- [X] T015 [P] [US1] Implement OpenAI Agent logic for generating answers from context (`backend/app/services/openai_agent.py`)
- [X] T016 [US1] Implement `POST /chat` endpoint (`backend/app/api/chat.py`) using services and streaming response
- [X] T017 [P] [US1] Create React component for Chatbot UI (`website/src/components/Chatbot/index.js`, `website/src/components/Chatbot/index.css`)
- [X] T018 [US1] Integrate Chatbot component into Docusaurus layout (`website/src/theme/Root.js`)

## Phase 4: User Story 2 (FR-2: The chatbot must answer questions based on user-selected text)

**Goal**: Allow users to ask questions specifically about selected text in the textbook.
**Independent Test Criteria**: A user can select text on a book page, click "Ask about this", and receive an answer based *only* on the selected text.

- [X] T019 [P] [US2] Implement `POST /chat/selected` endpoint (`backend/app/api/chat.py`)
- [X] T020 [P] [US2] Implement frontend JavaScript for text selection detection (`website/static/scripts/text-selection.js`)
- [X] T021 [US2] Implement "Ask about this" UI element in Docusaurus frontend (`website/src/components/Chatbot/TextSelectionButton.js`)

## Phase 5: User Story 3 (FR-3: The chatbot must provide citations for its answers)

**Goal**: Display source citations alongside chatbot answers, linking to relevant book sections.
**Independent Test Criteria**: Chatbot answers include clickable citations that accurately link to the source material within the textbook.

- [X] T022 [P] [US3] Ensure Qdrant payload stores required citation metadata (chapter, section, page_url, text_chunk) during ingestion process (`backend/scripts/ingest.py`)
- [X] T023 [P] [US3] Modify OpenAI Agent to extract and format citations into structured JSON (`backend/app/services/openai_agent.py`)
- [X] T024 [P] [US3] Update `chat_history` table's `sources` JSONB column based on `research.md` schema (`backend/scripts/init_db.sql`)
- [X] T025 [US3] Implement rendering of citations in the Chatbot UI (`website/src/components/Chatbot/index.js`)

## Phase 6: User Story 4 (FR-4: The chat history must be persisted and viewable within a session)

**Goal**: Store and retrieve user conversation history.
**Independent Test Criteria**: A user can close and reopen the chatbot within the same session and see their previous conversation history.

- [X] T026 [US4] Implement saving question and answer to `chat_history` table (`backend/app/services/chat_history_service.py`)
- [X] T027 [US4] Implement retrieving chat history for a given `session_id` (`backend/app/services/chat_history_service.py`)
- [X] T028 [US4] Integrate chat history loading/display into Chatbot UI (`website/src/components/Chatbot/index.js`)

## Phase 7: User Story 5 (FR-5: The chatbot UI must be embeddable in a Docusaurus site)

**Goal**: Seamless integration of the chatbot UI into the Docusaurus platform.
**Independent Test Criteria**: The chatbot UI is responsive, visually consistent with the Docusaurus theme, and does not interfere with existing site functionality.

- [X] T029 [US5] Finalize styling of Chatbot UI to match Docusaurus theme (`website/src/components/Chatbot/index.css`)
- [X] T030 [US5] Ensure Chatbot component lifecycle does not cause conflicts within Docusaurus React tree (`website/src/theme/Root.js`)

## Phase 8: User Story 6 (FR-6: The system must handle concurrent user requests)

**Goal**: Ensure the backend can manage multiple simultaneous user interactions efficiently.
**Independent Test Criteria**: Multiple simulated users can interact with the chatbot concurrently without significant performance degradation or errors.

- [X] T031 [P] [US6] Implement rate limiting for API endpoints (`backend/app/api/middleware/rate_limiter.py`)
- [X] T032 [US6] Conduct basic load testing on `/chat` and `/chat/selected` endpoints to verify performance (`backend/tests/integration/load_test.py`)

## Phase 9: Polish & Cross-Cutting Concerns

**Goal**: Enhance robustness, user experience, and overall system quality.
**Independent Test Criteria**: All NFRs are met, and the system provides a polished, reliable experience.

- [X] T033 [P] Implement detailed frontend error handling with toast notifications (`website/src/components/Chatbot/index.js`, `website/src/components/Chatbot/ErrorToast.js`)
- [X] T034 [P] Implement robust backend error handling and logging (`backend/app/main.py`, `backend/app/utils/logger.py`)
- [X] T035 [P] Implement document ingestion script (`backend/scripts/ingest.py`) to process markdown files, generate embeddings, and populate Qdrant.
- [X] T036 [P] Conduct security review for input sanitization and API key management (`backend/app/api/`)
- [X] T037 [P] Develop comprehensive unit and integration tests for backend services and API endpoints (`backend/tests/`)
- [X] T038 [P] Develop unit and integration tests for frontend components (`website/src/components/Chatbot/`)
- [X] T039 Deploy and configure FastAPI backend (CI/CD pipeline definition)
- [X] T040 Deploy and configure Docusaurus frontend (CI/CD pipeline definition)

## Dependencies

- Phase 1 (Setup) -> Phase 2 (Foundational)
- Phase 2 (Foundational) -> All User Story Phases (Phase 3-8)
- Phase 3 (US1) is independent of other user stories for core functionality.
- Phase 4 (US2) builds upon US1's core agent logic.
- Phase 5 (US3) depends on US1's answer generation and US4's history persistence.
- Phase 6 (US4) depends on database setup from Phase 2.
- Phase 7 (US5) is largely concurrent with other frontend integration tasks.
- Phase 8 (US6) depends on core API endpoints.
- Phase 9 (Polish) can run concurrently with many other tasks, but some sub-tasks depend on the completion of core features.

## Parallel Execution Examples

- **Backend Development (Concurrent)**:
    - Implement `T006` (Auth), `T007` (Config), `T008` (Qdrant Client), `T009` (OpenAI Client), `T010` (DB Init), `T011` (SQL Script)
- **Frontend Development (Concurrent)**:
    - Implement `T017` (Chatbot UI), `T018` (Docusaurus Integration)

## Implementation Strategy

The strategy will focus on an MVP-first approach, with incremental delivery. User Story 1 will be prioritized as the Minimum Viable Product, enabling core chatbot functionality. Subsequent user stories will be integrated incrementally, allowing for continuous feedback and validation. Cross-cutting concerns and polish will be addressed throughout development and in a dedicated final phase.
