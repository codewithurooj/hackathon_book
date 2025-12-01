# Implementation Plan: RAG Chatbot

**Branch**: `006-rag-chatbot` | **Date**: 2025-11-29 | **Spec**: [./spec.md](./spec.md)
**Input**: Feature specification from `specs/006-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of an intelligent, in-context chatbot for the Physical AI & Humanoid Robotics textbook. The chatbot will use a RAG (Retrieval-Augmented Generation) architecture to answer student questions based on the book's content. The backend will be built with FastAPI, using Qdrant for vector storage, Neon Postgres for chat history, and OpenAI's `text-embedding-3-small` and Agents SDK for the core logic. The frontend will be a React component using ChatKit, integrated into the existing Docusaurus website.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: FastAPI, OpenAI SDK, Qdrant Client, psycopg2, ChatKit SDK (React), Docusaurus
**Storage**: Qdrant Cloud (Vector DB), Neon Serverless Postgres (Relational DB)
**Testing**: Pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Docusaurus Website (frontend), Linux Server (backend)
**Project Type**: Web Application (frontend/backend)
**Performance Goals**: P95 response time for streamed response to begin in under 2 seconds.
**Constraints**: All API keys must be managed via environment variables. The agent must only use the provided context from the book.
**Scale/Scope**: To support concurrent users of the textbook website.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The project constitution at `.specify/memory/constitution.md` is currently a template. This plan will adhere to the principles outlined in the `spec.md`, including security NFRs (Non-Functional Requirements) like environment variable management for secrets and input sanitization.

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/
│   └── openapi.yaml     # OpenAPI schema for the backend
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure
backend/
├── app/
│   ├── api/             # FastAPI endpoints (e.g., chat.py)
│   ├── services/        # Business logic (e.g., openai_agent.py, qdrant_service.py)
│   ├── models/          # Pydantic models for requests/responses
│   └── main.py          # FastAPI app initialization
├── scripts/
│   └── ingest.py        # Data ingestion script
└── tests/
    ├── integration/
    └── unit/

website/ # Existing Docusaurus project
├── src/
│   ├── theme/
│   │   └── Root.js      # To wrap the site with chatbot context
│   └── components/
│       └── Chatbot/     # React component for the ChatKit UI
│           ├── index.js
│           └── index.css
└── static/
    └── scripts/
        └── text-selection.js # Script to handle "Ask about this" functionality
```

**Structure Decision**: A frontend/backend monorepo structure is chosen. The `backend/` directory will contain the new FastAPI application. The chatbot frontend will be integrated directly into the existing `website/` Docusaurus project to ensure seamless UI and access to the document content for text selection.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
|           |            |                                     |