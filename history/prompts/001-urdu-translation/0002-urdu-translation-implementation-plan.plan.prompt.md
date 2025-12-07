---
id: 0002
title: Urdu Translation Implementation Plan
stage: plan
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-urdu-translation
branch: 001-urdu-translation
user: pc1
command: /sp.plan
labels: ["implementation-plan", "architecture", "database-design", "api-contracts", "urdu-translation"]
links:
  spec: specs/001-urdu-translation/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-urdu-translation/plan.md
 - specs/001-urdu-translation/research.md
 - specs/001-urdu-translation/data-model.md
 - specs/001-urdu-translation/contracts/openapi.yaml
 - specs/001-urdu-translation/quickstart.md
tests:
 - Database schema validation
 - API contract completeness check
---

## Prompt

/sp.plan

## Response snapshot

Created comprehensive implementation plan for Urdu translation feature with complete Phase 0 (Research) and Phase 1 (Design & Contracts) deliverables:

**Phase 0: Research (research.md)**
- 10 technical decisions documented:
  1. Authentication: Use existing Docusaurus/React auth context
  2. Storage: PostgreSQL with 2 tables (translations, user_translation_points)
  3. Frontend: React state + localStorage for language preference
  4. API Design: RESTful with 5 endpoints
  5. Points Calculation: Linear 1 point/chapter, cap at 50
  6. Urdu Validation: Unicode range U+0600-U+06FF, 50% threshold
  7. First-Wins Policy: Database UNIQUE constraint on chapter_id
  8. UI Injection: Docusaurus theme swizzling
  9. Performance: Client + server-side caching
  10. Initial Seeding: Admin bulk upload endpoint

**Phase 1: Design & Contracts (data-model.md, contracts/, quickstart.md)**
- Database schema with full SQL DDL:
  - translations table (id, chapter_id, urdu_content, contributor_user_id, timestamps)
  - user_translation_points table (user_id, points, chapters_translated)
  - Indexes, foreign keys, CHECK constraints
  - Triggers for updated_at columns
- Pydantic models for request/response validation
- TypeScript types for frontend
- OpenAPI 3.0 specification (contracts/openapi.yaml):
  - 5 endpoints fully documented with schemas
  - Security schemes (JWT bearer auth)
  - Error responses and status codes
- Developer quickstart guide:
  - Step-by-step implementation instructions
  - Code examples for services, routers, components
  - Testing strategies and troubleshooting

**Implementation Strategy**:
- Priority-based approach (P1: View, P2: Points, P3: Contribute)
- Risk mitigation table with 5 identified risks
- Testing strategy (unit, integration, performance)
- Deployment checklist (backend, frontend, post-deployment)

**Technical Context**:
- Python 3.11+, FastAPI, PostgreSQL, React 19, Docusaurus 3.9.2
- Performance: < 3s translation load, < 500ms API response
- Scale: 37 chapters, 100-1000 users, ~50KB per translation

## Outcome

- ✅ Impact: Complete architectural plan ready for implementation with all design artifacts
- 🧪 Tests: Testing strategies defined for backend (pytest) and frontend (Jest)
- 📁 Files: 5 files created (plan.md, research.md, data-model.md, openapi.yaml, quickstart.md)
- 🔁 Next prompts: `/sp.tasks` to generate detailed implementation tasks with test cases
- 🧠 Reflection: Comprehensive planning phase completed all research and design work. Database schema handles first-wins policy atomically. API design follows REST best practices. Ready for task breakdown and TDD implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASS (no violations), All design artifacts complete
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Proceed to /sp.tasks for task generation with test cases
