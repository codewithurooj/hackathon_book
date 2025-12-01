# Research for RAG Chatbot

**Branch**: `006-rag-chatbot` | **Date**: 2025-11-29 | **Spec**: [./spec.md](./spec.md)

This document resolves open questions from the initial `plan.md`.

## 1. Dependency Versioning

**Topic**: The `spec.md` lists dependencies but does not pin versions.
**Research**: For Python projects, it is standard practice to pin dependencies for reproducible builds. Tools like `pip-tools` (which provides `pip-compile`) are excellent for managing this. You define abstract dependencies in a `requirements.in` file and `pip-compile` generates a fully-pinned `requirements.txt`.
**Decision**:
- The Python backend will use `pip-tools`.
- A `requirements.in` file will be created with the abstract dependencies (e.g., `fastapi`, `openai`, `qdrant-client`).
- The generated `requirements.txt` will be committed to the repository.
- For the frontend, `package.json` will use `~` or `^` for versions, and the `package-lock.json` will ensure reproducible builds.

## 2. Frontend Error Handling

**Topic**: The `spec.md` mentions high-level error handling but lacks detail for the UI.
**Research**: A good user experience requires clear, non-disruptive feedback. For web applications, "toast" notifications are a standard and effective way to display transient messages for events like API errors, network issues, or successful operations.
**Decision**:
- A toast notification library (e.g., `react-hot-toast`) will be added to the Docusaurus site.
- A global error handler in the React application will catch errors from the ChatKit service.
- Specific, user-friendly messages will be displayed for:
    - "I can't answer that question with the provided context."
    - "The chatbot service is temporarily unavailable. Please try again later."
    - "A network error occurred."

## 3. Backend Authentication

**Topic**: The `spec.md` mentions rate limiting and security but not a specific authentication scheme for the API. A public-facing chatbot backend should be protected.
**Research**: While full user-based authentication might be overkill for this stage, a simple API key check is a standard and effective way to prevent unauthorized access and abuse.
**Decision**:
- The FastAPI backend will be secured with an API key.
- A dependency `fastapi.Security` will be used to require an `X-API-Key` header on all endpoints.
- The API key will be configured via an environment variable (`BACKEND_API_KEY`) on the server.
- The Docusaurus frontend will be configured with this key (also via an environment variable, e.g., `REACT_APP_CHATBOT_API_KEY`) to include it in requests to the backend.

## 4. `sources` JSONB Schema Definition

**Topic**: The `chat_history` table in Neon Postgres has a `sources` column of type `JSONB`, but the exact schema is not defined.
**Research**: To provide accurate, clickable citations in the chat UI, the frontend needs a structured format. The structure should include the source document's path (which can be resolved to a URL) and the specific text chunk that was cited.
**Decision**:
- The `sources` field will be a JSON array of objects.
- Each object in the array will have the following structure:
  ```json
  {
    "text_chunk": "The specific text snippet from the book that was used as context.",
    "page_url": "/docs/chapter-name/section-name",
    "chapter": "Chapter Name",
    "section": "Section Name"
  }
  ```
- The backend agent will be responsible for populating this data from the metadata stored in the Qdrant vector payload.
