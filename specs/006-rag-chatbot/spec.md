# Feature Specification: RAG Chatbot for Physical AI Textbook

**Feature Branch**: `006-rag-chatbot`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "Integrate RAG chatbot in the book using FastAPI, OpenAI Agents SDK, Neon Postgres, and Qdrant"

# RAG Chatbot Specification

## 1. Title

**Physical AI Textbook RAG Chatbot**

## 2. Goal

To provide an intelligent, in-context chatbot for the Physical AI & Humanoid Robotics textbook. The chatbot will answer student questions using only the content of the published book, including the ability to answer questions about user-selected text. This system aims to enhance the learning experience by providing immediate, accurate, and citable answers.

## 3. Data Sources

-   **Content**: All markdown (`.md`) files within the `docs/` directory of the Docusaurus project.
-   **Format**: Plain text extracted from markdown, preserving chapter and section hierarchy for metadata.
-   **Location**: The local filesystem where the Docusaurus project is hosted. The ingestion process will read from this directory.

## 4. Embedding Model & Vector Storage

-   **Embedding Model**: `text-embedding-3-small` from OpenAI will be used to generate embeddings for both the book content and user questions.
-   **Vector Storage**: **Qdrant Cloud Free Tier** will be used to store and index the text embeddings.
    -   **Collection**: A single collection named `physical-ai-book` will be created.
    -   **Payload**: Each vector will include metadata: `chapter`, `section`, `page_url`, and the original `text_chunk`.

## 5. Backend Architecture (FastAPI)

The backend will be a FastAPI application with the following endpoints:

-   `POST /chat`:
    -   **Request**: `{ "question": "...", "session_id": "..." }`
    -   **Processing**:
        1.  Generate an embedding for the user's question.
        2.  Query Qdrant to retrieve the top-k relevant text chunks.
        3.  Invoke the OpenAI Agent with the retrieved context and the question.
        4.  Stream the response back to the client.
        5.  Save the question and final answer to Neon Postgres.
-   `POST /chat/selected`:
    -   **Request**: `{ "question": "...", "selected_text": "...", "session_id": "..." }`
    -   **Processing**:
        1.  Bypass Qdrant retrieval.
        2.  Invoke the OpenAI Agent using the `selected_text` as the primary context.
        3.  Stream the response and save the interaction history.

## 6. Agent Logic (OpenAI Agents SDK)

-   **Agent Type**: A retrieval-augmented agent will be configured.
-   **Retrieval Pipeline**:
    1.  For general questions, the agent's retrieval tool will query the Qdrant collection.
    2.  The top 5 most relevant chunks will be fetched.
-   **Generation Pipeline**:
    1.  A system prompt will instruct the agent to act as an expert on the Physical AI textbook and to **only use the provided context** to answer questions.
    2.  The user's question and the retrieved text chunks (or selected text) will be formatted into the final prompt.
-   **Selected-Text Handling**: When `selected_text` is provided, the retrieval step is skipped, and the agent focuses exclusively on that text.

## 7. Frontend Integration (ChatKit & Docusaurus)

-   **UI Library**: **ChatKit SDK** will be used to build the chatbot interface.
-   **Embedding**: The ChatKit component will be wrapped in a React component and integrated into the Docusaurus layout, likely as a floating action button that opens the chat window.
-   **UI Behavior**:
    -   The chat interface will display the conversation history.
    -   A text input field will be used for questions.
    -   A "pop-up" or similar mechanism will appear when text is selected in the book, allowing the user to "Ask about this."
    -   Responses will be rendered with citations linking back to the source sections.

## 8. Database Design (Neon Serverless Postgres)

A single table, `chat_history`, will be used:

-   `id`: SERIAL PRIMARY KEY
-   `session_id`: VARCHAR(255) NOT NULL
-   `question`: TEXT NOT NULL
-   `answer`: TEXT NOT NULL
-   `sources`: JSONB (storing a list of source sections)
-   `created_at`: TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP

## 9. Functional Requirements

-   **FR-1**: The chatbot must answer questions based on the book's content.
-   **FR-2**: The chatbot must answer questions based on user-selected text.
-   **FR-3**: The chatbot must provide citations for its answers.
-   **FR-4**: The chat history must be persisted and viewable within a session.
-   **FR-5**: The chatbot UI must be embeddable in a Docusaurus site.
-   **FR-6**: The system must handle concurrent user requests.

## 10. Non-Functional Requirements

-   **Security**:
    -   All API keys (OpenAI, Qdrant, Neon) must be stored as environment variables.
    -   API endpoints should have rate limiting to prevent abuse.
    -   Input must be sanitized to prevent injection attacks.
-   **Performance**:
    -   P95 response time for a streamed response should begin in under 2 seconds.
    -   The document ingestion process should be efficient.
-   **Error Handling**:
    -   If no relevant content is found, the chatbot should respond with a polite message indicating it cannot answer.
    -   If backend services are down, the chatbot UI should display a clear error message.
-   **Fallback Behaviors**: If the OpenAI API fails, the system will return a "service unavailable" message.

## 11. Deliverables

1.  **Backend**:
    -   A FastAPI application with all specified endpoints.
    -   A script (`ingest.py`) to process the markdown files, generate embeddings, and populate Qdrant.
2.  **Frontend**:
    -   A React component that integrates ChatKit.
    -   JavaScript to handle text selection and interaction with the chatbot.
3.  **Database**:
    -   An SQL script to create the `chat_history` table in Neon Postgres.