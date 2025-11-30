# Tasks: RAG Chatbot - OpenAI ChatKit Integration

**Input**: Design documents from `specs/004-rag-chatbot-openai/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md

## Phase 1: Setup (Shared Infrastructure)
**Purpose**: Initial project setup and environment configuration for the backend FastAPI application.

- [x] T001 Create `backend/` folder structure: `backend/app/main.py`, `models/`, `services/`, `api/`, `config.py`, `requirements.txt`, `.env.example`, `Dockerfile`
- [x] T002 Setup Python virtual environment in `backend/` and activate it.
- [ ] T003 Install all backend dependencies in `backend/requirements.txt` (`fastapi`, `uvicorn`, `openai`, `openai-agents` or `chatkit-sdk`, `qdrant-client`, `psycopg2-binary`, `python-dotenv`, `pydantic`).
- [ ] T004 Create `backend/.env.example` file based on the spec's environment variables.
- [ ] T005 Initialize basic FastAPI app in `backend/app/main.py` and configure CORS as per `CORS_ORIGINS` environment variable.

---

## Phase 2: Foundational (Blocking Prerequisites)
**Purpose**: Establish core database connections for PostgreSQL and Qdrant.

- [ ] T006 Setup Neon Postgres connection:
    - Configure connection string using `NEON_DATABASE_URL` environment variable.
    - Create `backend/app/models/database.py` for database connection management.
    - Define database schema in `backend/app/models/` for `chat_sessions`, `messages`, and `agent_memory` tables.
    - Write initial migration script (if ORM is used) or raw SQL to create tables.
    - Test database connection and table creation/insertion.
- [ ] T007 Setup Qdrant Vector DB:
    - Configure connection to Qdrant Cloud using `QDRANT_URL` and `QDRANT_API_KEY` environment variables.
    - Create `backend/app/services/qdrant_service.py` with helper functions for vector operations.
    - Implement collection creation for `book_content` with 1536 dimensions.
    - Test Qdrant connection and basic vector operations.

---

## Phase 3: User Story 1 - Ask Questions and Get Cited Answers (Priority: P1) 🎯 MVP
**Goal**: Implement the core RAG chatbot functionality to answer questions about book content with citations.

- [ ] T008 [US1] Develop Document Embedding Pipeline (`backend/app/services/embeddings_service.py`):
    - Write script to read all markdown files from `physical-ai-book/docs/`.
    - Implement text chunking logic (e.g., using `langchain_text_splitters`).
    - Generate embeddings for chunks using OpenAI `text-embedding-3-small` model.
    - Store embeddings and metadata (module, page, title, code_blocks) in Qdrant.
    - Create `POST /api/embeddings/generate` endpoint in `backend/app/api/embeddings.py` to trigger this process.
    - Test with sample Docusaurus documents.
- [ ] T009 [US1] Integrate OpenAI Agent (`backend/app/services/agent_service.py`):
    - Initialize OpenAI Agent with specified persona (teaching assistant) and `OPENAI_AGENT_MODEL`.
    - Define and implement agent tool: `search_book_content(query)` to interact with `qdrant_service.py`.
    - Define and implement agent tool: `get_module_info(module_name)` to retrieve module details (e.g., from Docusaurus config or metadata).
    - Define and implement agent tool: `get_code_examples(topic)` to retrieve code snippets (e.g., from Qdrant payload or direct file search).
    - Register these tools with the agent.
    - Test agent initialization and tool calling with sample queries.
- [ ] T010 [US1] Implement RAG Chat Endpoint (`backend/app/api/chat.py`):
    - Create `POST /api/chat` endpoint as per OpenAPI spec (`/chat`).
    - Implement the RAG pipeline: Receive user message → Agent analysis → Tool call → Qdrant retrieval → Agent synthesizes answer with citations → Save conversation to Postgres → Return streamed response.
    - Add robust error handling, retry logic, and handle multi-turn conversation state using `agent_state` in `chat_sessions`.
    - Test with various questions about book content.

---

## Phase 4: User Story 2 - Get Explanations for Selected Text (Priority: P1)
**Goal**: Enable the chatbot to provide contextual answers based on user-selected text.

- [ ] T011 [US2] Implement Text Selection Endpoint (`backend/app/api/chat.py`):
    - Create `POST /api/chat/selection` endpoint as per OpenAPI spec (`/chat/selection`).
    - Accept `selected_text` parameter.
    - Modify agent prompt to focus primarily on the `selected_text`.
    - Combine `selected_text` with relevant retrieved context for agent synthesis.
    - Test with sample text selections.

---

## Phase 5: User Story 3 - Engage in Multi-Turn Conversations (Priority: P2)
**Goal**: Ensure the chatbot maintains context across multiple turns. (This is primarily handled by T010's agent state management.)
- No dedicated tasks needed here beyond what's in T010.

---

## Phase 6: User Story 4 - Observe Agent's Processing (Priority: P2)
**Goal**: Provide transparency into the agent's actions and status.

- [ ] T012 [US4] Implement Health & Utility Endpoints (`backend/app/api/monitoring.py`):
    - Create `GET /api/health` endpoint as per OpenAPI spec, checking status of Postgres, Qdrant, and OpenAI connections.
    - Create `GET /api/agent/status` endpoint as per OpenAPI spec, returning agent initialization status and loaded tools.
    - Add structured logging to relevant backend services for debugging and monitoring.

---

## Phase 7: Polish & Cross-Cutting Concerns (Frontend & Deployment)
**Purpose**: Develop frontend components, integrate with Docusaurus, deploy backend/frontend, and perform final testing.

- [ ] T013 Create Frontend Chat Button Component (`physical-ai-book/src/components/ChatButton.tsx`):
    - Implement floating button (bottom-right, 60px circle) with AI/sparkle icon.
    - Implement click functionality to toggle chat window visibility.
    - Add smooth hover and click animations.
- [ ] T014 Create Frontend Chat Window Component (`physical-ai-book/src/components/ChatWindow.tsx`):
    - Implement slide-in animation from bottom-right.
    - Add header with title "AI Teaching Assistant" and close button.
    - Implement responsive dimensions (400x600px desktop, full screen mobile).
    - Apply styling using CSS modules or Tailwind.
- [ ] T015 Create Message Display Components (`physical-ai-book/src/components/MessageList.tsx`, `MessageBubble.tsx`):
    - Implement scrollable message area (`MessageList.tsx`) with auto-scroll to latest message.
    - Implement individual message bubble (`MessageBubble.tsx`) with user/bot roles, styling (right-aligned blue for user, left-aligned gray with avatar for bot), and timestamp display.
    - Implement clickable source citations within bot messages.
- [ ] T016 Create Message Input Component (`physical-ai-book/src/components/MessageInput.tsx`):
    - Implement text input field and send button.
    - Configure Enter key to send, Shift+Enter for new line.
    - Implement loading state (disable input during API calls) and character limit indicator.
- [ ] T017 Create Agent Thinking Indicator (`physical-ai-book/src/components/AgentThinking.tsx`):
    - Implement animated dots or spinner.
    - Display text: "AI is analyzing the textbook..."
    - Optionally show which tool agent is using (e.g., "Searching Module 2...") if available from backend.
- [ ] T018 Create API Service Integration (`physical-ai-book/src/services/api.ts`):
    - Create `api.ts` service with `sendMessage()`, `sendSelectionQuery()` functions.
    - Use `axios` or `fetch` with retry logic.
    - Implement error handling for API calls with user-friendly messages.
    - Configure API URL via environment variables.
    - Test basic API calls.
- [ ] T019 Create Text Selection Handler (`physical-ai-book/src/components/TextSelectionHandler.tsx`):
    - Implement `mouseup` event listener (`window.getSelection()`).
    - Display "Ask AI about this section" tooltip for text > 10 characters.
    - Implement tooltip positioning and click action (open chat with selected text context).
    - Handle edge cases (empty selection, cross-page).
- [ ] T020 Implement State Management with Context (`physical-ai-book/src/contexts/ChatContext.tsx`):
    - Create `ChatContext.tsx` with React Context to manage `messages`, `isOpen`, `isLoading`, `sessionId`, `selectedText`.
    - Implement `ChatProvider` wrapper for the Docusaurus app.
    - Create custom hooks for `useChatContext()`.
- [ ] T021 Integrate Chat in Docusaurus (`physical-ai-book/src/theme/Root.tsx`):
    - Wrap the Docusaurus app with `ChatProvider`.
    - Add `ChatButton` component.
    - Ensure chat functionality works consistently across all Docusaurus pages.
- [ ] T022 Backend Deployment:
    - Choose deployment platform (Railway or Render).
    - Create `Dockerfile` (if containerization is chosen).
    - Configure environment variables on chosen platform.
    - Deploy backend application.
    - Test deployed API endpoints.
    - Get production API URL.
- [ ] T023 Frontend Deployment:
    - Update frontend API URL to production backend URL.
    - Build Docusaurus site: `npm run build`.
    - Deploy Docusaurus site to GitHub Pages or Vercel.
    - Test chat on production site.
    - Verify CORS configuration.
- [ ] T024 End-to-End Testing:
    - Thoroughly test all user flows: asking questions about modules, text selection queries, multi-turn conversations, mobile responsiveness.
    - Debug and fix any bugs found.
    - Test error scenarios (e.g., API failures, invalid input).
    - Verify source citations work and link correctly.
- [ ] T025 Final Polish & Optimization:
    - Improve UI/UX polish (animations, transitions).
    - Add loading skeletons for better perceived performance.
    - Optimize frontend and backend response times.
    - Implement rate limiting on backend if not already present.
    - Perform final comprehensive testing.
    - Document any remaining assumptions or future work.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Phase 1 (Setup)** must be completed before other phases.
- **Phase 2 (Foundational)** depends on Phase 1 completion.
- **Phase 3 (US1)** depends on Phase 2 completion.
- **Phase 4 (US2)** depends on Phase 3 completion.
- **Phase 5 (US3)** depends on Phase 3 completion.
- **Phase 6 (US4)** depends on Phase 3 completion.
- **Phase 7 (Polish & Cross-Cutting Concerns)** depends on completion of all preceding User Story phases.

### User Story Dependencies
- **User Story 1 (Ask Questions and Get Cited Answers)**: Is the MVP, requires core backend (Phase 1-2, T008-T010) and core frontend components (T011-T016, T018-T020).
- **User Story 2 (Get Explanations for Selected Text)**: Depends on US1 functionality, requires specific backend endpoint (T011) and frontend handler (T015).
- **User Story 3 (Engage in Multi-Turn Conversations)**: Functionality is inherent in agent design (T009) and chat endpoint (T010), not separate tasks.
- **User Story 4 (Observe Agent's Processing)**: Requires backend utility endpoints (T012) and frontend indicator (T013, part of T015).

### Parallel Opportunities
- Frontend component development (T013-T021) can run in parallel with backend endpoint development (T010-T012) once initial backend setup (Phase 1-2, T001-T007) is complete.
- Testing and deployment can run in parallel for different parts of the system once respective components are ready.

---

## Implementation Strategy

### MVP First (User Story 1 - Core Chat Functionality)
1. Complete Phase 1: Backend Setup.
2. Complete Phase 2: Foundational (Database & Qdrant).
3. Complete Phase 3 (US1): Document Embedding, Agent Integration, RAG Chat Endpoint.
4. Complete Frontend Tasks for US1: Message Display, Message Input, Agent Thinking Indicator, API Integration, State Management.
5. **STOP and VALIDATE**: Core chat functionality works, user can ask questions, get cited answers.

### Incremental Delivery
1. After MVP, implement Phase 4 (US2): Text Selection.
2. Implement Phase 6 (US4): Health & Utility Endpoints.
3. Complete remaining Frontend components (Chat Button, Chat Window).
4. Deploy Backend and Frontend.
5. Perform End-to-End Testing and Polish.

### Total Estimated Time: ~12-14 hours (Can be reduced with focused execution: ~8-10 hours)
