# Feature Specification: RAG Chatbot - OpenAI ChatKit Integration

**Feature Branch**: `004-rag-chatbot-openai`
**Created**: 2025-11-30
**Status**: Draft
**Input**: User description: "RAG CHATBOT - COMPLETE SPECIFICATION OVERVIEW: Build a production-ready RAG chatbot using OpenAI Agents/ChatKit SDKs that answers questions about the Physical AI & Humanoid Robotics book content. PART 1: BACKEND ARCHITECTURE 1.1 TECHNOLOGY STACK: - Language: Python 3.11+ - Framework: FastAPI 0.104+ - AI SDK: OpenAI Agents SDK / ChatKit SDK (for agent-based responses) - Database: Neon Serverless Postgres - Vector DB: Qdrant Cloud (Free Tier) - Dependencies: fastapi, uvicorn, openai (with Agents SDK support), openai-agents or chatkit-sdk, qdrant-client, psycopg2-binary, python-dotenv, pydantic 1.2 OPENAI AGENTS/CHATKIT INTEGRATION: Agent Configuration: - Use OpenAI Agents SDK for conversational intelligence - Agent can call tools/functions to retrieve book content - Multi-turn conversation support - Context-aware responses Tools/Functions for Agent: 1. search_book_content(query: str) → Returns relevant chunks 2. get_module_info(module_name: str) → Returns module details 3. get_code_examples(topic: str) → Returns code snippets 1.3 DATABASE SCHEMAS: Postgres Tables: - chat_sessions: (id, user_id, created_at, agent_state) - messages: (id, session_id, role, content, timestamp, tool_calls) - agent_memory: (session_id, context_data, last_updated) Qdrant Collection: - book_content: vectors (1536 dimensions from OpenAI text-embedding-3-small) - payload: {text, module, page, title, code_blocks} 1.4 API ENDPOINTS: POST /api/chat Request: { message: string, session_id?: string, use_agent: boolean (default: true) } Response: { answer: string, sources: array, session_id: string, agent_thought_process?: string } POST /api/chat/selection Request: { message: string, selected_text: string, session_id?: string } Response: { answer: string, sources: array, focused_on_selection: boolean } POST /api/embeddings/generate Request: {force_refresh?: boolean} Response: {status: string, documents_embedded: number} GET /api/agent/status Response: {agent_initialized: boolean, tools_loaded: array} GET /api/health Response: { status: string, services: {postgres, qdrant, openai, agent_sdk} } 1.5 RAG PIPELINE WITH OPENAI AGENTS: Step 1: User sends question to agent Step 2: Agent analyzes query and decides if retrieval needed Step 3: Agent calls search_book_content() tool Step 4: Tool generates embedding and searches Qdrant Step 5: Relevant chunks returned to agent Step 6: Agent synthesizes answer using retrieved context Step 7: Agent returns conversational response with citations Step 8: Save conversation to Postgres with agent state Agent System Prompt: ""You are an expert teaching assistant for the Physical AI & Humanoid Robotics course. You have access to the complete textbook content. When answering questions: 1. Use the search_book_content tool to find relevant information 2. Cite specific modules and pages 3. Provide code examples when relevant 4. If user selects text, focus your answer on that specific content 5. Be clear, educational, and encouraging"" 1.6 TEXT SELECTION FEATURE WITH AGENT: When user selects text: - Send selected_text as additional context to agent - Agent instruction: ""Answer based primarily on this selected text: {selected_text}"" - Agent retrieves related content but prioritizes selection - Response focuses on explaining the selected content 1.7 ENVIRONMENT VARIABLES: - OPENAI_API_KEY - OPENAI_AGENT_MODEL (default: gpt-4-turbo) - QDRANT_URL - QDRANT_API_KEY - NEON_DATABASE_URL - CORS_ORIGINS - AGENT_TOOLS_ENABLED=true PART 2: FRONTEND ARCHITECTURE 2.1 COMPONENT STRUCTURE: - ChatButton.tsx (Floating button with AI badge) - ChatWindow.tsx (Main chat interface) - MessageList.tsx (Display messages with agent indicators) - MessageInput.tsx (User input field) - TextSelectionHandler.tsx (Detect and handle text selection) - AgentThinking.tsx (Show when agent is processing) - ChatContext.tsx (State management) 2.2 STATE MANAGEMENT: - messages: array of {role, content, timestamp, tool_calls} - isOpen: boolean - isAgentThinking: boolean (special loading state) - sessionId: string - selectedText: string | null - agentCapabilities: array of available tools 2.3 UI/UX REQUIREMENTS: - Floating button: ""Ask AI Assistant"" with sparkle icon - Chat window: Clean interface with agent branding - Agent thinking indicator: ""AI is analyzing the textbook..."" - Tool usage indicator: ""Searching Module 2..."" (show what agent is doing) - Source citations: Clickable links to specific pages - Text selection tooltip: ""Ask AI about this section"" 2.4 API INTEGRATION WITH CHATKIT: - Use OpenAI ChatKit SDK if available on frontend - Otherwise REST API calls to backend agent - Streaming responses for better UX - Show agent's reasoning process PART 3: OPENAI CHATKIT SDK USAGE 3.1 IF USING CHATKIT ON FRONTEND: - Initialize ChatKit with API endpoint - Configure agent tools - Handle streaming responses - Manage conversation state 3.2 IF USING BACKEND AGENT ONLY: - Frontend sends requests to FastAPI - Backend manages agent completely - Frontend receives formatted responses - Simpler but less interactive PART 4: DEPLOYMENT 4.1 BACKEND DEPLOYMENT: - Platform: Railway, Render, or Vercel (for FastAPI) - Environment variables properly set - Agent tools initialized on startup - Health checks for all services 4.2 FRONTEND INTEGRATION: - React component in Docusaurus - API URL configuration - Agent branding and UI polish PART 5: ERROR HANDLING 5.1 AGENT-SPECIFIC ERRORS: - Agent initialization failed → Fallback to basic RAG - Tool execution timeout → Retry or skip tool - Agent hallucination → Validate against retrieved content - Rate limits → Queue requests with user notification PART 6: TESTING 6.1 AGENT TESTING: - Test agent initialization - Test tool calling (search_book_content) - Test multi-turn conversations - Test text selection mode - Test agent's citation accuracy SUCCESS CRITERIA: ✅ OpenAI Agent/ChatKit SDK properly integrated ✅ Agent can search and retrieve book content ✅ Agent provides accurate, cited answers ✅ Text selection feature works with agent context ✅ Conversational, multi-turn dialogue works ✅ Agent shows reasoning/tool usage (transparency) ✅ Fast response times with streaming ✅ Professional UI with agent branding ✅ All services (FastAPI, Neon, Qdrant, OpenAI) connected ✅ Zero CORS or connection errors"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ask Questions and Get Cited Answers (Priority: P1)
As a user, I want to ask questions about the Physical AI & Humanoid Robotics book content and receive accurate, cited answers from the RAG chatbot so that I can easily find information and verify its source.

**Why this priority**: This is the core functionality of a RAG chatbot.
**Independent Test**: A user can ask a factual question related to the book content and receive a correct answer with at least one citation to a module or page.

**Acceptance Scenarios**:
1. **Given** the user inputs a question, **When** they submit it to the chat, **Then** the chatbot provides a relevant answer.
2. **Given** the chatbot provides an answer, **When** the answer uses information from the book, **Then** the answer includes clickable citations to the relevant module/page.
3. **Given** the chatbot provides an answer, **When** the question is about code examples, **Then** the chatbot includes a relevant code snippet.

---

### User Story 2 - Get Explanations for Selected Text (Priority: P1)
As a user, I want to select text directly from the book content and ask the RAG chatbot to explain that specific section in more detail so that I can get contextual help without re-typing.

**Why this priority**: This enhances the user experience by integrating the chatbot directly with the reading experience.
**Independent Test**: A user can select a paragraph of text, trigger the "Ask AI about this section" feature, and receive a focused explanation from the chatbot related to the selected content.

**Acceptance Scenarios**:
1. **Given** the user selects text on a page, **When** they click "Ask AI about this section," **Then** a chat prompt appears with the selected text as context.
2. **Given** the chat is prompted with selected text, **When** the chatbot responds, **Then** the answer primarily focuses on clarifying or expanding on the selected text.

---

### User Story 3 - Engage in Multi-Turn Conversations (Priority: P2)
As a user, I want to engage in multi-turn conversations with the RAG chatbot, where it maintains context from previous turns so that I can have a more natural and productive dialogue.

**Why this priority**: Conversational memory improves the utility and user satisfaction of the chatbot.
**Independent Test**: A user can ask a follow-up question that depends on the context of a previous question/answer, and the chatbot provides a relevant response demonstrating contextual understanding.

**Acceptance Scenarios**:
1. **Given** the chatbot answers a question about "ROS 2 Nodes," **When** the user then asks "What about Topics?", **Then** the chatbot discusses ROS 2 Topics without requiring a full re-statement of the context.

---

### User Story 4 - Observe Agent's Processing (Priority: P2)
As a user, I want to see indicators of the chatbot's processing (e.g., "AI is analyzing," "Searching Module X") so that I understand what the agent is doing and don't perceive it as frozen.

**Why this priority**: Transparency in processing improves user trust and patience.
**Independent Test**: When the chatbot is retrieving information or synthesizing an answer, a clear "Agent is thinking..." or "Searching Module X..." indicator is displayed.

**Acceptance Scenarios**:
1. **Given** the user sends a message, **When** the agent is processing a query, **Then** an "AI is analyzing the textbook..." indicator is visible.
2. **Given** the agent calls a tool (e.g., `search_book_content`), **When** the tool is active, **Then** an indicator like "Searching Module 2..." is displayed.

---

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The backend MUST be implemented using Python 3.11+ and FastAPI 0.104+.
- **FR-002**: The backend MUST utilize the OpenAI Agents SDK / ChatKit SDK for conversational intelligence and tool calling.
- **FR-003**: The backend MUST integrate with Neon Serverless Postgres for `chat_sessions`, `messages`, and `agent_memory` storage.
- **FR-004**: The backend MUST integrate with Qdrant Cloud (Free Tier) for `book_content` vector storage and retrieval.
- **FR-005**: The `book_content` Qdrant collection MUST store 1536-dimension vectors (from OpenAI `text-embedding-3-small`) and a payload including `{text, module, page, title, code_blocks}`.
- **FR-006**: The backend MUST expose a `POST /api/chat` endpoint as specified, supporting `message`, optional `session_id`, and `use_agent` (default: true).
- **FR-007**: The backend MUST expose a `POST /api/chat/selection` endpoint as specified, supporting `message`, `selected_text`, and optional `session_id`.
- **FR-008**: The backend MUST expose a `POST /api/embeddings/generate` endpoint as specified, supporting optional `force_refresh`.
- **FR-009**: The backend MUST expose a `GET /api/agent/status` endpoint to report `agent_initialized` status and `tools_loaded`.
- **FR-010**: The backend MUST expose a `GET /api/health` endpoint reporting the status of `postgres`, `qdrant`, `openai`, and `agent_sdk` services.
- **FR-011**: The RAG pipeline MUST follow the 8 steps outlined in the specification (User query → Agent analysis → Tool call → Qdrant search → Chunk return → Agent synthesis → Response with citations → Save conversation).
- **FR-012**: The agent system prompt MUST be configurable and align with the provided specification.
- **FR-013**: The text selection feature MUST send `selected_text` as additional context to the agent, instructing it to prioritize the selected content.
- **FR-014**: The frontend MUST comprise `ChatButton.tsx`, `ChatWindow.tsx`, `MessageList.tsx`, `MessageInput.tsx`, `TextSelectionHandler.tsx`, `AgentThinking.tsx`, and `ChatContext.tsx`.
- **FR-015**: Frontend state management MUST include `messages`, `isOpen`, `isAgentThinking`, `sessionId`, `selectedText`, and `agentCapabilities`.
- **FR-016**: The UI MUST include a floating "Ask AI Assistant" button with a sparkle icon.
- **FR-017**: The chat window UI MUST be clean with agent branding.
- **FR-018**: An "AI is analyzing the textbook..." indicator MUST be displayed when the agent is thinking.
- **FR-019**: A "Searching Module X..." indicator MUST be displayed when the agent is using a tool.
- **FR-020**: Source citations MUST be clickable links to specific pages in the Docusaurus content.
- **FR-021**: A "Ask AI about this section" tooltip MUST appear for text selection.
- **FR-022**: API integration MUST support streaming responses for better UX and show the agent's reasoning process.
- **FR-023**: Backend deployment MUST support platforms like Railway, Render, or Vercel, with proper environment variable management and health checks.
- **FR-024**: Frontend integration MUST involve embedding the React component in Docusaurus with configurable API URL and agent branding.
- **FR-025**: The system MUST handle agent-specific errors including initialization failure (fallback to basic RAG), tool execution timeouts (retry/skip), hallucination (validation against retrieved content), and rate limits (queueing/notification).

### Key Entities

-   **ChatSession**: Represents an ongoing conversation with the chatbot.
    *   `id`: UUID, primary key.
    *   `user_id`: String, identifies the user.
    *   `created_at`: Timestamp.
    *   `agent_state`: JSONB, stores the agent's internal state for multi-turn conversations.
-   **Message**: Stores individual messages within a chat session.
    *   `id`: UUID, primary key.
    *   `session_id`: UUID, foreign key to `ChatSession`.
    *   `role`: Enum (user, assistant, system, tool).
    *   `content`: Text.
    *   `timestamp`: Timestamp.
    *   `tool_calls`: JSONB, stores details if the message involved tool calls.
-   **AgentMemory**: Stores context data relevant to the agent's memory.
    *   `session_id`: UUID, foreign key to `ChatSession`.
    *   `context_data`: JSONB, adaptable structure for agent's long-term memory.
    *   `last_updated`: Timestamp.
-   **BookContent (Qdrant Payload)**: Metadata for each chunk of the textbook stored in Qdrant.
    *   `text`: Original text chunk.
    *   `module`: String, e.g., "Module 1: ROS 2 Fundamentals".
    *   `page`: String, e.g., "intro.md".
    *   `title`: String, title of the page/section.
    *   `code_blocks`: Array of strings, any code blocks within the chunk.

## Success Criteria *(mandatory)*

### Measurable Outcomes
-   **SC-001**: OpenAI Agent/ChatKit SDK is properly integrated, and the agent initializes without errors.
-   **SC-002**: The agent can successfully call `search_book_content` and retrieve relevant book content from Qdrant.
-   **SC-003**: The agent provides accurate, contextually relevant, and cited answers to user questions (evaluated by human review for >90% accuracy).
-   **SC-004**: The text selection feature correctly passes selected text to the agent, and the agent's response is focused on the provided context.
-   **SC-005**: The chatbot maintains conversational context across multiple turns, providing coherent responses.
-   **SC-006**: The UI correctly displays agent's reasoning and tool usage (e.g., "Searching Module 2...").
-   **SC-007**: Chat responses are streamed to the frontend, resulting in perceived fast response times (<2 seconds for initial response).
-   **SC-008**: The chatbot UI adheres to professional design standards and branding guidelines.
-   **SC-009**: All backend services (FastAPI, Neon, Qdrant, OpenAI, agent SDK) are successfully connected and reporting healthy status via `/api/health`.
-   **SC-010**: No CORS or API connection errors occur during normal operation.