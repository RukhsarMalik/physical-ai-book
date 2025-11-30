---
id: 1
title: Specify RAG Chatbot with OpenAI ChatKit
stage: spec
date: 2025-11-30
surface: "agent"
model: "gemini"
feature: "004-rag-chatbot-openai"
branch: "004-rag-chatbot-openai"
user: "user"
command: "/sp.specify"
labels: ["spec", "rag-chatbot", "openai", "chatkit", "fastapi"]
links:
  spec: "specs/004-rag-chatbot-openai/spec.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - specs/004-rag-chatbot-openai/spec.md
 - specs/004-rag-chatbot-openai/checklists/requirements.md
tests:
 - "none"
---

## Prompt

Create a detailed specification for RAG Chatbot using OpenAI Agents/ChatKit SDK.

Run: specify create "RAG Chatbot - OpenAI ChatKit Integration"

Include:

RAG CHATBOT - COMPLETE SPECIFICATION

OVERVIEW:
Build a production-ready RAG chatbot using OpenAI Agents/ChatKit SDKs that answers questions about the Physical AI & Humanoid Robotics book content.

PART 1: BACKEND ARCHITECTURE

1.1 TECHNOLOGY STACK:
- Language: Python 3.11+
- Framework: FastAPI 0.104+
- AI SDK: OpenAI Agents SDK / ChatKit SDK (for agent-based responses)
- Database: Neon Serverless Postgres
- Vector DB: Qdrant Cloud (Free Tier)
- Dependencies: 
  * fastapi
  * uvicorn
  * openai (with Agents SDK support)
  * openai-agents or chatkit-sdk
  * qdrant-client
  * psycopg2-binary
  * python-dotenv
  * pydantic

1.2 OPENAI AGENTS/CHATKIT INTEGRATION:

Agent Configuration:
- Use OpenAI Agents SDK for conversational intelligence
- Agent can call tools/functions to retrieve book content
- Multi-turn conversation support
- Context-aware responses

Tools/Functions for Agent:
1. search_book_content(query: str) → Returns relevant chunks
2. get_module_info(module_name: str) → Returns module details
3. get_code_examples(topic: str) → Returns code snippets

1.3 DATABASE SCHEMAS:

Postgres Tables:
- chat_sessions: (id, user_id, created_at, agent_state)
- messages: (id, session_id, role, content, timestamp, tool_calls)
- agent_memory: (session_id, context_data, last_updated)

Qdrant Collection:
- book_content: 
  * vectors (1536 dimensions from OpenAI text-embedding-3-small)
  * payload: {text, module, page, title, code_blocks}

1.4 API ENDPOINTS:

POST /api/chat
Request: {
  message: string, 
  session_id?: string,
  use_agent: boolean (default: true)
}
Response: {
  answer: string, 
  sources: array, 
  session_id: string,
  agent_thought_process?: string
}

POST /api/chat/selection
Request: {
  message: string, 
  selected_text: string, 
  session_id?: string
}
Response: {
  answer: string, 
  sources: array,
  focused_on_selection: boolean
}

POST /api/embeddings/generate
Request: {force_refresh?: boolean}
Response: {status: string, documents_embedded: number}

GET /api/agent/status
Response: {agent_initialized: boolean, tools_loaded: array}

GET /api/health
Response: {
  status: string, 
  services: {postgres, qdrant, openai, agent_sdk}
}

1.5 RAG PIPELINE WITH OPENAI AGENTS:

Step 1: User sends question to agent
Step 2: Agent analyzes query and decides if retrieval needed
Step 3: Agent calls search_book_content() tool
Step 4: Tool generates embedding and searches Qdrant
Step 5: Relevant chunks returned to agent
Step 6: Agent synthesizes answer using retrieved context
Step 7: Agent returns conversational response with citations
Step 8: Save conversation to Postgres with agent state

Agent System Prompt:
"You are an expert teaching assistant for the Physical AI & Humanoid Robotics course. You have access to the complete textbook content. When answering questions:
1. Use the search_book_content tool to find relevant information
2. Cite specific modules and pages
3. Provide code examples when relevant
4. If user selects text, focus your answer on that specific content
5. Be clear, educational, and encouraging"

1.6 TEXT SELECTION FEATURE WITH AGENT:

When user selects text:
- Send selected_text as additional context to agent
- Agent instruction: "Answer based primarily on this selected text: {selected_text}"
- Agent retrieves related content but prioritizes selection
- Response focuses on explaining the selected content

1.7 ENVIRONMENT VARIABLES:
- OPENAI_API_KEY
- OPENAI_AGENT_MODEL (default: gpt-4-turbo)
- QDRANT_URL
- QDRANT_API_KEY
- NEON_DATABASE_URL
- CORS_ORIGINS
- AGENT_TOOLS_ENABLED=true

PART 2: FRONTEND ARCHITECTURE

2.1 COMPONENT STRUCTURE:
- ChatButton.tsx (Floating button with AI badge)
- ChatWindow.tsx (Main chat interface)
- MessageList.tsx (Display messages with agent indicators)
- MessageInput.tsx (User input field)
- TextSelectionHandler.tsx (Detect and handle text selection)
- AgentThinking.tsx (Show when agent is processing)
- ChatContext.tsx (State management)

2.2 STATE MANAGEMENT:
- messages: array of {role, content, timestamp, tool_calls}
- isOpen: boolean
- isAgentThinking: boolean (special loading state)
- sessionId: string
- selectedText: string | null
- agentCapabilities: array of available tools

2.3 UI/UX REQUIREMENTS:
- Floating button: "Ask AI Assistant" with sparkle icon
- Chat window: Clean interface with agent branding
- Agent thinking indicator: "AI is analyzing the textbook..."
- Tool usage indicator: "Searching Module 2..." (show what agent is doing)
- Source citations: Clickable links to specific pages - Text selection tooltip: "Ask AI about this section"

2.4 API INTEGRATION WITH CHATKIT:
- Use OpenAI ChatKit SDK if available on frontend
- Otherwise REST API calls to backend agent
- Streaming responses for better UX
- Show agent's reasoning process

PART 3: OPENAI CHATKIT SDK USAGE

3.1 IF USING CHATKIT ON FRONTEND:
- Initialize ChatKit with API endpoint
- Configure agent tools
- Handle streaming responses
- Manage conversation state

3.2 IF USING BACKEND AGENT ONLY:
- Frontend sends requests to FastAPI
- Backend manages agent completely
- Frontend receives formatted responses
- Simpler but less interactive

PART 4: DEPLOYMENT

4.1 BACKEND DEPLOYMENT:
- Platform: Railway, Render, or Vercel (for FastAPI)
- Environment variables properly set
- Agent tools initialized on startup
- Health checks for all services

4.2 FRONTEND INTEGRATION:
- React component in Docusaurus
- API URL configuration
- Agent branding and UI polish

PART 5: ERROR HANDLING

5.1 AGENT-SPECIFIC ERRORS:
- Agent initialization failed → Fallback to basic RAG - Tool execution timeout → Retry or skip tool - Agent hallucination → Validate against retrieved content - Rate limits → Queue requests with user notification

PART 6: TESTING

6.1 AGENT TESTING:
- Test agent initialization - Test tool calling (search_book_content) - Test multi-turn conversations - Test text selection mode - Test agent's citation accuracy

SUCCESS CRITERIA:
✅ OpenAI Agent/ChatKit SDK properly integrated
✅ Agent can search and retrieve book content
✅ Agent provides accurate, cited answers
✅ Text selection feature works with agent context
✅ Conversational, multi-turn dialogue works
✅ Agent shows reasoning/tool usage (transparency)
✅ Fast response times with streaming
✅ Professional UI with agent branding
✅ All services (FastAPI, Neon, Qdrant, OpenAI) connected
✅ Zero CORS or connection errors

## Response snapshot

Successfully created the feature specification for 'RAG Chatbot - OpenAI ChatKit Integration'.
- Switched to new branch: `004-rag-chatbot-openai`
- Created specification: `specs/004-rag-chatbot-openai/spec.md`
- Created and passed quality checklist: `specs/004-rag-chatbot-openai/checklists/requirements.md`

The specification is ready for the next phase.

## Outcome

- ✅ Impact: High - Defines the full scope and requirements for the RAG Chatbot feature.
- 🧪 Tests: N/A
- 📁 Files: 2 files created/modified - `specs/004-rag-chatbot-openai/spec.md`, `specs/004-rag-chatbot-openai/checklists/requirements.md`.
- 🔁 Next prompts: User can now proceed with other setup commands like `/sp.plan` or `/sp.implement`.
- 🧠 Reflection: Despite initial parsing difficulties, the script successfully created the necessary files. The user provided a very detailed prompt which resulted in a high-quality specification.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-new-feature.ps1` script had a non-critical parameter binding error, likely due to argument length even with PowerShell here-strings. This indicates a limitation of passing very large feature descriptions directly as command arguments.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Implement a mechanism in `create-new-feature.ps1` to read the feature description from a temporary file if the direct argument causes parsing issues.
