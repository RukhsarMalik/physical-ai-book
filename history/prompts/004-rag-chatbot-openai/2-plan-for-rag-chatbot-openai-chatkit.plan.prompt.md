---
id: 2
title: Plan for RAG Chatbot with OpenAI ChatKit
stage: plan
date: 2025-11-30
surface: "agent"
model: "gemini"
feature: "004-rag-chatbot-openai"
branch: "004-rag-chatbot-openai"
user: "user"
command: "/sp.plan"
labels: ["plan", "rag-chatbot", "openai", "chatkit", "fastapi"]
links:
  spec: "specs/004-rag-chatbot-openai/spec.md"
  plan: "specs/004-rag-chatbot-openai/plan.md"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - specs/004-rag-chatbot-openai/plan.md
 - specs/004-rag-chatbot-openai/research.md
 - specs/004-rag-chatbot-openai/data-model.md
 - specs/004-rag-chatbot-openai/contracts/openapi.yaml
 - specs/004-rag-chatbot-openai/quickstart.md
tests:
 - "none"
---

## Prompt

/sp.plan Create a comprehensive implementation plan for RAG Chatbot with clear phases and time estimates.

Run: plan create

Include:

RAG CHATBOT IMPLEMENTATION PLAN

PHASE 1: BACKEND SETUP (Task 1 - 30 mins)
- Create backend project structure
- Setup Python virtual environment
- Install all dependencies (FastAPI, OpenAI SDK, Qdrant, Psycopg2, etc.)
- Create .env file template
- Setup basic FastAPI app with CORS
- Create folder structure:
  backend/
    app/
      main.py
      models/
      services/
      api/
      config.py
    requirements.txt
    .env.example

PHASE 2: DATABASE SETUP (Task 2 - 30 mins)
- Setup Neon Postgres connection
- Create database schema (chat_sessions, messages tables)
- Write database models using SQLAlchemy or Pydantic
- Test database connection
- Create migration scripts if needed

PHASE 3: QDRANT VECTOR DB SETUP (Task 3 - 30 mins)
- Setup Qdrant Cloud account (free tier)
- Create collection "book_content"
- Configure vector dimensions (1536 for OpenAI embeddings)
- Test Qdrant connection
- Write helper functions for vector operations

PHASE 4: DOCUMENT EMBEDDING PIPELINE (Task 4 - 45 mins)
- Create script to read all markdown files from docs/
- Split content into chunks (500-1000 tokens each)
- Generate embeddings using OpenAI text-embedding-3-small
- Store embeddings in Qdrant with metadata (module, page, title)
- Create endpoint: POST /api/embeddings/generate
- Test with sample documents

PHASE 5: OPENAI AGENT INTEGRATION (Task 5 - 60 mins)
- Initialize OpenAI Agent with ChatKit SDK or Agents SDK
- Define agent system prompt (teaching assistant persona)
- Create tool: search_book_content(query) function
- Create tool: get_module_info(module_name) function
- Create tool: get_code_examples(topic) function
- Test agent initialization
- Test tool calling with sample queries

PHASE 6: RAG CHAT ENDPOINT (Task 6 - 45 mins)
- Create POST /api/chat endpoint
- Implement RAG pipeline:
  * Receive user message
  * Pass to OpenAI Agent
  * Agent calls search_book_content tool
  * Retrieve from Qdrant
  * Agent synthesizes answer with citations
  * Save to Postgres
  * Return response
- Add error handling and retry logic
- Test with various questions

PHASE 7: TEXT SELECTION ENDPOINT (Task 7 - 30 mins)
- Create POST /api/chat/selection endpoint
- Accept selected_text parameter
- Modify agent prompt to focus on selected text
- Combine selected text with retrieved context
- Test with sample selections

PHASE 8: HEALTH & UTILITY ENDPOINTS (Task 8 - 15 mins)
- Create GET /api/health endpoint
- Check all services (Postgres, Qdrant, OpenAI)
- Create GET /api/agent/status endpoint
- Add logging for debugging

PHASE 9: FRONTEND CHAT BUTTON (Task 9 - 30 mins)
- Create ChatButton.tsx component
- Floating button bottom-right (60px circle)
- Click to open/close chat window
- Add sparkle/AI icon
- Smooth animations
- Integrate in Docusaurus (src/theme/Root.tsx)

PHASE 10: FRONTEND CHAT WINDOW (Task 10 - 45 mins)
- Create ChatWindow.tsx component
- Slide-in animation from bottom-right
- Header with "AI Teaching Assistant" title
- Close button
- Dimensions: 400px × 600px
- Mobile: full screen
- Styling with Tailwind or CSS modules

PHASE 11: MESSAGE COMPONENTS (Task 11 - 45 mins)
- Create MessageList.tsx (scrollable message area)
- Create MessageBubble.tsx (individual messages)
- User messages: right-aligned, blue
- Bot messages: left-aligned, gray, with avatar
- Timestamp display
- Source citations as clickable links
- Auto-scroll to latest message

PHASE 12: MESSAGE INPUT (Task 12 - 30 mins)
- Create MessageInput.tsx component
- Text input field
- Send button
- Enter key to send
- Shift+Enter for new line
- Loading state (disable during API call)
- Character limit indicator

PHASE 13: AGENT THINKING INDICATOR (Task 13 - 20 mins)
- Create AgentThinking.tsx component
- Show when isAgentThinking = true
- Animated dots or spinner
- Text: "AI is searching the textbook..."
- Show which tool agent is using if available

PHASE 14: API INTEGRATION (Task 14 - 30 mins)
- Create API service (api.ts)
- Functions: sendMessage(), sendSelectionQuery()
- Axios or Fetch with retry logic
- Error handling with user-friendly messages
- Environment variable for API URL
- Test API calls

PHASE 15: TEXT SELECTION FEATURE (Task 15 - 45 mins)
- Create TextSelectionHandler.tsx
- Listen to mouseup event
- Get window.getSelection()
- If text > 10 chars, show tooltip
- Tooltip: "Ask AI about this"
- Click → open chat with selected text
- Position tooltip near selection
- Handle edge cases (empty selection, cross-page)

PHASE 16: STATE MANAGEMENT (Task 16 - 30 mins)
- Create ChatContext.tsx with React Context
- State: messages, isOpen, isLoading, sessionId, selectedText
- Functions: openChat, closeChat, sendMessage, setSelectedText
- Wrap app with ChatProvider

PHASE 17: BACKEND DEPLOYMENT (Task 17 - 45 mins)
- Choose platform: Railway or Render
- Create Dockerfile (if needed)
- Configure environment variables
- Deploy backend
- Test deployed API endpoints
- Get production API URL

PHASE 18: FRONTEND DEPLOYMENT (Task 18 - 30 mins)
- Update API URL to production
- Build Docusaurus: npm run build
- Deploy to GitHub Pages or Vercel
- Test chat on production site
- Check CORS configuration

PHASE 19: TESTING & DEBUGGING (Task 19 - 45 mins)
- Test all user flows:
  * Ask question about Module 1
  * Select text and ask question
  * Multi-turn conversation
  * Mobile responsiveness
- Fix any bugs found
- Test error scenarios
- Verify source citations work

PHASE 20: POLISH & OPTIMIZATION (Task 20 - 30 mins)
- Improve UI/UX polish
- Add loading skeletons
- Optimize response times
- Add rate limiting if needed
- Final testing
- Documentation

TOTAL TIME ESTIMATE: ~12-14 hours
(Can be reduced with focused execution: ~8-10 hours)

CRITICAL PATH:
Phase 1-8 (Backend) → Phase 9-16 (Frontend) → Phase 17-18 (Deployment) → Phase 19-20 (Testing)

PARALLEL WORK POSSIBLE:
- Frontend components (9-13) can be built while backend endpoints (6-8) are being tested

Create this detailed plan now.

## Response snapshot

Successfully created the implementation plan for 'RAG Chatbot - OpenAI ChatKit Integration'.
- **Plan File**: `specs/004-rag-chatbot-openai/plan.md`
- **Generated Artifacts**:
  - `specs/004-rag-chatbot-openai/research.md`
  - `specs/004-rag-chatbot-openai/data-model.md`
  - `specs/004-rag-chatbot-openai/contracts/openapi.yaml`
  - `specs/004-rag-chatbot-openai/quickstart.md`
The plan is ready for task breakdown using `/sp.tasks`.

## Outcome

- ✅ Impact: High - Provides a clear, step-by-step technical plan for implementing the feature.
- 🧪 Tests: N/A
- 📁 Files: 5 files created/modified.
- 🔁 Next prompts: `/sp.tasks` to break the plan into concrete implementation tasks.
- 🧠 Reflection: The process was straightforward as the user provided a clear structure for the plan, which was integrated into the standard planning template. All necessary design artifacts were generated successfully.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): N/A