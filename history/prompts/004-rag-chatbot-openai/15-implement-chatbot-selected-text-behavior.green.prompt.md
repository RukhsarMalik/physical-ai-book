---
id: 15
title: Implement Chatbot Selected Text Behavior
stage: green
date: 2025-12-09
surface: agent
model: Gemini
feature: 004-rag-chatbot-openai
branch: main
user: '{{USER}}'
command: '{{COMMAND}}'
labels: [chatbot, feature, backend]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - backend/app/api/chat.py
tests:
---

## Prompt

When the frontend sends any message that contains:
SELECTED_TEXT: "<text>"

chatbot must ALWAYS:
1. Detect that it is selected text.
2. Treat it as the main query. Ignore all other user text.
3. Explain the selected text in clear, simple language.
4. Provide meaning, summary, examples, and context.
5. NEVER ask the user: "What text do you want to explain?"

If SELECTED_TEXT is empty or not included, behave as a normal chatbot.

## Response snapshot

I have modified `backend/app/api/chat.py` to handle the `SELECTED_TEXT: "<text>"` format. The `ChatRequest` model now extracts this text, and the `chat_with_bot` endpoint prioritizes it as the main query for the AI agent, providing a generic instruction for explanation.

## Outcome

- ✅ Impact: High. Implements a core feature for selected text analysis.
- 🧪 Tests: None.
- 📁 Files: 1 file updated.
- 🔁 Next prompts: User to verify the new behavior.
- 🧠 Reflection: Successfully implemented complex parsing and conditional logic to modify chatbot behavior based on specific message patterns.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): None.
