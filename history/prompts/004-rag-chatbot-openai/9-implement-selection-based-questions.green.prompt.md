---
id: 9
title: Implement selection-based questions in the chatbot
stage: green
date: 2025-12-09
surface: agent
model: gemini
feature: rag-chatbot-openai
branch: main
user: RukhsarMalik
command: "I want you to add a feature so that whenever the user selects some text in the book UI, that selected text is sent to the chatbot as part of the query."
labels: ["chatbot", "rag", "selection-based-questions"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - physical-ai-book/src/components/TextSelectionHandler.tsx
 - physical-ai-book/src/contexts/ChatContext.tsx
 - backend/app/api/chat.py
 - backend/app/services/agent_service.py
tests:
 - none
---

## Prompt

I want you to add a feature so that whenever the user selects some text in the book UI, that selected text is sent to the chatbot as part of the query.

Requirements:

Listen for text selection in the book reader (e.g. on mouseup / selectionchange).

If the user has selected non-empty text, open the chatbot panel (or focus the chat input) and send a message like:
"Explain this selection from the book and answer questions about it:\n\n<SELECTED_TEXT>".

On the backend, pass the selected text to the RAG pipeline as high-priority context (e.g. selected_passage) in addition to normal vector search.

The chatbot should always:

First interpret the selected passage.

Then answer the user’s question using both the selected passage and any retrieved chunks from the book.

If for any reason no selection is available, the bot should behave like a normal book assistant using RAG.

Please:

Modify the existing chatbot components/hooks instead of creating a completely new chatbot.

Clearly mark all new or changed code sections with comments like // NEW: selection-based question and // UPDATED:.

Keep everything consistent with the current tech stack (same framework, same state management, same styling).

You can adjust the framework details (Next.js, React, etc.) depending on what you actually used.

2️⃣ System prompt for your RAG chatbot (runtime behavior)

This is what you put as the system / assistant role prompt for the chatbot model itself (Gemini):

You are an AI Teaching Assistant for the textbook “Physical AI & Humanoid Robotics”.
The user is reading the book and may select a passage of text. You will often receive:

selected_passage: text that the user highlighted in the book

retrieved_chunks: relevant excerpts from the rest of the book (via RAG)

user_message: what the user typed

Your behavior:

If selected_passage is provided and non-empty:

First, briefly restate or summarize the selected passage in simple language.

Then, use both the selected passage and retrieved_chunks to answer the user’s question.

Make your answer clearly connected to the selected passage (quote or reference it when useful).

If there is no selected_passage, just act as a normal RAG tutor for the whole book.

Always:

Be precise and grounded only in the book content and provided context.

If the answer is not in the book, say that you don’t see it in the text and try to give a reasonable high-level explanation.

Use step-by-step reasoning when explaining technical concepts.

When useful, refer to sections/chapters (if IDs or titles are provided in the context).

Never invent page numbers or section names that are not present in the context.

Keep answers concise but clear. For deeper questions, you may add short bullet points or numbered steps.

## Response snapshot

I have implemented the selection-based question feature in the chatbot. The frontend now detects text selection and sends the selected text to the backend. The backend then uses this text as high-priority context for the RAG pipeline. The chatbot's system prompt has also been updated to handle this new functionality.

## Outcome

- ✅ Impact: The chatbot can now answer questions about user-selected text from the book.
- 🧪 Tests: none
- 📁 Files: 4 files modified
- 🔁 Next prompts: none
- 🧠 Reflection: The agent-native fallback for PHR creation is a good recovery mechanism when shell scripts fail.

## Evaluation notes (flywheel)

- Failure modes observed: The powershell script for PHR creation failed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Fix the powershell script pathing issue.
