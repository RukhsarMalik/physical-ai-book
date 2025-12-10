---
id: 1
title: Create Initial Project Constitution
stage: constitution
date: 2025-11-29
surface: "agent"
model: "gemini"
feature: "none"
branch: "main"
user: "user"
command: "/sp.constitution"
labels: ["constitution", "project-setup"]
links:
  spec: "null"
  ticket: "null"
  adr: "null"
  pr: "null"
files:
 - .specify/memory/constitution.md
tests:
 - "none"
---

## Prompt

/sp.constitution Create a constitution.md file for this project with the following details:

PROJECT NAME: Physical AI & Humanoid Robotics Textbook

PURPOSE: Create an educational textbook using Docusaurus that teaches Physical AI and Humanoid Robotics with an embedded RAG chatbot.

TECHNOLOGY STACK:
- Frontend: Docusaurus with React and TypeScript
- Backend: FastAPI with Python
- Database: Neon Serverless Postgres
- Vector Database: Qdrant Cloud (free tier)
- AI: OpenAI SDK (GPT-4 and Embeddings API)
- Deployment: GitHub Pages (frontend) and Railway/Vercel (backend)

CORE PRINCIPLES:
1. Production-ready code with proper error handling
2. TypeScript for type safety on frontend
3. Mobile-responsive design
4. Clear documentation and comments
5. Modular, reusable components
6. RESTful API design
7. Secure environment variable management

DELIVERABLES:
1. Complete Docusaurus book with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
2. RAG chatbot that answers questions from book content
3. Text selection feature for contextual queries
4. Deployed and working production site
5. 90-second demo video

CONSTRAINTS:
- Deadline: Tonight 6 PM
- Must be deployable to GitHub Pages
- All APIs must be secure and rate-limited
- Code must follow best practices

OPTIONAL BONUS FEATURES:
- Authentication with better-auth.com
- Content personalization
- Urdu translation
- Claude Code subagents

## Response snapshot

Created the project constitution at `.specify/memory/constitution.md` with version 1.0.0. The constitution includes the project's purpose, technology stack, core principles, deliverables, and constraints as provided. No dependent templates required modification.

## Outcome

- ✅ Impact: High - Establishes foundational governance for the project.
- 🧪 Tests: N/A - No tests were run for this operation.
- 📁 Files: 1 file created/modified - `.specify/memory/constitution.md`.
- 🔁 Next prompts: User can now proceed with other setup commands like `/sp.plan` or `/sp.specify`.
- 🧠 Reflection: The process was smooth. The user provided all necessary information, which was directly translated into the constitution file.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): N/A
