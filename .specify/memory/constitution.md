<!--
---
Sync Impact Report
---
Version change: 0.0.0 → 1.0.0
Modified principles:
- Principle 1: "PRINCIPLE_1_NAME" → "I. Production-Ready Code"
- Principle 2: "PRINCIPLE_2_NAME" → "II. TypeScript Type Safety"
- Principle 3: "PRINCIPLE_3_NAME" → "III. Mobile-Responsive Design"
- Principle 4: "PRINCIPLE_4_NAME" → "IV. Clear Documentation"
- Principle 5: "PRINCIPLE_5_NAME" → "V. Modular & Reusable Components"
Added sections:
- Principle 6: "VI. RESTful API Design"
- Principle 7: "VII. Secure Environment Management"
- "Technology Stack"
- "Project Deliverables & Constraints"
- "Optional Bonus Features"
Templates requiring updates:
- ✅ .specify/templates/plan-template.md
- ✅ .specify/templates/spec-template.md
- ✅ .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Production-Ready Code
All code must be production-ready with proper error handling.

### II. TypeScript Type Safety
Utilize TypeScript for strict type safety on the frontend to minimize runtime errors.

### III. Mobile-Responsive Design
All frontend components and pages must be fully responsive and functional on mobile devices.

### IV. Clear Documentation
Document all code, especially complex logic and public APIs, and maintain clear, in-code comments where necessary.

### V. Modular & Reusable Components
Design components and modules to be reusable and independent to promote a scalable and maintainable codebase.

### VI. RESTful API Design
Adhere to RESTful principles for all API endpoints, ensuring predictable and standardized communication.

### VII. Secure Environment Management
Manage all secrets and environment variables securely, ensuring they are never hardcoded or committed to version control.

## Technology Stack
- **Frontend**: Docusaurus with React and TypeScript
- **Backend**: FastAPI with Python
- **Database**: Neon Serverless Postgres
- **Vector Database**: Qdrant Cloud (free tier)
- **AI**: OpenAI SDK (GPT-4 and Embeddings API)
- **Deployment**: GitHub Pages (frontend) and Railway/Vercel (backend)

## Project Deliverables & Constraints

### Deliverables
1. Complete Docusaurus book with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA).
2. RAG chatbot that answers questions from book content.
3. Text selection feature for contextual queries.
4. Deployed and working production site.
5. 90-second demo video.

### Constraints
- Deadline: Tonight 6 PM.
- Must be deployable to GitHub Pages.
- All APIs must be secure and rate-limited.
- Code must follow best practices.

## Optional Bonus Features
- Authentication with better-auth.com
- Content personalization
- Urdu translation
- Claude Code subagents

## Governance
This constitution is the single source of truth for project standards. All development and review processes must adhere to these principles. Amendments require team consensus and documentation.

**Version**: 1.0.0 | **Ratified**: 2025-11-29 | **Last Amended**: 2025-11-29