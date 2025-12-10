# Quick Start: RAG Chatbot Setup

This document provides a quick guide to setting up the RAG Chatbot backend and frontend components.

## 1. Backend Setup (FastAPI, PostgreSQL, Qdrant)

### 1.1 Python Environment Setup

It is highly recommended to use a Python virtual environment for dependency management.

```bash
# Navigate to the backend directory (e.g., physical-ai-book/backend)
# cd physical-ai-book/backend

# Create a virtual environment
python3 -m venv venv

# Activate the virtual environment
# On Linux/macOS:
source venv/bin/activate
# On Windows (PowerShell):
.\venv\Scripts\activate
```

### 1.2 Install Dependencies

Install all required Python packages.

```bash
pip install -r requirements.txt
# Or if requirements.txt is not yet created, install individually:
pip install fastapi uvicorn openai qdrant-client psycopg2-binary python-dotenv pydantic openai-agents chatkit-sdk
```
**Note**: You might need to install `openai-agents` or `chatkit-sdk` specifically, depending on which OpenAI SDK version and approach is used.

### 1.3 Configure Environment Variables

Create a `.env` file in your backend project root (e.g., `physical-ai-book/backend/.env`) and populate it with your credentials and configuration.

```dotenv
OPENAI_API_KEY="sk-YOUR_OPENAI_API_KEY"
OPENAI_AGENT_MODEL="gpt-4-turbo" # or gpt-3.5-turbo, etc.
QDRANT_URL="https://YOUR_QDRANT_CLUSTER_URL.qdrant.tech"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
NEON_DATABASE_URL="postgres://user:password@host:port/database"
CORS_ORIGINS="http://localhost:3000,https://your-frontend-domain.com" # Comma-separated list
AGENT_TOOLS_ENABLED="true" # Set to "false" to disable agent tools
```
**IMPORTANT**: Replace placeholder values with your actual API keys and URLs. Never commit your `.env` file to version control.

### 1.4 Initialize Database

Ensure your Neon PostgreSQL database is created and accessible via `NEON_DATABASE_URL`. Run any necessary database migration scripts (to be developed) to create the `chat_sessions`, `messages`, and `agent_memory` tables.

### 1.5 Initialize Qdrant Collection

Ensure your Qdrant cluster is running. The backend will typically handle collection creation (`book_content`) upon startup or via a specific endpoint call (`POST /api/embeddings/generate`).

### 1.6 Run Backend

```bash
# Ensure virtual environment is active
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
```
The backend API will be accessible at `http://localhost:8000`.

## 2. Frontend Integration (Docusaurus)

### 2.1 Install OpenAI ChatKit SDK (if used)

If using the OpenAI ChatKit SDK directly on the frontend, install it:

```bash
# Navigate to Docusaurus project root (e.g., physical-ai-book)
# cd physical-ai-book
npm install @openai/chatkit
```
**Note**: The decision on whether to use ChatKit on frontend or backend will dictate this step. If the backend fully manages the agent, this step might be skipped.

### 2.2 Configure API URL

The frontend (Docusaurus React components) will need to know the URL of your deployed FastAPI backend. This can be configured via environment variables or a Docusaurus-specific configuration.

```javascript
// Example in a React component or configuration file
const BACKEND_API_URL = process.env.REACT_APP_BACKEND_API_URL || "http://localhost:8000/api";
```

### 2.3 Integrate Chat Components

Embed the custom React components (`ChatButton.tsx`, `ChatWindow.tsx`, etc.) into your Docusaurus site structure. The `ChatButton.tsx` is typically integrated in `src/theme/Root.tsx` to provide a site-wide floating button.

### 2.4 Test Frontend

Start your Docusaurus development server:

```bash
# Navigate to Docusaurus project root (e.g., physical-ai-book)
# cd physical-ai-book
npm start
```
Verify that the chat button appears, the chat window opens, and you can send messages to the backend.

## 3. Generate Embeddings

After the backend is running, trigger the embedding generation process to populate your Qdrant collection with book content.

```bash
# Example using curl (or any HTTP client)
curl -X POST "http://localhost:8000/api/embeddings/generate" -H "Content-Type: application/json" -d "{}"
```
This will read your Docusaurus `docs/` content, chunk it, embed it, and store it in Qdrant. This step is crucial for the RAG chatbot to function.
