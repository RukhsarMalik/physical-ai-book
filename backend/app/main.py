import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize FastAPI
app = FastAPI(
    title="Physical AI Chatbot API",
    description="RAG Chatbot for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# ✅ FINAL FIXED CORS CONFIG — DO NOT CHANGE
allowed_origins = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Root endpoint
@app.get("/")
def read_root():
    return {
        "message": "Welcome to the RAG Chatbot API!",
        "version": "1.0.0",
        "docs": "/docs",
        "health": "/api/health",
        "endpoints": {
            "chat": "/api/chat",
            "chat_selection": "/api/chat/selection",
            "embeddings": "/api/embeddings/generate",
            "agent_status": "/api/agent/status"
        }
    }

# Import and include API routers
try:
    from app.api import embeddings
    from app.api import chat
    from app.api import monitoring

    app.include_router(embeddings.router, prefix="/api", tags=["Embeddings"])
    app.include_router(chat.router, prefix="/api", tags=["Chat"])
    app.include_router(monitoring.router, prefix="/api", tags=["Monitoring", "Agent", "Health"])

    print("✅ All routers loaded successfully")

except ImportError as e:
    print(f"⚠️ Error loading routers: {e}")
    print("Some endpoints may not be available")


# Startup message
@app.on_event("startup")
async def startup_event():
    print("=" * 60)
    print("🚀 Physical AI Chatbot API Starting...")
    print("=" * 60)
    print("📝 Docs:          http://localhost:8000/docs")
    print("🏥 Health Check:  http://localhost:8000/api/health")
    print("💬 Chat API:      http://localhost:8000/api/chat")
    print(f"🔐 CORS Allowed:  {allowed_origins}")
    print("=" * 60)


# Shutdown message
@app.on_event("shutdown")
async def shutdown_event():
    print("👋 Shutting down Physical AI Chatbot API...")


# Run with: uvicorn app.main:app --reload
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
