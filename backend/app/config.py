import os
from typing import List, Optional

class Settings:
    # Backend settings
    APP_NAME: str = "RAG Chatbot API"
    APP_DESCRIPTION: str = "Backend for the Physical AI & Humanoid Robotics RAG Chatbot"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = os.getenv("DEBUG", "False").lower() == "true"
    
    # OpenAI settings
    OPENAI_API_KEY: str = os.environ.get("OPENAI_API_KEY")
    OPENAI_AGENT_MODEL: str = os.getenv("OPENAI_AGENT_MODEL", "gpt-4-turbo")

    # Qdrant settings
    QDRANT_URL: str = os.environ.get("QDRANT_URL")
    QDRANT_API_KEY: str = os.environ.get("QDRANT_API_KEY")

    # Postgres settings
    NEON_DATABASE_URL: str = os.environ.get("NEON_DATABASE_URL")

    # CORS settings
    CORS_ORIGINS: List[str] = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(',')

    # Agent settings
    AGENT_TOOLS_ENABLED: bool = os.getenv("AGENT_TOOLS_ENABLED", "True").lower() == "true"

settings = Settings()