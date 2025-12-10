import os
import uuid
from datetime import datetime
from sqlalchemy import create_engine, Column, String, DateTime, Text, Boolean, ForeignKey
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.orm import sessionmaker, declarative_base, relationship
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("NEON_DATABASE_URL")
if not DATABASE_URL:
    raise ValueError("NEON_DATABASE_URL environment variable is not set.")

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    agent_state = Column(JSONB, nullable=True) # Stores serializable state of the OpenAI agent

    messages = relationship("Message", back_populates="session")

class Message(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False) # Foreign key to chat_sessions.id - will add ForeignKeyConstraint later if needed
    role = Column(String, nullable=False) # user, assistant, system, tool
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow, nullable=False)
    tool_calls = Column(JSONB, nullable=True) # Stores details if the message involved agent tool calls

    session = relationship("ChatSession", back_populates="messages")

# Function to create all tables
def create_db_and_tables():
    Base.metadata.create_all(engine)

# Dependency to get the database session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

if __name__ == "__main__":
    print("Creating database tables...")
    create_db_and_tables()
    print("Tables created (if not already existing).")
