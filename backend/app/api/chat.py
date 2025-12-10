import re
from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.orm import Session
from typing import Optional, List
import uuid

from ..models.database import get_db, ChatSession, Message
from ..services.agent_service import rag_agent
from pydantic import BaseModel

router = APIRouter()


# ------------------ REQUEST & RESPONSE MODELS ------------------

class ChatRequest(BaseModel):
    message: str
    session_id: Optional[uuid.UUID] = None


class ChatResponse(BaseModel):
    answer: str
    sources: List[str]
    session_id: uuid.UUID
    agent_thought_process: Optional[str] = None


# ------------------ MAIN CHAT ENDPOINT ------------------

@router.post("/chat", response_model=ChatResponse)
async def chat_with_bot(chat_request: ChatRequest, db: Session = Depends(get_db)):

    session_id = chat_request.session_id
    user_id = "anonymous"

    # User input
    full_message = chat_request.message

    # ---------------------------------------
    # 1️⃣ Extract SELECTED TEXT in BOTH formats:
    #    A) Triple quotes:  SELECTED_TEXT: """ text """
    #    B) Single quotes:  SELECTED_TEXT: "text"
    # ---------------------------------------

    pattern = r"SELECTED_TEXT:\s*\"\"\"(.*?)\"\"\"|SELECTED_TEXT:\s*\"(.*?)\""
    match = re.search(pattern, full_message, re.DOTALL)

    selected_passage_for_agent = None
    user_message_for_agent = full_message  # default

    if match:
        # group(1) = triple-quoted text
        # group(2) = single-quoted text
        extracted_text = match.group(1) or match.group(2)
        selected_passage_for_agent = extracted_text.strip()

        # Override user message so agent focuses on the selected text
        user_message_for_agent = (
            "Explain the following selected text in clear, simple language. "
            "Provide meaning, summary, examples, and context. "
            "Do NOT ask me to re-enter or specify the text."
        )

    # ------------------ CREATE OR GET SESSION ------------------
    if session_id:
        chat_session = db.query(ChatSession).filter(ChatSession.id == session_id).first()

        if not chat_session:
            chat_session = ChatSession(id=session_id, user_id=user_id)
            db.add(chat_session)
            db.commit()
            db.refresh(chat_session)

    else:
        chat_session = ChatSession(user_id=user_id)
        db.add(chat_session)
        db.commit()
        db.refresh(chat_session)
        session_id = chat_session.id

    # ------------------ STORE USER MESSAGE ------------------
    stored_message_content = full_message
    if selected_passage_for_agent:
        stored_message_content = (
            f"[SELECTED TEXT]\n{selected_passage_for_agent}\n\n"
            f"[USER MESSAGE]\n{full_message}"
        )

    db.add(Message(
        session_id=session_id,
        role="user",
        content=stored_message_content
    ))
    db.commit()

    # ------------------ CALL AI AGENT ------------------
    try:
        agent_response = rag_agent.chat_with_agent(
            user_message=user_message_for_agent,
            selected_passage=selected_passage_for_agent,
            thread_id=str(session_id)
        )

        # Store assistant response
        db.add(Message(
            session_id=session_id,
            role="assistant",
            content=agent_response["answer"],
        ))
        db.commit()

        return ChatResponse(
            answer=agent_response["answer"],
            sources=agent_response["sources"],
            session_id=session_id,
            agent_thought_process=agent_response.get("agent_thought_process")
        )

    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(e))
