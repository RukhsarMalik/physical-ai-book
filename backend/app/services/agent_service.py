import os
import json
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from openai import OpenAI

from .qdrant_service import search_vectors
from .embeddings_service import get_embedding

load_dotenv()

openai_client = OpenAI()

# UPDATED: More specific system prompt for selection-based questions
AGENT_PERSONA = """
You are an AI Teaching Assistant for the textbook “Physical AI & Humanoid Robotics”.
Your primary and ONLY source of information is the book content provided to you as context.
You must not use any external knowledge or information you were trained on.

The user is reading the book and may select a passage of text. You will often receive:

- selected_passage: text that the user highlighted in the book
- retrieved_chunks: relevant excerpts from the rest of the book (via RAG)
- user_message: what the user typed

Your behavior:

1.  **If `selected_passage` is provided and non-empty:**
    - First, briefly restate or summarize the selected passage in simple language.
    - Then, use BOTH the `selected_passage` and `retrieved_chunks` to answer the user’s question.
    - Your answer must be directly connected to the provided passages. Quote or reference them when useful.

2.  **If there is no `selected_passage`:**
    - Act as a RAG tutor for the whole book, synthesizing answers from the `retrieved_chunks`.

Always:
- Be precise and grounded exclusively in the book content and provided context.
- If a direct answer cannot be found in the provided `selected_passage` or `retrieved_chunks`, but relevant information exists, synthesize a concise answer from that information.
- If absolutely no relevant information can be found in the provided context, state "Based on the provided text, I cannot find sufficient information to answer that question." Do NOT provide external knowledge.
- Use step-by-step reasoning when explaining technical concepts.
- When useful, refer to sections/chapters (if IDs or titles are provided in the context).
- Never invent page numbers or section names that are not present in the context.
- Keep answers concise but clear.
"""

AGENT_MODEL = os.getenv("OPENAI_AGENT_MODEL", "gpt-4o-mini")

# ------------------ RAG SEARCH ------------------


def search_book_content(query: str):
    embedding = get_embedding(query)
    if not embedding:
        return []

    hits = search_vectors(query_vector=embedding, limit=5)

    return [
        {
            "text": h.payload.get("text"),
            "module": h.payload.get("module"),
            "page": h.payload.get("page"),
            "title": h.payload.get("title"),
            "source": h.payload.get("source"),
            "score": h.score,
        }
        for h in hits
    ]


def get_module_info(module_name: str):
    return {"message": f"No metadata available for module '{module_name}'."}


def get_code_examples(topic: str):
    embedding = get_embedding(topic)
    if not embedding:
        return []

    hits = search_vectors(query_vector=embedding, limit=5)

    return [
        {
            "code": h.payload.get("code_blocks"),
            "context": h.payload.get("text"),
            "module": h.payload.get("module"),
            "page": h.payload.get("page")
        }
        for h in hits if h.payload.get("code_blocks")
    ]


# ------------------ MASTER AGENT CLASS ------------------


class OpenAIAgent:
    def __init__(self):
        self.client = openai_client
        self.thread = None
        self.assistant = None

        # tools
        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "search_book_content",
                    "description": "Search the book for relevant content.",
                    "parameters": {
                        "type": "object",
                        "properties": {"query": {"type": "string"}},
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_module_info",
                    "description": "Fetch module metadata.",
                    "parameters": {
                        "type": "object",
                        "properties": {"module_name": {"type": "string"}},
                        "required": ["module_name"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "get_code_examples",
                    "description": "Fetch code examples for a topic.",
                    "parameters": {
                        "type": "object",
                        "properties": {"topic": {"type": "string"}},
                        "required": ["topic"]
                    }
                }
            }
        ]

        self.initialize_agent()

    def initialize_agent(self):
        if not self.assistant:
            self.assistant = self.client.beta.assistants.create(
                name="Physical AI RAG Assistant",
                instructions=AGENT_PERSONA,
                model=AGENT_MODEL,
                tools=self.tools
            )

    def get_or_create_thread(self, thread_id: Optional[str]):
        if thread_id:
            try:
                self.thread = self.client.beta.threads.retrieve(thread_id)
                return
            except Exception:
                # if invalid / expired, new thread create karlo
                pass
        self.thread = self.client.beta.threads.create()

    # Execute tools
    def _execute_tool(self, call):
        fn = call.function.name
        args = json.loads(call.function.arguments)

        if fn == "search_book_content":
            return search_book_content(**args)
        if fn == "get_module_info":
            return get_module_info(**args)
        if fn == "get_code_examples":
            return get_code_examples(**args)

        return {"error": "Unknown tool called"}

    # ------------------ MAIN CHAT HANDLER ------------------
    def chat_with_agent(
        self,
        user_message: str,
        selected_passage: Optional[str] = None,
        thread_id: Optional[str] = None
    ):
        self.get_or_create_thread(thread_id)

        # MESSAGE BUILDING
        if selected_passage:
            # When a selected passage is provided, prioritize it strongly
            final_msg = (
                f"The user has selected the following text from the book for explanation:\n\n"
                f"```\n{selected_passage}\n```\n\n"
                f"Based on this selected text, please respond to the user's query: '{user_message}'. "
                f"Ensure your explanation is clear, simple, provides meaning, summary, examples, and context. "
                f"Do NOT ask the user what text they want to explain, as it is provided."
            )
        else:
            final_msg = user_message

        # Add to thread
        self.client.beta.threads.messages.create(
            thread_id=self.thread.id,
            role="user",
            content=final_msg
        )

        # Start run
        run = self.client.beta.threads.runs.create(
            thread_id=self.thread.id,
            assistant_id=self.assistant.id
        )

        # wait for first phase
        while run.status in ("queued", "in_progress"):
            run = self.client.beta.threads.runs.retrieve(
                thread_id=self.thread.id,
                run_id=run.id
            )

        agent_thought = ""

        # Tool calling required
        if run.status == "requires_action":
            outputs = []

            for call in run.required_action.submit_tool_outputs.tool_calls:
                result = self._execute_tool(call)

                outputs.append({
                    "tool_call_id": call.id,
                    "output": json.dumps(result)
                })

                agent_thought += f"[TOOL] {call.function.name} → {result}\n"

            # submit back to OpenAI
            run = self.client.beta.threads.runs.submit_tool_outputs(
                thread_id=self.thread.id,
                run_id=run.id,
                tool_outputs=outputs
            )

            # 🔁 IMPORTANT: ab dobara wait karo jab tak final completion na ho jaye
            while run.status in ("queued", "in_progress"):
                run = self.client.beta.threads.runs.retrieve(
                    thread_id=self.thread.id,
                    run_id=run.id
                )

        # ab run COMPLETED hona chahiye, warna error
        if run.status != "completed":
            return {
                "answer": f"Run did not complete properly. Status: {run.status}",
                "sources": [],
                "session_id": self.thread.id,
                "agent_thought_process": agent_thought or None
            }

        # Fetch last response
        messages = self.client.beta.threads.messages.list(
            thread_id=self.thread.id,
            order="desc",
            limit=1
        )

        msg = messages.data[0]

        answer = ""
        sources = []

        # Correct OpenAI parsing
        for block in msg.content:
            if block.type == "text":
                answer += block.text.value

                for ann in getattr(block.text, "annotations", []):
                    if hasattr(ann, "file_citation"):
                        sources.append(ann.text)

        return {
            "answer": answer.strip(),
            "sources": list(set(sources)),
            "session_id": self.thread.id,
            "agent_thought_process": agent_thought or None
        }


rag_agent = OpenAIAgent()
