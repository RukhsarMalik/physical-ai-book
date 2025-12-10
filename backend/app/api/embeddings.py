from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Dict
from pydantic import BaseModel
from ..services.embeddings_service import generate_embeddings_for_docs

router = APIRouter()

class EmbeddingGenerateRequest(BaseModel):
    force_refresh: bool = False

class EmbeddingGenerateResponse(BaseModel):
    status: str
    message: str
    documents_embedded: int = 0  # Will be updated when background task completes

@router.post("/embeddings/generate", response_model=EmbeddingGenerateResponse)
async def generate_embeddings(
    background_tasks: BackgroundTasks, 
    request: EmbeddingGenerateRequest = EmbeddingGenerateRequest()
):
    """
    Triggers the generation/refresh of book content embeddings.
    The process runs in the background to avoid blocking the API response.
    """
    try:
        # Run the embedding generation in a background task
        background_tasks.add_task(generate_embeddings_for_docs, request.force_refresh)
        
        return EmbeddingGenerateResponse(
            status="processing",
            message="Embedding generation started in the background.",
            documents_embedded=0  # Will be updated asynchronously
        )
    except Exception as e:
        raise HTTPException(
            status_code=500, 
            detail=f"Failed to start embedding generation: {str(e)}"
        )


# Optional: Add an endpoint to check embedding status
@router.get("/embeddings/status")
async def get_embedding_status():
    """
    Check the status of embeddings in Qdrant.
    """
    try:
        from ..services.qdrant_service import get_collection_info
        
        info = get_collection_info()
        return {
            "status": "ready",
            "total_documents": info.get("points_count", 0),
            "collection_name": "book_content"
        }
    except Exception as e:
        return {
            "status": "error",
            "message": str(e)
        }