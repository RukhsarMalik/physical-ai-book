from fastapi import APIRouter, HTTPException, status
from typing import Dict, List, Optional
from ..services.agent_service import rag_agent
from ..models.database import SessionLocal
from ..services.qdrant_service import client as qdrant_client
from openai import OpenAI
from pydantic import BaseModel
import os
from sqlalchemy import text

router = APIRouter()

class ServiceStatus(BaseModel):
    status: str
    message: Optional[str] = None

class HealthStatus(BaseModel):
    status: str
    services: Dict[str, ServiceStatus]

class AgentStatus(BaseModel):
    agent_initialized: bool
    tools_loaded: List[str]
    message: Optional[str] = None

@router.get("/health", response_model=HealthStatus, tags=["Health"])
async def get_health_status():
    """
    Check health status of all integrated services.
    Returns 'healthy' if all services are up, 'degraded' if any service is down.
    """
    overall_status = "healthy"
    services_status: Dict[str, ServiceStatus] = {}

    # Check PostgreSQL connection
    db = None
    try:
        db = SessionLocal()
        db.execute(text("SELECT 1"))
        services_status["postgres"] = ServiceStatus(status="healthy")
    except Exception as e:
        overall_status = "degraded"
        services_status["postgres"] = ServiceStatus(
            status="unhealthy", 
            message=f"Database connection failed: {str(e)[:100]}"
        )
    finally:
        if db is not None:
            db.close()

    # Check Qdrant connection
    try:
        collections = qdrant_client.get_collections()
        services_status["qdrant"] = ServiceStatus(
            status="healthy",
            message=f"Connected. Collections: {len(collections.collections)}"
        )
    except Exception as e:
        overall_status = "degraded"
        services_status["qdrant"] = ServiceStatus(
            status="unhealthy", 
            message=f"Qdrant connection failed: {str(e)[:100]}"
        )

    # Check OpenAI API connectivity
    try:
        if not os.getenv("OPENAI_API_KEY"):
            raise ValueError("OPENAI_API_KEY environment variable not set")
        
        openai_test_client = OpenAI()
        models = openai_test_client.models.list()
        services_status["openai"] = ServiceStatus(
            status="healthy",
            message="API key valid"
        )
    except Exception as e:
        overall_status = "degraded"
        services_status["openai"] = ServiceStatus(
            status="unhealthy", 
            message=f"OpenAI API error: {str(e)[:100]}"
        )
    
    # Check OpenAI Agent SDK initialization
    try:
        if rag_agent.assistant:
            agent_tools = [tool.get("function", {}).get("name", "unknown") for tool in rag_agent.tools]
            services_status["agent_sdk"] = ServiceStatus(
                status="healthy",
                message=f"Tools loaded: {', '.join(agent_tools)}"
            )
        else:
            overall_status = "degraded"
            services_status["agent_sdk"] = ServiceStatus(
                status="unhealthy", 
                message="Agent not initialized"
            )
    except Exception as e:
        overall_status = "degraded"
        services_status["agent_sdk"] = ServiceStatus(
            status="unhealthy",
            message=f"Agent error: {str(e)[:100]}"
        )

    return HealthStatus(status=overall_status, services=services_status)


@router.get("/agent/status", response_model=AgentStatus, tags=["Agent"])
async def get_agent_status():
    """
    Get detailed status of the OpenAI Agent.
    Includes initialization status, loaded tools, and capabilities.
    """
    try:
        agent_initialized = rag_agent.assistant is not None
        
        if agent_initialized:
            tools_loaded = [
                tool.get("function", {}).get("name", "unknown") 
                for tool in rag_agent.tools
            ]
            message = f"Agent ready with {len(tools_loaded)} tools"
        else:
            tools_loaded = []
            message = "Agent not initialized. Check logs for errors."
        
        return AgentStatus(
            agent_initialized=agent_initialized,
            tools_loaded=tools_loaded,
            message=message
        )
    except Exception as e:
        return AgentStatus(
            agent_initialized=False,
            tools_loaded=[],
            message=f"Error checking agent status: {str(e)}"
        )