from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import settings

app = FastAPI(
    title=settings.APP_NAME,
    description=settings.APP_DESCRIPTION, # Assuming APP_DESCRIPTION is added to settings
    version=settings.APP_VERSION,
    debug=settings.DEBUG
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/", tags=["Health"])
async def read_root():
    return {"message": f"{settings.APP_NAME} is running!"}

# Placeholder for API routers
# from .api import chat, embeddings, agent
# app.include_router(chat.router)
# app.include_router(embeddings.router)
# app.include_router(agent.router)
