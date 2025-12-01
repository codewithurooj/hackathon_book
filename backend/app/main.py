"""
FastAPI main application
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.routers import health, chat
from app.utils.config import settings
import structlog

# Configure structured logging
structlog.configure(
    processors=[
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.JSONRenderer()
    ]
)
logger = structlog.get_logger()

# Create FastAPI app
app = FastAPI(
    title="Physical AI & Humanoid Robotics Book API",
    description="RAG-powered chatbot API for Physical AI & Humanoid Robotics Book",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health.router, prefix="/api/v1")
app.include_router(chat.router, prefix="/api/v1")

# Startup event
@app.on_event("startup")
async def startup_event():
    logger.info(
        "application_startup",
        app_name="Physical AI & Humanoid Robotics Book API",
        environment=settings.ENVIRONMENT,
        cors_origins=settings.cors_origins_list
    )

# Root endpoint
@app.get("/")
async def root():
    return {
        "message": "Physical AI & Humanoid Robotics Book API",
        "docs": "/docs",
        "health": "/api/health"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
