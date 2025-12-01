from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base
from backend.app.config import settings

# Ensure DATABASE_URL is set
if not settings.DATABASE_URL:
    raise ValueError("DATABASE_URL must be set in environment variables for database connection.")

# Create the SQLAlchemy engine
# pool_pre_ping=True helps with connection resilience, especially in serverless environments like Neon
engine = create_engine(settings.DATABASE_URL, pool_pre_ping=True)

# Configure a sessionmaker for interacting with the database
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for our models
Base = declarative_base()

# Dependency to get a DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
