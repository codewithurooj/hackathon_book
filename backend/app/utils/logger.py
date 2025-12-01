# backend/app/utils/logger.py
import logging
import sys

def setup_logging():
    logging.basicConfig(
        level=logging.INFO, # Default logging level
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        handlers=[
            logging.StreamHandler(sys.stdout) # Log to stdout
        ]
    )
    # Optionally, configure specific loggers for libraries
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING) # Reduce noise from access logs
    logging.getLogger("uvicorn.error").setLevel(logging.INFO)
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.WARNING)
