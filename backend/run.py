#!/usr/bin/env python3
"""
Entry point for running the FastAPI application on Render.
"""
import sys
import os

# Get the directory containing this script (backend/)
backend_dir = os.path.dirname(os.path.abspath(__file__))

# Add backend directory to the FRONT of sys.path
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Set PYTHONPATH for uvicorn's import process
os.environ['PYTHONPATH'] = backend_dir

# Import the app AFTER setting up the path
from app.main import app

import uvicorn

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8000))
    print(f"Starting server on port {port}")
    print(f"Backend dir: {backend_dir}")

    # Pass the app object directly instead of a string
    uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
