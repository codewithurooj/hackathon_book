#!/usr/bin/env python3
"""
Entry point for running the FastAPI application on Render.
This script ensures proper module resolution for absolute imports.
"""
import sys
import os

# Get the directory containing this script (backend/)
backend_dir = os.path.dirname(os.path.abspath(__file__))

# Add backend directory to the FRONT of sys.path
# This ensures 'app' can be imported as a top-level module
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

# Also set PYTHONPATH environment variable for any subprocesses
os.environ['PYTHONPATH'] = backend_dir + os.pathsep + os.environ.get('PYTHONPATH', '')

# Now import and run uvicorn
import uvicorn

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8000))
    print(f"Starting server on port {port}")
    print(f"Python path: {sys.path[:3]}")  # Debug output
    uvicorn.run("app.main:app", host="0.0.0.0", port=port, log_level="info")
