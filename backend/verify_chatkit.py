"""
Quick verification script for ChatKit integration.
Run this to verify all components are properly set up.
"""

import sys
import os

# Add the project root to the Python path
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir))
sys.path.insert(0, project_root)


def check_environment():
    """Check if all required environment variables are set."""
    print("[*] Checking environment variables...")
    required_vars = [
        "BACKEND_API_KEY",
        "OPENAI_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY",
        "DATABASE_URL",
    ]

    missing = []
    for var in required_vars:
        if not os.getenv(var):
            missing.append(var)
            print(f"  [X] {var} - NOT SET")
        else:
            # Show first/last few chars for verification
            value = os.getenv(var)
            if len(value) > 10:
                masked = f"{value[:4]}...{value[-4:]}"
            else:
                masked = "***"
            print(f"  [OK] {var} - {masked}")

    if missing:
        print(f"\n[X] Missing variables: {', '.join(missing)}")
        print("Please set these in your .env file")
        return False

    print("\n[OK] All environment variables are set!")
    return True


def check_dependencies():
    """Check if all required Python packages are installed."""
    print("\n[*] Checking Python dependencies...")
    required_packages = [
        "fastapi",
        "openai",
        "qdrant_client",
        "psycopg2",
        "sqlalchemy",
        "uvicorn",
    ]

    missing = []
    for package in required_packages:
        try:
            __import__(package.replace("-", "_"))
            print(f"  [OK] {package}")
        except ImportError:
            missing.append(package)
            print(f"  [X] {package} - NOT INSTALLED")

    if missing:
        print(f"\n[X] Missing packages: {', '.join(missing)}")
        print("Run: pip install -r requirements.txt")
        return False

    print("\n[OK] All dependencies are installed!")
    return True


def check_database_connection():
    """Check database connection."""
    print("\n[*] Checking database connection...")
    try:
        from backend.app.config import settings

        if not settings.DATABASE_URL:
            print("  [X] DATABASE_URL not set")
            return False

        from backend.app.database import engine
        from sqlalchemy import text

        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            if result.fetchone():
                print("  [OK] Database connection successful!")
                return True
    except Exception as e:
        print(f"  [X] Database connection failed: {e}")
        return False


def check_qdrant_connection():
    """Check Qdrant connection."""
    print("\n[*] Checking Qdrant connection...")
    try:
        from qdrant_client import QdrantClient
        from backend.app.config import settings

        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            print("  [X] Qdrant credentials not set")
            return False

        client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
        collections = client.get_collections()
        print(f"  [OK] Qdrant connection successful!")
        print(f"  [INFO] Collections: {[c.name for c in collections.collections]}")

        # Check if our collection exists and has data
        from backend.app.config import settings

        collection_name = settings.QDRANT_COLLECTION_NAME
        if collection_name in [c.name for c in collections.collections]:
            count = client.count(collection_name=collection_name)
            print(f"  [INFO] Collection '{collection_name}' has {count.count} vectors")
            if count.count == 0:
                print(
                    "  [WARN] Collection is empty. Run: python -m backend.scripts.ingest"
                )
        else:
            print(f"  [WARN] Collection '{collection_name}' not found")
            print("  Run: python -m backend.scripts.ingest")

        return True
    except Exception as e:
        print(f"  [X] Qdrant connection failed: {e}")
        return False


def check_openai_connection():
    """Check OpenAI API connection."""
    print("\n[*] Checking OpenAI connection...")
    try:
        from openai import OpenAI
        from backend.app.config import settings

        if not settings.OPENAI_API_KEY:
            print("  [X] OPENAI_API_KEY not set")
            return False

        client = OpenAI(api_key=settings.OPENAI_API_KEY)
        # Test with a simple API call
        models = client.models.list()
        print("  [OK] OpenAI connection successful!")
        return True
    except Exception as e:
        print(f"  [X] OpenAI connection failed: {e}")
        return False


def check_chatkit_files():
    """Check if ChatKit files exist."""
    print("\n[*] Checking ChatKit integration files...")

    # Construct paths relative to the project root (one level up from backend)
    project_root = os.path.dirname(os.path.dirname(__file__))

    files_to_check = [
        ("Backend ChatKit endpoint", os.path.join(project_root, "backend", "app", "api", "chatkit.py")),
        ("Frontend ChatKit component", os.path.join(project_root, "website", "src", "components", "ChatbotChatKit", "index.js")),
        ("Frontend ChatKit styles", os.path.join(project_root, "website", "src", "components", "ChatbotChatKit", "index.module.css")),
        ("Root component", os.path.join(project_root, "website", "src", "theme", "Root.js")),
    ]

    all_exist = True
    for name, path in files_to_check:
        if os.path.exists(path):
            print(f"  [OK] {name}")
        else:
            print(f"  [X] {name} - NOT FOUND at {path}")
            all_exist = False

    if all_exist:
        print("\n[OK] All ChatKit files are in place!")
    return all_exist


def main():
    """Run all verification checks."""
    print("=" * 60)
    print("ChatKit Integration Verification")
    print("=" * 60)

    checks = [
        ("Environment Variables", check_environment),
        ("Python Dependencies", check_dependencies),
        ("ChatKit Files", check_chatkit_files),
        ("Database Connection", check_database_connection),
        ("Qdrant Connection", check_qdrant_connection),
        ("OpenAI Connection", check_openai_connection),
    ]

    results = {}
    for name, check_func in checks:
        try:
            results[name] = check_func()
        except Exception as e:
            print(f"\n[X] {name} check failed with error: {e}")
            results[name] = False

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)

    for name, passed in results.items():
        status = "[PASS]" if passed else "[FAIL]"
        print(f"{status} - {name}")

    all_passed = all(results.values())

    print("\n" + "=" * 60)
    if all_passed:
        print("[SUCCESS] All checks passed! ChatKit is ready to use.")
        print("\nNext steps:")
        print("1. Start backend: uvicorn backend.app.main:app --reload")
        print("2. Start frontend: cd website && npm start")
        print("3. Visit http://localhost:3000 and test the chatbot")
    else:
        print("[WARNING] Some checks failed. Please fix the issues above.")
        print("\nSee CHATKIT_SETUP.md for detailed setup instructions.")

    print("=" * 60)

    return 0 if all_passed else 1


if __name__ == "__main__":
    # Load environment variables from .env if available
    try:
        from dotenv import load_dotenv

        load_dotenv()
    except ImportError:
        print(
            "[WARN] python-dotenv not installed. Make sure to set environment variables manually.\n"
        )

    sys.exit(main())
