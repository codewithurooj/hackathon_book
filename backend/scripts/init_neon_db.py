import os
import sys
from sqlalchemy import create_engine, text
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the project root to the Python path if not already present
script_dir = os.path.dirname(__file__)
project_root = os.path.abspath(os.path.join(script_dir, os.pardir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Check if DATABASE_URL is set
database_url = os.getenv("DATABASE_URL")
if not database_url:
    print("Error: DATABASE_URL environment variable is not set.")
    sys.exit(1)

# Read the SQL script
try:
    with open(os.path.join(script_dir, "init_db.sql"), "r") as f:
        sql_script = f.read()
except FileNotFoundError:
    print(f"Error: init_db.sql not found in {script_dir}.")
    sys.exit(1)

# Connect to the database and execute the script
try:
    engine = create_engine(database_url)
    with engine.connect() as connection:
        connection.execute(text(sql_script))
        connection.commit()
    print("Successfully initialized Neon database with chat_history table.")
except Exception as e:
    print(f"Error initializing database: {e}")
    sys.exit(1)

