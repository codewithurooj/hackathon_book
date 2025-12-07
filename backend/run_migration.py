"""
Run database migration for Urdu translation feature
"""
import asyncio
import asyncpg
import sys
from app.utils.config import settings

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

async def run_migration():
    print("Connecting to database...")
    print(f"Database: {settings.DATABASE_URL[:50]}...")

    try:
        # Connect to database
        conn = await asyncpg.connect(settings.DATABASE_URL)
        print("Connected successfully!")

        # Read migration file
        print("\nReading migration file...")
        with open('migrations/001_urdu_translations.sql', 'r', encoding='utf-8') as f:
            migration_sql = f.read()

        print("Migration file loaded")

        # Execute migration
        print("\nRunning migration...")
        await conn.execute(migration_sql)
        print("Migration executed successfully!")

        # Verify tables created
        print("\nVerifying tables...")
        tables = await conn.fetch("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            ORDER BY table_name;
        """)

        print("\nTables created:")
        for table in tables:
            print(f"   - {table['table_name']}")

        # Check if our tables exist
        table_names = [t['table_name'] for t in tables]
        if 'translations' in table_names and 'user_translation_points' in table_names:
            print("\nSUCCESS! All required tables are present!")
        else:
            print("\nWarning: Some tables might be missing")

        await conn.close()
        print("\nMigration completed successfully!")

    except Exception as e:
        print(f"\nError: {e}")
        print("\nTroubleshooting:")
        print("1. Check DATABASE_URL in .env file")
        print("2. Verify you can access Neon dashboard")
        print("3. Make sure database is active in Neon")
        return False

    return True

if __name__ == "__main__":
    success = asyncio.run(run_migration())
    if success:
        print("\n" + "="*60)
        print("Database setup complete!")
        print("="*60)
        print("\nNext steps:")
        print("1. Start backend: python -m uvicorn app.main:app --reload")
        print("2. Start frontend: cd ../website && npm start")
        print("3. In browser console: localStorage.setItem('user_id', '123')")
        print("4. Navigate to any chapter to see the translation button!")
    else:
        print("\nMigration failed. Please check the errors above.")
