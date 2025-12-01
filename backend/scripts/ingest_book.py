"""
Script to ingest Physical AI textbook content into Qdrant
"""
import sys
import os
import asyncio
from pathlib import Path
import re
import frontmatter

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.embedding_service import EmbeddingService
from app.services.qdrant_service import QdrantService
import structlog

logger = structlog.get_logger()


def extract_chapter_info(file_path):
    """Extract chapter title and content from a markdown file"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract the first heading as title if it exists
    lines = content.split('\n')
    title = None
    for line in lines:
        if line.startswith('# '):
            title = line[2:].strip()  # Remove '# ' prefix
            break
    
    if not title:
        # Use filename as title if no heading found
        title = file_path.stem.replace('-', ' ').title()
    
    # Convert file path to URL - assuming docs are served from /docs
    relative_path = file_path.relative_to(Path(__file__).parent.parent.parent / 'docs')
    url = f"/docs/{relative_path.with_suffix('').as_posix().replace(os.sep, '-')}"
    
    return {
        "title": title,
        "url": url,
        "content": content.strip()
    }


def get_all_chapter_files(docs_path):
    """Get all markdown files from the docs directory"""
    docs_dir = Path(docs_path)
    chapter_files = []
    
    for md_file in docs_dir.rglob('*.md'):
        # Skip the index files in each chapter directory as they might be duplicates
        if 'index.md' in str(md_file) and len(str(md_file).split(os.sep)) > 2:
            continue
        chapter_files.append(md_file)
    
    return chapter_files


async def ingest_content():
    """Ingest Physical AI textbook content into Qdrant"""
    try:
        logger.info("ingestion_started")

        # Initialize services
        embedding_service = EmbeddingService()
        qdrant_service = QdrantService()

        # Create collection
        logger.info("creating_collection")
        await qdrant_service.create_collection(vector_size=1536)

        # Get all markdown files from docs directory
        docs_path = Path(__file__).parent.parent.parent / 'docs'
        chapter_files = get_all_chapter_files(docs_path)
        
        logger.info("found_chapter_files", count=len(chapter_files))

        # Extract content from each file
        chapters = []
        for file_path in chapter_files:
            try:
                chapter_info = extract_chapter_info(file_path)
                chapters.append(chapter_info)
                logger.info("processed_chapter", title=chapter_info["title"])
            except Exception as e:
                logger.error("failed_to_process_chapter", file=str(file_path), error=str(e))

        if not chapters:
            logger.warning("no_chapters_found", docs_path=str(docs_path))
            print(f"No chapters found in {docs_path}")
            return

        # Generate embeddings for all chapters
        logger.info("generating_embeddings", chapters=len(chapters))
        texts = [chapter["content"] for chapter in chapters]
        embeddings = await embedding_service.generate_embeddings_batch(texts)

        # Prepare payloads
        payloads = [
            {
                "title": chapter["title"],
                "url": chapter["url"],
                "content": chapter["content"].strip()
            }
            for chapter in chapters
        ]

        # Upsert to Qdrant
        logger.info("upserting_to_qdrant", points=len(embeddings))
        await qdrant_service.upsert_points(embeddings, payloads)

        logger.info("ingestion_completed", total_chapters=len(chapters))
        print(f"\nSuccessfully ingested {len(chapters)} chapters into Qdrant!")
        print("Chapters processed:")
        for chapter in chapters:
            print(f"  - {chapter['title']} ({chapter['url']})")

    except Exception as e:
        logger.error("ingestion_failed", error=str(e))
        print(f"\nIngestion failed: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    # Run ingestion
    asyncio.run(ingest_content())