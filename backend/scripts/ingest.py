import os
import asyncio
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from backend.app.config import settings
from backend.app.services.openai_service import generate_embedding
import logging
import sys

# Setup basic logging for the ingestion script
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# Path to your Docusaurus docs directory relative to the repository root
DOCS_PATH = "./docs"
QDRANT_COLLECTION_NAME = settings.QDRANT_COLLECTION_NAME

async def ingest_document(filepath: str, client: QdrantClient):
    """
    Reads a markdown file, processes it, generates embeddings, and uploads to Qdrant.
    """
    logger.info(f"Ingesting: {filepath}")
    
    with open(filepath, 'r', encoding='utf-8') as f:
        full_content = f.read()

    # Basic markdown parsing and chunking strategy: split by double newline
    # This is a rudimentary approach; a more sophisticated solution would use a library
    # like langchain.text_splitter.MarkdownTextSplitter
    content_chunks = [chunk.strip() for chunk in full_content.split('\n\n') if chunk.strip()]

    # Extract metadata from filepath
    relative_path = os.path.relpath(filepath, DOCS_PATH)
    path_parts = relative_path.split(os.sep)
    chapter_folder = path_parts[0] if path_parts else "Unknown Chapter"
    section_file = path_parts[1] if len(path_parts) > 1 else "Unknown Section"

    chapter_name = chapter_folder.replace('-', ' ').title()
    section_name = section_file.replace('.md', '').replace('-', ' ').title()
    page_url = f"/docs/{relative_path.replace(os.sep, '/')}"

    points_to_upsert = []
    for i, chunk_content in enumerate(content_chunks):
        if not chunk_content:
            continue

        # Generate embedding for each chunk
        try:
            embedding = await generate_embedding(chunk_content)
        except Exception as e:
            logger.error(f"Failed to generate embedding for chunk in {filepath} (Chunk {i+1}): {e}")
            continue

        # Prepare payload with required metadata
        payload = {
            "text_chunk": chunk_content,
            "page_url": page_url,
            "chapter": chapter_name,
            "section": section_name,
            "chunk_id": i # Unique ID for the chunk within the document
        }

        points_to_upsert.append(
            PointStruct(
                id=abs(hash(f"{filepath}-{i}")), # Unique ID for each chunk point
                vector=embedding,
                payload=payload,
            )
        )
    
    if points_to_upsert:
        client.upsert(
            collection_name=QDRANT_COLLECTION_NAME,
            wait=True,
            points=points_to_upsert
        )
        logger.info(f"Successfully ingested {len(points_to_upsert)} chunks from {filepath} into Qdrant.")
    else:
        logger.warning(f"No chunks to upsert from {filepath}.")


async def main():
    if not settings.QDRANT_URL or not settings.QDRANT_API_KEY or not settings.OPENAI_API_KEY:
        logger.error("Missing QDRANT_URL, QDRANT_API_KEY, or OPENAI_API_KEY environment variables. Exiting.")
        sys.exit(1) # Ensure sys is imported

    qdrant_client_instance = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY,
        timeout=60 # Increased timeout to 60 seconds
    )
    
    # Ensure collection exists before ingesting
    collections = qdrant_client_instance.get_collections()
    if QDRANT_COLLECTION_NAME not in [c.name for c in collections.collections]:
        logger.info(f"Creating Qdrant collection: {QDRANT_COLLECTION_NAME}")
        qdrant_client_instance.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=VectorParams(size=1536, distance=Distance.COSINE), # text-embedding-3-small dimensions
        )
    else:
        logger.info(f"Qdrant collection '{QDRANT_COLLECTION_NAME}' already exists.")


    for root, _, files in os.walk(DOCS_PATH):
        for file in files:
            if file.endswith(".md"):
                filepath = os.path.join(root, file)
                await ingest_document(filepath, qdrant_client_instance)

if __name__ == "__main__":
    # Ensure environment variables are loaded if running directly
    # from dotenv import load_dotenv
    # load_dotenv() # for BACKEND_URL, BACKEND_API_KEY
    import sys # Import sys for sys.exit
    asyncio.run(main())
