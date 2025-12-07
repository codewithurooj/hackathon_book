import asyncio
from app.services.qdrant_service import QdrantService
from app.services.embedding_service import EmbeddingService

async def test():
    qdrant = QdrantService()
    embed_service = EmbeddingService()

    embedding = await embed_service.generate_embedding('What is Physical AI?')
    results = await qdrant.search(embedding, limit=3)

    print('Sources from Qdrant:')
    for r in results:
        print(f"Title: {r['payload']['title']}")
        print(f"URL: {r['payload']['url']}")
        print()

asyncio.run(test())
