import asyncio
from app.services.qdrant_service import QdrantService

async def clear():
    q = QdrantService()
    await q.delete_collection()
    print('Collection deleted successfully!')

asyncio.run(clear())
