from openai import OpenAI
from ..utils.config import settings

openai_client: OpenAI = None

async def get_openai_client() -> OpenAI:
    """
    Initializes and returns an OpenAI client instance.
    """
    global openai_client
    if openai_client is None:
        if not settings.OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY must be set in environment variables.")
        openai_client = OpenAI(api_key=settings.OPENAI_API_KEY)
    return openai_client

async def generate_embedding(text: str) -> list[float]:
    """
    Generates an embedding for the given text using the configured OpenAI model.
    """
    client = await get_openai_client()
    response = client.embeddings.create(
        input=text,
        model=settings.EMBEDDING_MODEL
    )
    return response.data[0].embedding
