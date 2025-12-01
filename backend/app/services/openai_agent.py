from openai import OpenAI
from backend.app.config import settings
from typing import List, Dict, Optional
from pydantic import BaseModel, Field
import json

class AgentResponse(BaseModel):
    answer: str = Field(..., description="The generated answer to the user's question.")
    sources: List[Dict] = Field(default_factory=list, description="A list of source documents used to generate the answer.")

async def generate_answer_from_context(question: str, context_chunks: List[Dict]) -> AgentResponse:
    """
    Generates an answer to the question using only the provided context chunks,
    and extracts source information in structured JSON.
    """
    client = OpenAI(api_key=settings.OPENAI_API_KEY)

    system_message = {
        "role": "system",
        "content": (
            "You are an expert AI assistant specializing in the Physical AI & Humanoid Robotics textbook. "
            "Your primary goal is to answer student questions accurately and concisely. "
            "You MUST use ONLY the information provided in the context below to formulate your answers. "
            "If the answer cannot be found in the context, state that you cannot answer based on the provided information. "
            "Do NOT use external knowledge.\n\n"
            "Format your response as a JSON object with two fields: 'answer' (string) and 'sources' (array of objects). "
            "Each object in the 'sources' array should have 'text_chunk', 'page_url', 'chapter', and 'section'. "
            "Ensure 'sources' only includes context chunks actually used to formulate the answer."
        ),
    }

    context_messages = []
    if context_chunks:
        for i, chunk in enumerate(context_chunks):
            # Assuming chunk is a dict with 'text_chunk', 'chapter', 'section', 'page_url'
            context_messages.append({
                "role": "system",
                "content": (
                    f"Context chunk {i+1} from {chunk.get('chapter', 'Unknown Chapter')} - {chunk.get('section', 'Unknown Section')}:\n"
                    f"{chunk.get('text_chunk', '')}\n"
                    f"Page URL: {chunk.get('page_url', 'N/A')}\n"
                    "---"
                )
            })
    else:
        # This case should ideally be handled before calling the agent, but as a safeguard:
        return AgentResponse(answer="I cannot answer this question as no relevant context was provided.", sources=[])

    user_message = {"role": "user", "content": question}

    messages = [system_message] + context_messages + [user_message]

    try:
        response = client.chat.completions.create(
            model="gpt-4o-mini", # Using a smaller model for cost-effectiveness and speed
            response_format={"type": "json_object"}, # Explicitly request JSON output
            messages=messages,
            temperature=0.0, # Aim for factual and less creative answers
            stream=False # For now, not streaming, streaming will be handled at the FastAPI endpoint
        )
        response_content = response.choices[0].message.content
        # Parse the JSON response
        agent_response_dict = json.loads(response_content)
        return AgentResponse(**agent_response_dict)

    except json.JSONDecodeError as e:
        print(f"Error decoding agent response JSON: {e}")
        return AgentResponse(answer="I am currently unable to process the answer due to a format error. Please try again later.", sources=[])
    except Exception as e:
        print(f"Error generating answer: {e}")
        return AgentResponse(answer="I am currently unable to generate an answer. Please try again later.", sources=[])
