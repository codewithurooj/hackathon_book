"""
Test script to verify selected text functionality in RAG chatbot
"""
import asyncio
import sys
import os
from unittest.mock import patch, MagicMock

# Add the backend directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

# Mock the environment variables before importing the services
os.environ['OPENAI_API_KEY'] = 'fake-key-for-test'
os.environ['QDRANT_URL'] = 'https://fake-url.qdrant.io'
os.environ['QDRANT_API_KEY'] = 'fake-qdrant-key-for-test'

async def test_selected_text_functionality():
    """
    Test the selected text functionality in the RAG service using mocks
    """
    print("Testing selected text functionality...")

    # Import required modules after setting environment variables
    try:
        from app.services.rag_service import RAGService
        print("+ Successfully imported RAGService")
    except ImportError as e:
        print(f"- Failed to import RAGService: {e}")
        return False

    # Use mocks to bypass external service dependencies
    with patch('app.services.embedding_service.EmbeddingService') as mock_embedding, \
         patch('app.services.qdrant_service.QdrantService') as mock_qdrant, \
         patch('app.services.agent_service.AgentService') as mock_agent:

        # Setup mock returns
        mock_embedding_instance = MagicMock()
        mock_embedding_instance.generate_embedding.return_value = [0.1] * 1536  # Mock embedding
        mock_embedding.return_value = mock_embedding_instance

        mock_qdrant_instance = MagicMock()
        mock_qdrant_instance.search.return_value = []  # No additional context for testing
        mock_qdrant.return_value = mock_qdrant_instance

        mock_agent_instance = MagicMock()
        mock_agent_instance.generate_answer.return_value = "Deep reinforcement learning is a combination of deep learning and reinforcement learning."
        mock_agent.return_value = mock_agent_instance

        try:
            # Initialize RAG service
            rag_service = RAGService()
            print("+ Successfully initialized RAGService with mocks")

            # Test context-based question answering
            selected_text = """
            Deep reinforcement learning combines deep learning and reinforcement learning to
            enable agents to learn complex behaviors from raw sensory inputs. The agent learns
            through trial and error by interacting with an environment to maximize cumulative reward.
            """

            question = "What is deep reinforcement learning?"

            answer, sources, confidence = await rag_service.answer_context_question(
                question=question,
                context=selected_text,
                top_k=2
            )

            print(f"+ Successfully processed context question")
            print(f"Question: {question}")
            print(f"Answer: {answer}")
            print(f"Number of sources: {len(sources)}")
            print(f"Confidence: {confidence}")

            # Verify that the primary source is the selected context
            if sources and sources[0].title == "Selected Context":
                print("+ Selected context properly added as primary source")
            else:
                print("- Selected context not properly added as primary source")
                return False

            return True

        except Exception as e:
            print(f"- Failed to process context question: {e}")
            import traceback
            traceback.print_exc()
            return False

async def test_chatkit_endpoint_model():
    """
    Test the updated ChatKit request model
    """
    print("\nTesting ChatKit endpoint model...")

    try:
        from app.routers.chat import ChatKitRequest
        print("+ Successfully imported ChatKitRequest")
    except ImportError as e:
        print(f"- Failed to import ChatKitRequest: {e}")
        return False

    # Test creating a request with selected_text
    try:
        request = ChatKitRequest(
            messages=[
                {"role": "user", "content": "What is deep reinforcement learning?"}
            ],
            selected_text="Deep reinforcement learning combines deep learning and reinforcement learning...",
            session_id="test_session"
        )

        print("+ Successfully created ChatKitRequest with selected_text")
        print(f"Selected text: {request.selected_text}")

        # Check that the new field exists
        if hasattr(request, 'selected_text'):
            print("+ selected_text field exists in ChatKitRequest")
        else:
            print("- selected_text field missing from ChatKitRequest")
            return False

        return True
    except Exception as e:
        print(f"- Failed to create ChatKitRequest with selected_text: {e}")
        return False

async def main():
    """
    Main test function
    """
    print("Running tests for selected text functionality...\n")

    test1_passed = await test_selected_text_functionality()
    test2_passed = await test_chatkit_endpoint_model()

    print(f"\nTest Results:")
    print(f"Context functionality test: {'PASS' if test1_passed else 'FAIL'}")
    print(f"ChatKit model test: {'PASS' if test2_passed else 'FAIL'}")

    if test1_passed and test2_passed:
        print("\n+ All tests passed! Selected text functionality is working correctly.")
        return True
    else:
        print("\n- Some tests failed. Please check the implementation.")
        return False

if __name__ == "__main__":
    result = asyncio.run(main())
    sys.exit(0 if result else 1)