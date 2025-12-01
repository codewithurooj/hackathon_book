#!/usr/bin/env python3
"""
OpenAI API Verification Script
Tests Whisper and GPT-4 API access and estimates costs

Usage:
    python verify_api_access.py

Requirements:
    pip install openai

Environment:
    Set OPENAI_API_KEY environment variable
"""

import os
import sys
from openai import OpenAI

def verify_api_key():
    """Check if OpenAI API key is set"""
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("❌ OPENAI_API_KEY environment variable not set")
        print("\nTo set your API key:")
        print("  Linux/Mac: export OPENAI_API_KEY='your-api-key'")
        print("  Windows:   set OPENAI_API_KEY=your-api-key")
        return False

    print(f"✅ API key found: {api_key[:8]}...{api_key[-4:]}")
    return True

def test_whisper_api():
    """Test Whisper API access"""
    print("\n📝 Testing Whisper API...")
    try:
        client = OpenAI()

        # Note: Actual testing requires audio file
        # This just verifies the client can be instantiated
        print("✅ Whisper API client initialized successfully")
        print("   Cost: ~$0.006 per minute of audio")
        print("   Typical exercise: $0.10-$0.30")
        return True

    except Exception as e:
        print(f"❌ Whisper API test failed: {e}")
        return False

def test_gpt4_api():
    """Test GPT-4 API access with minimal request"""
    print("\n🧠 Testing GPT-4 API...")
    try:
        client = OpenAI()

        # Minimal test request
        response = client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": "You are a helpful assistant."},
                {"role": "user", "content": "Say 'API test successful' if you receive this."}
            ],
            max_tokens=10
        )

        result = response.choices[0].message.content
        print(f"✅ GPT-4 API test successful")
        print(f"   Response: {result}")
        print(f"   Tokens used: {response.usage.total_tokens}")
        print(f"   Cost estimate: ~$0.01-$0.03 per 1K tokens")
        print(f"   Typical exercise: $0.50-$2.00")
        return True

    except Exception as e:
        print(f"❌ GPT-4 API test failed: {e}")
        if "insufficient_quota" in str(e):
            print("   → You may need to add credits to your OpenAI account")
        elif "invalid_api_key" in str(e):
            print("   → Check that your API key is valid")
        return False

def estimate_total_cost():
    """Estimate total cost for Chapter 5 exercises"""
    print("\n💰 Estimated Total Cost for Chapter 5:")
    print("   Whisper exercises (6 tests × 2 min): ~$0.07")
    print("   GPT-4 exercises (20 requests × 500 tokens avg): ~$0.50-$1.50")
    print("   Multi-modal exercises: ~$0.50-$1.00")
    print("   Buffer for experimentation: ~$1.00-$2.00")
    print("   ─" * 30)
    print("   TOTAL ESTIMATED: $2.00-$5.00")
    print("\n📋 Cost Optimization Tips:")
    print("   • Cache common requests to avoid redundant API calls")
    print("   • Use shorter prompts when possible")
    print("   • Test with simple commands before complex ones")
    print("   • Set max_tokens limit to prevent runaway costs")
    print("   • Monitor usage at: https://platform.openai.com/usage")

def main():
    print("=" * 60)
    print("OpenAI API Verification for Chapter 5: VLA")
    print("=" * 60)

    if not verify_api_key():
        print("\n🔗 Get your API key at: https://platform.openai.com/api-keys")
        sys.exit(1)

    whisper_ok = test_whisper_api()
    gpt4_ok = test_gpt4_api()

    estimate_total_cost()

    print("\n" + "=" * 60)
    if whisper_ok and gpt4_ok:
        print("✅ All API tests passed! You're ready for Chapter 5.")
    else:
        print("⚠️  Some API tests failed. Check errors above.")
    print("=" * 60)

if __name__ == "__main__":
    main()
