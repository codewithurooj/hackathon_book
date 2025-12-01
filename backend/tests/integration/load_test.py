# backend/tests/integration/load_test.py
import httpx
import asyncio
import time
import os

# --- Configuration ---
BASE_URL = os.getenv("BACKEND_URL", "http://localhost:8000/api/v1")
NUM_REQUESTS = 100
CONCURRENCY = 10
API_KEY = os.getenv("BACKEND_API_KEY", "test-api-key") # Use a test API key

async def make_request(client: httpx.AsyncClient, endpoint: str, payload: dict):
    headers = {"X-API-Key": API_KEY}
    try:
        start_time = time.monotonic()
        response = await client.post(f"{BASE_URL}{endpoint}", json=payload, headers=headers, timeout=5)
        end_time = time.monotonic()
        latency = (end_time - start_time) * 1000 # milliseconds
        
        response.raise_for_status()
        print(f"Request to {endpoint} successful (Status: {response.status_code}, Latency: {latency:.2f}ms)")
        return latency, True
    except httpx.HTTPStatusError as e:
        print(f"Request to {endpoint} failed (Status: {e.response.status_code}): {e.response.text}")
        return 0, False
    except httpx.RequestError as e:
        print(f"Request to {endpoint} failed (Error: {e})")
        return 0, False

async def run_load_test():
    print(f"Starting load test on {BASE_URL} with {NUM_REQUESTS} requests and {CONCURRENCY} concurrency.")
    latencies = []
    successful_requests = 0
    total_requests = 0

    async with httpx.AsyncClient() as client:
        tasks = []
        for i in range(NUM_REQUESTS):
            endpoint = "/chat" # or "/chat/selected"
            payload = {"question": f"What is physical AI? Request {i+1}", "session_id": f"test-session-{i%CONCURRENCY}"}
            tasks.append(make_request(client, endpoint, payload))
            total_requests += 1

            if len(tasks) >= CONCURRENCY:
                results = await asyncio.gather(*tasks)
                for latency, success in results:
                    if success:
                        latencies.append(latency)
                        successful_requests += 1
                tasks = [] # Clear tasks for next batch

        # Process any remaining tasks
        if tasks:
            results = await asyncio.gather(*tasks)
            for latency, success in results:
                if success:
                    latencies.append(latency)
                    successful_requests += 1

    if latencies:
        avg_latency = sum(latencies) / len(latencies)
        p95_latency = sorted(latencies)[int(len(latencies) * 0.95)]
        print("\n--- Load Test Results ---")
        print(f"Total Requests: {total_requests}")
        print(f"Successful Requests: {successful_requests}")
        print(f"Failed Requests: {total_requests - successful_requests}")
        print(f"Success Rate: {(successful_requests / total_requests) * 100:.2f}%")
        print(f"Average Latency: {avg_latency:.2f}ms")
        print(f"P95 Latency: {p95_latency:.2f}ms")
    else:
        print("No successful requests to report latency.")

if __name__ == "__main__":
    # Ensure environment variables are loaded if running directly
    # from dotenv import load_dotenv
    # load_dotenv() # for BACKEND_URL, BACKEND_API_KEY
    asyncio.run(run_load_test())
