# backend/app/api/middleware/rate_limiter.py
from fastapi import Request
from starlette.responses import JSONResponse
from starlette.middleware.base import BaseHTTPMiddleware
from collections import defaultdict
import time

# A very basic in-memory rate limiter for demonstration
# In a real-world scenario, use a dedicated library like FastAPI-Limiter with Redis
# Or an API Gateway for more robust rate limiting.

RATE_LIMIT_DURATION = 60 # seconds
RATE_LIMIT_REQUESTS = 10 # requests per duration

class RateLimitMiddleware(BaseHTTPMiddleware):
    def __init__(self, app):
        super().__init__(app)
        self.clients = defaultdict(lambda: {'last_reset': time.time(), 'request_count': 0})

    async def dispatch(self, request: Request, call_next):
        client_ip = request.client.host

        current_time = time.time()
        client_data = self.clients[client_ip]

        if current_time - client_data['last_reset'] > RATE_LIMIT_DURATION:
            client_data['last_reset'] = current_time
            client_data['request_count'] = 0

        if client_data['request_count'] >= RATE_LIMIT_REQUESTS:
            return JSONResponse(
                status_code=429,
                content={"detail": "Rate limit exceeded. Try again later."},
                headers={"Retry-After": str(RATE_LIMIT_DURATION)}
            )
        
        client_data['request_count'] += 1
        response = await call_next(request)
        return response
