# OpenAI API Cost Optimization Guide

**Purpose**: Minimize API costs for Chapter 5 VLA exercises while maintaining learning outcomes
**Target**: Keep total student costs under $5 for all exercises
**Status**: CRITICAL - Must be communicated upfront to students

---

## Cost Breakdown

### Whisper API Pricing

**Rate**: $0.006 per minute of audio

| Exercise | Audio Duration | Cost |
|----------|----------------|------|
| Exercise 1: Whisper Integration | 5 clips × 30s = 2.5 min | $0.015 |
| Exercise 4: VLA Pipeline | 10 commands × 20s = 3.3 min | $0.020 |
| Exercise 5: Multi-Modal | 8 commands × 30s = 4.0 min | $0.024 |
| Exercise 6: Capstone Demo | 1 demo × 3 min = 3.0 min | $0.018 |
| Testing & Experimentation | ~5 min extra | $0.030 |
| **Whisper Total** | **~18 minutes** | **~$0.11** |

### GPT-4 API Pricing

**Rate** (gpt-4-turbo):
- Input: $0.01 per 1K tokens
- Output: $0.03 per 1K tokens

**Estimated Token Usage**:

| Exercise | Requests | Avg Tokens/Request | Cost |
|----------|----------|-------------------|------|
| Exercise 2: GPT-4 Planning | 15 | 400 (300 in, 100 out) | $0.15 |
| Exercise 3: Action Translation | 20 | 300 (250 in, 50 out) | $0.12 |
| Exercise 4: VLA Pipeline | 12 | 500 (350 in, 150 out) | $0.18 |
| Exercise 5: Multi-Modal | 10 | 700 (500 in, 200 out) | $0.26 |
| Exercise 6: Capstone | 8 | 800 (600 in, 200 out) | $0.26 |
| Testing & Debugging | 30 | 400 | $0.48 |
| **GPT-4 Total** | **~95 requests** | **~45K tokens** | **~$1.45** |

### Total Estimated Cost

| Service | Cost |
|---------|------|
| Whisper | $0.11 |
| GPT-4 | $1.45 |
| **Subtotal** | **$1.56** |
| Buffer (errors, extra testing) | $0.50-$1.00 |
| **Realistic Total** | **$2.00-$2.50** |
| **Worst Case** (extensive experimentation) | **$5.00** |

---

## Cost Optimization Techniques

### 1. Caching Common Requests

**Principle**: Store LLM responses for identical or similar requests to avoid redundant API calls.

#### Simple Cache Implementation

```python
import hashlib
import json
import os

class ResponseCache:
    def __init__(self, cache_file="vla_cache.json"):
        self.cache_file = cache_file
        self.cache = self.load_cache()

    def load_cache(self):
        if os.path.exists(self.cache_file):
            with open(self.cache_file, 'r') as f:
                return json.load(f)
        return {}

    def save_cache(self):
        with open(self.cache_file, 'w') as f:
            json.dump(self.cache, f, indent=2)

    def get_cache_key(self, prompt, model="gpt-4-turbo"):
        """Generate unique key from prompt and model"""
        content = f"{model}:{prompt}"
        return hashlib.md5(content.encode()).hexdigest()

    def get(self, prompt, model="gpt-4-turbo"):
        """Check cache for existing response"""
        key = self.get_cache_key(prompt, model)
        return self.cache.get(key)

    def set(self, prompt, response, model="gpt-4-turbo"):
        """Store response in cache"""
        key = self.get_cache_key(prompt, model)
        self.cache[key] = {
            "prompt": prompt,
            "response": response,
            "model": model,
            "timestamp": time.time()
        }
        self.save_cache()

# Usage
cache = ResponseCache()

def cached_gpt4_call(prompt):
    # Check cache first
    cached_response = cache.get(prompt)
    if cached_response:
        print("💰 Cache hit! Saved API call.")
        return cached_response["response"]

    # No cache, make API call
    response = client.chat.completions.create(
        model="gpt-4-turbo",
        messages=[{"role": "user", "content": prompt}]
    )
    result = response.choices[0].message.content

    # Save to cache
    cache.set(prompt, result)
    return result
```

**Savings**: If you test "go to kitchen" 10 times while debugging, cache saves 9 API calls (~$0.15 saved).

### 2. Prompt Compression

**Principle**: Reduce token count while maintaining meaning.

#### Before Optimization (435 tokens)

```python
system_prompt = """
You are a task planning assistant for a humanoid robot operating in an
indoor home environment. The robot has the following capabilities and
constraints that you must consider when generating plans:

Capabilities:
- The robot can navigate to different locations in the home using the
  navigate_to(location) action. Available locations include: kitchen,
  living room, bedroom, bathroom, hallway.
- The robot can pick up objects using the pick(object_id) action. The
  robot can detect and identify objects using its camera.
- The robot can place objects at locations using the place(object_id,
  location) action.

Constraints:
- The robot's maximum reach is 0.6 meters from its base.
- The robot cannot climb stairs or step over obstacles higher than 5cm.
- The robot's maximum payload is 2 kilograms.
- The robot must maintain a safe distance of at least 0.5 meters from
  humans at all times.

Please generate a step-by-step plan in JSON format with the following
schema: {task, plan: [{action, parameters, reasoning}]}
"""
```

#### After Optimization (187 tokens)

```python
system_prompt = """
Task planner for indoor humanoid robot.

Actions: navigate_to(location), pick(object_id), place(object_id, location)
Locations: kitchen, living_room, bedroom, bathroom, hallway
Limits: reach 0.6m, payload 2kg, step height 5cm, human clearance 0.5m

Output JSON: {task: str, plan: [{action: str, parameters: obj, reasoning: str}]}
"""
```

**Savings**: 248 tokens per request × 95 requests = 23,560 tokens saved (~$0.45 saved).

### 3. Rate Limiting & Batching

**Principle**: Prevent accidental expensive loops.

```python
import time
from functools import wraps

def rate_limit(max_calls_per_minute=10):
    """Decorator to limit API calls per minute"""
    calls = []

    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            now = time.time()

            # Remove calls older than 1 minute
            calls[:] = [call_time for call_time in calls if now - call_time < 60]

            if len(calls) >= max_calls_per_minute:
                wait_time = 60 - (now - calls[0])
                print(f"⏳ Rate limit reached. Waiting {wait_time:.1f}s...")
                time.sleep(wait_time)
                calls.clear()

            calls.append(now)
            return func(*args, **kwargs)

        return wrapper
    return decorator

@rate_limit(max_calls_per_minute=10)
def gpt4_plan(task):
    response = client.chat.completions.create(
        model="gpt-4-turbo",
        messages=[{"role": "user", "content": task}]
    )
    return response.choices[0].message.content
```

**Protection**: Prevents runaway loops (e.g., buggy re-planning logic) from spending $20 before you notice.

### 4. Max Tokens Limit

**Principle**: Cap output length to prevent verbose responses.

```python
response = client.chat.completions.create(
    model="gpt-4-turbo",
    messages=[...],
    max_tokens=150  # Limit response length
)
```

**Example**: Task planning typically needs 50-150 tokens output. Default (unlimited) might generate 500+ tokens of unnecessary explanation.

**Savings**: If LLM averages 300 tokens output without limit vs 100 with limit:
- 200 tokens saved per request × 95 requests = 19K tokens
- At $0.03/1K tokens (output): ~$0.57 saved

### 5. Use Cheaper Models Where Appropriate

**Strategy**: Use `gpt-3.5-turbo` for simple tasks, `gpt-4-turbo` only when necessary.

| Model | Input Cost | Output Cost | Use Case |
|-------|------------|-------------|----------|
| gpt-3.5-turbo | $0.0005/1K | $0.0015/1K | Simple commands: "go to kitchen" |
| gpt-4-turbo | $0.01/1K | $0.03/1K | Complex planning: "clean the living room" |
| gpt-4 | $0.03/1K | $0.06/1K | Avoid (use gpt-4-turbo instead) |

```python
def choose_model(task_complexity):
    """Select model based on task complexity"""
    if len(task.split()) <= 5:  # Simple 1-5 word commands
        return "gpt-3.5-turbo"
    else:
        return "gpt-4-turbo"

model = choose_model("go to kitchen")  # → gpt-3.5-turbo
response = client.chat.completions.create(model=model, ...)
```

**Savings**: If 40% of requests are simple enough for GPT-3.5:
- 38 requests × 400 tokens avg = 15.2K tokens
- Cost with gpt-4-turbo: $0.30
- Cost with gpt-3.5-turbo: $0.02
- **Saved: $0.28**

### 6. Audio Preprocessing

**Principle**: Send only speech segments to Whisper, not silence.

#### Voice Activity Detection (VAD)

```python
import webrtcvad
import numpy as np

def extract_speech_segments(audio_data, sample_rate=16000):
    """Extract only speech segments from audio"""
    vad = webrtcvad.Vad(3)  # Aggressiveness 0-3 (3 = most aggressive)

    # Process audio in 30ms frames
    frame_duration = 30  # ms
    frame_length = int(sample_rate * frame_duration / 1000)

    speech_frames = []
    for i in range(0, len(audio_data), frame_length):
        frame = audio_data[i:i+frame_length]
        if len(frame) < frame_length:
            break

        # Check if frame contains speech
        is_speech = vad.is_speech(frame.tobytes(), sample_rate)
        if is_speech:
            speech_frames.extend(frame)

    return np.array(speech_frames)

# Usage
full_audio = record_audio(duration=10)  # 10 seconds recorded
speech_only = extract_speech_segments(full_audio)  # Maybe only 4 seconds of speech

# Send only speech to Whisper
whisper_cost_full = 10 * 0.006 / 60  # $0.001
whisper_cost_optimized = 4 * 0.006 / 60  # $0.0004
# Savings: 60% reduction
```

**Savings**: If average recording is 50% silence:
- 18 minutes full audio → 9 minutes speech
- **Saved: $0.054**

---

## Cost Monitoring

### Track Usage in Code

```python
class CostTracker:
    def __init__(self):
        self.whisper_minutes = 0.0
        self.gpt4_input_tokens = 0
        self.gpt4_output_tokens = 0

    def log_whisper_call(self, audio_duration_seconds):
        minutes = audio_duration_seconds / 60.0
        self.whisper_minutes += minutes
        cost = minutes * 0.006
        print(f"💸 Whisper: {audio_duration_seconds}s = ${cost:.4f}")

    def log_gpt4_call(self, response):
        input_tokens = response.usage.prompt_tokens
        output_tokens = response.usage.completion_tokens
        self.gpt4_input_tokens += input_tokens
        self.gpt4_output_tokens += output_tokens

        cost_input = input_tokens / 1000 * 0.01
        cost_output = output_tokens / 1000 * 0.03
        total_cost = cost_input + cost_output

        print(f"💸 GPT-4: {input_tokens}+{output_tokens} tokens = ${total_cost:.4f}")

    def print_total_cost(self):
        whisper_cost = self.whisper_minutes * 0.006
        gpt4_cost = (self.gpt4_input_tokens / 1000 * 0.01 +
                     self.gpt4_output_tokens / 1000 * 0.03)
        total = whisper_cost + gpt4_cost

        print("\n" + "="*50)
        print("API COST SUMMARY")
        print("="*50)
        print(f"Whisper: {self.whisper_minutes:.2f} min = ${whisper_cost:.4f}")
        print(f"GPT-4: {self.gpt4_input_tokens + self.gpt4_output_tokens} tokens = ${gpt4_cost:.4f}")
        print(f"TOTAL: ${total:.4f}")
        print("="*50)

# Usage
tracker = CostTracker()

# Track Whisper
audio_duration = 5.2  # seconds
tracker.log_whisper_call(audio_duration)

# Track GPT-4
response = client.chat.completions.create(...)
tracker.log_gpt4_call(response)

# At end of exercise
tracker.print_total_cost()
```

### OpenAI Dashboard

Monitor real-time usage at: https://platform.openai.com/usage

**Set Budget Alerts**:
1. Go to https://platform.openai.com/account/billing/limits
2. Set monthly budget limit (e.g., $10)
3. OpenAI emails you when you hit 75%, 90%, 100% of limit

---

## Free Tier & Credits

### Free Trial Credits

**New Accounts** (as of 2024):
- $5 free credit upon sign-up
- Expires after 3 months
- Sufficient for Chapter 5 exercises (~$2.50 usage)

### Educational Credits

**OpenAI Educator Program**:
- Apply at: https://openai.com/educators
- Free API credits for verified educators
- Must be teaching accredited course
- Limited availability

**Institutional Agreements**:
- Some universities negotiate bulk API credits
- Contact your institution's CS/Robotics department
- Ask if research grants cover API costs

---

## Alternatives for Students Without API Access

### 1. Pre-Recorded Demos

**Approach**: Watch instructor videos demonstrating VLA pipeline.

**Pros**:
- Zero cost
- Still learn concepts

**Cons**:
- No hands-on practice
- Less engaging

**Implementation**: Instructors record exercise solutions, students follow along without running code.

### 2. Shared Classroom API Key

**Approach**: Instructor provides API key for in-class exercises only.

**Risks**:
- Key exposure (students could abuse)
- Difficult cost control

**Mitigation**:
- Rotate keys weekly
- Set strict rate limits
- Monitor usage closely

### 3. Local Whisper (Free)

**Setup**:
```bash
pip install openai-whisper
whisper audio.mp3 --model medium
```

**Pros**:
- Free, unlimited usage
- Privacy-preserving (local processing)

**Cons**:
- Requires GPU (CPU very slow)
- Setup complexity
- Slower than API (~5-10x)

**Recommendation**: Mention in privacy guide, but use API for course (time constraints).

### 4. Local LLMs (Free, Advanced)

**Options**:
- Llama 3 8B/70B (Meta)
- Mistral 7B (Mistral AI)
- GPT4All (local deployment)

**Setup**:
```bash
# Example: Ollama (easiest local LLM)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3
```

**Pros**:
- Free, unlimited
- Privacy-preserving

**Cons**:
- Requires significant GPU (8GB+ VRAM for 8B models)
- Lower quality than GPT-4
- Setup complexity
- Out of scope for 10-14 hour module

**Recommendation**: Mention in content, but out of scope for exercises.

---

## Summary: Cost Optimization Checklist

**Before Exercises**:
- [ ] Set OpenAI budget alert ($10 limit)
- [ ] Implement response caching
- [ ] Set max_tokens limits on all API calls
- [ ] Add cost tracking to code

**During Development**:
- [ ] Use compressed prompts (remove verbose instructions)
- [ ] Test with simple commands first before complex ones
- [ ] Check cache before every API call
- [ ] Monitor costs with `tracker.print_total_cost()`

**If Costs Exceed $5**:
- [ ] Review: Are you re-running same requests? → Add caching
- [ ] Review: Are responses too long? → Reduce max_tokens
- [ ] Review: Can simple tasks use GPT-3.5? → Add model selection
- [ ] Review: Is VAD reducing audio duration? → Check preprocessing

**Expected Outcome**: $2-3 total cost for all Chapter 5 exercises with optimization techniques applied.

---

## Questions & Support

**Cost Concerns?**
- Email instructor before Week 11 (before starting Module 4)
- Discuss alternatives (pre-recorded demos, shared keys)
- Some institutions provide API credits for students

**Technical Issues?**
- Check OpenAI status: https://status.openai.com
- Verify API key: `echo $OPENAI_API_KEY`
- Check quotas: https://platform.openai.com/account/limits

**Unexpectedly High Costs?**
- Review usage: https://platform.openai.com/usage
- Check for runaway loops in code
- Verify caching is working (`print("Cache hit!")` messages)
