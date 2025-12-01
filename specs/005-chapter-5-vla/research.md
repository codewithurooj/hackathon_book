# VLA Research: Vision-Language-Action Models for Robotics

**Date**: 2025-11-28
**Feature**: chapter-5-vla
**Purpose**: Research foundation for Chapter 5 implementation

---

## 1. OpenAI API Educational Access and Pricing

### Current Pricing (as of 2024)

**Whisper API**:
- Model: `whisper-1`
- Cost: $0.006 per minute of audio
- Input: Audio files (mp3, mp4, mpeg, mpga, m4a, wav, webm)
- Output: Text transcription
- Languages: 99+ languages supported
- Typical latency: 500ms-2s per clip

**GPT-4 API**:
- Model: `gpt-4` or `gpt-4-turbo`
- Cost (gpt-4):
  - Input: $0.03 per 1K tokens
  - Output: $0.06 per 1K tokens
- Cost (gpt-4-turbo):
  - Input: $0.01 per 1K tokens
  - Output: $0.03 per 1K tokens
- Typical latency: 1-5s per request
- Context window: 8K-128K tokens (model dependent)

**GPT-4V (Vision)**:
- Same pricing as GPT-4
- Additional image processing costs: ~$0.01 per image (depends on size/detail)

### Educational Access Options

1. **Personal API Keys** (Most Common)
   - Students create free OpenAI account
   - $5 free trial credit (expires after 3 months)
   - Students pay directly after free credit exhausted
   - Estimated cost for Chapter 5: $2-5 total

2. **Institutional/Educational Credits**
   - Some universities negotiate educational pricing
   - Bootcamps may provide API credits
   - Research labs often have grants
   - Contact: education@openai.com

3. **OpenAI Educator Program** (Limited)
   - Free API credits for verified educators
   - Must apply and be approved
   - Limited slots available

### Cost Estimation for Chapter 5

**Whisper Exercises**:
- Exercise 1: 5 test clips × 30 seconds = $0.015
- Exercise 4: 10 voice commands × 20 seconds = $0.02
- Exercise 5: 8 multi-modal commands × 30 seconds = $0.024
- Exercise 6: Capstone demo × 3 minutes = $0.018
- **Whisper Total: ~$0.08**

**GPT-4 Exercises**:
- Exercise 2: 15 planning requests × 300 tokens avg = $0.18
- Exercise 3: 20 translation requests × 200 tokens = $0.12
- Exercise 4: 12 VLA pipeline runs × 500 tokens = $0.36
- Exercise 5: 10 multi-modal requests × 600 tokens = $0.36
- Exercise 6: Capstone demo × 1500 tokens = $0.135
- **GPT-4 Total: ~$1.00-$1.50** (using gpt-4-turbo)

**Buffer for Experimentation**: +$1.00-$2.00

**TOTAL CHAPTER 5 COST: $2.00-$5.00**

---

## 2. Prompt Engineering for Robotics

### Literature Review

#### SayCan (Google Research, 2022)
**Paper**: "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances"

**Key Insights**:
- Combine LLM's task planning with robot's affordance model
- LLM generates high-level plan, affordance model filters by feasibility
- Two-stage process:
  1. LLM proposes actions given task and available skills
  2. Value functions score each action by feasibility and relevance
- Prompting strategy: Provide available skills as context in prompt

**Example Prompt Pattern**:
```
You are a robot assistant. You can perform these skills:
- navigate_to(location)
- pick(object)
- place(object, location)

Task: "Bring me a drink from the kitchen"
Plan the sequence of actions:
```

#### RT-2 (Robotic Transformer 2, Google DeepMind, 2023)
**Paper**: "RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control"

**Key Insights**:
- Vision-language models fine-tuned for robot control
- End-to-end: image + instruction → robot actions
- Leverages pre-trained vision-language models (PaLI-X)
- Generalization from internet-scale knowledge to robotic tasks

**Prompting Strategy**:
- Direct instruction format: "pick up the [object]"
- Image + text → action tokens
- Not directly applicable to GPT-4 (RT-2 is specialized), but demonstrates VLA potential

#### PaLM-E (Google Research, 2023)
**Paper**: "PaLM-E: An Embodied Multimodal Language Model"

**Key Insights**:
- 562B parameter model integrating vision, language, and continuous sensor data
- Embodied reasoning: language model "sees" the world through robot's sensors
- Task decomposition with visual grounding

**Prompting Strategy**:
- Multi-modal prompts: text + images + sensor state
- Chain-of-thought reasoning for complex manipulation
- Example: "What should I do to clean the table?" → observes table → generates plan

### Best Practices for Robotics Prompting

#### 1. System Prompt Structure

**Components**:
- **Role definition**: "You are a navigation planner for a humanoid robot..."
- **Capabilities**: List available actions, sensors, workspace
- **Constraints**: Safety rules, physical limits, workspace boundaries
- **Output format**: Specify JSON schema or structured format
- **Few-shot examples**: 2-3 example task decompositions

**Template**:
```yaml
system_prompt:
  role: "You are a task planner for a humanoid robot operating in an indoor environment."

  capabilities:
    - "Navigate to named locations using navigate_to(location)"
    - "Pick objects using pick(object_id)"
    - "Place objects using place(object_id, location)"
    - "Detect objects using camera: /camera/image_raw"

  constraints:
    - "Robot height: 1.5m, reach: 0.6m"
    - "Cannot climb stairs"
    - "Must avoid collision with humans"
    - "Maximum object weight: 2kg"

  output_format:
    type: "json"
    schema:
      task: "string"
      plan:
        - action: "string"
          parameters: "object"
          reasoning: "string"

  examples:
    - task: "Bring the red mug to the living room"
      plan:
        - action: "navigate_to"
          parameters: {location: "kitchen"}
          reasoning: "Red mug likely in kitchen"
        - action: "pick"
          parameters: {object: "red_mug"}
          reasoning: "Grasp the target object"
        - action: "navigate_to"
          parameters: {location: "living_room"}
          reasoning: "Transport to destination"
        - action: "place"
          parameters: {object: "red_mug", location: "coffee_table"}
          reasoning: "Place at convenient location"
```

#### 2. Few-Shot Examples

**Principles**:
- Include 2-4 examples (more increases cost without much benefit)
- Cover diverse task types (navigation, manipulation, multi-step)
- Show correct output format
- Include reasoning/explanation

**Example Set**:
1. **Simple navigation**: "Go to the kitchen" → [navigate_to(kitchen)]
2. **Object manipulation**: "Pick up the book" → [detect(book), navigate_to(book_location), pick(book_id)]
3. **Multi-step task**: "Clean the table" → [navigate_to(table), detect(objects_on_table), pick(each_object), navigate_to(trash), place(objects)]

#### 3. Structured Output Parsing

**JSON Mode** (GPT-4 Turbo feature):
```python
response = client.chat.completions.create(
    model="gpt-4-turbo",
    messages=[...],
    response_format={"type": "json_object"}
)
```

**Function Calling**:
```python
functions = [
    {
        "name": "navigate_to",
        "description": "Navigate robot to a named location",
        "parameters": {
            "type": "object",
            "properties": {
                "location": {"type": "string", "description": "Target location name"},
                "approach_distance": {"type": "number", "description": "Stop distance in meters"}
            },
            "required": ["location"]
        }
    }
]

response = client.chat.completions.create(
    model="gpt-4",
    messages=[...],
    functions=functions,
    function_call="auto"
)
```

#### 4. Safety Constraint Encoding

**Explicit Safety Rules in Prompt**:
```
SAFETY RULES (MUST FOLLOW):
1. Never navigate if humans detected in path
2. Always check object weight before picking (max 2kg)
3. Never approach stairs or ledges
4. Stop immediately if battery < 20%
5. Confirm destructive actions with user
```

**Safety Validation Prompt** (separate call):
```
Given this action plan: [PLAN]
And these safety rules: [RULES]
Identify any safety violations. Return JSON:
{
  "is_safe": boolean,
  "violations": [list of rule violations],
  "suggested_fixes": [list of modifications]
}
```

---

## 3. VLA Latency Mitigation Strategies

### Problem: LLM Latency vs Real-Time Control

**Challenge**: GPT-4 API calls take 1-5 seconds, too slow for low-level control (e.g., 100Hz motor control loops).

**Solution**: Hybrid architecture separating deliberative and reactive layers.

### Deliberative vs Reactive Control

```
┌─────────────────────────────────────────────┐
│           Deliberative Layer                │
│  (LLM Task Planning: Seconds-Minutes)       │
│                                             │
│  "Bring the red mug from kitchen"           │
│      ↓ GPT-4 (3 seconds)                    │
│  [navigate_to(kitchen), pick(red_mug),      │
│   navigate_to(living_room), place(mug)]     │
└────────────────┬────────────────────────────┘
                 │
                 ↓ High-level commands
┌────────────────────────────────────────────┐
│           Reactive Layer                    │
│  (Traditional Controllers: Milliseconds)    │
│                                             │
│  • Nav2 path planning (10Hz)                │
│  • MoveIt motion planning (10Hz)            │
│  • Motor controllers (100Hz)                │
│  • Collision avoidance (100Hz)              │
└─────────────────────────────────────────────┘
```

**Key Principle**: LLM plans what to do (task level), traditional controllers execute how to do it (control level).

### Asynchronous Processing Patterns

#### Pattern 1: Plan-Then-Execute

```python
class VLAPipeline:
    def __init__(self):
        self.current_plan = None
        self.execution_status = "idle"

    async def voice_command_callback(self, voice_msg):
        # 1. Speech-to-text (async, ~1s)
        text = await self.whisper_transcribe(voice_msg.audio)

        # 2. LLM planning (async, ~3s)
        plan = await self.gpt4_plan(text)

        # 3. Execute plan (synchronous, seconds-minutes)
        await self.execute_plan(plan)

    async def execute_plan(self, plan):
        for action in plan:
            # Each action handled by reactive layer
            await self.execute_action(action)
```

#### Pattern 2: Concurrent Planning

```python
class ConcurrentVLA:
    def __init__(self):
        self.planning_thread = Thread(target=self.continuous_planning)
        self.execution_thread = Thread(target=self.continuous_execution)

    def continuous_planning(self):
        """Runs in background, updates plan based on new information"""
        while True:
            if self.needs_replan():
                new_plan = self.gpt4_plan(self.current_context)
                self.plan_queue.put(new_plan)
            time.sleep(5)  # Replan every 5 seconds if needed

    def continuous_execution(self):
        """Executes current plan, polls for new plans"""
        while True:
            if not self.plan_queue.empty():
                self.current_plan = self.plan_queue.get()
            self.execute_next_action()
```

### Caching Strategies

#### Common Task Caching

```python
class PlanCache:
    def __init__(self):
        self.cache = {}

    def get_plan(self, task):
        # Check cache first
        cache_key = self.normalize_task(task)
        if cache_key in self.cache:
            print(f"Cache hit: {cache_key}")
            return self.cache[cache_key]

        # Call LLM if not cached
        plan = self.gpt4_plan(task)
        self.cache[cache_key] = plan
        return plan

    def normalize_task(self, task):
        # "go to kitchen" and "navigate to the kitchen" → same key
        return task.lower().strip().replace("navigate to", "go to")
```

**Savings**: Common tasks (e.g., "go to kitchen") cached → 0ms latency, $0 cost on repeat.

#### Semantic Similarity Caching

```python
from sentence_transformers import SentenceTransformer

class SemanticCache:
    def __init__(self, threshold=0.9):
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')
        self.cache = {}  # embedding → plan
        self.threshold = threshold

    def get_plan(self, task):
        task_embedding = self.encoder.encode(task)

        # Find most similar cached task
        for cached_embedding, plan in self.cache.items():
            similarity = cosine_similarity(task_embedding, cached_embedding)
            if similarity > self.threshold:
                return plan  # Cache hit

        # No match, call LLM
        plan = self.gpt4_plan(task)
        self.cache[task_embedding] = plan
        return plan
```

**Benefit**: "bring me water" and "get me a glass of water" recognized as similar → cache hit.

---

## 4. Multi-Modal Perception Integration

### Vision + Language for Object Grounding

**Problem**: Language commands reference objects ("the red mug"), but robots need object IDs and poses.

**Solution**: Combine object detection (vision) with language understanding.

### Architecture

```
Voice Command: "Pick up the red mug"
    ↓ Whisper
Text: "pick up the red mug"
    ↓ GPT-4 (extract object description)
Object Query: {color: "red", type: "mug"}
    ↓ Vision System (Isaac ROS, YOLOv8)
Detected Objects: [
    {id: "obj_1", class: "mug", color: "red", pose: (x, y, z)},
    {id: "obj_2", class: "mug", color: "blue", pose: (x2, y2, z2)}
]
    ↓ Grounding (match description to detections)
Target Object: {id: "obj_1", pose: (x, y, z)}
    ↓ Action Execution
pick(object_id="obj_1", approach_pose=(x, y, z+0.1))
```

### GPT-4V (Vision) Integration

**Use Case**: LLM sees robot's camera feed directly.

```python
import base64

def gpt4v_plan(task, camera_image_path):
    with open(camera_image_path, "rb") as image_file:
        base64_image = base64.b64encode(image_file.read()).decode('utf-8')

    response = client.chat.completions.create(
        model="gpt-4-vision-preview",
        messages=[
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": f"Task: {task}\n\nWhat objects do you see? Plan the actions."},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{base64_image}"
                        }
                    }
                ]
            }
        ]
    )
    return response.choices[0].message.content
```

**Example**:
- Task: "Clean the table"
- Image: [camera view of cluttered table]
- GPT-4V response: "I see a coffee cup, a book, and a pen on the table. Plan: 1) Pick up cup, 2) Pick up book, 3) Pick up pen."

### Combining Isaac ROS Object Detection with LLM

```python
class MultiModalVLA:
    def __init__(self):
        # Subscribe to Isaac ROS detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        self.latest_detections = []

    def detection_callback(self, msg):
        self.latest_detections = msg.detections

    async def execute_command(self, voice_command):
        # 1. LLM extracts object description
        object_desc = await self.extract_object_description(voice_command)
        # e.g., {"color": "red", "type": "mug"}

        # 2. Match description to detections
        target_obj = self.find_matching_object(object_desc, self.latest_detections)

        # 3. LLM plans actions with concrete object ID
        plan = await self.gpt4_plan(
            task=voice_command,
            available_objects=[target_obj]
        )

        # 4. Execute
        await self.execute_plan(plan)

    def find_matching_object(self, description, detections):
        """Match object description to detected objects"""
        for det in detections:
            if det.class_name == description["type"]:
                # TODO: Check color, size, etc.
                return {
                    "id": det.id,
                    "pose": det.pose,
                    "class": det.class_name
                }
        return None
```

---

## 5. Privacy and Security Best Practices

### Privacy Concerns

**Data Sent to OpenAI**:
1. **Voice audio** → Whisper API
2. **Text transcriptions** → GPT-4 API
3. **Camera images** (if using GPT-4V) → GPT-4V API

**Risks**:
- Personal conversations recorded
- Private environments photographed
- Sensitive information in tasks (e.g., "unlock the safe")

### OpenAI Data Retention Policies (as of 2024)

**API Data Usage**:
- OpenAI states: "Data submitted via API is not used to train models"
- Retention: 30 days for abuse monitoring, then deleted
- Zero Data Retention (ZDR): Available for enterprise customers
- Source: https://openai.com/policies/api-data-usage-policies

**Recommendations for Chapter 5**:
- Use sample/test audio files (not real private conversations)
- Avoid camera feeds of private spaces for GPT-4V exercises
- Inform students about data policies
- Suggest local alternatives for privacy-sensitive applications

### Data Minimization Strategies

#### 1. Local Processing Where Possible

```python
# Instead of sending full audio
audio_full = record_audio(duration=10)  # 10 seconds

# Use Voice Activity Detection (VAD) to send only speech
import webrtcvad
vad = webrtcvad.Vad(3)  # Aggressiveness 0-3

speech_segments = extract_speech(audio_full, vad)
# Send only speech_segments to Whisper (e.g., 3 seconds instead of 10)
```

#### 2. Anonymization

```python
def anonymize_task(task_text):
    """Remove names, addresses, sensitive info before sending to LLM"""
    # Replace names with placeholders
    anonymized = task_text.replace("John", "[Person_1]")
    anonymized = anonymized.replace("123 Main St", "[Address_1]")
    return anonymized

task = "Tell John at 123 Main St that I'll be late"
anonymized_task = anonymize_task(task)
# Send to LLM: "Tell [Person_1] at [Address_1] that I'll be late"
```

#### 3. Consent Mechanisms

```python
class PrivacyConsentVLA:
    def __init__(self):
        self.user_consented = False

    def request_consent(self):
        print("This robot uses OpenAI APIs. Your voice and camera data will be sent to OpenAI.")
        print("Data retention: 30 days for monitoring, then deleted.")
        print("Data is not used for training.")
        response = input("Do you consent? (yes/no): ")
        self.user_consented = (response.lower() == "yes")
        return self.user_consented

    async def process_command(self, audio):
        if not self.user_consented:
            print("Cannot process command without consent.")
            return

        # Proceed with API calls
        text = await self.whisper_api(audio)
        plan = await self.gpt4_api(text)
        await self.execute(plan)
```

### Safety Validation Before Execution

**Problem**: LLM may hallucinate dangerous actions.

**Solution**: Hard-coded validation layer.

```python
class SafetyValidator:
    UNSAFE_ZONES = ["stairs", "balcony", "street"]
    MAX_OBJECT_WEIGHT = 5.0  # kg

    def validate_plan(self, plan):
        violations = []

        for action in plan:
            # Check navigation safety
            if action["type"] == "navigate_to":
                if action["target"] in self.UNSAFE_ZONES:
                    violations.append(f"Cannot navigate to unsafe zone: {action['target']}")

            # Check manipulation safety
            if action["type"] == "pick":
                obj_weight = self.get_object_weight(action["object_id"])
                if obj_weight > self.MAX_OBJECT_WEIGHT:
                    violations.append(f"Object too heavy: {obj_weight}kg > {self.MAX_OBJECT_WEIGHT}kg")

        return {
            "is_safe": len(violations) == 0,
            "violations": violations
        }
```

### Local Alternatives (Out of Scope for Exercises, Mentioned in Content)

**Whisper Local**:
```bash
# Run Whisper on local GPU (no API calls)
pip install openai-whisper
whisper audio.mp3 --model medium
```

**Local LLMs**:
- Llama 3 (Meta)
- Mistral 7B/Mixtral
- Requires GPU, more setup complexity
- Trade-off: Privacy ↑, Quality ↓, Setup complexity ↑

**Recommendation**: Mention local alternatives in privacy guide, but use OpenAI for course exercises (accessibility/time).

---

## Summary

### Key Findings for Chapter 5 Implementation

1. **API Costs**: $2-5 total is acceptable, provide cost transparency upfront
2. **Prompt Engineering**: System prompts + few-shot examples + structured outputs
3. **Latency**: Use hybrid architecture (LLM planning + traditional control)
4. **Multi-Modal**: Combine vision (Isaac ROS) with language (GPT-4) for grounding
5. **Privacy**: Educate on data policies, provide consent mechanisms, mention local alternatives

### Implementation Priorities

**Must Have**:
- API cost guide with optimization techniques
- System prompt template with robot capabilities
- Safety validation layer
- Privacy consent mechanism

**Nice to Have**:
- Semantic caching for cost savings
- GPT-4V integration (advanced exercise)
- Local Whisper mention (privacy guide)

### Recommended Architecture for Capstone

```
┌──────────────────────────────────────────────────────┐
│                  VLA Pipeline                         │
│                                                       │
│  Voice Input → Whisper → GPT-4 Planning →            │
│  Action Translation → Nav2/MoveIt Execution           │
│                                                       │
│  Camera Feed → Isaac ROS Detection → Object Grounding │
└──────────────────────────────────────────────────────┘
         ↓                    ↓                    ↓
    [Safety]            [Caching]            [Privacy]
   Validation       (Cost Optimization)     (Consent)
```

This architecture balances functionality, cost, safety, and privacy for educational humanoid robotics.
