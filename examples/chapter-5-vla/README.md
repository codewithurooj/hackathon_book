# Chapter 5: Vision-Language-Action (VLA) - Code Examples

This directory contains all code examples for Chapter 5 (Module 4) of the Physical AI & Humanoid Robotics textbook.

## Overview

**Vision-Language-Action (VLA)** systems integrate AI (voice recognition, LLMs) with robotics (ROS 2, Nav2) to enable voice-controlled autonomous robots. This module teaches the complete pipeline:

```
Voice Command → Whisper API → GPT-4 Planning → Action Translation → Robot Execution
```

## Prerequisites

- **ROS 2 Humble** installed
- **Python 3.10+**
- **OpenAI API Key** (required for exercises)
- **Isaac Sim** or **Gazebo** (simulation environment)
- Modules 1-3 completed (ROS 2, Nav2, Isaac basics)

## Estimated Cost

**Total**: $2-5 for all Chapter 5 exercises

| Service | Usage | Cost |
|---------|-------|------|
| Whisper API | ~18 minutes audio | $0.11 |
| GPT-4 API | ~95 requests (~45K tokens) | $1.45 |
| **Total** | All exercises + experimentation | **$2.00-$5.00** |

See `../../specs/005-chapter-5-vla/api-cost-guide.md` for cost optimization strategies.

## Setup

### 1. Install Dependencies

```bash
cd ~/ros2_ws/src
git clone [your-repo]/physical-ai-book
cd physical-ai-book/examples/chapter-5-vla

pip install openai pyaudio numpy python-dotenv webrtcvad
```

### 2. Set OpenAI API Key

Create `.env` file (never commit to Git!):

```bash
# Create .env file
cat > .env << EOF
OPENAI_API_KEY=your-api-key-here
EOF

# Verify it's in .gitignore
grep ".env" ../../../.gitignore
```

Get your API key at: https://platform.openai.com/api-keys

### 3. Verify API Access

```bash
python verify_api_access.py
```

Expected output:
```
✅ API key found: sk-proj-...
✅ Whisper API client initialized successfully
✅ GPT-4 API test successful
💰 TOTAL ESTIMATED: $2.00-$5.00
```

## Directory Structure

```
chapter-5-vla/
├── README.md                       ← You are here
├── verify_api_access.py            ← API verification script
├── vla_nodes/                      ← ROS 2 VLA nodes
│   ├── whisper_voice_node.py       ← Whisper speech-to-text
│   ├── gpt4_planning_node.py       ← GPT-4 task planning
│   ├── action_translator.py        ← NL → ROS 2 actions
│   └── vla_pipeline.py             ← Complete VLA system
├── prompts/                        ← Prompt engineering templates
│   ├── system_prompt.yaml          ← Robot capabilities, constraints
│   ├── few_shot_examples.yaml      ← Example task decompositions
│   └── safety_validation.yaml      ← Safety checking prompts
├── configs/                        ← Configuration files
│   ├── whisper_config.yaml         ← Whisper settings
│   ├── gpt4_config.yaml            ← GPT-4 model, temperature, max_tokens
│   └── action_mapping.yaml         ← Action type → ROS 2 message mapping
├── launch/                         ← ROS 2 launch files
│   ├── whisper_node.launch.py      ← Launch Whisper node
│   ├── vla_pipeline.launch.py      ← Launch complete VLA pipeline
│   └── capstone_demo.launch.py     ← Capstone autonomous humanoid demo
└── exercises/                      ← Hands-on exercises
    ├── exercise_1_whisper_integration/
    ├── exercise_2_gpt4_planning/
    ├── exercise_3_action_translation/
    ├── exercise_4_complete_vla/
    ├── exercise_5_multimodal_perception/
    └── exercise_6_capstone_system/
```

## Quick Start

### Example 1: Whisper Voice Recognition

```bash
# Launch Whisper node
ros2 launch launch/whisper_node.launch.py

# In another terminal, send test audio
ros2 topic echo /voice_commands
```

### Example 2: GPT-4 Task Planning

```python
from vla_nodes.gpt4_planning_node import GPT4PlanningNode

planner = GPT4PlanningNode()
plan = planner.plan_task("Go to the kitchen and bring me a glass of water")

print(plan)
# Output:
# [
#   {"action": "navigate_to", "parameters": {"location": "kitchen"}},
#   {"action": "pick", "parameters": {"object": "glass"}},
#   {"action": "navigate_to", "parameters": {"location": "living_room"}},
#   {"action": "place", "parameters": {"object": "glass", "location": "table"}}
# ]
```

### Example 3: Complete VLA Pipeline

```bash
# Launch complete VLA system (Whisper + GPT-4 + Action Translator + Isaac Sim)
ros2 launch launch/vla_pipeline.launch.py

# Speak voice command (or play test audio file)
# Example: "Robot, go to the bedroom"
# Pipeline:
#   1. Whisper transcribes: "go to the bedroom"
#   2. GPT-4 plans: [{"action": "navigate_to", "parameters": {"location": "bedroom"}}]
#   3. Action translator executes: Nav2 navigation to bedroom
#   4. Robot moves in Isaac Sim
```

## Exercises

### Exercise 1: Whisper Integration (30-45 min)

**Goal**: Implement Whisper API integration and create ROS 2 voice command node.

**Tasks**:
1. Capture audio from microphone
2. Call Whisper API to transcribe
3. Publish transcriptions to `/voice_commands` topic
4. Test with 5 voice commands

**Cost**: ~$0.015 (2.5 minutes audio)

### Exercise 2: GPT-4 Planning (45-60 min)

**Goal**: Create LLM-driven task decomposition with prompt engineering.

**Tasks**:
1. Design system prompt with robot capabilities
2. Implement GPT-4 API call with structured output
3. Test with 15 different tasks
4. Measure latency and cost

**Cost**: ~$0.18 (15 requests × 400 tokens avg)

### Exercise 3: Action Translation (45-60 min)

**Goal**: Translate LLM outputs to ROS 2 action calls.

**Tasks**:
1. Parse LLM JSON output
2. Map actions to Nav2 goals
3. Implement action clients
4. Execute in Isaac Sim

**Cost**: ~$0.12 (20 requests for testing)

### Exercise 4: Complete VLA Pipeline (60-90 min)

**Goal**: Integrate Whisper + GPT-4 + action execution.

**Tasks**:
1. Connect all components
2. Test multi-step tasks ("clean the table")
3. Add error handling
4. Profile end-to-end latency

**Cost**: ~$0.38 (Whisper + GPT-4 for 12 tasks)

### Exercise 5: Multi-Modal Perception (60-90 min)

**Goal**: Integrate vision + language for object grounding.

**Tasks**:
1. Connect Isaac ROS object detection
2. Match object descriptions to detections
3. Test: "pick the red mug"
4. Handle object not found

**Cost**: ~$0.28 (multi-modal requests)

### Exercise 6: Capstone System (90-120 min)

**Goal**: Build complete autonomous humanoid with voice control.

**Tasks**:
1. Design capstone architecture
2. Implement long-running missions
3. Add interruption handling
4. Demo: "Go to kitchen, find red mug, bring to living room"

**Cost**: ~$0.40 (capstone demo + testing)

## Common Issues

### Issue: "API key not found"

```bash
# Check environment variable
echo $OPENAI_API_KEY

# If empty, add to .env file
echo "OPENAI_API_KEY=sk-proj-..." > .env

# Load .env in Python
from dotenv import load_dotenv
load_dotenv()
```

### Issue: "insufficient_quota" error

**Cause**: No credits in OpenAI account.

**Solution**:
1. Check balance: https://platform.openai.com/account/usage
2. Add credits: https://platform.openai.com/account/billing/payment-methods
3. New accounts get $5 free credit (expires after 3 months)

### Issue: High latency (>10s per command)

**Cause**: Network latency or model selection.

**Solutions**:
- Use `gpt-4-turbo` instead of `gpt-4` (faster, cheaper)
- Reduce max_tokens limit
- Check internet connection
- Implement caching for repeated commands

### Issue: Whisper transcription errors

**Causes**: Noisy audio, non-English speech, accents.

**Solutions**:
- Use better microphone
- Implement voice activity detection (VAD)
- Specify language: `language="en"`
- Use `model="large"` for better accuracy (slower, not recommended for course)

## Cost Optimization

See `../../specs/005-chapter-5-vla/api-cost-guide.md` for detailed strategies:

1. **Caching**: Store LLM responses for identical requests → Save ~$0.45
2. **Prompt compression**: Remove verbose instructions → Save ~$0.45
3. **Rate limiting**: Prevent runaway loops
4. **Max tokens**: Cap output length → Save ~$0.57
5. **Model selection**: Use gpt-3.5-turbo for simple tasks → Save ~$0.28
6. **VAD**: Send only speech, not silence → Save ~$0.05

**Total Potential Savings**: $1.80 → Final cost ~$0.70 (from $2.50)

## Privacy & Security

See `../../specs/005-chapter-5-vla/privacy-guide.md` for full details:

**Data Sent to OpenAI**:
- Voice audio → Whisper API
- Text commands → GPT-4 API
- Camera images (if using GPT-4V) → GPT-4V API

**OpenAI Policy**:
- Data NOT used for training
- Stored 30 days, then deleted
- Encrypted in transit (TLS)

**Best Practices**:
- ✅ Use test data (not real personal conversations)
- ✅ Implement consent mechanism
- ✅ Store API keys in .env (never commit)
- ✅ Implement input validation
- ✅ Validate LLM outputs before execution

## Additional Resources

- **Documentation**: `../../docs/chapter-5-vla/`
- **Specification**: `../../specs/005-chapter-5-vla/spec.md`
- **API Cost Guide**: `../../specs/005-chapter-5-vla/api-cost-guide.md`
- **Privacy Guide**: `../../specs/005-chapter-5-vla/privacy-guide.md`
- **OpenAI API Docs**: https://platform.openai.com/docs
- **Whisper API**: https://platform.openai.com/docs/guides/speech-to-text
- **GPT-4 API**: https://platform.openai.com/docs/guides/gpt

## Support

**Questions?**
- Check troubleshooting: `../../docs/chapter-5-vla/12-troubleshooting.md`
- Review exercises: `../../docs/chapter-5-vla/11-exercises.md`
- Email instructor
- Post in course forum

**API Issues?**
- OpenAI Status: https://status.openai.com
- Usage Dashboard: https://platform.openai.com/usage
- API Documentation: https://platform.openai.com/docs

---

**Ready to start?** Run `python verify_api_access.py` and begin Exercise 1!
