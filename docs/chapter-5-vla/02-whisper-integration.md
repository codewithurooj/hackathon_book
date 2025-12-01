# Whisper Integration: Voice-to-Text for Robots

## Introduction

Natural language control starts with **hearing**. Before your humanoid robot can understand commands, it needs to convert speech into text. In this section, you'll integrate **OpenAI Whisper**, a state-of-the-art speech recognition model, into a ROS 2 node. By the end, your robot will transcribe voice commands and publish them to a ROS topic for downstream planning.

## What is Whisper?

**Whisper** is OpenAI's automatic speech recognition (ASR) model trained on 680,000 hours of multilingual audio. It's designed for:
- **Robustness**: Works with noisy audio, accents, and background sounds
- **Multilingual**: Supports 99 languages (English, Spanish, Chinese, etc.)
- **Zero-shot**: No fine-tuning required for new domains

**Key Features**:
- **Word Error Rate (WER)**: ~5-10% on clean audio (human-level)
- **Real-time capable**: ~500ms-2s latency depending on audio length
- **Free tier**: OpenAI API provides generous free usage

### Whisper Models

Whisper comes in several sizes:

| Model | Parameters | Speed | Accuracy | Use Case |
|-------|-----------|-------|----------|----------|
| **tiny** | 39M | ~10x real-time | 80% | Quick prototyping |
| **base** | 74M | ~7x real-time | 85% | Embedded devices |
| **small** | 244M | ~4x real-time | 90% | General use |
| **medium** | 769M | ~2x real-time | 95% | High accuracy |
| **large-v3** | 1.5B | ~1x real-time | 97%+ | Production |

**For this chapter**: We'll use the **OpenAI API** (uses `large-v3` automatically), which is faster and more accurate than running models locally.

## Why Whisper for Robotics?

### Advantage 1: Noise Robustness

Robots operate in **noisy environments**:
- Motors humming
- People talking in background
- Wind (outdoors)
- Echoes in large rooms

**Whisper handles this** because it's trained on diverse, real-world audio (not just clean studio recordings).

### Advantage 2: Multilingual Support

**Example**: International research team
- Engineer (English): *"Go to the lab"*
- Colleague (Spanish): *"Ve al laboratorio"*
- Both transcribed correctly without language switching

### Advantage 3: No Fine-Tuning

**Traditional ASR** (e.g., Google Speech API) often struggles with:
- Technical jargon ("Nav2", "cuVSLAM", "ROS 2")
- Domain-specific terms ("gripper", "manipulator")

**Whisper** handles robotics terminology out-of-the-box due to its massive training corpus.

## OpenAI Whisper API vs. Local Models

### Option 1: OpenAI API (Recommended)

**Advantages**:
- Fastest (cloud GPUs)
- Most accurate (`large-v3` model)
- No local setup required
- Always up-to-date

**Disadvantages**:
- Requires internet
- Costs $0.006/minute of audio (~$0.10 for 15min exercise)
- Privacy: Audio sent to OpenAI servers

### Option 2: Local Whisper (whisper.cpp or Faster Whisper)

**Advantages**:
- No internet required
- Free (after initial setup)
- Privacy: Audio stays local

**Disadvantages**:
- Slower (unless you have a strong GPU)
- Requires installation and model downloads
- Slightly lower accuracy

**For learning**: Use OpenAI API. For production/privacy-sensitive deployments: Use local models.

## Setting Up OpenAI API

### Step 1: Create OpenAI Account

1. Go to https://platform.openai.com/signup
2. Sign up with email
3. Verify email address

### Step 2: Get API Key

1. Navigate to **API Keys** section
2. Click **Create new secret key**
3. Copy key (starts with `sk-...`)
4. **Important**: Never commit API keys to GitHub!

### Step 3: Install OpenAI Python SDK

```bash
# Activate ROS 2 workspace
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# Install OpenAI SDK
pip3 install openai
```

### Step 4: Test API Access

```python
#!/usr/bin/env python3
import openai

# Set API key (replace with yours)
openai.api_key = "sk-YOUR_API_KEY_HERE"

# Test connection
models = openai.Model.list()
print("✓ API connection successful!")
print(f"Available models: {len(models['data'])}")
```

**Expected output**:
```
✓ API connection successful!
Available models: 50+
```

### Step 5: Secure API Key

**Never hardcode API keys!** Use environment variables:

```bash
# Add to ~/.bashrc
export OPENAI_API_KEY="sk-YOUR_API_KEY_HERE"
source ~/.bashrc
```

**In Python**:
```python
import os
import openai

openai.api_key = os.getenv("OPENAI_API_KEY")
```

## Creating a Whisper ROS 2 Node

Now let's build a ROS 2 node that captures audio and publishes transcriptions.

### Package Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python whisper_voice_node --dependencies rclpy std_msgs
cd whisper_voice_node
```

**Directory structure**:
```
whisper_voice_node/
├── whisper_voice_node/
│   ├── __init__.py
│   └── whisper_node.py       # Main node
├── package.xml
├── setup.py
└── config/
    └── whisper_config.yaml   # Configuration
```

### Node Implementation

**File**: `whisper_voice_node/whisper_voice_node/whisper_node.py`

```python
#!/usr/bin/env python3
"""
Whisper Voice Node - Captures audio and transcribes to text using OpenAI Whisper API
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import scipy.io.wavfile as wav
import openai
import tempfile

class WhisperVoiceNode(Node):
    """ROS 2 node for voice-to-text using Whisper API"""

    def __init__(self):
        super().__init__('whisper_voice_node')

        # Declare parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('duration', 5.0)  # Record 5 seconds at a time
        self.declare_parameter('device_index', None)  # Auto-select microphone

        # Get parameters
        self.sample_rate = self.get_parameter('sample_rate').value
        self.duration = self.get_parameter('duration').value
        self.device_index = self.get_parameter('device_index').value

        # Set up OpenAI API
        openai.api_key = os.getenv("OPENAI_API_KEY")
        if not openai.api_key:
            self.get_logger().error("OPENAI_API_KEY environment variable not set!")
            raise ValueError("Missing API key")

        # Create publisher for transcribed text
        self.publisher = self.create_publisher(String, '/voice_commands', 10)

        # Create timer to record audio periodically
        self.timer = self.create_timer(self.duration + 0.5, self.record_and_transcribe)

        self.get_logger().info(f'Whisper Voice Node started')
        self.get_logger().info(f'Recording {self.duration}s audio every {self.duration + 0.5}s')
        self.get_logger().info('Publishing transcriptions to /voice_commands')

    def record_audio(self):
        """Record audio from microphone"""
        self.get_logger().info('🎤 Recording...')

        # Record audio
        audio_data = sd.rec(
            int(self.duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype='int16',
            device=self.device_index
        )
        sd.wait()  # Wait until recording is finished

        return audio_data

    def transcribe_audio(self, audio_data):
        """Transcribe audio using Whisper API"""
        # Save audio to temporary WAV file (API requires file input)
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_audio:
            wav.write(temp_audio.name, self.sample_rate, audio_data)
            temp_path = temp_audio.name

        try:
            # Call Whisper API
            with open(temp_path, 'rb') as audio_file:
                transcript = openai.Audio.transcribe(
                    model="whisper-1",
                    file=audio_file,
                    language="en"  # Force English (remove for auto-detect)
                )

            transcribed_text = transcript['text'].strip()
            return transcribed_text

        except Exception as e:
            self.get_logger().error(f'Transcription failed: {e}')
            return None

        finally:
            # Clean up temporary file
            os.remove(temp_path)

    def record_and_transcribe(self):
        """Main callback: record audio and transcribe"""
        # Record audio
        audio_data = self.record_audio()

        # Transcribe
        text = self.transcribe_audio(audio_data)

        if text:
            # Publish to ROS topic
            msg = String()
            msg.data = text
            self.publisher.publish(msg)
            self.get_logger().info(f'📝 Transcribed: "{text}"')
        else:
            self.get_logger().warn('No transcription (silence or error)')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Dependencies

**Add to `package.xml`**:
```xml
<exec_depend>std_msgs</exec_depend>
<exec_depend>python3-sounddevice</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<exec_depend>python3-openai</exec_depend>
```

**Install Python packages**:
```bash
pip3 install sounddevice scipy openai
```

### Configuration File

**File**: `config/whisper_config.yaml`

```yaml
whisper_voice_node:
  ros__parameters:
    sample_rate: 16000      # 16 kHz (Whisper's native rate)
    duration: 5.0           # Record 5 seconds at a time
    device_index: null      # Auto-select (or specify device number)
```

### Setup.py Entry Point

**File**: `setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'whisper_voice_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Whisper voice-to-text ROS 2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'whisper_node = whisper_voice_node.whisper_node:main',
        ],
    },
)
```

## Building and Running

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select whisper_voice_node --symlink-install
source install/setup.bash
```

### Run the Node

```bash
ros2 run whisper_voice_node whisper_node
```

**Expected output**:
```
[INFO] [whisper_voice_node]: Whisper Voice Node started
[INFO] [whisper_voice_node]: Recording 5.0s audio every 5.5s
[INFO] [whisper_voice_node]: Publishing transcriptions to /voice_commands
[INFO] [whisper_voice_node]: 🎤 Recording...
[INFO] [whisper_voice_node]: 📝 Transcribed: "go to the kitchen"
```

**Speak into your microphone** during the 5-second recording window.

### Listen to Transcriptions

**Terminal 2**:
```bash
ros2 topic echo /voice_commands
```

**Output**:
```
data: 'go to the kitchen'
---
data: 'pick up the red cup'
---
```

## Improving Audio Quality

### Issue 1: Background Noise

**Problem**: Robot motors, HVAC, people talking
**Solution**: Use noise suppression

**Install `noisereduce`**:
```bash
pip3 install noisereduce
```

**Add to node**:
```python
import noisereduce as nr

def record_audio(self):
    audio_data = sd.rec(...)
    sd.wait()

    # Apply noise reduction
    audio_data_clean = nr.reduce_noise(
        y=audio_data.flatten(),
        sr=self.sample_rate,
        stationary=True  # For constant background noise (motors)
    )

    return audio_data_clean.reshape(-1, 1).astype('int16')
```

### Issue 2: Voice Activity Detection (VAD)

**Problem**: Transcribing silence wastes API calls
**Solution**: Only transcribe when voice detected

**Install `webrtcvad`**:
```bash
pip3 install webrtcvad
```

**Add VAD check**:
```python
import webrtcvad

vad = webrtcvad.Vad(3)  # Aggressiveness 0-3 (3 = most aggressive)

def has_voice(self, audio_data):
    # Convert to bytes
    audio_bytes = audio_data.tobytes()

    # Check if voice present
    is_speech = vad.is_speech(audio_bytes, self.sample_rate)
    return is_speech

def record_and_transcribe(self):
    audio_data = self.record_audio()

    if not self.has_voice(audio_data):
        self.get_logger().info('No voice detected (silence)')
        return

    text = self.transcribe_audio(audio_data)
    # ... rest of code
```

## Privacy Considerations

### Data Handling

**What OpenAI receives**:
- Audio file (WAV format)
- API key (for billing)

**What OpenAI does NOT receive**:
- User identity (unless in audio content)
- Location data
- Camera feeds

**OpenAI's Policy** (as of 2025):
- Audio data used to improve models (opt-out available)
- Not used for advertising
- Deleted after 30 days

### Best Practices

1. **Inform users**: Display message "Voice commands are processed by OpenAI Whisper"
2. **Minimize data**: Only send audio when voice detected (VAD)
3. **Local alternative**: Use `whisper.cpp` for sensitive environments
4. **Anonymize**: Filter out personal information before sending

## Cost Optimization

### Whisper API Pricing

**Current pricing**: $0.006 / minute of audio

**Example usage** (15-minute exercise):
- 15 minutes audio × $0.006 = **$0.09 total**
- For 30 students: $2.70

**Optimization strategies**:

### 1. Use VAD (Voice Activity Detection)

Only transcribe when voice detected → **Save 50-70%** (no silence transcribed)

### 2. Adjust Recording Duration

```yaml
duration: 3.0  # Shorter clips = less audio sent
```

**Trade-off**: Users must speak within 3-second windows

### 3. Batch Processing

Record 30 seconds, split into chunks, transcribe once → **Reduce API calls**

### 4. Local Whisper for Testing

Use local `whisper.cpp` during development, switch to API for demos.

## Summary

You've now integrated OpenAI Whisper into a ROS 2 node for voice-to-text:

1. **Whisper API**: State-of-the-art speech recognition (97%+ accuracy)
2. **ROS 2 Node**: `whisper_voice_node` publishes transcriptions to `/voice_commands`
3. **Audio Capture**: Uses `sounddevice` to record from microphone
4. **Improvements**: Noise reduction, Voice Activity Detection (VAD)
5. **Privacy**: Understanding OpenAI's data handling, local alternatives available
6. **Cost**: ~$0.09 for 15min exercise, optimizations can reduce by 50-70%

In the next section, you'll take transcribed commands and use GPT-4 to decompose them into robot action sequences.

## Review Questions

1. **What is the main advantage of Whisper over traditional speech recognition models?**
   <details>
   <summary>Answer</summary>
   Whisper is highly robust to noise, accents, and background sounds because it's trained on 680,000 hours of diverse, real-world audio (not just clean studio recordings). It also supports 99 languages zero-shot.
   </details>

2. **How much does the OpenAI Whisper API cost per minute of audio?**
   <details>
   <summary>Answer</summary>
   $0.006 per minute of audio (as of 2025). For a 15-minute exercise, this costs approximately $0.09 total.
   </details>

3. **What is Voice Activity Detection (VAD) and why is it useful?**
   <details>
   <summary>Answer</summary>
   VAD detects whether audio contains speech or just silence. It's useful for saving API costs (don't transcribe silence) and reducing unnecessary processing.
   </details>

4. **What ROS 2 topic does the Whisper node publish transcriptions to?**
   <details>
   <summary>Answer</summary>
   `/voice_commands` (type: `std_msgs/String`)
   </details>

5. **What are two privacy concerns with using the OpenAI Whisper API?**
   <details>
   <summary>Answer</summary>
   1) Audio data is sent to OpenAI servers (external processing), 2) OpenAI may use the data to improve models (though users can opt-out, and data is deleted after 30 days).
   </details>

---
