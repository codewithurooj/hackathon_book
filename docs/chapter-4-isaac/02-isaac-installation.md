# Isaac Sim Installation Guide

## Learning Objectives

By the end of this section, you will:

- Understand the different Isaac Sim installation options (native, container, cloud)
- Install Isaac Sim on your local machine or set up cloud access
- Verify your NVIDIA GPU and driver installation
- Obtain educational licensing for Isaac Sim
- Configure ROS 2 integration with Isaac Sim
- Troubleshoot common installation issues

## Before You Begin: GPU Check

Isaac Sim **requires an NVIDIA GPU**. Let's verify your hardware first.

### Check GPU on Linux

```bash
# Check if NVIDIA GPU is present
lspci | grep -i nvidia

# Check NVIDIA driver version
nvidia-smi
```

**Expected output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 525.147.05   Driver Version: 525.147.05   CUDA Version: 12.0     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|                               |                      |               MIG M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
|  0%   45C    P8    10W / 200W |    500MiB /  8192MiB |      2%      Default |
|                               |                      |                  N/A |
+-------------------------------+----------------------+----------------------+
```

**Minimum Requirements**:
- **GPU**: NVIDIA GTX 1060 (6GB VRAM) or better
- **Driver**: Version 525+ recommended
- **CUDA**: Version 11.8 or 12.0+

### Check GPU on Windows

```powershell
# Open PowerShell and run:
nvidia-smi
```

Or open **NVIDIA Control Panel** and check your GPU model under "System Information."

### No GPU? Choose an Alternative

If you don't have an NVIDIA GPU:

1. **Cloud GPU** (recommended): Rent GPU compute ($1-2/hour, ~$15-25 total)
2. **Shared University Lab**: Use GPU workstations on campus
3. **Pre-Recorded Demos**: Learn concepts without hands-on execution

Continue reading for cloud setup instructions.

---

## Installation Option 1: Native Install (Ubuntu 22.04)

**Best for**: Students with local NVIDIA GPU who want full control.

**Time**: 45-60 minutes
**Difficulty**: Intermediate

### Step 1: Install NVIDIA Drivers

```bash
# Update package lists
sudo apt update

# Install NVIDIA driver
sudo apt install nvidia-driver-525

# Reboot to load driver
sudo reboot

# Verify installation after reboot
nvidia-smi
```

**Troubleshooting**: If `nvidia-smi` shows errors, try:
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Step 2: Install Docker and NVIDIA Container Toolkit

Isaac Sim can run in a Docker container (recommended for easier setup):

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-docker2
sudo systemctl restart docker

# Test GPU access in Docker
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

**Expected**: You should see the same `nvidia-smi` output as before.

### Step 3: Get Educational License

NVIDIA Isaac Sim is **free for students** but requires registration:

1. Go to [NVIDIA Developer Program](https://developer.nvidia.com/isaac-sim)
2. Click **"Get Started"** or **"Download"**
3. Sign in with NVIDIA account (create if needed)
4. Select **"Educational Use"** when prompted
5. Accept terms and conditions
6. You'll get access to download links and NGC credentials

**Educational License Benefits**:
- Free access to Isaac Sim
- Access to NVIDIA NGC container registry
- Access to pre-trained models and datasets
- Community forum support

### Step 4: Download and Install Isaac Sim

**Option A: Omniverse Launcher (Easier)**

1. Download [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/download/)
2. Install and sign in with your NVIDIA account
3. In Launcher, go to **"Exchange"** tab
4. Search for **"Isaac Sim"**
5. Click **"Install"** (select version 2023.1.1)
6. Wait for download (10-20 GB)

**Option B: Direct Download**

```bash
# Download Isaac Sim (requires NGC credentials)
# This command downloads the standalone package
wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/nvidia/isaac-sim/versions/2023.1.1/zip' \
  -O isaac-sim-2023.1.1.zip

# Extract
unzip isaac-sim-2023.1.1.zip -d ~/isaac-sim

# Run setup script
cd ~/isaac-sim
./setup_python_env.sh
```

### Step 5: Run Isaac Sim

```bash
# Navigate to Isaac Sim directory
cd ~/isaac-sim

# Launch Isaac Sim
./isaac-sim.sh

# Or use Omniverse Launcher and click "Launch" button
```

**First Launch**:
- May take 2-5 minutes to start
- Will download additional dependencies
- May show shader compilation progress

**Expected**: Isaac Sim GUI opens with the main viewport.

### Step 6: Configure ROS 2 Integration

```bash
# Install ROS 2 Humble if not already installed
sudo apt install ros-humble-desktop-full

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install Isaac Sim ROS 2 bridge dependencies
sudo apt install ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge

# Add to your ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 7: Verify Installation

Run the verification script:

```bash
# Download verification script
cd ~/isaac-sim
python -c "
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({'headless': False})
from omni.isaac.core import World
world = World()
print('Isaac Sim installed successfully!')
simulation_app.close()
"
```

**Expected output**: `Isaac Sim installed successfully!`

---

## Installation Option 2: NVIDIA NGC Container (Recommended)

**Best for**: Students who want easier setup with Docker.

**Time**: 20-30 minutes
**Difficulty**: Beginner-Intermediate

### Step 1: Set Up NGC Account

1. Go to [NVIDIA NGC](https://ngc.nvidia.com/)
2. Sign up with your educational email
3. Go to **"Setup"** → **"Get API Key"**
4. Generate an API key (save it securely)

### Step 2: Log in to NGC Registry

```bash
# Log in to NGC container registry
docker login nvcr.io
# Username: $oauthtoken
# Password: <your-ngc-api-key>
```

### Step 3: Pull Isaac Sim Container

```bash
# Pull the Isaac Sim container (8-12 GB)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Verify the image
docker images | grep isaac-sim
```

### Step 4: Run Isaac Sim Container

```bash
# Create directory for persistent data
mkdir -p ~/isaac-sim-data

# Run Isaac Sim container
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -v ~/isaac-sim-data:/root/workspace \
  -p 8211:8211 -p 8899:8899 \
  nvcr.io/nvidia/isaac-sim:2023.1.1
```

**Inside the container**:
```bash
# Launch Isaac Sim
./runapp.sh

# Or run headless (no GUI) for data generation
./runheadless.native.sh
```

### Step 5: Access Web UI (Optional)

Isaac Sim container supports **web-based streaming**:

1. Run container with web streaming:
```bash
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" \
  -v ~/isaac-sim-data:/root/workspace \
  -p 8211:8211 -p 8899:8899 -p 8888:8888 \
  nvcr.io/nvidia/isaac-sim:2023.1.1

# Inside container, start streaming
./runapp.sh --enable-webrtc
```

2. Open browser to `http://localhost:8211`
3. You'll see Isaac Sim viewport in your browser

**Benefits**: Access Isaac Sim from any device on your network.

---

## Installation Option 3: Cloud GPU (AWS)

**Best for**: Students without local NVIDIA GPU.

**Cost**: ~$1-2/hour (~$15-25 for full module)
**Time**: 30-45 minutes setup
**Difficulty**: Intermediate

### Step 1: Create AWS Account

1. Go to [AWS](https://aws.amazon.com/)
2. Sign up (requires credit card, free tier available)
3. Verify email and complete account setup

### Step 2: Request GPU Instance Quota

AWS limits GPU instances by default:

1. Go to **Service Quotas** in AWS Console
2. Search for **EC2**
3. Find **"Running On-Demand G instances"**
4. Request quota increase to **4 vCPUs** (allows 1× g5.xlarge)
5. Wait for approval (usually 1-2 business days)

### Step 3: Launch EC2 Instance

1. Go to **EC2** Dashboard
2. Click **"Launch Instance"**
3. Configure:
   - **Name**: isaac-sim-instance
   - **AMI**: Ubuntu Server 22.04 LTS (HVM), SSD Volume Type
   - **Instance Type**: **g5.xlarge** (NVIDIA A10G GPU, 4 vCPUs, 16 GB RAM)
   - **Storage**: 50 GB gp3 (SSD)
   - **Security Group**: Allow SSH (port 22), HTTP (8211), custom ports if needed
4. Click **"Launch"**
5. Download the `.pem` key file

### Step 4: Connect to Instance

```bash
# Set permissions on key file
chmod 400 your-key.pem

# Connect to instance
ssh -i your-key.pem ubuntu@<instance-public-ip>
```

### Step 5: Install Isaac Sim on AWS

Once connected:

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install NVIDIA drivers
sudo apt install nvidia-driver-525 -y
sudo reboot
```

Reconnect after reboot:

```bash
ssh -i your-key.pem ubuntu@<instance-public-ip>

# Verify GPU
nvidia-smi
```

Now follow **Option 2 (NGC Container)** steps to pull and run Isaac Sim.

### Step 6: Access from Local Machine

**Option A: X11 Forwarding** (if you have X server on local machine):
```bash
ssh -X -i your-key.pem ubuntu@<instance-public-ip>
```

**Option B: Web Streaming** (recommended):
- Run Isaac Sim with `--enable-webrtc`
- Access via `http://<instance-public-ip>:8211`

### Step 7: Cost Management

**Important**: Stop instance when not in use!

```bash
# From AWS Console: EC2 → Instances → Select → Instance State → Stop
```

**Costs**:
- **g5.xlarge**: ~$1.00-1.50/hour (varies by region)
- **Storage**: ~$5/month for 50 GB
- **Total for module**: ~$15-25 if you stop instance when not using

**Tips**:
- Use **Spot Instances** for 60-70% discount (may be interrupted)
- Delete instance after completing module to avoid ongoing charges
- Set up **billing alerts** to avoid surprises

---

## Installation Option 4: Google Cloud Platform

Similar to AWS, but with different pricing:

1. Create GCP account
2. Enable Compute Engine API
3. Launch instance:
   - **Machine type**: n1-standard-4 + 1× NVIDIA T4
   - **OS**: Ubuntu 22.04 LTS
   - **Storage**: 50 GB SSD
4. Install NVIDIA drivers and Isaac Sim (same as AWS)

**Cost**: ~$0.80-1.20/hour + storage

---

## Installation Option 5: Shared University Lab

Many universities provide GPU workstations:

1. **Contact Your Department**:
   - Ask about GPU lab access for coursework
   - Request Isaac Sim installation if not already available

2. **Schedule Lab Time**:
   - Reserve specific time slots for Isaac work
   - Coordinate with other students

3. **Remote Access**:
   - Ask for SSH/VNC access to work remotely
   - Use web streaming if available

---

## Post-Installation: Verify Setup

### Test 1: Launch Isaac Sim

```bash
# Native or container
./isaac-sim.sh
# Or from Omniverse Launcher
```

**Expected**: GUI opens without errors.

### Test 2: Create a Simple Scene

In Isaac Sim:

1. **File → New Stage** (or Ctrl+N)
2. **Create → Mesh → Cube**
3. Click **Play** button (bottom-left)
4. Cube should fall due to gravity

**Expected**: Cube falls and hits ground plane.

### Test 3: Check GPU Utilization

While Isaac Sim is running:

```bash
nvidia-smi
```

**Expected**: GPU utilization > 0%, memory usage 1-4 GB.

### Test 4: ROS 2 Bridge Test

```bash
# Terminal 1: Source ROS 2
source /opt/ros/humble/setup.bash

# Terminal 2: Launch Isaac Sim with ROS 2 bridge
./isaac-sim.sh
```

In Isaac Sim:
1. **Isaac Examples → ROS2 → Clock**
2. Click **Play**

In Terminal 1:
```bash
ros2 topic list
```

**Expected**: You should see `/clock` topic published by Isaac Sim.

---

## Troubleshooting Common Issues

### Issue 1: `nvidia-smi` Not Found

**Solution**:
```bash
# Reinstall NVIDIA driver
sudo apt install --reinstall nvidia-driver-525
sudo reboot
```

### Issue 2: Docker Permission Denied

**Solution**:
```bash
sudo usermod -aG docker $USER
newgrp docker
# Or log out and log back in
```

### Issue 3: Isaac Sim Crashes on Launch

**Solution**:
```bash
# Check GPU memory
nvidia-smi

# Close other GPU applications
# Try launching with reduced resolution
./isaac-sim.sh --width 1280 --height 720
```

### Issue 4: Container Cannot Access GPU

**Solution**:
```bash
# Verify NVIDIA Container Toolkit
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

# If fails, reinstall
sudo apt purge nvidia-docker2
sudo apt install nvidia-docker2
sudo systemctl restart docker
```

### Issue 5: Slow Performance

**Possible causes**:
- Insufficient VRAM (< 4 GB)
- CPU bottleneck
- Thermal throttling

**Solutions**:
```bash
# Check GPU temperature
nvidia-smi

# Reduce physics rate in Isaac Sim
# Edit → Preferences → Physics → Simulation Rate → 30 Hz

# Disable ray tracing if enabled
# Viewport → Rendering → Real-Time
```

### Issue 6: ROS 2 Bridge Not Working

**Solution**:
```bash
# Ensure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Check ROS_DOMAIN_ID matches
echo $ROS_DOMAIN_ID
# Set if needed
export ROS_DOMAIN_ID=0

# Restart Isaac Sim
```

---

## Hardware Recommendations by Budget

### Budget: $0 (No GPU)

**Options**:
1. **Shared University Lab** (free)
2. **Pre-Recorded Demos** (learn concepts without hands-on)

### Budget: $15-50

**Recommended**: **AWS/GCP Cloud GPU**
- Rent for 10-20 hours
- Stop instance when not using
- Good for learning and short projects

### Budget: $200-500

**Recommended**: Used **NVIDIA GTX 1060/1070** or **RTX 2060**
- Buy used on eBay/Craigslist
- Sufficient for Isaac Sim
- Useful beyond this course

### Budget: $800-1500

**Recommended**: New **NVIDIA RTX 3060/4060** or better
- Excellent performance
- Future-proof for robotics work
- Good for research and professional development

---

## Summary

You have multiple options for running Isaac Sim:

| Option | Time | Cost | Difficulty | Best For |
|--------|------|------|------------|----------|
| **Native Install** | 45-60 min | $0 (if GPU) | Medium | Local GPU owners |
| **NGC Container** | 20-30 min | $0 (if GPU) | Easy-Medium | Docker users |
| **AWS Cloud** | 30-45 min | $15-25 | Medium | No local GPU |
| **GCP Cloud** | 30-45 min | $15-25 | Medium | No local GPU |
| **University Lab** | Varies | $0 | Easy | Students with lab access |
| **Pre-Recorded** | 0 min | $0 | Easy | Learn without hands-on |

**Recommended Path**:
- Have GPU? → Native Install or NGC Container
- No GPU? → Cloud (AWS/GCP) or University Lab
- Limited budget? → Pre-Recorded Demos (concept learning)

**Next Steps**:

1. Choose your installation option
2. Complete setup and verification
3. Continue to [Creating Photorealistic Scenes](./03-photorealistic-scenes.md)

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/app_isaacsim/)
- [NGC Registry](https://ngc.nvidia.com/)
- [NVIDIA Isaac Forums](https://forums.developer.nvidia.com/c/isaac/)
- [Isaac Sim Troubleshooting](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/troubleshooting.html)
- [AWS EC2 GPU Instances](https://aws.amazon.com/ec2/instance-types/g5/)
