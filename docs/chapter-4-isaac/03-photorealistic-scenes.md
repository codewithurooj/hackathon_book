# Creating Photorealistic Scenes in Isaac Sim

## Learning Objectives

By the end of this section, you will:

- Create and navigate Isaac Sim scenes using the USD format
- Add realistic materials (PBR) to objects for photorealistic rendering
- Configure lighting (HDR, directional, point lights) for realistic shadows
- Import robots from URDF into Isaac Sim as USD
- Understand scene composition and layering in USD
- Set up cameras for perception tasks
- Optimize scene performance for real-time simulation

## Introduction to USD Scenes

Isaac Sim uses **USD (Universal Scene Description)** to define 3D scenes. USD is a powerful format that allows:

- **Composition**: Combine multiple USD files (layers) into complex scenes
- **Non-Destructive Editing**: Modify scenes without altering original files
- **Version Control**: Text-based format works with Git
- **Interoperability**: Import/export from Blender, Maya, Unreal Engine

### USD Hierarchy

A USD scene is organized as a **hierarchy of prims** (primitives):

```
World (root)
├── GroundPlane
├── Lighting
│   ├── DomeLight (HDR environment)
│   └── DirectionalLight (sun)
├── Props
│   ├── Table
│   ├── Chair
│   └── Box
└── Robots
    └── Humanoid
```

Each **prim** can have:
- **Transforms**: position, rotation, scale
- **Geometry**: meshes, shapes
- **Materials**: colors, textures, properties
- **Physics**: mass, collision shapes
- **Semantics**: labels for synthetic data

---

## Your First Isaac Sim Scene

Let's create a simple indoor room scene from scratch.

### Step 1: Launch Isaac Sim

```bash
# Native installation
./isaac-sim.sh

# Or from Omniverse Launcher
# Click "Isaac Sim" → "Launch"

# Or in container
./runapp.sh
```

### Step 2: Create New Stage

1. **File → New** (or Ctrl+N)
2. This creates an empty USD stage

### Step 3: Add Ground Plane

1. **Create → Physics → Ground Plane**
2. A large flat plane appears at z=0
3. This provides a surface for objects to rest on

### Step 4: Add Lighting

Good lighting is crucial for photorealistic rendering.

**Add Dome Light (HDR Environment)**:

1. **Create → Light → Dome Light**
2. In **Property** panel (right side):
   - **Intensity**: 1000
   - **Texture**: Click folder icon
   - Navigate to `isaac-sim/materials/HDRI/` and select `kloofendal_48d_partly_cloudy_4k.exr`
3. This adds realistic environment lighting

**Add Directional Light (Sun)**:

1. **Create → Light → Directional Light**
2. Set properties:
   - **Intensity**: 3000
   - **Angle**: 0.5 (soft shadows)
   - **Rotation**: X=-45°, Y=30° (sun angle)

### Step 5: Add Simple Room Geometry

**Create Floor**:

1. **Create → Mesh → Cube**
2. Rename to "Floor" (right-click → Rename)
3. Set transform:
   - **Scale**: X=10, Y=10, Z=0.1
   - **Position**: X=0, Y=0, Z=-0.05

**Create Walls**:

1. **Create → Mesh → Cube** (Repeat 4 times)
2. Name them "WallNorth", "WallSouth", "WallEast", "WallWest"

Configure each wall:

**WallNorth**:
- **Position**: X=0, Y=5, Z=1.5
- **Scale**: X=10, Y=0.2, Z=3

**WallSouth**:
- **Position**: X=0, Y=-5, Z=1.5
- **Scale**: X=10, Y=0.2, Z=3

**WallEast**:
- **Position**: X=5, Y=0, Z=1.5
- **Scale**: X=0.2, Y=10, Z=3

**WallWest**:
- **Position**: X=-5, Y=0, Z=1.5
- **Scale**: X=0.2, Y=10, Z=3

### Step 6: Apply Realistic Materials

Now let's make the scene photorealistic with materials.

**Apply Wood Floor Material**:

1. Select "Floor" prim
2. In **Property** panel → **Material** section
3. Click **"+"** → **Create New Material**
4. Name it "WoodFloor"
5. In Material properties:
   - **Base Color**: Load texture from `materials/textures/wood_floor_diffuse.jpg` (if available)
   - Or set RGB: (0.6, 0.4, 0.2) for brown color
   - **Roughness**: 0.6
   - **Metallic**: 0.0

**Apply White Paint to Walls**:

1. Select all wall prims (Shift+Click)
2. **Create → Material** → Name "WhitePaint"
3. Properties:
   - **Base Color**: RGB (0.95, 0.95, 0.95)
   - **Roughness**: 0.8
   - **Metallic**: 0.0

### Step 7: Add Props (Table, Chairs)

**Add Table**:

1. **Create → Mesh → Cube** (for table top)
2. **Position**: X=0, Y=0, Z=0.8
3. **Scale**: X=2, Y=1, Z=0.05

Add 4 legs (create 4 cubes):
- **Scale**: X=0.1, Y=0.1, Z=0.8
- **Positions**: (±0.9, ±0.4, 0.4)

Apply **Wood Material**:
- **Base Color**: RGB (0.4, 0.25, 0.15)
- **Roughness**: 0.5

### Step 8: Add Physics

To make objects interact realistically:

1. Select each prop (table, walls)
2. **Add → Physics → Collider** (for static objects)
3. For the floor: already has physics from Ground Plane

### Step 9: Save Scene

1. **File → Save As**
2. Name: `simple_room.usd`
3. Save location: `examples/chapter-4-isaac/isaac_sim_scenes/`

### Step 10: Test with Play

1. Click **Play** button (triangle, bottom-left)
2. Physics simulation starts
3. Objects should remain stable

**Expected**: Room remains static, lighting looks realistic.

---

## Advanced Scene Creation: Office Environment

Let's create a more complex scene suitable for VSLAM and navigation testing.

### Scene Requirements

- Multiple rooms with corridors
- Varied textures (walls, floors, furniture)
- Obstacles for navigation
- Good feature points for VSLAM (corners, textures)

### Step 1: Create Floor Plan

Instead of creating manually, we can use **pre-built assets** or **import from USD libraries**.

**Option A: Use Isaac Assets**:

```python
# In Isaac Sim Python console (Window → Script Editor → Python)
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add office asset from Isaac library
add_reference_to_stage(
    usd_path="/Isaac/Environments/Office/office.usd",
    prim_path="/World/Office"
)
```

**Option B: Build Custom Layout**:

Create multiple rooms following the simple room approach:

1. Room 1: 10m × 10m (start area)
2. Corridor: 2m × 8m (connecting rooms)
3. Room 2: 8m × 8m (navigation goal area)

### Step 2: Add Realistic Props

**Import from Content Browser**:

1. **Content** panel (bottom) → **Props**
2. Drag and drop items:
   - Desks
   - Chairs
   - Bookshelves
   - Potted plants
   - Computer monitors

**Or download from Sketchfab/TurboSquid**:
- Export as `.obj` or `.fbx`
- Import: **File → Import** → Select file
- Isaac Sim converts to USD automatically

### Step 3: Configure Realistic Materials

For each prop, apply **PBR materials**:

**Desk Material**:
- **Base Color**: Wood texture or RGB (0.5, 0.35, 0.2)
- **Roughness**: 0.4
- **Metallic**: 0.0

**Metal Chair**:
- **Base Color**: RGB (0.2, 0.2, 0.2)
- **Roughness**: 0.3
- **Metallic**: 0.9

**Glass Window** (if adding windows):
- **Base Color**: RGB (0.95, 0.95, 0.95)
- **Roughness**: 0.05
- **Opacity**: 0.3
- **IOR (Index of Refraction)**: 1.5

### Step 4: Advanced Lighting Setup

**Dome Light with HDR**:
- Use `indoor_studio.exr` or similar HDRI for indoor scenes
- **Intensity**: 800-1200

**Additional Point Lights** (ceiling lights):

1. **Create → Light → Sphere Light**
2. **Position**: Ceiling height (Z=2.8)
3. **Radius**: 0.2
4. **Intensity**: 10000
5. Duplicate for multiple ceiling lights (Ctrl+D)

**Window Lights** (if room has windows):

1. **Create → Light → Rect Light**
2. **Position**: Window location
3. **Scale**: Match window size
4. **Intensity**: 5000
5. **Color Temperature**: 6500K (daylight)

### Step 5: Texture Variation for VSLAM

VSLAM requires **texture features** to track. Ensure:

- Walls have texture or patterns (not blank white)
- Floor has tile or wood grain patterns
- Props provide visual features

**Add Wall Texture**:
- Apply **brick texture** or **wallpaper texture**
- Download from [Poly Haven](https://polyhaven.com/textures) (free PBR textures)
- In Material: **Base Color** → Load downloaded texture

---

## Importing Robots from URDF

Isaac Sim can import ROS URDF files and convert them to USD.

### Step 1: Prepare URDF

Ensure your URDF has:
- Correct mesh file paths
- Material definitions
- Collision geometries

Example: Use a humanoid URDF from `examples/chapter-2-ros2/urdf/`

### Step 2: Import URDF in Isaac Sim

**Method 1: GUI Import**:

1. **Isaac Utils → URDF Importer**
2. Click **"Select URDF File"**
3. Navigate to your `.urdf` file
4. Configure import settings:
   - **Import Inertia**: ✓
   - **Import Materials**: ✓
   - **Fix Base Link**: ✓ (for fixed-base robots)
5. Click **"Import"**

**Method 2: Python Script**:

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.urdf")

from omni.isaac.urdf import _urdf

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()

# Convert URDF to USD
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True

result, prim_path = urdf_interface.parse_urdf(
    urdf_path="/path/to/robot.urdf",
    import_config=import_config
)

print(f"Robot imported at: {prim_path}")
```

### Step 3: Adjust Robot Placement

1. Select robot root prim in **Stage** panel
2. Set **Transform**:
   - **Position**: X=0, Y=0, Z=0.1 (above ground)
   - **Rotation**: as needed

### Step 4: Add Physics to Robot

Isaac Sim should automatically add:
- **Articulation Root**: Makes robot controllable
- **Revolute/Prismatic Joints**: For each joint
- **Collision Meshes**: From URDF

Verify:
1. Select robot root
2. Check **Property** panel for **Articulation Root**
3. Click **Play** → robot should fall/settle under gravity

---

## Setting Up Cameras for Perception

Cameras are essential for VSLAM and object detection.

### Add RGB Camera

1. **Create → Camera**
2. Rename to "RGBCamera"
3. **Position**: X=0, Y=0, Z=1.5 (eye height)
4. **Rotation**: Point forward

**Configure Camera Properties**:

- **Focal Length**: 24mm (wide angle for VSLAM)
- **Horizontal Aperture**: 20.955mm
- **Resolution**: 1280×720 (HD)
- **Clipping Range**: Near=0.1m, Far=1000m

### Add Depth Camera

1. **Create → Camera**
2. Rename to "DepthCamera"
3. **Add → Isaac Sensors → Camera Sensor**
4. Configure:
   - **Type**: Depth
   - **Resolution**: 640×480
   - **Render Product**: Create new

### Camera on Robot

To attach camera to robot:

1. Drag camera prim under robot's head link in **Stage** hierarchy
2. Adjust relative transform

Example hierarchy:
```
/World/Humanoid
└── head_link
    └── RGBCamera
```

### Visualize Camera View

1. Select camera in **Stage**
2. **Viewport** → **Camera** → Select your camera
3. View switches to camera's perspective
4. Press **Play** → see real-time camera output

### Publish Camera to ROS 2

```python
# In Isaac Sim script
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

from omni.isaac.ros2_bridge import CameraBridge

# Create ROS 2 bridge for camera
camera_bridge = CameraBridge(
    camera_prim_path="/World/Humanoid/head_link/RGBCamera",
    topic_name="/camera/image_raw",
    frame_id="camera_link"
)
```

When you run simulation, camera images will publish to ROS 2 topic `/camera/image_raw`.

---

## Scene Optimization for Real-Time Performance

Complex scenes can slow down simulation. Optimize:

### 1. Reduce Polygon Count

- Use **low-poly meshes** for background objects
- Apply **Level of Detail (LOD)** for distant objects

### 2. Optimize Materials

- Avoid complex shader networks
- Use simple PBR materials
- Limit number of textures

### 3. Adjust Physics Rate

1. **Edit → Preferences → Physics**
2. **Physics Rate**: 60 Hz (default) → 30 Hz for slower physics
3. **Rendering Rate**: Keep at 60 Hz for smooth visuals

### 4. Disable Unnecessary Features

- Disable **Ray Tracing** if not needed: **Viewport → Rendering → Real-Time**
- Disable **Ambient Occlusion**
- Reduce **Shadow Quality**

### 5. Use Instancing

For repeated objects (chairs, plants):

```python
# Instead of duplicating, use instancing
from pxr import Usd, UsdGeom

stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/World/Props/Chair")
prim.GetReferences().AddReference("/path/to/chair.usd")
```

Instances share geometry, reducing memory.

---

## Example: Complete Office Scene Script

Save this as `create_office_scene.py`:

```python
from omni.isaac.kit import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Create world
world = World(stage_units_in_meters=1.0)

# Add ground plane
world.scene.add_default_ground_plane()

# Add office environment
add_reference_to_stage(
    usd_path="/Isaac/Environments/Office/office.usd",
    prim_path="/World/Office"
)

# Add some dynamic obstacles (boxes)
for i in range(5):
    box = world.scene.add(
        DynamicCuboid(
            prim_path=f"/World/Obstacles/Box_{i}",
            name=f"box_{i}",
            position=np.array([np.random.uniform(-3, 3),
                              np.random.uniform(-3, 3),
                              0.5]),
            size=np.array([0.5, 0.5, 0.5]),
            color=np.array([np.random.rand(), np.random.rand(), np.random.rand()])
        )
    )

# Add lighting
from pxr import UsdLux, Gf
stage = omni.usd.get_context().get_stage()

# Dome light
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1000)

# Directional light (sun)
dir_light = UsdLux.DistantLight.Define(stage, "/World/SunLight")
dir_light.CreateIntensityAttr(3000)
dir_light.CreateAngleAttr(0.5)

# Save scene
omni.usd.get_context().save_as_stage("/path/to/office_environment.usd")

print("Office scene created successfully!")

# Run simulation
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

Run:
```bash
cd ~/isaac-sim
./python.sh /path/to/create_office_scene.py
```

---

## Scene Composition with USD Layers

USD allows **layered composition** for modular scenes.

### Example: Modular Scene Structure

**Base Layer** (`base_room.usd`):
- Ground plane
- Walls
- Basic lighting

**Props Layer** (`office_props.usd`):
- Furniture
- Decorations

**Robot Layer** (`humanoid_robot.usd`):
- Robot model
- Sensors

**Combine layers**:

```python
from pxr import Usd

stage = Usd.Stage.CreateNew("complete_scene.usd")

# Add base room
stage.GetRootLayer().subLayerPaths.append("base_room.usd")

# Add props
stage.GetRootLayer().subLayerPaths.append("office_props.usd")

# Add robot
stage.GetRootLayer().subLayerPaths.append("humanoid_robot.usd")

stage.Save()
```

**Benefits**:
- Reuse base scenes
- Swap props without changing base
- Version control each layer independently

---

## Summary

You learned to:

1. **Create USD scenes** in Isaac Sim with realistic geometry
2. **Apply PBR materials** for photorealistic rendering (wood, metal, glass)
3. **Configure lighting** (dome lights, directional lights, point lights)
4. **Import robots** from URDF files
5. **Set up cameras** for perception (RGB, depth) and ROS 2 integration
6. **Optimize scenes** for real-time performance
7. **Use USD layers** for modular scene composition

**Key Takeaways**:

- **Photorealism matters**: Realistic scenes improve sim-to-real transfer
- **USD is powerful**: Layered composition enables complex, maintainable scenes
- **Performance optimization**: Balance visual quality with simulation speed
- **VSLAM needs texture**: Ensure sufficient visual features in scenes

**Next Steps**:

With photorealistic scenes ready, continue to [Isaac ROS Introduction](./04-isaac-ros-intro.md) to learn GPU-accelerated perception.

## Practice Exercises

1. **Exercise 1**: Create a simple room with wood floor, white walls, and a table
2. **Exercise 2**: Import a humanoid URDF and place it in the room
3. **Exercise 3**: Add an RGB camera to the robot's head and publish to ROS 2
4. **Exercise 4**: Create an office environment with multiple rooms and realistic props
5. **Exercise 5**: Optimize a complex scene to run at 60 FPS

## Additional Resources

- [USD Introduction](https://graphics.pixar.com/usd/docs/index.html)
- [Isaac Sim Scene Creation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_gui_simple_scene.html)
- [PBR Materials Guide](https://marmoset.co/posts/basic-theory-of-physically-based-rendering/)
- [Poly Haven Textures](https://polyhaven.com/textures) (free PBR textures)
- [Sketchfab](https://sketchfab.com/) (3D models, many free)
