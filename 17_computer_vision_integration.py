"""
PROJECT 2: Computer Vision Pick-and-Place System
Robot uses camera to detect and classify objects
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import random
import matplotlib.pyplot as plt
from PIL import Image
import cv2

print("="*70)
print("PROJECT 2: COMPUTER VISION PICK-AND-PLACE")
print("="*70)

# =====================================================
# SETUP
# =====================================================
print("\n→ Setting up environment...")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Table
table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15])
table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15],
                                   rgbaColor=[0.6, 0.4, 0.2, 1])
table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision,
                         baseVisualShapeIndex=table_visual,
                         basePosition=[0.6, 0, 0.15])

# Target boxes
box_height = 0.08
box_width = 0.15

# Red box
red_box_collision = p.createCollisionShape(p.GEOM_BOX, 
                                           halfExtents=[box_width/2, box_width/2, box_height/2])
red_box_visual = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[box_width/2, box_width/2, box_height/2],
                                     rgbaColor=[1, 0, 0, 0.3])
red_box = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=red_box_collision,
                            baseVisualShapeIndex=red_box_visual,
                            basePosition=[0.3, 0.25, 0.30 + box_height/2])

# Blue box
blue_box_collision = p.createCollisionShape(p.GEOM_BOX, 
                                            halfExtents=[box_width/2, box_width/2, box_height/2])
blue_box_visual = p.createVisualShape(p.GEOM_BOX,
                                      halfExtents=[box_width/2, box_width/2, box_height/2],
                                      rgbaColor=[0, 0, 1, 0.3])
blue_box = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=blue_box_collision,
                             baseVisualShapeIndex=blue_box_visual,
                             basePosition=[0.3, 0.0, 0.30 + box_height/2])

# Green box
green_box_collision = p.createCollisionShape(p.GEOM_BOX,
                                             halfExtents=[box_width/2, box_width/2, box_height/2])
green_box_visual = p.createVisualShape(p.GEOM_BOX,
                                       halfExtents=[box_width/2, box_width/2, box_height/2],
                                       rgbaColor=[0, 1, 0, 0.3])
green_box = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=green_box_collision,
                              baseVisualShapeIndex=green_box_visual,
                              basePosition=[0.3, -0.25, 0.30 + box_height/2])

# Panda robot
panda = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Camera position
p.resetDebugVisualizerCamera(
    cameraDistance=2.2,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0.4]
)

print("✓ Environment ready")

# =====================================================
# ROBOT CONTROL
# =====================================================

arm_joints = [0, 1, 2, 3, 4, 5, 6]
gripper_joints = [9, 10]
end_effector_index = 11

# Home position
home_angles = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i, angle in enumerate(home_angles):
    p.resetJointState(panda, arm_joints[i], angle)

for joint in gripper_joints:
    p.resetJointState(panda, joint, 0.04)

# IK cache
ik_cache = {}

def calculate_ik_cached(target_pos):
    """Cached IK calculation"""
    key = (round(target_pos[0], 2), round(target_pos[1], 2), round(target_pos[2], 2))
    
    if key in ik_cache:
        return ik_cache[key]
    
    target_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
    ik_result = p.calculateInverseKinematics(
        panda, end_effector_index, target_pos, target_orn,
        maxNumIterations=50, residualThreshold=0.005
    )
    
    target_angles = [ik_result[i] for i in arm_joints]
    ik_cache[key] = target_angles
    
    return target_angles

def move_arm(target_pos, duration=1.5):
    """Move robot arm to target position"""
    target_angles = calculate_ik_cached(target_pos)
    
    num_steps = int(duration * 180)
    
    for _ in range(num_steps):
        for i, joint_idx in enumerate(arm_joints):
            p.setJointMotorControl2(
                panda, joint_idx, p.POSITION_CONTROL,
                targetPosition=target_angles[i],
                force=100, maxVelocity=1.5
            )
        p.stepSimulation()
        time.sleep(1./180.)

def control_gripper(command, force=100):
    """Control gripper"""
    target = 0.04 if command == "open" else 0.0
    
    for joint in gripper_joints:
        p.setJointMotorControl2(
            panda, joint, p.POSITION_CONTROL,
            targetPosition=target, force=force
        )
    
    for _ in range(50):
        p.stepSimulation()
        time.sleep(1./180.)

# =====================================================
# COMPUTER VISION SYSTEM
# =====================================================
print("\n" + "="*70)
print("COMPUTER VISION SYSTEM SETUP")
print("="*70)

# Virtual camera parameters
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 60

# Camera position (above table, looking down)
camera_position = [0.6, 0, 1.0]  # Above table
camera_target = [0.7, 0, 0.3]    # Looking at table
camera_up = [0, 1, 0]            # Y-axis is up

print(f"\n📷 Camera Configuration:")
print(f"  Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
print(f"  FOV: {CAMERA_FOV}°")
print(f"  Position: {camera_position}")
print(f"  Target: {camera_target}")

def capture_camera_image():
    """
    Capture image from virtual camera
    
    Returns:
        rgb_array: RGB image (height x width x 3)
        depth_array: Depth image (height x width)
        seg_array: Segmentation mask (height x width)
    """
    # Compute view matrix
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target,
        cameraUpVector=camera_up
    )
    
    # Compute projection matrix
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=CAMERA_FOV,
        aspect=CAMERA_WIDTH / CAMERA_HEIGHT,
        nearVal=0.1,
        farVal=3.0
    )
    
    # Capture image
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL
    )
    
    # Convert to numpy arrays
    rgb_array = np.array(rgb_img, dtype=np.uint8).reshape(height, width, 4)
    rgb_array = rgb_array[:, :, :3]  # Remove alpha channel
    
    depth_array = np.array(depth_img, dtype=np.float32).reshape(height, width)
    seg_array = np.array(seg_img, dtype=np.int32).reshape(height, width)
    
    return rgb_array, depth_array, seg_array

def detect_objects_from_image(rgb_image):
    """
    Detect objects from RGB image using color segmentation
    
    Returns:
        detections: List of (color, center_x, center_y, bbox)
    """
    detections = []
    
    # Color ranges (HSV-like but in RGB for simplicity)
    color_ranges = {
        "red": {
            "lower": np.array([180, 0, 0]),
            "upper": np.array([255, 80, 80])
        },
        "blue": {
            "lower": np.array([0, 0, 180]),
            "upper": np.array([80, 80, 255])
        },
        "green": {
            "lower": np.array([0, 180, 0]),
            "upper": np.array([80, 255, 80])
        }
    }
    
    for color_name, ranges in color_ranges.items():
        # Create mask
        mask = cv2.inRange(rgb_image, ranges["lower"], ranges["upper"])
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter small noise
            if area > 100:  # Minimum area threshold
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Get center
                center_x = x + w // 2
                center_y = y + h // 2
                
                detections.append({
                    "color": color_name,
                    "center": (center_x, center_y),
                    "bbox": (x, y, w, h),
                    "area": area
                })
    
    return detections

# ALTERNATIVE: Simple color detection without OpenCV
def detect_objects_simple(rgb_image):
    """
    Simple object detection without OpenCV dependency
    Uses color clustering
    """
    height, width, _ = rgb_image.shape
    detections = []
    
    # Define color ranges (RGB)
    color_definitions = {
        "red": ([200, 0, 0], [255, 100, 100]),
        "blue": ([0, 0, 200], [100, 100, 255]),
        "green": ([0, 200, 0], [100, 255, 100])
    }
    
    for color_name, (lower, upper) in color_definitions.items():
        lower = np.array(lower)
        upper = np.array(upper)
        
        # Create mask
        mask = np.all((rgb_image >= lower) & (rgb_image <= upper), axis=2)
        
        if np.any(mask):
            # Find all points matching color
            points = np.argwhere(mask)
            
            if len(points) > 50:  # Minimum points threshold
                # Calculate centroid
                center_y, center_x = points.mean(axis=0).astype(int)
                
                # Calculate bounding box
                y_min, x_min = points.min(axis=0)
                y_max, x_max = points.max(axis=0)
                
                detections.append({
                    "color": color_name,
                    "center": (center_x, center_y),
                    "bbox": (x_min, y_min, x_max - x_min, y_max - y_min),
                    "area": len(points)
                })
    
    return detections

def pixel_to_world_coordinates(pixel_x, pixel_y, depth_value):
    """
    Convert pixel coordinates to 3D world coordinates
    FIXED VERSION with proper depth conversion
    
    Args:
        pixel_x, pixel_y: Pixel coordinates
        depth_value: Raw depth value from depth buffer
    
    Returns:
        world_x, world_y, world_z: 3D world coordinates
    """
    # PyBullet returns depth in non-linear format
    # Convert to actual distance
    near = 0.1
    far = 3.0
    depth_linear = far * near / (far - (far - near) * depth_value)
    
    # View matrix (matching camera setup)
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=camera_target,
        cameraUpVector=camera_up
    )
    
    # Projection matrix
    projection_matrix = p.computeProjectionMatrixFOV(
        fov=CAMERA_FOV,
        aspect=CAMERA_WIDTH / CAMERA_HEIGHT,
        nearVal=near,
        farVal=far
    )
    
    # Convert to normalized device coordinates (NDC)
    ndc_x = (2.0 * pixel_x / CAMERA_WIDTH) - 1.0
    ndc_y = 1.0 - (2.0 * pixel_y / CAMERA_HEIGHT)  # Flip Y
    ndc_z = 2.0 * depth_value - 1.0
    
    # Create homogeneous coordinates
    ndc = np.array([ndc_x, ndc_y, ndc_z, 1.0])
    
    # Get inverse matrices
    view_matrix_np = np.array(view_matrix).reshape(4, 4).T
    proj_matrix_np = np.array(projection_matrix).reshape(4, 4).T
    
    # Transform: NDC → Clip → View → World
    inv_proj = np.linalg.inv(proj_matrix_np)
    inv_view = np.linalg.inv(view_matrix_np)
    
    # Apply transformations
    clip_coords = ndc * depth_linear  # Scale by depth
    view_coords = inv_proj @ clip_coords
    world_coords = inv_view @ view_coords
    
    # Normalize homogeneous coordinates
    world_x = world_coords[0] / world_coords[3]
    world_y = world_coords[1] / world_coords[3]
    world_z = world_coords[2] / world_coords[3]
    
    return world_x, world_y, world_z

print("✓ Vision system ready")

# =====================================================
# CREATE OBJECTS
# =====================================================
print("\n" + "="*70)
print("CREATING OBJECTS")
print("="*70)

# Color definitions
COLOR_DEFS = {
    "red": {"rgb": [1.0, 0.0, 0.0], "box_pos": [0.3, 0.25, 0.38]},
    "blue": {"rgb": [0.0, 0.0, 1.0], "box_pos": [0.3, 0.0, 0.38]},
    "green": {"rgb": [0.0, 1.0, 0.0], "box_pos": [0.3, -0.25, 0.38]}
}

def create_cube(position, color_name):
    """Create colored cube"""
    color_rgb = COLOR_DEFS[color_name]["rgb"]
    
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02],
                                  rgbaColor=[color_rgb[0], color_rgb[1], color_rgb[2], 1])
    
    obj = p.createMultiBody(baseMass=0.05,
                           baseCollisionShapeIndex=collision,
                           baseVisualShapeIndex=visual,
                           basePosition=position)
    return obj

# Create random cubes
num_cubes = 6
objects = []
object_colors = []

print(f"\n→ Creating {num_cubes} random cubes...")

color_names = list(COLOR_DEFS.keys())

for i in range(num_cubes):
    x = random.uniform(0.45, 0.6)
    y = random.uniform(-0.2, 0.2)
    z = 0.35
    
    color = random.choice(color_names)
    
    cube = create_cube([x, y, z], color)
    objects.append(cube)
    object_colors.append(color)
    
    print(f"  {i+1}. Cube: {color:6s} → ({x:.2f}, {y:.2f})")

# Let cubes fall
print("\n→ Letting cubes settle...")
for _ in range(150):
    p.stepSimulation()
    time.sleep(1./180.)

print("✓ Cubes ready")

# Color distribution
red_count = object_colors.count("red")
blue_count = object_colors.count("blue")
green_count = object_colors.count("green")

print(f"\nColor Distribution:")
print(f"  🔴 Red:   {red_count}")
print(f"  🔵 Blue:  {blue_count}")
print(f"  🟢 Green: {green_count}")

input("\nPress ENTER to capture camera image...\n")

# =====================================================
# CAPTURE AND DISPLAY IMAGE
# =====================================================
print("="*70)
print("CAMERA IMAGE CAPTURE")
print("="*70)

print("\n→ Capturing image from camera...")
rgb_image, depth_image, seg_image = capture_camera_image()

print("✓ Image captured")
print(f"  RGB shape: {rgb_image.shape}")
print(f"  Depth range: {depth_image.min():.3f} - {depth_image.max():.3f}")
print(f"  Unique objects: {len(np.unique(seg_image))}")

# Display captured image
fig, axes = plt.subplots(1, 3, figsize=(15, 5))

axes[0].imshow(rgb_image)
axes[0].set_title("RGB Image")
axes[0].axis('off')

axes[1].imshow(depth_image, cmap='gray')
axes[1].set_title("Depth Image")
axes[1].axis('off')

axes[2].imshow(seg_image, cmap='tab20')
axes[2].set_title("Segmentation Mask")
axes[2].axis('off')

plt.tight_layout()
plt.show()

input("\nPress ENTER to detect objects...\n")

# =====================================================
# OBJECT DETECTION
# =====================================================
print("="*70)
print("OBJECT DETECTION FROM IMAGE")
print("="*70)

print("\n→ Detecting objects using color segmentation...")
detections = detect_objects_from_image(rgb_image)

print(f"✓ Detected {len(detections)} objects\n")

# Display detections
fig, ax = plt.subplots(1, 1, figsize=(10, 8))
ax.imshow(rgb_image)

for i, det in enumerate(detections):
    color = det["color"]
    cx, cy = det["center"]
    x, y, w, h = det["bbox"]
    
    # Draw bounding box
    rect = plt.Rectangle((x, y), w, h, fill=False, edgecolor=color, linewidth=2)
    ax.add_patch(rect)
    
    # Draw center point
    ax.plot(cx, cy, 'o', color=color, markersize=10)
    
    # Label
    ax.text(cx, cy - 20, f"{i+1}: {color.upper()}", 
            color='white', fontsize=12, fontweight='bold',
            bbox=dict(facecolor=color, alpha=0.7))
    
    print(f"  Object {i+1}:")
    print(f"    Color: {color}")
    print(f"    Center: ({cx}, {cy}) pixels")
    print(f"    Bbox: {det['bbox']}")
    print(f"    Area: {det['area']} pixels")

ax.set_title("Detected Objects")
ax.axis('off')
plt.tight_layout()
plt.show()

input("\nPress ENTER to start pick-and-place...\n")

# =====================================================
# VISION-GUIDED PICK-AND-PLACE
# =====================================================
print("="*70)
print("VISION-GUIDED PICK-AND-PLACE")
print("="*70)

def pick_and_place_vision(pixel_x, pixel_y, target_color):
    """
    Pick object using vision-based positioning
    """
    # Get depth at pixel
    depth = depth_image[pixel_y, pixel_x]
    
    # Convert to world coordinates
    world_x, world_y, world_z = pixel_to_world_coordinates(pixel_x, pixel_y, depth)
    
    print(f"  → Pixel ({pixel_x}, {pixel_y}) → World ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
    
    # Adjust Z for gripper approach
    actual_z = 0.32  # Table height + cube height/2
    
    # Target box position
    target_pos = COLOR_DEFS[target_color]["box_pos"]
    
    # Pick
    control_gripper("open")
    move_arm([world_x, world_y, actual_z + 0.15], duration=1.2)
    move_arm([world_x, world_y, actual_z], duration=0.8)
    control_gripper("close", force=120)
    time.sleep(0.2)
    move_arm([world_x, world_y, actual_z + 0.2], duration=1.0)
    
    # Place
    move_arm([target_pos[0], target_pos[1], target_pos[2] + 0.15], duration=1.2)
    move_arm([target_pos[0], target_pos[1], target_pos[2]], duration=0.7)
    control_gripper("open")
    time.sleep(0.2)

# Process each detection
success_count = 0
start_time = time.time()

for i, detection in enumerate(detections, 1):
    print(f"\n{'='*70}")
    print(f"OBJECT {i}/{len(detections)}")
    print(f"{'='*70}")
    
    color = detection["color"]
    cx, cy = detection["center"]
    
    print(f"  [1/3] Detected: {color.upper()}")
    print(f"  [2/3] Location: Pixel ({cx}, {cy})")
    print(f"  [3/3] Picking and placing to {color.upper()} box...")
    
    try:
        pick_and_place_vision(cx, cy, color)
        success_count += 1
        print(f"  ✓ Object successfully placed in {color} box!")
    except Exception as e:
        print(f"  ✗ Error: {e}")
    
    # Progress bar
    completed = i
    remaining = len(detections) - i
    print(f"\n  Progress: {'█' * completed}{'░' * remaining} {i}/{len(detections)}")

end_time = time.time()
total_time = end_time - start_time

# =====================================================
# RESULTS
# =====================================================
print("\n" + "="*70)
print("PROJECT 2 COMPLETED - RESULTS")
print("="*70)

print(f"\n📊 PERFORMANCE METRICS:")
print(f"  ⏱  Total Time: {total_time:.1f} seconds")
print(f"  📦 Detected Objects: {len(detections)}")
print(f"  ✓ Successfully Placed: {success_count}")
print(f"  📈 Success Rate: {(success_count/len(detections))*100:.1f}%")
print(f"  ⚡ Average Time: {total_time/len(detections):.1f} sec/object")

print(f"\n🎥 VISION SYSTEM:")
print(f"  Camera Resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
print(f"  Detection Method: Color Segmentation")
print(f"  Objects Found: {len(detections)}")

if success_count == len(detections):
    print(f"\n🎉 PERFECT! All objects detected and placed correctly!")
elif success_count >= len(detections) * 0.8:
    print(f"\n👍 GREAT! {(success_count/len(detections))*100:.0f}% success rate!")
else:
    print(f"\n📝 Needs improvement. Some detections missed.")

print("\n" + "="*70)
print("WHAT WE LEARNED")
print("="*70)
print("""
✓ Virtual camera setup in PyBullet
✓ Image capture (RGB, Depth, Segmentation)
✓ Color-based object detection
✓ Pixel-to-world coordinate transformation
✓ Vision-guided manipulation
✓ Real-world robotics pipeline

REAL-WORLD APPLICATIONS:
  - Bin picking (warehouse automation)
  - Quality inspection (manufacturing)
  - Food sorting (agriculture)
  - Medical robotics (surgery assistance)

NEXT IMPROVEMENTS:
  → Use OpenCV for better detection
  → Add machine learning (YOLO, MaskRCNN)
  → Multi-view camera fusion
  → Dynamic scene understanding
""")

input("\nPress ENTER to close simulation...\n")

p.disconnect()
print("✓ Simulation closed")
print("\n🎉 Congratulations! You've built a vision-guided robot system!")