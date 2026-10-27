"""
2-Joint Robot - USING A READY-MADE ROBOT
We will learn forward kinematics using the Kuka robot
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("Robot Control with PyBullet - READY-MADE ROBOT")
print("="*60)

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Floor
p.loadURDF("plane.urdf")
print("✓ Floor loaded")

# Set the camera
p.resetDebugVisualizerCamera(
    cameraDistance=2.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.5]
)
print("✓ Camera set")

# Load the KUKA IIWA robot (7-DOF industrial robot)
# We will use only the first 2 joints
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print(f"✓ Kuka robot loaded (ID: {robot})")

# Get joint information
num_joints = p.getNumJoints(robot)
print(f"✓ Total number of joints: {num_joints}")
print("\nJoint details:")

# Find only the movable joints
movable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]

    # Type 0 = REVOLUTE (rotating joint)
    # Type 4 = FIXED (fixed connection)
    if joint_type == p.JOINT_REVOLUTE:
        movable_joints.append(i)
        print(f"  Joint {i}: {joint_name} (Movable)")

print(f"\n✓ Number of movable joints: {len(movable_joints)}")
print("  We will use the first 2 joints (for 2-DOF)")

# Joints we will use
joint1_idx = movable_joints[0]
joint2_idx = movable_joints[1]

def set_joint_position(joint_idx, angle, max_force=200):
    """
    Move a joint to the specified angle.

    Args:
        joint_idx: Joint index
        angle: Target angle (radians)
        max_force: Maximum motor force
    """
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle,
        force=max_force,
        maxVelocity=1.0,
        positionGain=0.3,  # PD controller - P gain
        velocityGain=1.0    # PD controller - D gain
    )

def get_joint_position(joint_idx):
    """Get the actual joint angle."""
    state = p.getJointState(robot, joint_idx)
    return state[0]  # Position (radians)

def get_end_effector_position():
    """
    Get the end-effector position.
    (Position of the last link)
    """
    # Link state: [pos, orn, local_inertial_pos, ...]
    # Link 6 = end-effector (Kuka has 7 links, 0-6)
    state = p.getLinkState(robot, 6)
    return state[0]  # (x, y, z) position

def reset_robot():
    """Move the robot to the initial position."""
    print("\n→ Resetting the robot (all joints to 0°)...")
    for i in movable_joints:
        set_joint_position(i, 0, max_force=200)

    # Wait for the movement to complete
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)
    print("  ✓ Robot reset")

# Global variable for the marker (green sphere)
marker_id = None

def visualize_target(x, y, z, color=[0, 1, 0, 0.7]):
    """
    Visualize the target position.

    Args:
        x, y, z: Target position
        color: RGBA color [r, g, b, a]
    """
    global marker_id

    # Remove the old marker
    if marker_id is not None:
        try:
            p.removeBody(marker_id)
        except:
            pass

    # Create a new marker
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=color
    )

    marker_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x, y, z]
    )

print("\n" + "="*60)
print("FORWARD KINEMATICS TESTS")
print("="*60)

# Reset the robot
reset_robot()

# TEST 1: Both joints at 0°
print("\n[TEST 1] Both joints at 0° (default pose)")
target_j1 = 0.0
target_j2 = 0.0

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Moving...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

# Show the results
real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Target: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Actual: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector position: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2])

input("\n→ Press ENTER to continue...")

# TEST 2: First joint at 45°
print("\n[TEST 2] First joint at 45°, second joint at 0°")
target_j1 = math.radians(45)  # 45 derece
target_j2 = 0.0

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Moving...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Target: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Actual: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector position: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [1, 0, 0, 0.7])  # Red

input("\n→ Press ENTER to continue...")

# TEST 3: Both joints at 45°
print("\n[TEST 3] Both joints at 45°")
target_j1 = math.radians(45)
target_j2 = math.radians(45)

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Moving...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Target: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Actual: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector position: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [0, 0, 1, 0.7])  # Blue

input("\n→ Press ENTER to continue...")

# TEST 4: Negative angles
print("\n[TEST 4] First joint at -45°, second joint at 30°")
target_j1 = math.radians(-45)
target_j2 = math.radians(30)

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Moving...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Target: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Actual: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector position: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [1, 1, 0, 0.7])  # Yellow

input("\n→ Press ENTER to start the animation...")

# CONTINUOUS ANIMATION
print("\n" + "="*60)
print("CONTINUOUS ANIMATION")
print("The robot will swing smoothly")
print("Press CTRL+C to close")
print("="*60)

t = 0
try:
    while True:
        # Sinusoidal movement (smooth)
        theta1 = 0.5 * math.sin(t * 0.5)      # Slow swing
        theta2 = 0.3 * math.cos(t * 0.7)      # At a different frequency

        set_joint_position(joint1_idx, theta1)
        set_joint_position(joint2_idx, theta2)

        # Show the end-effector position
        ee_pos = get_end_effector_position()
        visualize_target(ee_pos[0], ee_pos[1], ee_pos[2])

        p.stepSimulation()
        time.sleep(1./240.)
        t += 0.02

except KeyboardInterrupt:
    print("\n\n✓ Simulation stopped")

p.disconnect()
print("✓ PyBullet closed")
print("\n" + "="*60)
print("SUMMARY")
print("="*60)
print("✓ Forward Kinematics tested successfully")
print("✓ Robot control system is working")
print("✓ Next lesson: Inverse Kinematics")
print("="*60)
