"""
LESSON 4: Inverse Kinematics
Calculating joint angles from a target position
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Set the camera
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.5]
)

print("✓ Simulation started")

# Load the robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot loaded\n")

# =====================================================
# HELPER FUNCTIONS
# =====================================================
def position_control_joint(joint_no, target, max_force=200, max_velocity=1.0):
    """Move a joint to the specified angle."""
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=max_force,
        maxVelocity=max_velocity,
        positionGain=0.3,
        velocityGain=1.0
    )

def read_joint_state(joint_no):
    """Read the joint state."""
    joint_params = p.getJointState(robot, joint_no)
    return joint_params[0]  # Angle

def end_effector_position():
    """Return the end-effector position."""
    # The last link (end-effector) in Kuka IIWA is usually link 6 or 7
    num_links = p.getNumJoints(robot)
    end_effector_link = num_links - 1  # Last link
    
    link_state = p.getLinkState(robot, end_effector_link)
    return link_state[0]  # (x, y, z) position

def simulate(no_steps):
    """Run the simulation."""
    for i in range(no_steps):
        p.stepSimulation()
        time.sleep(1./2400.)

def visualize_target(x, y, z, color=[1, 0, 0, 0.7]):
    """Create a marker to visualize the target position."""
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=color
    )
    marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x, y, z]
    )
    return marker

# =====================================================
# INVERSE KINEMATICS FUNCTION
# =====================================================
def inverse_kinematics(target_position, target_orientation=None):
    """
    Calculate the joint angles required to reach the target position.
    
    Args:
        target_position: (x, y, z) target position
        target_orientation: (x, y, z, w) quaternion (optional)
    
    Returns:
        joint_angles: List of joint angles (radians)
    """
    num_joints = p.getNumJoints(robot)
    end_effector_link = num_joints - 1
    
    # Use PyBullet's built-in IK solver
    if target_orientation is None:
        # Position target only (orientation does not matter)
        target_orientation = [0, 0, 0, 1]  # Default quaternion
    
    # IK solver parameters
    ik_solver = 0  # 0 = ikSolver, 1 = ikSolverPosOnly
    max_iterations = 20
    residual_threshold = 0.01
    
    # Find movable joints
    movable_joints = []
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            movable_joints.append(i)
    
    # Get the current joint positions (initial values)
    current_positions = []
    for joint_idx in movable_joints:
        current_positions.append(read_joint_state(joint_idx))
    
    # Calculate IK
    ik_result = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_link,
        targetPosition=target_position,
        targetOrientation=target_orientation,
        lowerLimits=[-math.pi] * len(movable_joints),
        upperLimits=[math.pi] * len(movable_joints),
        jointRanges=[2*math.pi] * len(movable_joints),
        restPoses=current_positions,
        maxNumIterations=max_iterations,
        residualThreshold=residual_threshold
    )
    
    return list(ik_result)

# =====================================================
# TEST 1: Basic IK - Single Target Position
# =====================================================
print("="*60)
print("TEST 1: Basic Inverse Kinematics")
print("="*60)

# Target position
target_x, target_y, target_z = 0.4, 0.3, 0.6
print(f"\nTarget position: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

 # Create a marker
marker = visualize_target(target_x, target_y, target_z, [1, 0, 0, 0.7])
print("✓ The red marker shows the target position")

# Calculate IK
print("\nCalculating Inverse Kinematics...")
joint_targets = inverse_kinematics([target_x, target_y, target_z])

# Find movable joints
num_joints = p.getNumJoints(robot)
movable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    if info[2] == p.JOINT_REVOLUTE:
        movable_joints.append(i)

# Move the joints to the calculated angles
print(f"\nCalculated joint angles:")
for idx, joint_idx in enumerate(movable_joints[:7]):  # First 7 joints
    target = joint_targets[idx]
    info = p.getJointInfo(robot, joint_idx)
    joint_velocity = info[1].decode('utf-8')
    print(f"  Joint {joint_idx} ({joint_velocity}): {math.degrees(target):.2f}°")
    position_control_joint(joint_idx, target)

# Wait for the movement
print("\nRobot moving to the target...")
simulate(2000)

# Check the results
link_position = end_effector_position()
error = math.sqrt(
    (link_position[0] - target_x)**2 +
    (link_position[1] - target_y)**2 +
    (link_position[2] - target_z)**2
)

print(f"\n✓ Movement complete!")
print(f"  Target position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
print(f"  Actual position: ({link_position[0]:.3f}, {link_position[1]:.3f}, {link_position[2]:.3f})")
print(f"  Distance error: {error*1000:.2f} mm")

input("\nPress ENTER to continue...")
p.removeBody(marker)#remove task 1 marker
# =====================================================
# TEST 2: Multiple Target Positions
# =====================================================
print("\n" + "="*60)
print("TEST 2: Multiple Target Positions")
print("="*60)

# Markers in different colors
targets = [
    ([0.5, 0.2, 0.5], [1, 0, 0, 0.7]),  # Red
    ([0.3, 0.4, 0.6], [0, 1, 0, 0.7]),  # Green
    ([0.4, -0.3, 0.5], [0, 0, 1, 0.7]),  # Blue
    ([0.2, 0.0, 0.7], [1, 1, 0, 0.7]),  # Yellow
]

print(f"\n{len(targets)} different target positions will be tested\n")

for i, (target_pos, color) in enumerate(targets):
    print(f"[Target {i+1}] Position: ({target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f})")
    
    # Create a marker
    marker = visualize_target(target_pos[0], target_pos[1], target_pos[2], color)
    
    # Calculate IK
    joint_targets = inverse_kinematics(target_pos)
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, joint_targets[idx])
    
    # Wait for the movement
    simulate(40)
    
    # Check the result
    joint_pos = end_effector_position()
    error = math.sqrt(
        (joint_pos[0] - target_pos[0])**2 +
        (joint_pos[1] - target_pos[1])**2 +
        (joint_pos[2] - target_pos[2])**2
    )
    
    print(f"  → Actual: ({joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f})")
    print(f"  → Error: {error*1000:.2f} mm\n")
    
    time.sleep(0.5)  # Brief pause
    p.removeBody(marker)

input("\nPress ENTER to continue...")

# =====================================================
# TEST 3: Trajectory Following
# =====================================================
print("\n" + "="*60)
print("TEST 3: Trajectory Following")
print("="*60)
print("The robot will draw a circle\n")

# Circle parameters
center = [0.4, 0.0, 0.6]
radius = 0.15
no_points = 20

print(f"Circle center: {center}")
print(f"Radius: {radius}m")
print(f"Number of points: {no_points}\n")

# Calculate the points on the circle
points = []
for i in range(no_points):
    target = 2 * math.pi * i / no_points
    x = center[0] + radius * math.cos(target)
    y = center[1] + radius * math.sin(target)
    z = center[2]
    points.append([x, y, z])
    visualize_target(x,y,z,)

print("The robot is drawing the circle...")
for i, coordinates in enumerate(points):
    # Calculate IK
    joint_targets = inverse_kinematics(coordinates)
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, joint_targets[idx])
    
    # Run a short simulation at each point
    simulate(100)
    
    # Show progress
    if i % 5 == 0:
        joint_pos = end_effector_position()
        print(f"  Point {i+1}/{no_points}: "
              f"({joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f})")

print("\n✓ Trajectory following complete!")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Real-Time IK Tracking
# =====================================================
print("\n" + "="*60)
print("TEST 4: Real-Time IK Tracking")
print("="*60)
print("The robot will track a moving target for 10 seconds\n")

start_time = time.time()
t = 0

while time.time() - start_time < 10:
    # Moving target (sinusoidal)
    target_x = 0.4 + 0.1 * math.sin(t)
    target_y = 0.0 + 0.15 * math.cos(t * 0.8)
    target_z = 0.6 + 0.05 * math.sin(t * 1.2)
    
    # Calculate IK
    joint_targets = inverse_kinematics([target_x, target_y, target_z])
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, joint_targets[idx])
    
    # Read the actual position
    joint_pos = end_effector_position()
    error = math.sqrt(
        (joint_pos[0] - target_x)**2 +
        (joint_pos[1] - target_y)**2 +
        (joint_pos[2] - target_z)**2
    )
    
    # Print to the screen
    print(f"\rTarget: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f}) | "
          f"Actual: ({joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f}) | "
          f"Error: {error*1000:.1f}mm | "
          f"Time: {time.time()-start_time:.1f}s", end="")
    
    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Real-time tracking complete!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 4 SUMMARY: Inverse Kinematics")
print("="*60)
print("✓ Used PyBullet's IK solver")
print("✓ Calculated joint angles from the target position")
print("✓ Tested multiple target positions")
print("✓ Performed trajectory tracking")
print("✓ Performed real-time target tracking")
print("\nNext lesson: Collision Detection")
print("="*60)

p.disconnect()
print("\n✓ Simulation closed")
