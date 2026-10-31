"""
LESSON 6: Inverse Kinematics
"Move the hand to this point" → Calculate the joint angles
"""

import pybullet as p
import pybullet_data
import time
import math

print("="*60)
print("LESSON 6: INVERSE KINEMATICS")
print("="*60)

# =====================================================
# SECTION 1: What Is Inverse Kinematics?
# =====================================================
print("\nSECTION 1: Concept")
print("-"*60)
print("""
Forward Kinematics (FK):
  Input  : θ1, θ2 (joint angles)
  Output : (x, y) hand position
  Example: θ1=45°, θ2=30° → x=1.2m, y=0.8m

Inverse Kinematics (IK):
  Input  : (x, y) target position
  Output : θ1, θ2 (required joint angles)
  Example: x=1.2m, y=0.8m → θ1=?, θ2=?

Problem: The mathematics is more difficult!
  - Sometimes there are multiple solutions
  - Sometimes there is no solution (unreachable point)
""")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 2: Using PyBullet's IK
# =====================================================
print("="*60)
print("SECTION 2: PyBullet's Automatic IK")
print("="*60)

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ Simulation started")

# Load the robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot loaded")

# Add a floor
p.loadURDF("plane.urdf")

# Set the camera
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

def position_control_joint(joint_no, target):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=200,
        maxVelocity=2.0
    )

def simulate(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)

# Marker for showing the target
marker = None
def show_target(x, y, z, color=[1, 0, 0, 0.8]):
    """Show the target position with a red sphere."""
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass

    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=color)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

# =====================================================
# PyBullet IK Function
# =====================================================
print("\nPyBullet's calculateInverseKinematics() function:")
print("  Provide a target position → It calculates all joint angles\n")

def pybullet_ik(target_x, target_y, target_z):
    """
    Use PyBullet's IK solver.

    Parameters:
        target_x, target_y, target_z: Target position

    Returns:
        joint_angles: Angles for 7 joints (tuple)
    """
    # End-effector = Link 6
    end_effector_index = 6
    max_iterations = 100
    residual_threshold = 0.001

    # Calculate IK
    joint_angles = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_index,
        targetPosition=[target_x, target_y, target_z],
        maxNumIterations=max_iterations,
        residualThreshold=residual_threshold
    )

    return joint_angles

# =====================================================
# TEST 1: First IK Test
# =====================================================
print("="*60)
print("TEST 1: First IK Calculation")
print("="*60)

target = [0.4, 0.3, 0.5]  # (x, y, z)
print(f"\nTarget position: x={target[0]}, y={target[1]}, z={target[2]}")

# Show the target (red sphere)
show_target(target[0], target[1], target[2])

print("  → Calculating IK...")
joint_targets = pybullet_ik(target[0], target[1], target[2])

print(f"  ✓ Calculated joint angles (degrees):")
for i in range(7):
    print(f"    Joint {i}: {math.degrees(joint_targets[i]):6.1f}°")

print("\n  → Moving the robot to the calculated angles...")
for i in range(7):
    position_control_joint(i, joint_targets[i])

simulate(500)

# Check the actual position
joint_params = p.getLinkState(robot, 6)
joint_pos = joint_params[0]
print(f"\n  ✓ The robot hand reached the target!")
print(f"    Target   : ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
print(f"    Actual   : ({joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f})")

# Calculate the error
error = math.sqrt(
    (target[0]-joint_pos[0])**2 +
    (target[1]-joint_pos[1])**2 +
    (target[2]-joint_pos[2])**2
)
print(f"    Error    : {error*1000:.1f} mm")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 2: Multiple Targets
# =====================================================
print("="*60)
print("TEST 2: The Robot Will Move to 5 Different Points")
print("="*60)

targets = [
    ([0.5, 0.0, 0.5], [1, 0, 0, 0.8]),   # Red
    ([0.3, 0.3, 0.8], [0, 1, 0, 0.8]),   # Green
    ([0.0, 0.5, 0.4], [0, 0, 1, 0.8]),   # Blue
    ([0.4, -0.2, 0.6], [1, 1, 0, 0.8]),  # Yellow
    ([0.2, 0.2, 0.3], [1, 0, 1, 0.8])    # Purple
]

for idx, (target, color) in enumerate(targets, 1):
    x, y, z = target

    print(f"\nTarget {idx}: ({x:.2f}, {y:.2f}, {z:.2f})")

    # Show the target
    show_target(x, y, z, color)

    # Calculate IK
    joint_targets = pybullet_ik(x, y, z)

    # Move the robot
    for i in range(7):
        position_control_joint(i, joint_targets[i])

    simulate(400)

    # Verify
    joint_params = p.getLinkState(robot, 6)
    joint_pos = joint_params[0]
    error = math.sqrt((x-joint_pos[0])**2 + (y-joint_pos[1])**2 + (z-joint_pos[2])**2)
    print(f"  ✓ Reached! Error: {error*1000:.1f} mm")

    time.sleep(0.5)

print("\n✓ All targets reached successfully!")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 3: Unreachable Point
# =====================================================
print("="*60)
print("TEST 3: Unreachable Point")
print("="*60)

# A point that is very far away
unreachable_target = [2.0, 2.0, 2.0]
print(f"\nVery distant target: ({unreachable_target[0]}, {unreachable_target[1]}, {unreachable_target[2]})")
print("  (The robot's maximum reach is around ~1.5m)")

show_target(unreachable_target[0], unreachable_target[1], unreachable_target[2], [1, 0.5, 0, 0.8])

print("\n  → Calculating IK...")
joint_targets = pybullet_ik(unreachable_target[0], unreachable_target[1], unreachable_target[2])

print("  → The robot is moving to the nearest position...")
for i in range(7):
    position_control_joint(i, joint_targets[i])

simulate(500)

# Actual position
joint_params = p.getLinkState(robot, 6)
joint_pos = joint_params[0]
error = math.sqrt(
    (unreachable_target[0]-joint_pos[0])**2 +
    (unreachable_target[1]-joint_pos[1])**2 +
    (unreachable_target[2]-joint_pos[2])**2
)

print(f"\n  Target   : ({unreachable_target[0]:.3f}, {unreachable_target[1]:.3f}, {unreachable_target[2]:.3f})")
print(f"  Reached  : ({joint_pos[0]:.3f}, {joint_pos[1]:.3f}, {joint_pos[2]:.3f})")
print(f"  Error    : {error:.3f} m = {error*100:.1f} cm")
print("\n  ⚠ The robot could not reach the target exactly (too far away!)")
print("     But it moved to the nearest possible point.")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 4: Interactive Mode
# =====================================================
print("="*60)
print("TEST 4: Interactive - Set a Target with the Mouse")
print("="*60)
print("""
You can now set a target by clicking with the mouse!

How to use:
  1. Click somewhere in the PyBullet window
  2. The robot will try to move to that point
  3. Stop with CTRL+C

Note: This is a simple example; there is no mouse_click event, so
      we will generate random targets and send the robot to them (for the demo).
""")

input("Press ENTER to start interactive mode...\n")

print("The robot will move to random targets for 10 seconds...")
print("(In a real application, mouse clicks would be used)\n")

import random

start_time = time.time()
while time.time() - start_time < 10:
    # Random target (within the workspace)
    x = random.uniform(0.2, 0.6)
    y = random.uniform(-0.3, 0.3)
    z = random.uniform(0.3, 0.8)

    print(f"\rTarget: ({x:.2f}, {y:.2f}, {z:.2f})", end="")

    # IK and movement
    show_target(x, y, z)
    joint_targets = pybullet_ik(x, y, z)

    for i in range(7):
        position_control_joint(i, joint_targets[i])

    simulate(100)

print("\n\n✓ Interactive mode complete!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 6 SUMMARY")
print("="*60)
print("""
WHAT WE LEARNED:
✓ The concept of Inverse Kinematics
✓ PyBullet's IK solver (calculateInverseKinematics)
✓ Target position → Joint angles
✓ Reachable/unreachable points
✓ Error calculation

USAGE:
  eklem_targetlari = p.calculateInverseKinematics(
      robot,
      end_effector_index,
      [target_x, target_y, target_z]
  )

NEXT LESSON:
→ Writing your own IK algorithm (Jacobian method)
  (Advanced, but let's continue if you're curious!)
""")

p.disconnect()
print("\n✓ PyBullet closed")
