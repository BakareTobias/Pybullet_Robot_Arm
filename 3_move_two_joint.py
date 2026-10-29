"""
LESSON 2: Two-Joint Movement
We will move two joints - simple robot control
"""

import pybullet as p
import pybullet_data
import time

# STEP 1: Start the simulation (open the window)
p.connect(p.GUI)
print("✓ Window opened")

# STEP 1.5: Add PyBullet's robot directory (IMPORTANT!)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print("✓ Robot directory added")

# STEP 2: Load the Kuka robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot loaded")

# STEP 3: Enable gravity
p.setGravity(0, 0, -9.81)
print("✓ Gravity set")

# STEP 4: Move the first joint to 45 degrees
def position_control_joint(joint_no, target):
    """
    Move a joint to the specified angle.

    joint_no: Which joint (0, 1, 2, ...)
    target: Target angle (in radians)
    """
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target
    )

def simulate(no_steps):
    """
    Run the simulation for the specified number of steps.

    no_steps: Number of steps (e.g., 50 = ~5 seconds)
    """
    for i in range(no_steps):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: Move two joints sequentially
# =====================================================
print("\n[TEST 1] Move the joints sequentially")

print("  → Move joint 0 to 45 degrees...")
position_control_joint(joint_no=0, target=0.785)
simulate(50)

print("  → Move joint 1 to 30 degrees...")
position_control_joint(joint_no=1, target=0.524)  # 30 degrees
simulate(50)

print("  ✓ Test 1 complete\n")
input("Press ENTER to continue...")

# =====================================================
# TEST 2: Move two joints AT THE SAME TIME
# =====================================================

print("\n[TEST 2] Move two joints at the same time")

print("  → Move both joints to 0 degrees (reset)...")
position_control_joint(0, 0)
position_control_joint(1, 0)
simulate(50)

print("  ✓ Test 2 complete\n")
input("Press ENTER to continue...")

# =====================================================
# TEST 3: Continuous movement (animation)
# =====================================================
print("\n[TEST 3] Continuous movement - Robot swinging")
print("Press CTRL+C to close\n")

import math

t = 0
try:
    while True:
        # Smooth movement using a sine wave
        target_0 = 0.5 * math.sin(t)
        target_1 = 0.3 * math.cos(t * 1.5)

        position_control_joint(0, target_0)
        position_control_joint(1, target_1)

        p.stepSimulation()
        time.sleep(1./240.)

        t += 0.05 # Advance time

except KeyboardInterrupt:
    print("\n✓ Simulation stopped")

p.disconnect()
print("✓ PyBullet closed")
