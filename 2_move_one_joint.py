"""
LESSON 1: Single Joint Movement
We will move only one joint - the simplest robot control
"""

import pybullet as p
import pybullet_data
import time

# STEP 1: Start the simulation (open the window)
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
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
print("\nMoving the robot's first joint to 45 degrees...")

# 45 degrees = 0.785 radians
target_angle = 0.785

# Control joint 0 (the first joint)
p.setJointMotorControl2(
    bodyUniqueId=robot,      # Which robot
    jointIndex=1,            # Which joint (0 = first)
    controlMode=p.POSITION_CONTROL,  # "Move to this angle" mode
    targetPosition=target_angle  # Target angle
)

# STEP 5: Run the simulation (the robot will move)
for i in range(1000):
    p.stepSimulation()   # Advance physics by one step
    time.sleep(1./240.)  # Wait to observe the motion slowly

print("✓ Movement complete!")
print("\nPress CTRL+C to close the window")

# Keep the window open
while True:
    p.stepSimulation()
    time.sleep(1./240.)
