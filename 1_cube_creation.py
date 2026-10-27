"""
PyBullet installation test code.
This code simply opens a window and drops a cube.
Purpose: Verify that PyBullet is installed correctly.
"""

import pybullet as p
import pybullet_data
import time

# STEP 1: Open the simulation window (GUI = Graphical User Interface)
# If you use p.DIRECT, no window opens; only calculations are performed.
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # Simplifies the display.
print("PyBullet started successfully!")

# STEP 2: Add PyBullet's built-in model directory.
# This directory contains ready-made models such as planes, cars, and robots.
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# STEP 3: Set gravity (m/s^2).
# On Earth, the z-axis gravity is -9.81 (downward).
p.setGravity(0, 0, -9.81)

# STEP 4: Add a floor (a flat platform).
planeId = p.loadURDF("plane.urdf")
print("Floor added")

# STEP 5: Add a cube (starting position: x=0, y=0, z=5).
# URDF = Unified Robot Description Format (robot description file).
cubeStartPos = [0, 0, 5]  # Initial z-coordinate: 5 meters.
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation.
boxId = p.loadURDF("cube.urdf", cubeStartPos, cubeStartOrientation)
print("Cube added; it will fall...")

# STEP 6: Run the simulation (240 steps = approximately 1 second).
# Each call to stepSimulation() advances the physics by 1/240 of a second.
for i in range(240):
    p.stepSimulation()  # Advance the physics by one step.
    time.sleep(1./240.)  # Wait to observe the simulation in real time.

# STEP 7: Get the cube's final position.
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final cube position: {cubePos}")
print("It fell to the floor! (z-coordinate should be approximately 0.5; the cube is 0.5 m tall)")

# Keep the window open.
print("\nPress CTRL+C to close the window")
while True:
    p.stepSimulation()
    time.sleep(1./240.)
