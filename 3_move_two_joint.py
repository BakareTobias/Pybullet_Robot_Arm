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
def eklem_hareket_ettir(eklem_no, aci):
    """
    Move a joint to the specified angle.

    eklem_no: Which joint (0, 1, 2, ...)
    aci: Target angle (in radians)
    """
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def simule_et(adim_sayisi):
    """
    Run the simulation for the specified number of steps.

    adim_sayisi: Number of steps (e.g., 500 = ~2 seconds)
    """
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: Move two joints sequentially
# =====================================================
print("\n[TEST 1] Move the joints sequentially")

print("  → Move joint 0 to 45 degrees...")
eklem_hareket_ettir(eklem_no=0, aci=0.785)
simule_et(500)

print("  → Move joint 1 to 30 degrees...")
eklem_hareket_ettir(eklem_no=1, aci=0.524)  # 30 derece
simule_et(500)

print("  ✓ Test 1 complete\n")
input("Press ENTER to continue...")

# =====================================================
# TEST 2: Move two joints AT THE SAME TIME
# =====================================================

print("\n[TEST 2] Move two joints at the same time")

print("  → Move both joints to 0 degrees (reset)...")
eklem_hareket_ettir(0, 0)
eklem_hareket_ettir(1, 0)
simule_et(1000)

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
        aci_0 = 0.5 * math.sin(t)
        aci_1 = 0.3 * math.cos(t * 1.5)

        eklem_hareket_ettir(0, aci_0)
        eklem_hareket_ettir(1, aci_1)

        p.stepSimulation()
        time.sleep(1./240.)

        t += 0.05 # Advance time

except KeyboardInterrupt:
    print("\n✓ Simulation stopped")

p.disconnect()
print("✓ PyBullet closed")
