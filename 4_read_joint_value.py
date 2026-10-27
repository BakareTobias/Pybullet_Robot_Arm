"""
LESSON 3: Reading Joint State
We will read the robot's actual position and velocity
"""

import pybullet as p
import pybullet_data
import time
import math

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ Simulation started")

# Load the robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot loaded\n")

# =====================================================
# NEW FUNCTION: Read joint state
# =====================================================
def eklem_durumunu_oku(eklem_no):
    """
    Read the actual state of a joint.

    Returns:
        aci: Current joint angle (radians)
        hiz: Joint rotational velocity (radians/second)
    """
    durum = p.getJointState(robot, eklem_no)

    # durum is a tuple (multiple values):
    # durum[0] = position (angle)
    # durum[1] = velocity
    # durum[2] = reaction forces (not used for now)
    # durum[3] = applied torque

    aci = durum[0]
    hiz = durum[1]

    return aci, hiz

# =====================================================
# Previous functions
# =====================================================
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def simule_et(adim_sayisi):
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: Read the initial position
# =====================================================
print("="*50)
print("TEST 1: Initial Position")
print("="*50)

# Read the state of the first 3 joints
for eklem in range(4):
    aci, hiz = eklem_durumunu_oku(eklem)
    print(f"Joint {eklem}:")
    print(f"  Angle: {aci:.3f} radians = {math.degrees(aci):.1f} degrees")
    print(f"  Velocity: {hiz:.3f} rad/s")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 2: Compare Target vs. Actual
# =====================================================
print("\n" + "="*50)
print("TEST 2: Target vs. Actual Angle")
print("="*50)

hedef_aci = 1.0  # ~57 degrees

print(f"\nTarget: Move joint 0 to {hedef_aci:.3f} radians ({math.degrees(hedef_aci):.1f}°)")
eklem_hareket_ettir(0, hedef_aci)

# Show the state every 100 steps during movement
print("\nMoving...")
for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    # Report every 100 steps
    if adim % 100 == 0:
        aci, hiz = eklem_durumunu_oku(0)
        print(f"  Step {adim:3d}: Angle={math.degrees(aci):6.2f}°, Velocity={hiz:6.3f} rad/s")

# Final state
aci_final, hiz_final = eklem_durumunu_oku(0)
print(f"\n✓ Movement complete!")
print(f"  Target angle : {math.degrees(hedef_aci):.2f}°")
print(f"  Actual angle: {math.degrees(aci_final):.2f}°")
print(f"  Difference      : {math.degrees(aci_final - hedef_aci):.2f}°")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 3: Coordinated Movement of Two Joints
# =====================================================
print("\n" + "="*50)
print("TEST 3: Coordinated Movement of Two Joints")
print("="*50)

# Targets
hedef_0 = 0.8   # ~46 degrees
hedef_1 = -0.6  # ~-34 degrees
hedef_2 = 1.57
hedef_3 = -1.048

print(f"\nTargets:")
print(f"  Joint 0: {math.degrees(hedef_0):.1f}°")
print(f"  Joint 1: {math.degrees(hedef_1):.1f}°")

eklem_hareket_ettir(0, hedef_0)
eklem_hareket_ettir(1, hedef_1)
eklem_hareket_ettir(2, hedef_2)
eklem_hareket_ettir(3, hedef_3)

print("\nMoving...")
for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    if adim % 100 == 0:
        aci_0, hiz_0 = eklem_durumunu_oku(0)
        aci_1, hiz_1 = eklem_durumunu_oku(1)
        aci_2, hiz_2 = eklem_durumunu_oku(2)
        aci_3, hiz_3 = eklem_durumunu_oku(3)
        print(f"  Step {adim:3d}:")
        print(f"    Joint 0: {math.degrees(aci_0):6.2f}°")
        print(f"    Joint 1: {math.degrees(aci_1):6.2f}°")
        print(f"    Joint 2: {math.degrees(aci_2):6.2f}°")
        print(f"    Joint 2: {math.degrees(aci_3):6.2f}°")

print("\n✓ Coordinated movement complete!")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Reset Angles
# =====================================================
print("\n" + "="*50)
print("TEST 4: Reset Angles")
print("="*50)

eklem_hareket_ettir(0, 0)
eklem_hareket_ettir(1, 0)
eklem_hareket_ettir(2, 0)
eklem_hareket_ettir(3, 0)

for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    if adim % 100 == 0:
        aci_0, hiz_0 = eklem_durumunu_oku(0)
        aci_1, hiz_1 = eklem_durumunu_oku(1)
        aci_2, hiz_2 = eklem_durumunu_oku(2)
        aci_3, hiz_3 = eklem_durumunu_oku(3)
        print(f"  Step {adim:3d}:")
        print(f"    Joint 0: {math.degrees(aci_0):6.2f}°")
        print(f"    Joint 1: {math.degrees(aci_1):6.2f}°")
        print(f"    Joint 2: {math.degrees(aci_2):6.2f}°")
        print(f"    Joint 2: {math.degrees(aci_3):6.2f}°")

print("\n✓ Angles reset!")

input("\nPress ENTER to continue...")
# =====================================================
# TEST 5: Live Monitoring (Real Time)
# =====================================================
print("\n" + "="*50)
print("TEST 5: Live Monitoring - Robot Swinging")
print("="*50)
print("The robot will swing for 10 seconds")
print("Joint states will be displayed in real time\n")

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:  # 10 saniye
    # Sinusoidal movement
    hedef_0 = 0.6 * math.sin(t)
    hedef_1 = 0.4 * math.cos(t * 1.3)

    eklem_hareket_ettir(0, hedef_0)
    eklem_hareket_ettir(1, hedef_1)

    # Read the actual state
    gercek_0, hiz_0 = eklem_durumunu_oku(0)
    gercek_1, hiz_1 = eklem_durumunu_oku(1)

    # Print to the screen (update the same line)
    print(f"\rJoint 0: {math.degrees(gercek_0):6.1f}° | "
          f"Joint 1: {math.degrees(gercek_1):6.1f}° | "
          f"Time: {time.time()-baslangic:.1f}s", end="")

    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Live monitoring complete!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*50)
print("LESSON 3 SUMMARY")
print("="*50)
print("✓ Learned to read joint states")
print("✓ Compared target and actual angles")
print("✓ Performed real-time monitoring")
print("\nNext lesson: Forward Kinematics")
print("(Calculating the robot hand position from joint angles)")
print("="*50)

p.disconnect()
