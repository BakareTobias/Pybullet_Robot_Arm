"""
LESSON 8: Collision Avoidance
The robot will reach the target while avoiding obstacles
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("LESSON 8: COLLISION AVOIDANCE")
print("="*60)

# =====================================================
# SECTION 1: Collision Detection
# =====================================================
print("\nSECTION 1: How Does Collision Detection Work?")
print("-"*60)
print("""
PyBullet has two collision-checking methods:

1. getContactPoints(): Has physical contact occurred?
   - Real-time
   - Detects collisions AFTER they happen
   - "Oops, we hit it!" ✗

2. getClosestPoints(): What is the closest distance?
   - Checks in advance
   - Detects collisions BEFORE they happen
   - "We're getting close; turn!" ✓

We will use the second method (a proactive approach).
""")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 2: Simulation Setup
# =====================================================
print("="*60)
print("SECTION 2: Preparing the Environment")
print("="*60)

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ Simulation started")

# Ground
p.loadURDF("plane.urdf")

# Robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot loaded")

# Set the camera
p.resetDebugVisualizerCamera(
    cameraDistance=2.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

# Add obstacles
print("\n→ Adding obstacles...")

engeller = []

# Obstacle 1: Red cube
engel1_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.3])
engel1_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.3],
                                     rgbaColor=[1, 0, 0, 0.7])
engel1 = p.createMultiBody(
    baseMass=0,  # Fixed (does not move)
    baseCollisionShapeIndex=engel1_collision,
    baseVisualShapeIndex=engel1_visual,
    basePosition=[0.4, 0.1, 0.3]
)
engeller.append(engel1)
print(f"  ✓ Obstacle 1 (Red cube): [0.4, 0.1, 0.3]")

# Obstacle 2: Blue cube
engel2_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, 0.2])
engel2_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, 0.2],
                                     rgbaColor=[0, 0, 1, 0.7])
engel2 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=engel2_collision,
    baseVisualShapeIndex=engel2_visual,
    basePosition=[0.2, -0.2, 0.2]
)
engeller.append(engel2)
print(f"  ✓ Obstacle 2 (Blue cube): [0.2, -0.2, 0.2]")

# Obstacle 3: Green cylinder
engel3_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.4)
engel3_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.08, length=0.4,
                                     rgbaColor=[0, 1, 0, 0.7])
engel3 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=engel3_collision,
    baseVisualShapeIndex=engel3_visual,
    basePosition=[0.3, 0.3, 0.2]
)
engeller.append(engel3)
print(f"  ✓ Obstacle 3 (Green cylinder): [0.3, 0.3, 0.2]")

print(f"\n✓ Added {len(engeller)} obstacles in total")

input("\nPress ENTER to continue...\n")

# =====================================================
# HELPER FUNCTIONS
# =====================================================
def position_control_joint(joint_no, target):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=200,
        maxVelocity=2.0
    )

def ik_hesapla(x, y, z):
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

marker = None
def marker_goster(x, y, z, renk=[0, 1, 0, 0.8]):
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=renk)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

def carpisma_var_mi(guvenlik_mesafesi=0.05):
    """
    Is the robot too close to any obstacle?

    Parameters:
        guvenlik_mesafesi: Minimum safe distance (meters)

    Returns:
        True: Collision risk detected
        False: Safe
    """
    for engel in engeller:
        # Closest distance between the obstacle and all robot links
        yakin_points = p.getClosestPoints(
            bodyA=robot,
            bodyB=engel,
            distance=guvenlik_mesafesi  # Report if closer than this distance
        )

        if len(yakin_points) > 0:
            # Find the closest distance
            min_mesafe = min([nokta[8] for nokta in yakin_points])

            if min_mesafe < guvenlik_mesafesi:
                return True, min_mesafe

    return False, float('inf')

# =====================================================
# SECTION 3: TEST - Movement Without Collision
# =====================================================
print("="*60)
print("SECTION 3: Movement in a Safe Area")
print("="*60)

print("\n→ The robot will move in a safe area (away from obstacles)")

guvenli_hedefler = [
    [0.5, 0.5, 0.6],   # Outside the obstacles
    [0.6, -0.4, 0.5],  # Outside the obstacles
]

for idx, hedef in enumerate(guvenli_hedefler, 1):
    print(f"\n  Target {idx}: {hedef}")

    marker_goster(*hedef, renk=[0, 1, 0, 0.8])
    eklem_targetlari = ik_hesapla(*hedef)

    for i in range(7):
        position_control_joint(i, eklem_targetlari[i])

    # Check for collisions while moving
    for _ in range(300):
        p.stepSimulation()
        time.sleep(1./240.)

        carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.1)
        if carpti:
            print(f"    ⚠ Too close to an obstacle! Distance: {mesafe*100:.1f} cm")

    print(f"    ✓ Target reached (safe)")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 4: TEST - Attempting to Hit an Obstacle
# =====================================================
print("="*60)
print("SECTION 4: Dangerous Target (Collision Risk!)")
print("="*60)

tehlikeli_hedef = [0.4, 0.1, 0.3]  # Directly inside obstacle 1!
print(f"\n⚠ Dangerous target: {tehlikeli_hedef}")
print("  (This target is inside obstacle 1!)")

marker_goster(*tehlikeli_hedef, renk=[1, 0, 0, 0.8])

print("\n→ The robot will move toward the dangerous target...")
eklem_targetlari = ik_hesapla(*tehlikeli_hedef)

for i in range(7):
    position_control_joint(i, eklem_targetlari[i])

# Monitor during movement
carpti_mi = False
for adim in range(400):
    p.stepSimulation()
    time.sleep(1./240.)

    carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.1)

    if carpti and not carpti_mi:
        print(f"  ⚠⚠⚠ WARNING: Approached the obstacle to within {mesafe*100:.1f} cm!")
        carpti_mi = True

    # Report the state every 50 steps
    if adim % 50 == 0 and carpti:
        print(f"    Step {adim}: Distance to obstacle = {mesafe*100:.1f} cm")

if carpti_mi:
    print("\n  ✗ Collision risk detected!")
else:
    print("\n  ✓ Strange, no collision occurred (is the obstacle too small?)")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 5: Simple Avoidance Strategy
# =====================================================
print("="*60)
print("SECTION 5: Smart Avoidance - Go Overhead!")
print("="*60)

print("""
Strategy: Pass OVER the obstacles
  1. Start → Move up (to a safe height)
  2. At height → Move toward the target
  3. Above the target → Move down
""")

baslangic = [0.5, 0.5, 0.5]
hedef_son = [0.3, -0.3, 0.4]  # Near the obstacles

guvenli_yukseklik = 0.9  # Above the obstacles

print(f"\nStart: {baslangic}")
print(f"Target: {hedef_son}")
print(f"Safe height: {guvenli_yukseklik}m")

# Create waypoints
waypoints = [
    np.array(baslangic),                                    # 1. Start
    np.array([baslangic[0], baslangic[1], guvenli_yukseklik]),  # 2. Move up
    np.array([hedef_son[0], hedef_son[1], guvenli_yukseklik]),  # 3. Move above the target
    np.array(hedef_son)                                     # 4. Move down
]

print("\nWaypoints:")
for idx, wp in enumerate(waypoints, 1):
    print(f"  {idx}. {wp}")

# Display the waypoints
for wp in waypoints:
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 1, 0, 0.9])
    p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=wp.tolist())

print("\n→ The robot will move safely through the waypoints...")

# Move to each waypoint
for idx in range(len(waypoints) - 1):
    baslangic_wp = waypoints[idx]
    bitis_wp = waypoints[idx + 1]

    print(f"\n  Segment {idx+1}/{len(waypoints)-1}")

    # Smooth movement using cubic interpolation
    no_steps = 200
    for adim in range(no_steps):
        t = adim / no_steps
        s = 3 * t**2 - 2 * t**3  # Cubic

        hedef_anlik = baslangic_wp + (bitis_wp - baslangic_wp) * s

        marker_goster(*hedef_anlik, renk=[0, 1, 1, 0.8])
        eklem_targetlari = ik_hesapla(*hedef_anlik)

        for i in range(7):
            position_control_joint(i, eklem_targetlari[i])

        p.stepSimulation()
        time.sleep(1./240.)

        # Collision check
        carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.15)
        if carpti and adim % 50 == 0:
            print(f"    ⚠ Distance to obstacle: {mesafe*100:.1f} cm (but at a safe height!)")

    print(f"    ✓ Reached waypoint {idx+2}")

print("\n✓ The robot passed through all waypoints!")
print("✓ Successfully avoided the obstacles!")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 6: Advanced - Dynamic Avoidance
# =====================================================
print("="*60)
print("SECTION 6: Dynamic Avoidance (Bonus)")
print("="*60)

print("""
Advanced technique: Real-time avoidance
  - Check for collisions at every step
  - Change direction if it gets too close

This is a simple demo; real applications are more complex:
  - RRT (Rapidly-exploring Random Tree)
  - A* algorithm
  - Potential Field methods
""")

print("\n→ The robot will stop if it gets too close to an obstacle...")

hedef_dinamik = [0.35, 0.0, 0.4]
marker_goster(*hedef_dinamik, renk=[1, 0.5, 0, 0.8])

eklem_targetlari = ik_hesapla(*hedef_dinamik)

for i in range(7):
    position_control_joint(i, eklem_targetlari[i])

for adim in range(500):
    carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.12)

    if carpti:
        if mesafe < 0.08:  # Too close!
            print(f"\n  🛑 STOP! {mesafe*100:.1f} cm from the obstacle - too close!")
            print("     An alternative path would be searched for in a real application.")
            break
        elif adim % 50 == 0:
            print(f"  ⚠ Caution: obstacle at a distance of {mesafe*100:.1f} cm")

    p.stepSimulation()
    time.sleep(1./240.)

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 8 SUMMARY")
print("="*60)
print("""
WHAT WE LEARNED:
✓ Collision detection (getClosestPoints)
✓ The concept of safety distance
✓ Creating obstacles (collision & visual shapes)
✓ A simple avoidance strategy (go overhead)
✓ Safe waypoint-based movement

IMPORTANT FUNCTIONS:
  p.getClosestPoints(robot, engel, distance)
  p.createCollisionShape(...)
  p.createVisualShape(...)

REAL-WORLD APPLICATIONS:
  - Autonomous vehicles
  - Industrial robots (near people)
  - Drones
  - Mobile robots (floor cleaning, warehouses)

ADVANCED TECHNIQUES:
  - RRT (Rapidly-exploring Random Tree)
  - A* path planning
  - Potential Fields
  - Dynamic Window Approach

NEXT LESSON:
→ Advanced IK (Jacobian method)
  Or you can suggest another topic!
""")

p.disconnect()
print("\n✓ PyBullet disconnected")
