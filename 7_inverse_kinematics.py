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

def position_control_joint(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci,
        force=200,
        maxVelocity=2.0
    )

def simule_et(adim):
    for _ in range(adim):
        p.stepSimulation()
        time.sleep(1./240.)

# Marker for showing the target
marker = None
def hedef_goster(x, y, z, renk=[1, 0, 0, 0.8]):
    """Show the target position with a red sphere."""
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass

    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.08, rgbaColor=renk)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

# =====================================================
# PyBullet IK Function
# =====================================================
print("\nPyBullet's calculateInverseKinematics() function:")
print("  Provide a target position → It calculates all joint angles\n")

def pybullet_ik(hedef_x, hedef_y, hedef_z):
    """
    Use PyBullet's IK solver.

    Parameters:
        hedef_x, hedef_y, hedef_z: Target position

    Returns:
        eklem_acilari: Angles for 7 joints (tuple)
    """
    # End-effector = Link 6
    end_effector_index = 6

    # Calculate IK
    eklem_acilari = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_index,
        targetPosition=[hedef_x, hedef_y, hedef_z]
    )

    return eklem_acilari

# =====================================================
# TEST 1: First IK Test
# =====================================================
print("="*60)
print("TEST 1: First IK Calculation")
print("="*60)

hedef = [0.4, 0.3, 0.5]  # (x, y, z)
print(f"\nTarget position: x={hedef[0]}, y={hedef[1]}, z={hedef[2]}")

# Show the target (red sphere)
hedef_goster(hedef[0], hedef[1], hedef[2])

print("  → Calculating IK...")
eklem_acilari = pybullet_ik(hedef[0], hedef[1], hedef[2])

print(f"  ✓ Calculated joint angles (degrees):")
for i in range(7):
    print(f"    Joint {i}: {math.degrees(eklem_acilari[i]):6.1f}°")

print("\n  → Moving the robot to the calculated angles...")
for i in range(7):
    position_control_joint(i, eklem_acilari[i])

simule_et(500)

# Check the actual position
durum = p.getLinkState(robot, 6)
gercek_pos = durum[0]
print(f"\n  ✓ The robot hand reached the target!")
print(f"    Target   : ({hedef[0]:.3f}, {hedef[1]:.3f}, {hedef[2]:.3f})")
print(f"    Actual   : ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")

# Calculate the error
hata = math.sqrt(
    (hedef[0]-gercek_pos[0])**2 +
    (hedef[1]-gercek_pos[1])**2 +
    (hedef[2]-gercek_pos[2])**2
)
print(f"    Error    : {hata*1000:.1f} mm")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 2: Multiple Targets
# =====================================================
print("="*60)
print("TEST 2: The Robot Will Move to 5 Different Points")
print("="*60)

hedefler = [
    ([0.5, 0.0, 0.5], [1, 0, 0, 0.8]),   # Red
    ([0.3, 0.3, 0.8], [0, 1, 0, 0.8]),   # Green
    ([0.0, 0.5, 0.4], [0, 0, 1, 0.8]),   # Blue
    ([0.4, -0.2, 0.6], [1, 1, 0, 0.8]),  # Yellow
    ([0.2, 0.2, 0.3], [1, 0, 1, 0.8])    # Purple
]

for idx, (hedef, renk) in enumerate(hedefler, 1):
    x, y, z = hedef

    print(f"\nTarget {idx}: ({x:.2f}, {y:.2f}, {z:.2f})")

    # Show the target
    hedef_goster(x, y, z, renk)

    # Calculate IK
    eklem_acilari = pybullet_ik(x, y, z)

    # Move the robot
    for i in range(7):
        position_control_joint(i, eklem_acilari[i])

    simule_et(400)

    # Verify
    durum = p.getLinkState(robot, 6)
    gercek = durum[0]
    hata = math.sqrt((x-gercek[0])**2 + (y-gercek[1])**2 + (z-gercek[2])**2)
    print(f"  ✓ Reached! Error: {hata*1000:.1f} mm")

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
hedef_uzak = [2.0, 2.0, 2.0]
print(f"\nVery distant target: ({hedef_uzak[0]}, {hedef_uzak[1]}, {hedef_uzak[2]})")
print("  (The robot's maximum reach is around ~1.5m)")

hedef_goster(hedef_uzak[0], hedef_uzak[1], hedef_uzak[2], [1, 0.5, 0, 0.8])

print("\n  → Calculating IK...")
eklem_acilari = pybullet_ik(hedef_uzak[0], hedef_uzak[1], hedef_uzak[2])

print("  → The robot is moving to the nearest position...")
for i in range(7):
    position_control_joint(i, eklem_acilari[i])

simule_et(500)

# Actual position
durum = p.getLinkState(robot, 6)
gercek = durum[0]
hata = math.sqrt(
    (hedef_uzak[0]-gercek[0])**2 +
    (hedef_uzak[1]-gercek[1])**2 +
    (hedef_uzak[2]-gercek[2])**2
)

print(f"\n  Target   : ({hedef_uzak[0]:.3f}, {hedef_uzak[1]:.3f}, {hedef_uzak[2]:.3f})")
print(f"  Reached  : ({gercek[0]:.3f}, {gercek[1]:.3f}, {gercek[2]:.3f})")
print(f"  Error    : {hata:.3f} m = {hata*100:.1f} cm")
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

baslangic = time.time()
while time.time() - baslangic < 10:
    # Random target (within the workspace)
    x = random.uniform(0.2, 0.6)
    y = random.uniform(-0.3, 0.3)
    z = random.uniform(0.3, 0.8)

    print(f"\rTarget: ({x:.2f}, {y:.2f}, {z:.2f})", end="")

    # IK and movement
    hedef_goster(x, y, z)
    eklem_acilari = pybullet_ik(x, y, z)

    for i in range(7):
        position_control_joint(i, eklem_acilari[i])

    simule_et(100)

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
  eklem_acilari = p.calculateInverseKinematics(
      robot,
      end_effector_index,
      [hedef_x, hedef_y, hedef_z]
  )

NEXT LESSON:
→ Writing your own IK algorithm (Jacobian method)
  (Advanced, but let's continue if you're curious!)
""")

p.disconnect()
print("\n✓ PyBullet closed")
