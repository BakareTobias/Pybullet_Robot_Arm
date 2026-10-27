"""
LESSON 10: Gripper Control and Object Management
Basic skills for pick-and-place
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("LESSON 10: GRIPPER AND OBJECT MANAGEMENT")
print("="*60)

# =====================================================
# SECTION 1: What Is a Gripper?
# =====================================================
print("\nSECTION 1: The Gripper Concept")
print("-"*60)
print("""
Gripper: A gripping mechanism at the end of a robot arm

Types:
  1. Parallel gripper (most common)
     └─┬─┘  Closed
     └───┘  Open

  2. Vacuum gripper (suction)
  3. Multi-finger gripper (humanoid hand)
  4. Magnetic gripper (for metal objects)

The Kuka robot's gripper:
  - 2 fingers (parallel)
  - Opening/closing motion
  - Force control
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
p.loadURDF("plane.urdf")
print("✓ Simulation started")

# Load the robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot loaded")

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0.3, 0, 0.3]
)

# Inspect the robot joints
num_joints = p.getNumJoints(robot)
print(f"✓ The robot has {num_joints} joints")

print("\nJoint information:")
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]
    print(f"  {i}: {joint_name} (Tip: {joint_type})")

# Find gripper joints (usually the last 2 joints)
# The Kuka has no gripper, but it does have an end effector
# We will simulate a simple gripper mechanism

print("\nNote: The Kuka model does not include a gripper.")
print("     We will simulate a simple gripping mechanism.")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 3: Creating Objects
# =====================================================
print("="*60)
print("SECTION 3: Creating Objects in the Simulation")
print("="*60)

print("\nWe will create 3 different types of objects:")
print("  1. Cube (Box)")
print("  2. Sphere")
print("  3. Cylinder\n")

# Helper function: create an object
def nesne_olustur(tip, pozisyon, renk, boyut=0.05):
    """
    Create an object in the simulation.

    Parameters:
        tip: "box", "sphere", "cylinder"
        pozisyon: [x, y, z]
        renk: [r, g, b, a]
        boyut: Object size (meters)

    Returns:
        nesne_id: PyBullet object ID
    """
    if tip == "box":
        # Cube
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[boyut/2, boyut/2, boyut/2]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[boyut/2, boyut/2, boyut/2],
            rgbaColor=renk
        )

    elif tip == "sphere":
        # Sphere
        collision_shape = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=boyut/2
        )
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=boyut/2,
            rgbaColor=renk
        )

    elif tip == "cylinder":
        # Cylinder
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=boyut/2,
            height=boyut
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=boyut/2,
            length=boyut,
            rgbaColor=renk
        )

    # Create the object
    nesne_id = p.createMultiBody(
        baseMass=0.05,  # Lightweight (50 grams)
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=pozisyon
    )

    return nesne_id

# Test: create 3 different objects
print("→ Creating objects...")

kutu1 = nesne_olustur("box", [0.4, 0.2, 0.3], [1, 0, 0, 1], boyut=0.05)
print(f"  ✓ Red cube created (ID: {kutu1})")

kutu2 = nesne_olustur("sphere", [0.4, 0.0, 0.3], [0, 0, 1, 1], boyut=0.05)
print(f"  ✓ Blue sphere created (ID: {kutu2})")

kutu3 = nesne_olustur("cylinder", [0.4, -0.2, 0.3], [0, 1, 0, 1], boyut=0.05)
print(f"  ✓ Green cylinder created (ID: {kutu3})")

# Wait for the objects to fall
print("\n→ Objects are falling...")
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

print("✓ Objects have fallen to the ground")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 4: Reading Object Information
# =====================================================
print("="*60)
print("SECTION 4: Reading Object Positions")
print("="*60)

def nesne_pozisyonu_oku(nesne_id):
    """Read the object's position and orientation."""
    pos, orn = p.getBasePositionAndOrientation(nesne_id)
    return pos, orn

print("\nCurrent object positions:")
for idx, nesne in enumerate([kutu1, kutu2, kutu3], 1):
    pos, _ = nesne_pozisyonu_oku(nesne)
    print(f"  Object {idx}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 5: Simple "Grasping" Mechanism
# =====================================================
print("="*60)
print("SECTION 5: Simple Grasping Mechanism")
print("="*60)

print("""
Simulating a real gripper is complex. Here is a simple method:

Use a CONSTRAINT:
  - When the robot gets close to the end effector
  - "Attach" the object to the end effector (virtual connection)
  - This makes the object move with the robot
  - To release: remove the connection

This method is called "Kinematic Grasping".
""")

# Helper functions
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        robot, eklem_no,
        p.POSITION_CONTROL,
        targetPosition=aci,
        force=200
    )

def ik_hesapla(x, y, z):
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

def robot_hareket_ettir(hedef_pos, sure=2.0):
    """Move the robot to the target position (using a trajectory)."""
    eklem_acilari = ik_hesapla(*hedef_pos)

    adim_sayisi = int(sure * 240)
    for _ in range(adim_sayisi):
        for i in range(7):
            eklem_hareket_ettir(i, eklem_acilari[i])
        p.stepSimulation()
        time.sleep(1./240.)

def end_effector_pozisyonu():
    """Return the current position of the end effector."""
    durum = p.getLinkState(robot, 6)
    return durum[0]

# Global variables for grasping/releasing
tutulan_nesne = None
constraint_id = None

def nesne_tut(nesne_id):
    """
    Grasp an object (create a constraint).
    """
    global tutulan_nesne, constraint_id

    if tutulan_nesne is not None:
        print("  ⚠ You are already holding an object!")
        return False

    # End-effector position
    ee_pos = end_effector_pozisyonu()

    # Object position
    nesne_pos, _ = nesne_pozisyonu_oku(nesne_id)

    # Check the distance
    mesafe = math.sqrt(
        (ee_pos[0]-nesne_pos[0])**2 +
        (ee_pos[1]-nesne_pos[1])**2 +
        (ee_pos[2]-nesne_pos[2])**2
    )

    if mesafe > 0.15:  # Cannot grasp if farther than 15 cm
        print(f"  ✗ Object is too far away! Distance: {mesafe*100:.1f}cm")
        return False

    # Create a constraint (connection)
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=6,  # End effector
        childBodyUniqueId=nesne_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,  # Fixed connection
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    tutulan_nesne = nesne_id
    print(f"  ✓ Object grasped! (Constraint ID: {constraint_id})")
    return True

def nesne_birak():
    """
    Release the held object (remove the constraint).
    """
    global tutulan_nesne, constraint_id

    if tutulan_nesne is None:
        print("  ⚠ You are not holding an object!")
        return False

    # Remove the constraint
    p.removeConstraint(constraint_id)

    print(f"  ✓ Object released!")
    tutulan_nesne = None
    constraint_id = None
    return True

input("Press ENTER to continue...\n")

# =====================================================
# TEST 1: Basic Pick and Place
# =====================================================
print("="*60)
print("TEST 1: Basic Pick and Place")
print("="*60)

# Pick up the red cube
print("\n→ STEP 1: Move above the red cube")
kutu1_pos, _ = nesne_pozisyonu_oku(kutu1)
hedef_ustte = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.2]
robot_hareket_ettir(hedef_ustte, sure=2.0)
print("  ✓ Above it")

print("\n→ STEP 2: Move down to the cube's level")
hedef_yakin = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.05]
robot_hareket_ettir(hedef_yakin, sure=1.5)
print("  ✓ Near the cube")

print("\n→ STEP 3: Grasp the cube")
nesne_tut(kutu1)
time.sleep(0.5)

print("\n→ STEP 4: Lift it")
hedef_yukari = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.3]
robot_hareket_ettir(hedef_yukari, sure=1.5)
print("  ✓ Cube is in the air!")

print("\n→ STEP 5: Move it to a new position")
yeni_pos = [0.2, -0.3, 0.3]
robot_hareket_ettir(yeni_pos, sure=2.0)
print("  ✓ At the new position")

print("\n→ STEP 6: Release the cube")
nesne_birak()

# Wait for it to fall a little
for _ in range(120):
    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Cube released and fell")

# Check the final position
final_pos, _ = nesne_pozisyonu_oku(kutu1)
print(f"\nRed cube moved:")
print(f"  Start: ({kutu1_pos[0]:.2f}, {kutu1_pos[1]:.2f}, {kutu1_pos[2]:.2f})")
print(f"  End  : ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})")

input("\n✓ TEST 1 complete! Press ENTER to continue...\n")

# =====================================================
# TEST 2: Collecting All Objects
# =====================================================
print("="*60)
print("TEST 2: Gathering All Objects Together")
print("="*60)

hedef_alan = [0.0, 0.0, 0.05]  # Gather at the center

nesneler = [kutu1, kutu2, kutu3]
renkler = ["Red", "Blue", "Green"]

for idx, (nesne, renk) in enumerate(zip(nesneler, renkler)):
    print(f"\n→ Gathering the {renk.lower()} object...")

    # Object position
    pos, _ = nesne_pozisyonu_oku(nesne)

    # Move above it
    print("  - Moving above it...")
    robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.25], sure=1.5)

    # Move down
    print("  - Moving down...")
    robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.05], sure=1.0)

    # Grasp
    print("  - Grasping...")
    if nesne_tut(nesne):
        # Lift
        print("  - Lifting...")
        robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.3], sure=1.0)

        # Move to the target area
        print("  - Moving to the target...")
        offset_x = idx * 0.08  # Arrange side by side
        robot_hareket_ettir([hedef_alan[0] + offset_x, hedef_alan[1], 0.3], sure=1.5)

        # Release
        print("  - Releasing...")
        robot_hareket_ettir([hedef_alan[0] + offset_x, hedef_alan[1], 0.15], sure=0.8)
        nesne_birak()

        # Wait for it to fall
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)

        print(f"  ✓ {renk} object gathered!")
    else:
        print(f"  ✗ Could not grasp the {renk.lower()} object")

print("\n✓ All objects gathered!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 10 SUMMARY")
print("="*60)
print("""
WHAT WE LEARNED:
✓ The gripper concept
✓ Creating objects (createCollisionShape + createVisualShape)
✓ Reading object positions (getBasePositionAndOrientation)
✓ Grasping with a constraint (createConstraint)
✓ Releasing by removing a constraint (removeConstraint)
✓ A basic pick-and-place operation

IMPORTANT FUNCTIONS:
  p.createCollisionShape()  - Physics shape
  p.createVisualShape()     - Visual shape
  p.createMultiBody()       - Create an object
  p.createConstraint()      - Create a connection
  p.removeConstraint()      - Remove a connection

CONSTRAINT TYPES:
  - JOINT_FIXED: Fixed connection (for a gripper)
  - JOINT_POINT2POINT: Point connection
  - JOINT_HINGE: Hinge (rotating)
  - JOINT_PRISMATIC: Sliding

REAL GRIPPER:
  Real robots have their own gripper joints
  Example: Robotiq 2F-85
    - 2 fingers
    - Each finger is motor-controlled
    - Force sensor

Our method (kinematic grasping):
  ✓ Simple
  ✓ Fast
  ✓ Sufficient for training
  ✗ No realistic physics

NEXT STEP:
→ PROJECT 1: Advanced Pick-and-Place
  - Multiple objects
  - Color recognition
  - Placing objects in target boxes
""")

input("\nLesson complete! Press ENTER...\n")

p.disconnect()
print("✓ PyBullet disconnected")
