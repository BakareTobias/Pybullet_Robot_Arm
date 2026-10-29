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
def position_control_joint(eklem_no, aci, max_force=200, max_velocity=1.0):
    """Move a joint to the specified angle."""
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci,
        force=max_force,
        maxVelocity=max_velocity,
        positionGain=0.3,
        velocityGain=1.0
    )

def read_joint_state(eklem_no):
    """Read the joint state."""
    durum = p.getJointState(robot, eklem_no)
    return durum[0]  # Angle

def end_effector_pozisyonu():
    """Return the end-effector position."""
    # The last link (end-effector) in Kuka IIWA is usually link 6 or 7
    num_links = p.getNumJoints(robot)
    end_effector_link = num_links - 1  # Last link
    
    link_state = p.getLinkState(robot, end_effector_link)
    return link_state[0]  # (x, y, z) position

def simule_et(adim_sayisi):
    """Run the simulation."""
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

def marker_olustur(x, y, z, renk=[1, 0, 0, 0.7]):
    """Create a marker to visualize the target position."""
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=renk
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
def inverse_kinematics(hedef_pozisyon, hedef_orientasyon=None):
    """
    Calculate the joint angles required to reach the target position.
    
    Args:
        hedef_pozisyon: (x, y, z) target position
        hedef_orientasyon: (x, y, z, w) quaternion (optional)
    
    Returns:
        eklem_acilari: List of joint angles (radians)
    """
    num_joints = p.getNumJoints(robot)
    end_effector_link = num_joints - 1
    
    # Use PyBullet's built-in IK solver
    if hedef_orientasyon is None:
        # Position target only (orientation does not matter)
        hedef_orientasyon = [0, 0, 0, 1]  # Default quaternion
    
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
        targetPosition=hedef_pozisyon,
        targetOrientation=hedef_orientasyon,
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
hedef_x, hedef_y, hedef_z = 0.4, 0.3, 0.6
print(f"\nTarget position: ({hedef_x:.2f}, {hedef_y:.2f}, {hedef_z:.2f})")

 # Create a marker
marker = marker_olustur(hedef_x, hedef_y, hedef_z, [1, 0, 0, 0.7])
print("✓ The red marker shows the target position")

# Calculate IK
print("\nCalculating Inverse Kinematics...")
eklem_acilari = inverse_kinematics([hedef_x, hedef_y, hedef_z])

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
    aci = eklem_acilari[idx]
    info = p.getJointInfo(robot, joint_idx)
    eklem_adi = info[1].decode('utf-8')
    print(f"  Joint {joint_idx} ({eklem_adi}): {math.degrees(aci):.2f}°")
    position_control_joint(joint_idx, aci)

# Wait for the movement
print("\nRobot moving to the target...")
simule_et(500)

# Check the results
gercek_pozisyon = end_effector_pozisyonu()
mesafe = math.sqrt(
    (gercek_pozisyon[0] - hedef_x)**2 +
    (gercek_pozisyon[1] - hedef_y)**2 +
    (gercek_pozisyon[2] - hedef_z)**2
)

print(f"\n✓ Movement complete!")
print(f"  Target position: ({hedef_x:.3f}, {hedef_y:.3f}, {hedef_z:.3f})")
print(f"  Actual position: ({gercek_pozisyon[0]:.3f}, {gercek_pozisyon[1]:.3f}, {gercek_pozisyon[2]:.3f})")
print(f"  Distance error: {mesafe*1000:.2f} mm")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 2: Multiple Target Positions
# =====================================================
print("\n" + "="*60)
print("TEST 2: Multiple Target Positions")
print("="*60)

# Markers in different colors
hedefler = [
    ([0.5, 0.2, 0.5], [1, 0, 0, 0.7]),  # Red
    ([0.3, 0.4, 0.6], [0, 1, 0, 0.7]),  # Green
    ([0.4, -0.3, 0.5], [0, 0, 1, 0.7]),  # Blue
    ([0.2, 0.0, 0.7], [1, 1, 0, 0.7]),  # Yellow
]

print(f"\n{len(hedefler)} different target positions will be tested\n")

for i, (hedef_pos, renk) in enumerate(hedefler):
    print(f"[Target {i+1}] Position: ({hedef_pos[0]:.2f}, {hedef_pos[1]:.2f}, {hedef_pos[2]:.2f})")
    
    # Create a marker
    marker = marker_olustur(hedef_pos[0], hedef_pos[1], hedef_pos[2], renk)
    
    # Calculate IK
    eklem_acilari = inverse_kinematics(hedef_pos)
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, eklem_acilari[idx])
    
    # Wait for the movement
    simule_et(400)
    
    # Check the result
    gercek_pos = end_effector_pozisyonu()
    mesafe = math.sqrt(
        (gercek_pos[0] - hedef_pos[0])**2 +
        (gercek_pos[1] - hedef_pos[1])**2 +
        (gercek_pos[2] - hedef_pos[2])**2
    )
    
    print(f"  → Actual: ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")
    print(f"  → Error: {mesafe*1000:.2f} mm\n")
    
    time.sleep(0.5)  # Brief pause

input("\nPress ENTER to continue...")

# =====================================================
# TEST 3: Trajectory Following
# =====================================================
print("\n" + "="*60)
print("TEST 3: Trajectory Following")
print("="*60)
print("The robot will draw a circle\n")

# Circle parameters
merkez = [0.4, 0.0, 0.6]
yaricap = 0.15
nokta_sayisi = 20

print(f"Circle center: {merkez}")
print(f"Radius: {yaricap}m")
print(f"Number of points: {nokta_sayisi}\n")

# Calculate the points on the circle
noktalar = []
for i in range(nokta_sayisi):
    aci = 2 * math.pi * i / nokta_sayisi
    x = merkez[0] + yaricap * math.cos(aci)
    y = merkez[1] + yaricap * math.sin(aci)
    z = merkez[2]
    noktalar.append([x, y, z])

print("The robot is drawing the circle...")
for i, nokta in enumerate(noktalar):
    # Calculate IK
    eklem_acilari = inverse_kinematics(nokta)
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, eklem_acilari[idx])
    
    # Run a short simulation at each point
    simule_et(50)
    
    # Show progress
    if i % 5 == 0:
        gercek_pos = end_effector_pozisyonu()
        print(f"  Point {i+1}/{nokta_sayisi}: "
              f"({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")

print("\n✓ Trajectory following complete!")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Real-Time IK Tracking
# =====================================================
print("\n" + "="*60)
print("TEST 4: Real-Time IK Tracking")
print("="*60)
print("The robot will track a moving target for 10 seconds\n")

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:
    # Moving target (sinusoidal)
    hedef_x = 0.4 + 0.1 * math.sin(t)
    hedef_y = 0.0 + 0.15 * math.cos(t * 0.8)
    hedef_z = 0.6 + 0.05 * math.sin(t * 1.2)
    
    # Calculate IK
    eklem_acilari = inverse_kinematics([hedef_x, hedef_y, hedef_z])
    
    # Move the joints
    for idx, joint_idx in enumerate(movable_joints[:7]):
        position_control_joint(joint_idx, eklem_acilari[idx])
    
    # Read the actual position
    gercek_pos = end_effector_pozisyonu()
    mesafe = math.sqrt(
        (gercek_pos[0] - hedef_x)**2 +
        (gercek_pos[1] - hedef_y)**2 +
        (gercek_pos[2] - hedef_z)**2
    )
    
    # Print to the screen
    print(f"\rTarget: ({hedef_x:.3f}, {hedef_y:.3f}, {hedef_z:.3f}) | "
          f"Actual: ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f}) | "
          f"Error: {mesafe*1000:.1f}mm | "
          f"Time: {time.time()-baslangic:.1f}s", end="")
    
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
