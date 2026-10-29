"""
LESSON 4: Forward Kinematics
Calculating the robot hand position from joint angles
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
print("✓ Robot loaded")

# Add a floor (for reference)
p.loadURDF("plane.urdf")

# =====================================================
# HELPER FUNCTIONS
# =====================================================
def position_control_joint(joint_no, target):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target
    )

def read_joint_state(joint_no):
    joint_params = p.getJointState(robot, joint_no)
    return joint_params[0], joint_params[1]  # angle, velocity

def simulate(no_steps):
    for i in range(no_steps):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# NEW FUNCTION: Read the End-Effector Position
# =====================================================
def read_end_effector_position():
    """
    Read the actual position of the robot hand (end-effector).

    Returns:
        (x, y, z): 3D position (in meters)
    """
    # getLinkState: Read the state of a link (part)
    # Link 6 = the Kuka robot's hand (last link)
    joint_params = p.getLinkState(robot, 6)

    # joint_params[0] = position (x, y, z)
    # joint_params[1] = orientation (rotation, quaternion)

    position = joint_params[0]  # (x, y, z) tuple

    return position

# =====================================================
# NEW: Visual Marker
# =====================================================
sphere = None

def draw_sphere(x, y, z, renk=[0, 1, 0, 0.8]):
    """
    Draw a colored sphere at the specified position.

    Parameters:
        x, y, z: Position
        renk: [R, G, B, transparency] (from 0 to 1)
    """
    global sphere

    # Remove the old marker
    if sphere is not None:
        try:
            p.removeBody(sphere)
        except:
            pass

    # define a new sphere
    visualshape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=renk
    )
    #create the sphere in simulation
    sphere = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visualshape,
        basePosition=[x, y, z]
    )

# =====================================================
# TEST 1: Initial Position
# =====================================================
print("\n" + "="*60)
print("TEST 1: Initial Position of the Robot Hand")
print("="*60)

# Reset all joints
print("\n→ Resetting the robot (all joints to 0°)...")
for i in range(7):
    position_control_joint(i, 0)
simulate(300)

# Read the hand position
x, y, z = read_end_effector_position()
print(f"\n✓ Robot hand position:")
print(f"   X = {x:.3f} meters")
print(f"   Y = {y:.3f} meters")
print(f"   Z = {z:.3f} meters")

# Mark with a green sphere
draw_sphere(x, y, z, renk=[0, 1, 0, 0.8])

input("\nPress ENTER to continue...")

# =====================================================
# TEST 2: Forward Kinematics - Single Joint
# =====================================================
print("\n" + "="*60)
print("TEST 2: Single Joint Movement and Position Change")
print("="*60)

print("\n→ Rotate joint 0 by 45°...")
position_control_joint(0, math.radians(45))
simulate(40)

x, y, z = read_end_effector_position()
print(f"\n✓ New hand position:")
print(f"   X = {x:.3f} meters")
print(f"   Y = {y:.3f} meters")
print(f"   Z = {z:.3f} meters")

# Mark with a red sphere
draw_sphere(x, y, z, renk=[1, 0, 0, 0.8])

print("\nObservation:")
print("  Joint 0 rotates around the Z axis")
print("  → X and Y coordinates change, while Z remains constant")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 3: Two-Joint Movement
# =====================================================
print("\n" + "="*60)
print("TEST 3: Two Joints Moving Together")
print("="*60)

# Reset
print("\n→ Resetting the robot...")
for i in range(7):
    position_control_joint(i, 0)
simulate(30)

print("→ Joint 0: 30°, Joint 1: 45°")
position_control_joint(0, math.radians(30))
position_control_joint(1, math.radians(45))
simulate(40)

x, y, z = read_end_effector_position()
print(f"\n✓ Hand position:")
print(f"   X = {x:.3f} m")
print(f"   Y = {y:.3f} m")
print(f"   Z = {z:.3f} m")

draw_sphere(x, y, z, renk=[0, 0, 1, 0.8])

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Different Angles = Different Positions
# =====================================================
print("\n" + "="*60)
print("TEST 4: Various Positions")
print("="*60)
print("The robot will move to 5 different positions")
print("Each position will be marked with a different color\n")

positionArray = [
    ([0, 0], [0, 1, 0, 0.7]),           # Green
    ([45, 0], [1, 0, 0, 0.7]),          # Red
    ([0, 45], [0, 0, 1, 0.7]),          # Blue
    ([45, 45], [1, 1, 0, 0.7]),         # Yellow
    ([-30, 60], [1, 0, 1, 0.7])         # Purple
]

for idx, (targetlar, renk) in enumerate(positionArray, 1):
    target0, target1 = targetlar

    print(f"Position {idx}: Joint 0={target0:3d}°, Joint 1={target1:3d}°")

    position_control_joint(0, math.radians(target0))
    position_control_joint(1, math.radians(target1))
    simulate(30)

    x, y, z = read_end_effector_position()
    print(f"   → Position: ({x:.3f}, {y:.3f}, {z:.3f})")

    # Create a persistent marker (do not remove it)
    visualShape = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=renk)
    p.createMultiBody(0, baseVisualShapeIndex=visualShape, basePosition=[x, y, z])

    time.sleep(0.5)

print("\n✓ 5 positions marked (colored spheres)")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 5: Live Monitoring - Moving Hand
# =====================================================
print("\n" + "="*60)
print("TEST 5: Live Monitoring - Hand Position While the Robot Swings")
print("="*60)
print("The robot will swing for 10 seconds")
print("The hand position will be displayed in real time\n")

start_time = time.time()
t = 0

while time.time() - start_time < 10:
    # Sinusoidal movement
    target0 = 0.5 * math.sin(t)
    target1 = 0.4 * math.cos(t * 1.2)

    position_control_joint(0, target0)
    position_control_joint(1, target1)

    # Read the hand position
    x, y, z = read_end_effector_position()

    # Update the marker
    draw_sphere(x, y, z)

    # Print to the screen
    print(f"\rHand Position: X={x:6.3f} Y={y:6.3f} Z={z:6.3f} | Time: {time.time()-start_time:.1f}s", end="")

    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Live monitoring complete!")

# =====================================================
# SUMMARY AND EXPLANATION
# =====================================================
print("\n" + "="*60)
print("WHAT IS FORWARD KINEMATICS?")
print("="*60)
print("""
Forward Kinematics:
  Joint angles → Hand position

Example:
  Input : Joint 0 = 45°, Joint 1 = 30°
  Output: Hand position = (X, Y, Z)

Mathematically:
  - Each joint creates a transformation matrix
  - These matrices are multiplied
  - Result: The hand's 3D coordinates

PyBullet calculates this for us:
  p.getLinkState(robot, link_index)

In the next lesson:
  We will write our own FK function (for a simple 2-link robot)
""")

print("="*60)
print("LESSON 4 COMPLETE!")
print("="*60)

p.disconnect()
