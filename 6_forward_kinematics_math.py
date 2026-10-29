"""
LESSON 5: Forward Kinematics Mathematics
We will write our own FK function for a 2-link robot
"""

import pybullet as p
import pybullet_data
import time
import math
import matplotlib.pyplot as plt

# =====================================================
# SECTION 1: MATHEMATICS - Forward Kinematics
# =====================================================
print("="*60)
print("SECTION 1: Forward Kinematics Mathematics")
print("="*60)

print("""
2-Link Planar Robot:

        ● Hand (x2, y2)
        |
   L2   | Link 2
        |
        ● Joint 2 (x1, y1)
        |
   L1   | Link 1
        |
        ● Joint 1 (0, 0)

Joint 1: rotated by θ1 degrees (around the Z axis)
Joint 2: rotated by θ2 degrees (around the Z axis)
""")

# Robot parameters
L1 = 1.0  # Link 1 length (meters)
L2 = 0.8  # Link 2 length (meters)

print(f"Robot Parameters:")
print(f"  L1 (Link 1 length) = {L1} meters")
print(f"  L2 (Link 2 length) = {L2} meters")

input("\nPress ENTER to continue...\n")

# =====================================================
# Forward Kinematics Fonksiyonu
# =====================================================
def forward_kinematics(theta1, theta2, L1, L2):
    """
    Forward Kinematics for a 2-link robot.

    Parameters:
        theta1: Joint 1 angle (radians)
        theta2: Joint 2 angle (radians)
        L1: Link 1 length
        L2: Link 2 length

    Returns:
        (x, y): Hand position
    """
    # Position of joint 2 (end of link 1)
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)

    # Hand position (end of link 2)
    # Link 2 is at angle theta1 + theta2
    x2 = x1 + L2 * math.cos(theta1 + theta2)
    y2 = y1 + L2 * math.sin(theta1 + theta2)

    return (x2, y2)

print("="*60)
print("Forward Kinematics Function Defined!")
print("="*60)
print("""
Mathematics:
  Joint 2 position:
    x1 = L1 * cos(θ1)
    y1 = L1 * sin(θ1)

  Hand position:
    x2 = x1 + L2 * cos(θ1 + θ2)
    y2 = y1 + L2 * sin(θ1 + θ2)
""")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 2: TEST - Different Angles
# =====================================================
print("="*60)
print("SECTION 2: Test with Different Angles")
print("="*60)

test_targetlar = [
    (0, 0, "Both joints at 0°"),
    (90, 0, "Joint 1: 90°, Joint 2: 0°"),
    (45, 45, "Both joints at 45°"),
    (90, -90, "Joint 1: 90°, Joint 2: -90°"),
    (0, 90, "Joint 1: 0°, Joint 2: 90°")
]

for theta1_derece, theta2_derece, targetklama in test_targetlar:
    # Convert degrees to radians
    theta1 = math.radians(theta1_derece)
    theta2 = math.radians(theta2_derece)

    # Calculate Forward Kinematics
    x, y = forward_kinematics(theta1, theta2, L1, L2)

    print(f"\n{targetklama}")
    print(f"  Input : θ1={theta1_derece}°, θ2={theta2_derece}°")
    print(f"  Output: x={x:.3f}m, y={y:.3f}m")

    # Distance from the base
    uzaklik = math.sqrt(x**2 + y**2)
    print(f"  Distance from the base: {uzaklik:.3f}m")

input("\n\nPress ENTER to continue...\n")

# =====================================================
# SECTION 3: VISUALIZATION
# =====================================================
print("="*60)
print("SECTION 3: Visualization with Matplotlib")
print("="*60)

def robot_ciz(theta1, theta2, L1, L2, baslik=""):
    """
    Draw the robot in a 2D plane.
    """
    # Joint positions
    x0, y0 = 0, 0  # Base
    x1 = L1 * math.cos(theta1)
    y1 = L1 * math.sin(theta1)
    x2, y2 = forward_kinematics(theta1, theta2, L1, L2)

    # Plot
    plt.figure(figsize=(8, 8))

    # Link 1
    plt.plot([x0, x1], [y0, y1], 'r-', linewidth=8, label='Link 1')

    # Link 2
    plt.plot([x1, x2], [y1, y2], 'b-', linewidth=8, label='Link 2')

    # Joints
    plt.plot(x0, y0, 'ko', markersize=15, label='Base')
    plt.plot(x1, y1, 'go', markersize=12, label='Joint 2')
    plt.plot(x2, y2, 'mo', markersize=15, label='Hand')

    # Grid and settings
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlim(-2, 2)
    plt.ylim(-2, 2)
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.title(baslik, fontsize=14, fontweight='bold')
    plt.legend(fontsize=10)

    # Axis lines
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)

    plt.tight_layout()
    plt.show()

print("Example 1: Both joints at 0° (straight up)")
robot_ciz(0, 0, L1, L2, "θ1=0°, θ2=0°")

print("\nExample 2: Joint 1: 45°, Joint 2: 0°")
robot_ciz(math.radians(45), 0, L1, L2, "θ1=45°, θ2=0°")

print("\nExample 3: Joint 1: 90°, Joint 2: -45°")
robot_ciz(math.radians(90), math.radians(-45), L1, L2, "θ1=90°, θ2=-45°")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 4: WORKSPACE
# =====================================================
print("="*60)
print("SECTION 4: Workspace")
print("="*60)
print("We will plot ALL points the robot hand can reach")
print("This may take a few seconds...\n")

# Try many angle combinations
x_noktalar = []
y_noktalar = []

for theta1_derece in range(0, 360, 5):  # From 0° to 360°, in 5° steps
    for theta2_derece in range(-180, 180, 5):  # From -180° to 180°
        theta1 = math.radians(theta1_derece)
        theta2 = math.radians(theta2_derece)

        x, y = forward_kinematics(theta1, theta2, L1, L2)
        x_noktalar.append(x)
        y_noktalar.append(y)

# Plot
plt.figure(figsize=(10, 10))
plt.scatter(x_noktalar, y_noktalar, c='blue', s=1, alpha=0.5)
plt.grid(True, alpha=0.3)
plt.axis('equal')
plt.xlabel('X (meters)', fontsize=12)
plt.ylabel('Y (meters)', fontsize=12)
plt.title('Robot Workspace', fontsize=14, fontweight='bold')
plt.axhline(y=0, color='k', linestyle='--', alpha=0.3)
plt.axvline(x=0, color='k', linestyle='--', alpha=0.3)

# Min/Max reach
max_uzaklik = L1 + L2
min_uzaklik = abs(L1 - L2)
print(f"✓ Workspace calculated!")
print(f"  Maximum reach: {max_uzaklik:.3f}m (arms straight)")
print(f"  Minimum reach: {min_uzaklik:.3f}m (arms overlapping)")

plt.tight_layout()
plt.show()

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 5: COMPARISON WITH PYBULLET
# =====================================================
print("="*60)
print("SECTION 5: Validation with PyBullet")
print("="*60)
print("Does our FK function match PyBullet's?\n")

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ PyBullet started")

# Load the Kuka robot (we will use its first 2 joints)
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot loaded")

def position_control_joint(joint_no, target):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target
    )

def simulate(adim):
    for _ in range(adim):
        p.stepSimulation()
        time.sleep(1./240.)

# Test angles
test_kombinasyonlar = [
    (0, 0),
    (math.radians(30), math.radians(45)),
    (math.radians(90), math.radians(-30))
]

print("\nComparison:")
for theta1, theta2 in test_kombinasyonlar:
    # Our calculation
    x_bizim, y_bizim = forward_kinematics(theta1, theta2, L1, L2)

    # Give PyBullet the same angles
    position_control_joint(0, theta1)
    position_control_joint(1, theta2)
    simulate(300)

    # PyBullet's calculation
    joint_params = p.getLinkState(robot, 6)
    x_pybullet, y_pybullet, z_pybullet = joint_params[0]

    print(f"\nθ1={math.degrees(theta1):.0f}°, θ2={math.degrees(theta2):.0f}°")
    print(f"  Our calculation: x={x_bizim:.3f}, y={y_bizim:.3f}")
    print(f"  PyBullet       : x={x_pybullet:.3f}, y={y_pybullet:.3f}, z={z_pybullet:.3f}")
    print(f"  Note: PyBullet is 3D, while ours is 2D. We compare X and Y.")

print("\n" + "="*60)
print("LESSON 5 COMPLETE!")
print("="*60)
print("""
WHAT WE LEARNED:
✓ The Forward Kinematics formula
✓ Wrote our own FK function
✓ Visualized with Matplotlib
✓ Understood the workspace concept
✓ Validated with PyBullet

NEXT LESSON:
→ Inverse Kinematics
  "Place the hand at this point" → Calculate the joint angles
""")

p.disconnect()
