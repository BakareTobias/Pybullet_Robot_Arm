"""
LESSON 7: Trajectory Planning
Moving the robot smoothly from point A to point B
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt

print("="*60)
print("LESSON 7: TRAJECTORY PLANNING")
print("="*60)

# =====================================================
# SECTION 1: Problem Definition
# =====================================================
print("\nSECTION 1: Why Trajectory Planning?")
print("-"*60)
print("""
Scenario: Move the robot's end effector from (0, 0.3, 0.5) to (0.4, 0.2, 0.7).

WRONG METHOD: Sudden jump
  t=0.0s: (0.0, 0.3, 0.5)
  t=0.1s: (0.4, 0.2, 0.7)  ← ZIPLAAA!

  Problems:
  ✗ Vibration
  ✗ Motor overload
  ✗ Risk of colliding with objects
  ✗ Unprofessional

CORRECT METHOD: Smooth transition
  t=0.0s: (0.00, 0.30, 0.50)
  t=0.5s: (0.10, 0.28, 0.55)
  t=1.0s: (0.20, 0.26, 0.60)
  t=1.5s: (0.30, 0.24, 0.65)
  t=2.0s: (0.40, 0.20, 0.70) ✓

  Advantages:
  ✓ Smooth movement
  ✓ Safe
  ✓ Predictable
""")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 2: Linear Interpolation
# =====================================================
print("="*60)
print("SECTION 2: Linear Interpolation (The Simplest Method)")
print("="*60)

def linear_interpolation(start_value, end_value, t):
    """
    Performs linear interpolation between two points.

    Parameters:
        start_value: Starting value
        end_value: Ending value
        t: Time (0.0 = start, 1.0 = end)

    Returns:
        Intermediate value
    """
    return start_value + (end_value - start_value) * t

print("\nFormula: p(t) = p_start + (p_end - p_start) * t")
print("  t = 0.0 → Start")
print("  t = 0.5 → Midpoint")
print("  t = 1.0 → End")

# Example
print("\nExample: Moving from 10 to 50")
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    point = linear_interpolation(10, 50, t)
    print(f"  t={t:.2f} → {point:.1f}")

input("Press ENTER to continue...\n")

# Plot the graph
time_passed = np.linspace(0, 1, 100)
position = [linear_interpolation(10, 50, t) for t in time_passed]
hiz = [50 - 10 for _ in time_passed]  # Constant velocity (derivative)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time_passed, position, 'b-', linewidth=2)
ax1.set_ylabel('Position', fontsize=12)
ax1.set_title('Linear Interpolation', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)

ax2.plot(time_passed, hiz, 'r-', linewidth=2)
ax2.set_xlabel('Time (t)', fontsize=12)
ax2.set_ylabel('Velocity', fontsize=12)
ax2.set_title('Velocity Profile (Constant)', fontsize=12)
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("\nObservation:")
print("  ✓ Position: Straight line (linear)")
print("  ✗ Velocity: Constant (sudden start and stop!)")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 3: Cubic Interpolation
# =====================================================
print("="*60)
print("SECTION 3: Cubic Interpolation (Better!)")
print("="*60)

def cubic_interpolation(start_value, end_value, t):
    """
    Cubic interpolation (smooth start and end).

    Formula: p(t) = p_start + (p_end - p_start) * (3t² - 2t³)
    """
    # Smooth transition function
    s = 3 * t**2 - 2 * t**3
    return start_value + (end_value - start_value) * s

print("\nFormula: p(t) = p_start + (p_end - p_start) * (3t² - 2t³)")
print("  This formula smooths the velocity:")
print("  - Starts slowly")
print("  - Speeds up in the middle")
print("  - Slows down at the end\n")

# Example
print("Example: Moving from 10 to 50 (Cubic)")
for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
    point = cubic_interpolation(10, 50, t)
    print(f"  t={t:.2f} → {point:.1f}")

input("Press ENTER to continue...\n")

# Compare the plots
time_passed = np.linspace(0, 1, 100)

linear_pos = [linear_interpolation(10, 50, t) for t in time_passed]
cubic_pos = [cubic_interpolation(10, 50, t) for t in time_passed]

# Calculate velocity (derivative)
linear_vel = np.gradient(linear_pos, time_passed)
cubic_vel = np.gradient(cubic_pos, time_passed)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time_passed, linear_pos, 'b-', linewidth=2, label='Linear')
ax1.plot(time_passed, cubic_pos, 'r-', linewidth=2, label='Cubic')
ax1.set_ylabel('Position', fontsize=12)
ax1.set_title('Linear vs Cubic Interpolation', fontsize=14, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

ax2.plot(time_passed, linear_vel, 'b-', linewidth=2, label='Linear Velocity')
ax2.plot(time_passed, cubic_vel, 'r-', linewidth=2, label='Cubic Velocity')
ax2.set_xlabel('Time (t)', fontsize=12)
ax2.set_ylabel('Velocity', fontsize=12)
ax2.set_title('Velocity Profiles', fontsize=12)
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.show()

print("\nComparison:")
print("  Linear : Starts/stops abruptly (constant velocity is a straight line)")
print("  Cubic  : Starts/stops smoothly (velocity changes gradually)")
print("  ✓ Cubic is more professional!")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 4: Implementation in PyBullet
# =====================================================
print("="*60)
print("SECTION 4: Test with a Real Robot")
print("="*60)

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
print("✓ Simulation started")

robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot loaded")

p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

def position_control_joint(joint_no, target):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=target,
        force=200,
        maxVelocity=3.0
    )

def simulate(steps):
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(1./240.)

def calc_ik(x, y, z):
    """Calculate joint angles using IK."""
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

# Marker
marker = None
def show_target(x, y, z, color=[0, 1, 0, 0.8]):
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass
    sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=color)
    marker = p.createMultiBody(0, baseVisualShapeIndex=sphere, basePosition=[x, y, z])

# =====================================================
# TEST 1: Linear Trajectory
# =====================================================
print("\n" + "="*60)
print("TEST 1: Linear Trajectory")
print("="*60)

start_value_pos = np.array([0.3, 0.3, 0.5])
end_value_pos = np.array([0.5, -0.2, 0.7])

print(f"\nStart: {start_value_pos}")
print(f"End  : {end_value_pos}")
print("\n→ Robot will move LINEARLY (3 seconds)")

# Move to the starting position
eklem_targetlari = calc_ik(*start_value_pos)
for i in range(7):
    position_control_joint(i, eklem_targetlari[i])
simulate(300)

print("  Movement starting...")
sure = 3.0  # seconds
no_steps = int(sure * 240)  # 240 Hz

for steps in range(no_steps):
    t = steps / no_steps  # 0.0 → 1.0

    # Linear interpolation
    target = start_value_pos + (end_value_pos - start_value_pos) * t

    # IK and movement
    show_target(*target, color=[1, 0, 0, 0.8])
    eklem_targetlari = calc_ik(*target)
    for i in range(7):
        position_control_joint(i, eklem_targetlari[i])

    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Linear movement complete")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 2: Cubic Trajectory
# =====================================================
print("="*60)
print("TEST 2: Cubic Trajectory (Smoother)")
print("="*60)

# Same start and end points
print(f"\nStart: {start_value_pos}")
print(f"End  : {end_value_pos}")
print("\n→ Robot will move using CUBIC interpolation (3 seconds)")

# Return to the start
eklem_targetlari = calc_ik(*start_value_pos)
for i in range(7):
    position_control_joint(i, eklem_targetlari[i])
simulate(300)

print("  Movement starting...")

for steps in range(no_steps):
    t = steps / no_steps  # 0.0 → 1.0

    # Cubic interpolation
    s = 3 * t**2 - 2 * t**3  # Smooth transition
    target = start_value_pos + (end_value_pos - start_value_pos) * s

    # IK and movement
    show_target(*target, color=[0, 1, 0, 0.8])
    eklem_targetlari = calc_ik(*target)
    for i in range(7):
        position_control_joint(i, eklem_targetlari[i])

    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Cubic movement complete")
print("\nDid you notice the difference?")
print("  Linear: Abrupt start/end")
print("  Cubic : Smooth start/end")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 3: Multiple Points (Waypoints)
# =====================================================
print("="*60)
print("TEST 3: Waypoints (Passing Through Intermediate Points)")
print("="*60)

waypoints = [
    np.array([0.4, 0.2, 0.5]),   # Point 1
    np.array([0.5, -0.1, 0.7]),  # Point 2
    np.array([0.3, -0.3, 0.6]),  # Point 3
    np.array([0.2, 0.0, 0.4])    # Point 4
]

print(f"\nThe robot will pass through {len(waypoints)} waypoints:")
for idx, wp in enumerate(waypoints, 1):
    print(f"  coordinates {idx}: {wp}")

# Show the points (persistent spheres)
for wp in waypoints:
    sphere = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 1, 0, 0.7])
    p.createMultiBody(0, baseVisualShapeIndex=sphere, basePosition=wp.tolist())

print("\n→ The robot will pass through all points...")

for idx in range(len(waypoints) - 1):
    start_value = waypoints[idx]
    end_value = waypoints[idx + 1]

    print(f"  Segment {idx+1}: {start_value} → {end_value}")

    segment_sure = 2.0
    segment_steps = int(segment_sure * 240)

    for steps in range(segment_steps):
        t = steps / segment_steps
        s = 3 * t**2 - 2 * t**3  # Cubic
        target = start_value + (end_value - start_value) * s

        show_target(*target, color=[0, 0, 1, 0.8])
        eklem_targetlari = calc_ik(*target)
        for i in range(7):
            position_control_joint(i, eklem_targetlari[i])

        p.stepSimulation()
        time.sleep(1./240.)

print("\n✓ Passed through all waypoints!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 7 SUMMARY")
print("="*60)
print("""
WHAT WE LEARNED:
✓ What trajectory planning is
✓ Linear interpolation (simple but abrupt)
✓ Cubic interpolation (smooth and professional)
✓ Waypoints (passing through intermediate points)

FORMULAS:
  Linear: p(t) = p₀ + (p₁ - p₀) * t
  Cubic : p(t) = p₀ + (p₁ - p₀) * (3t² - 2t³)

REAL-WORLD APPLICATIONS:
  - Industrial robots always use trajectory planning
  - Pick-and-place applications
  - Welding, painting, assembly

NEXT LESSON:
→ Collision Avoidance
  Reaching the target while avoiding obstacles!
""")

p.disconnect()
print("\n✓ PyBullet disconnected")
