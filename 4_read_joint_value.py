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
def read_joint_state(joint_no):
    """
    Read the actual state of a joint.

    Returns:
        joint_angle: Current joint angle (radians)
        velocity: Joint rotational velocity (radians/second)
    """
    joint_params = p.getJointState(robot, joint_no)

    # joint_params is a tuple (multiple values):
    # joint_params[0] = position (angle)
    # joint_params[1] = velocity
    # joint_params[2] = reaction forces (not used for now)
    # joint_params[3] = applied torque

    joint_angle = joint_params[0]
    velocity = joint_params[1]

    return joint_angle, velocity

# =====================================================
# Previous functions
# =====================================================
def position_control_joint(joint_no, joint_angle):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=joint_angle
    )

def simulate(no_steps):
    for i in range(no_steps):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: Read the initial position
# =====================================================
print("="*50)
print("TEST 1: Initial Position")
print("="*50)

# Read the state of the first 3 joints
for joint in range(4):
    joint_angle, joint_velocity = read_joint_state(joint)
    print(f"Joint {joint}:")
    print(f"  Angle: {joint_angle:.3f} radians = {math.degrees(joint_angle):.1f} degrees")
    print(f"  Velocity: {joint_velocity:.3f} rad/s")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 2: Compare Target vs. Actual
# =====================================================
print("\n" + "="*50)
print("TEST 2: Target vs. Actual Angle")
print("="*50)

joint_target = 1.0  # ~57 degrees

print(f"\nTarget: Move joint 0 to {joint_target:.3f} radians ({math.degrees(joint_target):.1f}°)")
position_control_joint(0, joint_target)

# Show the state every 10 steps during movement
print("\nMoving...")
for step in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

    # Report every 10 steps
    if step % 10 == 0:
        joint_angle, joint_velocity = read_joint_state(0)
        print(f"  Step {step:3d}: Angle={math.degrees(joint_angle):6.2f}°, Velocity={joint_velocity:6.3f} rad/s")

# Final state
joint_angle_final, joint_velocity_final = read_joint_state(0)
print(f"\n✓ Movement complete!")
print(f"  Target angle : {math.degrees(joint_target):.2f}°")
print(f"  Actual angle: {math.degrees(joint_angle_final):.2f}°")
print(f"  Difference      : {math.degrees(joint_angle_final - joint_target):.2f}°")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 3: Coordinated Movement of Two Joints
# =====================================================
print("\n" + "="*50)
print("TEST 3: Coordinated Movement of Two Joints")
print("="*50)

# Targets
joint_target_0 = 0.8   # ~46 degrees
joint_target_1 = -0.6  # ~-34 degrees
joint_target_2 = 1.57
joint_target_3 = -1.048

print(f"\nTargets:")
print(f"  Joint 0: {math.degrees(joint_target_0):.1f}°")
print(f"  Joint 1: {math.degrees(joint_target_1):.1f}°")

position_control_joint(0, joint_target_0)
position_control_joint(1, joint_target_1)
position_control_joint(2, joint_target_2)
position_control_joint(3, joint_target_3)

print("\nMoving...")
for step in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

    if step % 10 == 0:
        joint_angle_0, joint_velocity_0 = read_joint_state(0)
        joint_angle_1, joint_velocity_1 = read_joint_state(1)
        joint_angle_2, joint_velocity_2 = read_joint_state(2)
        joint_angle_3, joint_velocity_3 = read_joint_state(3)
        print(f"    Step {step:3d}:")
        print(f"    Joint 0: {math.degrees(joint_angle_0):6.2f}°")
        print(f"    Joint 1: {math.degrees(joint_angle_1):6.2f}°")
        print(f"    Joint 2: {math.degrees(joint_angle_2):6.2f}°")
        print(f"    Joint 3: {math.degrees(joint_angle_3):6.2f}°")

print("\n✓ Coordinated movement complete!")

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Reset Angles
# =====================================================
print("\n" + "="*50)
print("TEST 4: Reset Angles")
print("="*50)

position_control_joint(0, 0)
position_control_joint(1, 0)
position_control_joint(2, 0)
position_control_joint(3, 0)

for step in range(50):
    p.stepSimulation()
    time.sleep(1./240.)

    if step % 10 == 0:
        joint_angle_0, joint_velocity_0 = read_joint_state(0)
        joint_angle_1, joint_velocity_1 = read_joint_state(1)
        joint_angle_2, joint_velocity_2 = read_joint_state(2)
        joint_angle_3, joint_velocity_3 = read_joint_state(3)
        print(f"    Step {step:3d}:")
        print(f"    Joint 0: {math.degrees(joint_angle_0):6.2f}°")
        print(f"    Joint 1: {math.degrees(joint_angle_1):6.2f}°")
        print(f"    Joint 2: {math.degrees(joint_angle_2):6.2f}°")
        print(f"    Joint 3: {math.degrees(joint_angle_3):6.2f}°")

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

start_time = time.time()
t = 0

while time.time() - start_time < 10:  # 10 seconds 
    # Sinusoidal movement
    target_0 = 0.6 * math.sin(t)
    target_1 = 0.4 * math.cos(t * 1.3)

    position_control_joint(0, target_0)
    position_control_joint(1, target_1)

    # Read the actual state
    joint_angle_0, joint_velocity_0 = read_joint_state(0)
    joint_angle_1, joint_velocity_1 = read_joint_state(1)

    # Print to the screen (update the same line)
    print(f"\rJoint 0: {math.degrees(joint_angle_0):6.1f}° | "
          f"Joint 1: {math.degrees(joint_angle_1):6.1f}° | "
          f"Time: {time.time()-start_time:.1f}s", end="")

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
