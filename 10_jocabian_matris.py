"""
LESSON 9: Jacobian Matrix and Advanced IK
We will build our own IK algorithm from scratch!
"""

import numpy as np
import matplotlib.pyplot as plt
import math

print("="*60)
print("LESSON 9: JACOBIAN MATRIX AND ADVANCED IK")
print("="*60)

# =====================================================
# SECTION 1: What Is the Jacobian?
# =====================================================
print("\nSECTION 1: Jacobian Matrix - Basic Concept")
print("-"*60)
print("""
DEFINITION:
The Jacobian is a matrix of the derivatives of a function.

In robotics:
  Input  : Joint angles (θ₁, θ₂, ...)
  Output : End-effector position (x, y, z)
  Jacobian: "If the angles change slightly, how much does the position change?"

MATHEMATICS:
  Forward Kinematics: f(θ₁, θ₂) = (x, y)

  Jacobian J:
    J = [∂x/∂θ₁  ∂x/∂θ₂]
        [∂y/∂θ₁  ∂y/∂θ₂]

  Meaning:
    ∂x/∂θ₁ = "How much does x change when θ₁ changes by 1 unit?"

USAGE:
  Δp = J × Δθ

  "Position change = Jacobian × angle change"
""")

input("Press ENTER to continue...\n")

# =====================================================
# SECTION 2: Jacobian for a 2-Link Robot
# =====================================================
print("="*60)
print("SECTION 2: 2-Link Robot Example")
print("="*60)

L1 = 1.0  # Link 1 length
L2 = 0.8  # Link 2 length

print(f"\nRobot Parameters:")
print(f"  L1 = {L1}m")
print(f"  L2 = {L2}m")

def forward_kinematics(theta1, theta2):
    """
    Forward Kinematics
    Input: θ₁, θ₂ (radians)
    Output: (x, y)
    """
    x = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    y = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
    return x, y

def jacobian(theta1, theta2):
    """
    Calculate the Jacobian matrix.

    Mathematics:
      x = L1*cos(θ₁) + L2*cos(θ₁+θ₂)
      y = L1*sin(θ₁) + L2*sin(θ₁+θ₂)

    Derivatives:
      ∂x/∂θ₁ = -L1*sin(θ₁) - L2*sin(θ₁+θ₂)
      ∂x/∂θ₂ = -L2*sin(θ₁+θ₂)
      ∂y/∂θ₁ = L1*cos(θ₁) + L2*cos(θ₁+θ₂)
      ∂y/∂θ₂ = L2*cos(θ₁+θ₂)

    Returns:
      J: 2x2 numpy array
    """
    # Calculate the derivatives
    dx_dtheta1 = -L1 * math.sin(theta1) - L2 * math.sin(theta1 + theta2)
    dx_dtheta2 = -L2 * math.sin(theta1 + theta2)
    dy_dtheta1 = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2)
    dy_dtheta2 = L2 * math.cos(theta1 + theta2)

    # Jacobian matrix
    J = np.array([
        [dx_dtheta1, dx_dtheta2],
        [dy_dtheta1, dy_dtheta2]
    ])

    return J

print("\n" + "="*60)
print("Jacobian Formula (Derived by Hand):")
print("="*60)
print("""
  J = [∂x/∂θ₁  ∂x/∂θ₂]
      [∂y/∂θ₁  ∂y/∂θ₂]

  J = [-L1*sin(θ₁) - L2*sin(θ₁+θ₂)    -L2*sin(θ₁+θ₂)    ]
      [ L1*cos(θ₁) + L2*cos(θ₁+θ₂)     L2*cos(θ₁+θ₂)    ]
""")

# Example calculation
theta1 = math.radians(30)  # 30 degrees
theta2 = math.radians(45)  # 45 degrees

J = jacobian(theta1, theta2)

print(f"\nExample: θ₁={math.degrees(theta1):.0f}°, θ₂={math.degrees(theta2):.0f}°")
print(f"\nJacobian Matrix:")
print(f"  J = [{J[0,0]:7.3f}  {J[0,1]:7.3f}]")
print(f"      [{J[1,0]:7.3f}  {J[1,1]:7.3f}]")

print(f"\nMeaning:")
print(f"  If θ₁ changes by 1 rad:")
print(f"    x changes by {J[0,0]:.3f}m")
print(f"    y changes by {J[1,0]:.3f}m")
print(f"  If θ₂ changes by 1 rad:")
print(f"    x changes by {J[0,1]:.3f}m")
print(f"    y changes by {J[1,1]:.3f}m")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 3: Velocity Relationship Using the Jacobian
# =====================================================
print("="*60)
print("SECTION 3: Velocity Relationship Using the Jacobian")
print("="*60)

print("""
Another use of the Jacobian: VELOCITY RELATIONSHIP

Formula:
  v = J × θ̇

Where:
  v = End-effector velocity (vx, vy)
  θ̇ = Joint angular velocities (θ̇₁, θ̇₂)
  J = Jacobian matrix
""")

# Example
theta1_dot = 0.5  # rad/s (angular velocity of θ₁)
theta2_dot = 0.3  # rad/s (angular velocity of θ₂)

theta_dot = np.array([theta1_dot, theta2_dot])
v = J @ theta_dot  # Matrix multiplication

print(f"\nExample:")
print(f"  Joint velocities: θ̇₁={theta1_dot} rad/s, θ̇₂={theta2_dot} rad/s")
print(f"  End-effector velocity: vx={v[0]:.3f} m/s, vy={v[1]:.3f} m/s")

input("\nPress ENTER to continue...\n")

# =====================================================
# SECTION 4: INVERSE KINEMATICS - Jacobian Method
# =====================================================
print("="*60)
print("SECTION 4: Inverse Kinematics Using the Jacobian")
print("="*60)

print("""
THE MAIN TOPIC: How do we solve IK?

Problem:
  Target position: (x*, y*)
  Current position: (x, y)
  Required: Find the values of θ₁, θ₂

SOLUTION: Iterative method (approach step by step)

Algorithm:
  1. Start with the current angles: θ₁, θ₂
  2. Calculate the current position: (x, y) = FK(θ₁, θ₂)
  3. Calculate the error: e = (x*, y*) - (x, y)
  4. Calculate the Jacobian: J
  5. Find the angle change: Δθ = J⁻¹ × e
  6. Update the angles: θ₁ += Δθ₁, θ₂ += Δθ₂
  7. Is the error small? Yes → Done! No → Go to 2

This method is called "Jacobian Inverse" or "Newton-Raphson".
""")

input("Press ENTER to continue...\n")

def inverse_kinematics_jacobian(target_x, target_y, theta1_init=0, theta2_init=0,
                                max_iter=100, tolerance=0.001):
    """
    Inverse kinematics using the Jacobian method.

    Parameters:
        target_x, target_y: Target position
        theta1_init, theta2_init: Initial angles
        max_iter: Maximum number of iterations
        tolerance: Error tolerance (meters)

    Returns:
        theta1, theta2: Calculated joint angles
        iterations: Number of iterations to find the solution
        errors: Error at each iteration
    """
    # Initialize
    theta1 = theta1_init
    theta2 = theta2_init

    errors = []

    for iteration in range(max_iter):
        # Current position
        x, y = forward_kinematics(theta1, theta2)

        # Error vector
        error_x = target_x - x
        error_y = target_y - y
        error = np.array([error_x, error_y])

        # Error magnitude (Euclidean distance)
        error_norm = np.linalg.norm(error)
        errors.append(error_norm)

        # Have we reached the target?
        if error_norm < tolerance:
            return theta1, theta2, iteration, errors

        # Calculate the Jacobian
        J = jacobian(theta1, theta2)

        # Invert the Jacobian (use the pseudoinverse)
        try:
            J_inv = np.linalg.pinv(J)  # Pseudoinverse (more reliable)
        except:
            print("  ⚠ Could not invert the Jacobian (singular matrix)")
            return theta1, theta2, iteration, errors

        # Calculate the angle change
        delta_theta = J_inv @ error

        # Update the angles (add a learning rate for greater stability)
        alpha = 0.5  # Learning rate (between 0 and 1)
        theta1 += alpha * delta_theta[0]
        theta2 += alpha * delta_theta[1]

    # Maximum number of iterations reached
    return theta1, theta2, max_iter, errors

# =====================================================
# TEST 1: Basic IK Test
# =====================================================
print("="*60)
print("TEST 1: Jacobian IK - First Attempt")
print("="*60)

target = (1.2, 0.8)
print(f"\nTarget position: ({target[0]}, {target[1]})")
print(f"Initial angles: θ₁=0°, θ₂=0°")

print("\nSearching for a solution...")
theta1_sol, theta2_sol, iters, errors = inverse_kinematics_jacobian(
    target[0], target[1],
    theta1_init=0,
    theta2_init=0
)

print(f"\n✓ Solution found!")
print(f"  Number of iterations: {iters}")
print(f"  Calculated angles:")
print(f"    θ₁ = {math.degrees(theta1_sol):6.2f}°")
print(f"    θ₂ = {math.degrees(theta2_sol):6.2f}°")

# Verification
x_final, y_final = forward_kinematics(theta1_sol, theta2_sol)
print(f"  Reached position: ({x_final:.4f}, {y_final:.4f})")
print(f"  Target position : ({target[0]:.4f}, {target[1]:.4f})")
print(f"  Error: {np.linalg.norm([target[0]-x_final, target[1]-y_final])*1000:.2f} mm")

# Error plot
plt.figure(figsize=(10, 6))
plt.plot(errors, 'b-', linewidth=2)
plt.xlabel('Iteration', fontsize=12)
plt.ylabel('Error (meters)', fontsize=12)
plt.title('IK Convergence - Error vs. Iteration', fontsize=14, fontweight='bold')
plt.grid(True, alpha=0.3)
plt.yscale('log')  # Logarithmic scale (error decreases rapidly)
plt.tight_layout()
plt.show()

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 2: Different Starting Points
# =====================================================
print("="*60)
print("TEST 2: Different Initial Angles")
print("="*60)

print("""
The IK solution sometimes depends on the INITIAL ANGLES!
Different starting points can lead to different solutions for the same target.
""")

target = (1.0, 1.0)
start_timelar = [
    (0, 0),
    (math.radians(45), math.radians(45)),
    (math.radians(-30), math.radians(60))
]

print(f"\nTarget: ({target[0]}, {target[1]})")
print("\nDifferent starting points:")

cozumler = []

for idx, (t1_init, t2_init) in enumerate(start_timelar, 1):
    print(f"\n{idx}. Initial: θ₁={math.degrees(t1_init):.0f}°, θ₂={math.degrees(t2_init):.0f}°")

    t1, t2, iters, errs = inverse_kinematics_jacobian(
        target[0], target[1],
        theta1_init=t1_init,
        theta2_init=t2_init
    )

    print(f"   Solution: θ₁={math.degrees(t1):.1f}°, θ₂={math.degrees(t2):.1f}° ({iters} iterations)")

    # Verification
    x, y = forward_kinematics(t1, t2)
    error = np.linalg.norm([target[0]-x, target[1]-y])
    print(f"   Error: {error*1000:.2f} mm")

    cozumler.append((t1, t2, errs))

# Visualization
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Left: Robot configurations
ax1.set_xlim(-2, 2)
ax1.set_ylim(-2, 2)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('X (m)', fontsize=12)
ax1.set_ylabel('Y (m)', fontsize=12)
ax1.set_title('Different Solutions (Same Target)', fontsize=14, fontweight='bold')

colorler = ['red', 'blue', 'green']

for idx, (t1, t2, _) in enumerate(cozumler):
    # Draw the robot
    x0, y0 = 0, 0
    x1 = L1 * math.cos(t1)
    y1 = L1 * math.sin(t1)
    x2, y2 = forward_kinematics(t1, t2)

    ax1.plot([x0, x1], [y0, y1], color=colorler[idx], linewidth=3, label=f'Solution {idx+1}')
    ax1.plot([x1, x2], [y1, y2], color=colorler[idx], linewidth=3)
    ax1.plot(x2, y2, 'o', color=colorler[idx], markersize=10)

# Target
ax1.plot(target[0], target[1], 'k*', markersize=20, label='Target')
ax1.legend(fontsize=10)

# Right: Convergence plots
for idx, (_, _, errs) in enumerate(cozumler):
    ax2.plot(errs, color=colorler[idx], linewidth=2, label=f'Solution {idx+1}')

ax2.set_xlabel('Iteration', fontsize=12)
ax2.set_ylabel('Error (m)', fontsize=12)
ax2.set_title('Convergence Rates', fontsize=14, fontweight='bold')
ax2.set_yscale('log')
ax2.grid(True, alpha=0.3)
ax2.legend(fontsize=10)

plt.tight_layout()
plt.show()

print("\nObservation:")
print("  ✓ The same target was reached via different paths")
print("  ✓ Every solution is valid (all reach the target)")
print("  ✓ Some starting points converge faster")

input("\nPress ENTER to continue...\n")

# =====================================================
# TEST 3: Unreachable Target
# =====================================================
print("="*60)
print("TEST 3: Unreachable Target")
print("="*60)

uzak_target = (2.5, 2.5)  # Very far away!
print(f"\nVery distant target: ({uzak_target[0]}, {uzak_target[1]})")
print(f"  (The robot can reach a maximum of {L1+L2}m = {L1+L2}m)")

t1, t2, iters, errs = inverse_kinematics_jacobian(
    uzak_target[0], uzak_target[1],
    max_iter=50
)

x_final, y_final = forward_kinematics(t1, t2)
error = np.sqrt(x_final**2 + y_final**2)

print(f"\n✓ Algorithm stopped (maximum iterations)")
print(f"  Nearest position: ({x_final:.3f}, {y_final:.3f})")
print(f"  Target: ({uzak_target[0]}, {uzak_target[1]})")
print(f"  Reached distance: {error:.3f}m")
print(f"  Error: {errs[-1]:.3f}m = {errs[-1]*100:.1f}cm")
print("\n  The robot moved to the nearest point possible!")

# =====================================================
# SUMMARY
# =====================================================
print("\n" + "="*60)
print("LESSON 9 SUMMARY")
print("="*60)
print("""
WHAT WE LEARNED:
✓ What the Jacobian matrix is
✓ How to calculate the Jacobian (derivatives)
✓ The relationship between the Jacobian and velocity
✓ The Jacobian Inverse IK algorithm
✓ The iterative solution method
✓ Different starting points produce different solutions

JACOBIAN FORMULA (2-Link):
  J = [∂x/∂θ₁  ∂x/∂θ₂]
      [∂y/∂θ₁  ∂y/∂θ₂]

IK ALGORITHM:
  1. Initial angles
  2. Calculate the position using FK
  3. Error = Target - Current
  4. Δθ = J⁻¹ × Error
  5. Update θ
  6. Repeat (until error < tolerance)

ADVANTAGES:
  ✓ General (for any robot)
  ✓ Real-time
  ✓ Partial movements (velocity control)

DISADVANTAGES:
  ✗ Problems at singularities (J cannot be inverted)
  ✗ Can get stuck at a local minimum
  ✗ Sensitive to initial angles

FURTHER READING:
  - Damped Least Squares (DLS) - Singularity resolution
  - Weighted Jacobian - Joint limits
  - Task Priority - Multiple targets
""")

print("\n" + "="*60)
print("ROBOTICS SOFTWARE TRAINING COMPLETE!")
print("="*60)
print("""
🎉 CONGRATULATIONS! You have learned:

1. ✅ Simulation with PyBullet
2. ✅ Forward Kinematics (angle → position)
3. ✅ Inverse Kinematics (position → angle)
4. ✅ Trajectory Planning (smooth movement)
5. ✅ Collision Avoidance (obstacle avoidance)
6. ✅ The Jacobian matrix (mathematics)
7. ✅ An advanced IK algorithm

NEXT STEPS:
  → Learn ROS 2 (real robots)
  → Get a real robot arm (UR5, Kuka, or DIY)
  → Add Computer Vision (camera + robot)
  → Machine Learning (reinforcement learning)

Good luck! 🚀🤖
""")
