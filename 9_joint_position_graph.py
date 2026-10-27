# ADVANCED ROBOT CONTROL SIMULATION
# LESSON 3+: Advanced Joint State Reading and Control

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import threading

class RobotSimulator:
    def __init__(self, gui=True):
        """Starts the robot simulator and loads the robot."""
        self.gui = gui
        self.simulation_running = False
        self.joint_states_history = {i: deque(maxlen=100) for i in range(7)}  # Store the last 100 steps
        self.time_history = deque(maxlen=100)

        # Start the simulation
        if self.gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Simplify the GUI
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)  # Step-by-step simulation
        p.setTimeStep(1./240.)

        # Add the ground
        p.loadURDF("plane.urdf")

        # Load the robot
        self.robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot)

        # Get joint information
        self.joint_info = []
        for i in range(self.num_joints):
            info = p.getJointInfo(self.robot, i)
            self.joint_info.append({
                'name': info[1].decode('utf-8'),
                'type': info[2],
                'lower_limit': info[8],
                'upper_limit': info[9],
                'max_force': info[10],
                'max_velocity': info[11]
            })

        # Joint control parameters
        self.joint_gains = [0.1] * self.num_joints
        self.joint_damping = [0.5] * self.num_joints

        print("✓ Simulation started")
        print(f"✓ Robot loaded: {self.num_joints} joints detected")

        # Reset the starting position
        self.reset_robot()

    def reset_robot(self):
        """Resets the robot to its starting position."""
        for i in range(self.num_joints):
            p.resetJointState(self.robot, i, 0.0)
        print("✓ Robot reset to its starting position")

    def get_joint_state(self, joint_index):
        """
        Reads the current state of a joint.

        Parameters:
            joint_index: Joint index

        Returns:
            position: Current joint angle (radians)
            velocity: Joint angular velocity (radians/second)
            reaction_forces: Reaction forces
            applied_torque: Applied torque
        """
        try:
            state = p.getJointState(self.robot, joint_index)
            position, velocity, reaction_forces, applied_torque = state
            return position, velocity, reaction_forces, applied_torque
        except Exception as e:
            print(f"Error: Could not read the state of joint {joint_index}: {e}")
            return None, None, None, None

    def set_joint_position(self, joint_index, target_position, max_force=None):
        """
        Moves a joint to the target position.

        Parameters:
            joint_index: Joint index
            target_position: Target angle (radians)
            max_force: Maximum applied force (automatic if None)
        """
        if max_force is None:
            max_force = self.joint_info[joint_index]['max_force']

        p.setJointMotorControl2(
            bodyUniqueId=self.robot,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=max_force,
            positionGain=self.joint_gains[joint_index],
            velocityGain=self.joint_damping[joint_index]
        )

    def set_joint_positions(self, target_positions):
        """
        Moves multiple joints to their target positions simultaneously.

        Parameters:
            target_positions: List of target angles (radians)
        """
        for i, target in enumerate(target_positions):
            if i < self.num_joints:
                self.set_joint_position(i, target)

    def simulate(self, steps=1):
        """Advances the simulation by the specified number of steps."""
        for _ in range(steps):
            p.stepSimulation()
            if self.gui:
                time.sleep(1./240.)

    def update_joint_history(self):
        """Updates the joint state history."""
        current_time = time.time()
        self.time_history.append(current_time)

        for i in range(self.num_joints):
            pos, vel, _, _ = self.get_joint_state(i)
            self.joint_states_history[i].append((pos, vel))

    def print_joint_states(self, joint_indices=None):
        """
        Prints joint states to the console.

        Parameters:
            joint_indices: Joint indices to print (all joints if None)
        """
        if joint_indices is None:
            joint_indices = range(self.num_joints)

        print("\n" + "="*50)
        print("JOINT STATES")
        print("="*50)

        for i in joint_indices:
            if i < self.num_joints:
                pos, vel, _, _ = self.get_joint_state(i)
                if pos is not None:
                    print(f"Joint {i} ({self.joint_info[i]['name']}):")
                    print(f"  Angle: {pos:.3f} radians = {math.degrees(pos):.1f} degrees")
                    print(f"  Velocity: {vel:.3f} rad/s")

    def get_end_effector_position(self):
        """Returns the position of the robot's end effector."""
        end_effector_link = self.num_joints - 1
        state = p.getLinkState(self.robot, end_effector_link)
        position, orientation = state[0], state[1]
        return position, orientation

    def plot_joint_positions(self, joint_indices=None, show_plot=True):
        """
        Plots joint positions.

        Parameters:
            joint_indices: Joint indices to plot (all joints if None)
            show_plot: Show the plot immediately (True) or return the figure (False)
        """
        if joint_indices is None:
            joint_indices = range(self.num_joints)

        plt.figure(figsize=(10, 6))

        for i in joint_indices:
            if i < self.num_joints and len(self.joint_states_history[i]) > 0:
                positions = [state[0] for state in self.joint_states_history[i]]
                times = list(range(len(positions)))
                plt.plot(times, positions, label=f"Joint {i} ({self.joint_info[i]['name']})")

        plt.xlabel("Time (step)")
        plt.ylabel("Position (radians)")
        plt.title("Joint Positions")
        plt.legend()
        plt.grid(True)

        if show_plot:
            plt.show()
            return None
        else:
            return plt.gcf()

    def run_test_1(self):
        """TEST 1: Read the starting position."""
        print("\n" + "="*50)
        print("TEST 1: Starting Position")
        print("="*50)

        self.print_joint_states(range(3))

        input("\nPress ENTER to continue...")

    def run_test_2(self):
        """TEST 2: Compare target and actual positions."""
        print("\n" + "="*50)
        print("TEST 2: Target vs. Actual Angle")
        print("="*50)

        target_angle = 1.0  # ~57 degrees
        print(f"\nTarget: Move joint 0 to {target_angle:.3f} radians ({math.degrees(target_angle):.1f}°)")
        self.set_joint_position(0, target_angle)

        # Show the state every 100 steps while moving
        print("\nMoving...")
        for step in range(500):
            self.simulate(1)
            self.update_joint_history()

            # Report every 100 steps
            if step % 100 == 0:
                pos, vel, _, _ = self.get_joint_state(0)
                print(f"  Step {step:3d}: Angle={math.degrees(pos):6.2f}°, Velocity={vel:6.3f} rad/s")

        # Final state
        pos_final, vel_final, _, _ = self.get_joint_state(0)
        print(f"\n✓ Movement complete!")
        print(f"  Target angle : {math.degrees(target_angle):.2f}°")
        print(f"  Actual angle: {math.degrees(pos_final):.2f}°")
        print(f"  Difference      : {math.degrees(pos_final - target_angle):.2f}°")

        input("\nPress ENTER to continue...")

    def run_test_3(self):
        """TEST 3: Coordinated movement of two joints."""
        print("\n" + "="*50)
        print("TEST 3: Coordinated Movement of Two Joints")
        print("="*50)

        # Targets
        target_0 = 0.8   # ~46 degrees
        target_1 = -0.6  # ~-34 degrees

        print(f"\nTargets:")
        print(f"  Joint 0: {math.degrees(target_0):.1f}°")
        print(f"  Joint 1: {math.degrees(target_1):.1f}°")

        self.set_joint_position(0, target_0)
        self.set_joint_position(1, target_1)

        print("\nMoving...")
        for step in range(500):
            self.simulate(1)
            self.update_joint_history()

            if step % 100 == 0:
                pos_0, vel_0, _, _ = self.get_joint_state(0)
                pos_1, vel_1, _, _ = self.get_joint_state(1)
                print(f"  Step {step:3d}:")
                print(f"    Joint 0: {math.degrees(pos_0):6.2f}°")
                print(f"    Joint 1: {math.degrees(pos_1):6.2f}°")

        print("\n✓ Coordinated movement complete!")

        input("\nPress ENTER to continue...")

    def run_test_4(self):
        """TEST 4: Live monitoring (real time)."""
        print("\n" + "="*50)
        print("TEST 4: Live Monitoring - Robot Oscillation")
        print("="*50)
        print("The robot will oscillate for 10 seconds")
        print("Joint states will be displayed in real time\n")

        start_time = time.time()
        t = 0

        while time.time() - start_time < 10:  # 10 seconds
            # Sinusoidal motion
            target_0 = 0.6 * math.sin(t)
            target_1 = 0.4 * math.cos(t * 1.3)

            self.set_joint_position(0, target_0)
            self.set_joint_position(1, target_1)

            # Read the actual state
            pos_0, vel_0, _, _ = self.get_joint_state(0)
            pos_1, vel_1, _, _ = self.get_joint_state(1)

            # Print to the console (update the same line)
            print(f"\rJoint 0: {math.degrees(pos_0):6.1f}° | "
                  f"Joint 1: {math.degrees(pos_1):6.1f}° | "
                  f"Time: {time.time()-start_time:.1f}s", end="")

            self.simulate(1)
            self.update_joint_history()
            t += 0.02

        print("\n\n✓ Live monitoring complete!")

        input("\nPress ENTER to continue...")

    def run_test_5(self):
        """TEST 5: End-effector tracking."""
        print("\n" + "="*50)
        print("TEST 5: End-Effector Tracking")
        print("="*50)
        print("The robot's end-effector position will be tracked during movement\n")

        # Record the starting position
        start_pos, _ = self.get_end_effector_position()
        print(f"Starting position: [{start_pos[0]:.3f}, {start_pos[1]:.3f}, {start_pos[2]:.3f}]")

        # Target positions
        targets = [
            [0.5, 0.0, 0.5],
            [0.5, 0.5, 0.5],
            [0.0, 0.5, 0.5],
            [0.0, 0.0, 0.5]
        ]

        for i, target in enumerate(targets):
            print(f"\nTarget {i+1}: [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")

            # A simple inverse-kinematics approach (for demonstration only)
            # A more advanced algorithm is needed in real applications
            if target[0] != 0:
                self.set_joint_position(0, math.atan2(target[1], target[0]))
            else:
                self.set_joint_position(0, 0)

            if target[1] != 0:
                self.set_joint_position(1, math.atan2(target[2], math.sqrt(target[0]**2 + target[1]**2)))
            else:
                self.set_joint_position(1, math.pi/2)

            # Move and wait
            for step in range(200):
                self.simulate(1)
                self.update_joint_history()

            # Read the final position
            end_pos, _ = self.get_end_effector_position()
            print(f"Final position: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

            # Calculate the error
            error = math.sqrt(sum((end_pos[j] - target[j])**2 for j in range(3)))
            print(f"Error: {error:.3f} m")

        input("\nPress ENTER to continue...")

    def run_test_6(self):
        """TEST 6: Joint position plot."""
        print("\n" + "="*50)
        print("TEST 6: Joint Position Plot")
        print("="*50)
        print("Joint movements from previous tests will be plotted\n")

        # Make random movements and collect data
        print("Making random movements...")
        for _ in range(300):
            # Random targets
            targets = [np.random.uniform(-1, 1) for _ in range(3)]
            self.set_joint_positions(targets)

            for _ in range(10):
                self.simulate(1)
                self.update_joint_history()

        print("Data collected; plotting...")
        self.plot_joint_positions(range(3))

        input("\nPress ENTER to continue...")

    def run_all_tests(self):
        """Runs all tests in sequence."""
        self.run_test_1()
        self.run_test_2()
        self.run_test_3()
        self.run_test_4()
        self.run_test_5()
        self.run_test_6()

        # Summary
        print("\n" + "="*50)
        print("ALL TESTS COMPLETE")
        print("="*50)
        print("✓ Learned to read joint states")
        print("✓ Compared target and actual angles")
        print("✓ Performed real-time monitoring")
        print("✓ Tracked the end-effector position")
        print("✓ Plotted joint movements")
        print("\nNext steps:")
        print("- More advanced kinematics calculations")
        print("- Path-planning algorithms")
        print("- Multi-robot coordination")
        print("="*50)

    def disconnect(self):
        """Closes the simulation connection."""
        p.disconnect()
        print("✓ Simulation connection closed")


def main():
    """Main function."""
    try:
        # Start the simulator
        sim = RobotSimulator(gui=True)

        # Run all tests
        sim.run_all_tests()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the connection
        if 'sim' in locals():
            sim.disconnect()


if __name__ == "__main__":
    main()
