"""
LESSON 5: PICK & PLACE WITH A PYBULLET CONVEYOR BELT (ADVANCED)
================================================================

PyBullet advantages:
✓ 100% Python - no Lua scripts!
✓ Powerful physics simulation
✓ Easy transition to a real robot
✓ ROS integration
✓ Ideal for machine learning

Installation:
pip install pybullet numpy

IMPROVEMENTS:
✓ Franka Panda robot (including gripper!)
✓ Conveyor belt positioned to avoid colliding with the robot
✓ Better positioning
✓ Improved gripper control

"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math

class PyBulletConveyorSystem:
    """
    Conveyor belt pick-and-place system using PyBullet
    With a Franka Panda robot
    """
    
    def __init__(self, gui=True):
        """
        Args:
            gui: If True, open the graphical interface
        """
        
        # Start PyBullet
        if gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # Simplify the GUI
            print("✅ PyBullet GUI mode started")
        else:
            self.physics_client = p.connect(p.DIRECT)
            print("✅ PyBullet DIRECT mode started")
        
        # Add additional data paths
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Set gravity
        p.setGravity(0, 0, -9.81)
        
        # Time step
        p.setTimeStep(1./240.)
        
        # Objects
        self.robot_id = None
        self.conveyor_id = None
        self.objects = []
        self.gripper_constraint = None
        
        # Conveyor parameters (NEXT TO the robot, not above it!)
        self.conveyor_speed = 0.08  # m/s
        self.conveyor_length = 1.5
        self.conveyor_width = 0.4
        self.conveyor_position = [0.5, 0.5, 0.05]  # Beside the robot
        
        # Robot position
        self.robot_base_position = [0, 0, 0]
        
        # Statistics
        self.stats = {
            'spawned': 0,
            'detected': 0,
            'picked': 0,
            'dropped': 0,
            'failed': 0
        }
        
        # Set up the scene
        self._setup_scene()
        
        print("✅ PyBullet conveyor system ready!")
        print(f"   Robot: Franka Panda @ {self.robot_base_position}")
        print(f"   Conveyor: @ {self.conveyor_position}")
    
    # ==================== SCENE SETUP ====================
    
    def _setup_scene(self):
        """Set up the scene: ground, robot, and conveyor belt."""
        
        print("\n🏗️  Setting up the scene...")
        
        # 1. Ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # 2. Load the robot (Franka Panda, including gripper)
        self._load_franka_panda()
        
        # 3. Create the conveyor belt (BESIDE the robot)
        self._create_conveyor_belt()
        
        # 4. Drop box (drop zone)
        self._create_drop_zone()
        
        # 5. Set the camera
        p.resetDebugVisualizerCamera(
            cameraDistance=2.0,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=[0.3, 0.3, 0.3]
        )
        
        print("✅ Scene setup complete!")
    
    def _load_franka_panda(self):
        """Load the Franka Panda robot (including gripper)."""
        
        print("   → Loading Franka Panda...")
        
        try:
            # Franka Panda robot
            self.robot_id = p.loadURDF(
                "franka_panda/panda.urdf",
                basePosition=self.robot_base_position,
                baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION
            )
            
            # Get joint information
            self.num_joints = p.getNumJoints(self.robot_id)
            
            # Franka Panda joint layout:
            # 0-6: Arm joints (7-DOF)
            # 7-8: Gripper joints (finger joints)
            self.arm_joints = list(range(7))  # 7-DOF arm
            self.gripper_joints = [9, 10]     # Gripper fingers
            
            # End-effector link (panda_hand)
            self.end_effector_link = 11  # panda_hand link
            
            print(f"   ✅ Franka Panda loaded (ID: {self.robot_id})")
            print(f"      Total joints: {self.num_joints}")
            print(f"      Arm joints: {self.arm_joints}")
            print(f"      Gripper joints: {self.gripper_joints}")
            
            # Get joint limits
            self._get_joint_limits()
            
            # Move to the home position
            home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
            self.move_to_joint_positions(home_position)
            
            # Open the gripper initially
            self.gripper_open()
            
        except Exception as e:
            print(f"❌ Failed to load Franka Panda: {e}")
            print("   Please make sure PyBullet is up to date!")
            raise
    
    def _create_conveyor_belt(self):
        """Create the conveyor belt (beside the robot!)."""
        
        print("   → Creating the conveyor belt...")
        
        # Visual shape
        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05],
            rgbaColor=[0.3, 0.3, 0.3, 1]
        )
        
        # Collision shape
        collision_shape = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.conveyor_length/2, self.conveyor_width/2, 0.05]
        )
        
        # Create the conveyor belt
        self.conveyor_id = p.createMultiBody(
            baseMass=0,  # Sabit
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=self.conveyor_position
        )
        
        print(f"   ✅ Conveyor belt created @ {self.conveyor_position}")
        
        # Direction indicator (arrow) - optional
        arrow_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.05, 0.02, 0.001],
            rgbaColor=[1, 1, 0, 1]
        )
        
        arrow_pos = [
            self.conveyor_position[0] - self.conveyor_length/2 + 0.1,
            self.conveyor_position[1],
            self.conveyor_position[2] + 0.11
        ]
        
        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=arrow_visual,
            basePosition=arrow_pos
        )
    
    def _create_drop_zone(self):
        """Create the drop box."""
        
        print("   → Creating the drop box...")
        
        # Box visual
        box_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[0.15, 0.15, 0.1],
            rgbaColor=[0.2, 0.8, 0.2, 0.3]
        )
        
        # Box position (the other side of the robot)
        drop_position = [0, -0.5, 0.1]
        
        self.drop_zone_id = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=box_visual,
            basePosition=drop_position
        )
        
        self.drop_position = [drop_position[0], drop_position[1], drop_position[2] + 0.15]
        
        print(f"   ✅ Drop box @ {drop_position}")
    
    def _get_joint_limits(self):
        """Get joint limits."""
        self.joint_limits = []
        
        for joint_idx in self.arm_joints:
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            self.joint_limits.append((lower_limit, upper_limit))
    
    # ==================== ROBOT CONTROL ====================
    
    def move_to_joint_positions(self, target_positions, max_velocity=0.5):
        """
        Move to joint positions (smooth motion).
        
        Args:
            target_positions: Target joint angles (radians)
            max_velocity: Maximum velocity
        """
        
        # Control the position of each joint
        for i, joint_idx in enumerate(self.arm_joints):
            if i < len(target_positions):
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot_id,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_positions[i],
                    maxVelocity=max_velocity,
                    force=200
                )
        
        # Run the simulation to complete the movement
        # Wait for the movement to finish
        for _ in range(480):  # ~1 saniye
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Check whether the target has been approached
            current_positions = [p.getJointState(self.robot_id, j)[0] for j in self.arm_joints[:len(target_positions)]]
            diff = np.abs(np.array(current_positions) - np.array(target_positions))
            if np.all(diff < 0.01):  # Close enough
                break
    
    def get_end_effector_pose(self):
        """
        Get the end-effector position and orientation.
        
        Returns:
            (position, orientation) tuple
        """
        
        link_state = p.getLinkState(
            self.robot_id,
            self.end_effector_link,
            computeForwardKinematics=True
        )
        
        position = link_state[4]  # World position
        orientation = link_state[5]  # World orientation (quaternion)
        
        return position, orientation
    
    def inverse_kinematics(self, target_position, target_orientation=None):
        """
        Solve inverse kinematics.
        
        Args:
            target_position: [x, y, z]
            target_orientation: Quaternion (optional)
        
        Returns:
            Joint angles
        """
        
        if target_orientation is None:
            # Default orientation (ftargetng downward)
            target_orientation = p.getQuaternionFromEuler([math.pi, 0, 0])
        
        joint_poses = p.calculateInverseKinematics(
            bodyUniqueId=self.robot_id,
            endEffectorLinkIndex=self.end_effector_link,
            targetPosition=target_position,
            targetOrientation=target_orientation,
            maxNumIterations=100,
            residualThreshold=0.001
        )
        
        return joint_poses[:len(self.arm_joints)]
    
    def move_to_position(self, target_position, target_orientation=None, velocity=0.5):
        """
        Move to a Cartesian position (using IK).
        
        Args:
            target_position: [x, y, z]
            target_orientation: Quaternion (optional)
            velocity: Movement speed
        """
        
        # Calculate IK
        joint_positions = self.inverse_kinematics(target_position, target_orientation)
        
        # Move to the joint positions
        self.move_to_joint_positions(joint_positions, max_velocity=velocity)
    
    # ==================== GRIPPER CONTROL ====================
    
    def gripper_open(self, width=0.04):
        """
        Open the gripper.
        
        Args:
            width: Opening width (meters)
        """
        
        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=width,
                force=20
            )
        
        # Wait for the gripper to open
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def gripper_close(self, force=40):
        """
        Close the gripper.
        
        Args:
            force: Closing force
        """
        
        for joint_idx in self.gripper_joints:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=force
            )
        
        # Wait for the gripper to close
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)
    
    def check_gripper_grasp(self):
        """
        Check whether there is an object in the gripper.
        
        Returns:
            bool: Whether an object was grasped
        """
        
        # Get the current positions of the gripper joints
        finger_states = [p.getJointState(self.robot_id, j)[0] for j in self.gripper_joints]
        
        # If the gripper has not fully closed, it is holding something
        avg_position = np.mean(finger_states)
        
        # Closing tolerance
        return avg_position > 0.005  # An opening wider than 5 mm indicates an object is present
    
    # ==================== OBJECT MANAGEMENT ====================
    
    def spawn_object(self, position=None, size=0.03):
        """
        Create an object on the belt.
        
        Args:
            position: [x, y, z] - automatically assigned if None
            size: Object size
        
        Returns:
            object_id
        """
        
        if position is None:
            # Random position at the start of the conveyor
            conv_start_x = self.conveyor_position[0] - self.conveyor_length/2 + 0.15
            conv_y = self.conveyor_position[1] + np.random.uniform(-0.12, 0.12)
            conv_z = self.conveyor_position[2] + 0.15
            position = [conv_start_x, conv_y, conv_z]
        
        # Choose a random shape
        shape_choice = np.random.choice(['box', 'cylinder', 'sphere'])
        
        if shape_choice == 'box':
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, size],
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[size, size, size]
            )
        elif shape_choice == 'cylinder':
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size,
                length=size*2,
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_CYLINDER,
                radius=size,
                height=size*2
            )
        else:  # sphere
            visual_shape = p.createVisualShape(
                shapeType=p.GEOM_SPHERE,
                radius=size,
                rgbaColor=[np.random.rand(), np.random.rand(), np.random.rand(), 1]
            )
            collision_shape = p.createCollisionShape(
                shapeType=p.GEOM_SPHERE,
                radius=size
            )
        
        # Create the object
        obj_id = p.createMultiBody(
            baseMass=0.05,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=position
        )
        
        self.objects.append({
            'id': obj_id,
            'spawn_time': time.time(),
            'picked': False,
            'shape': shape_choice
        })
        
        self.stats['spawned'] += 1
        
        print(f"📦 {shape_choice.upper()} created (ID: {obj_id}) @ [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
        
        return obj_id
    
    def update_conveyor(self, dt=1./240.):
        """
        Simulate conveyor belt movement.
        Move objects on the belt.
        
        Args:
            dt: Time step (seconds)
        """
        
        # Find and move all objects on the belt
        for obj_data in self.objects[:]:  # Iterate over a copy (safe for removal)
            obj_id = obj_data['id']
            
            # Check whether the object still exists
            try:
                position, orientation = p.getBasePositionAndOrientation(obj_id)
                
                # Move forward along the X axis (belt direction)
                new_x = position[0] + self.conveyor_speed * dt
                
                # Remove the object if it has reached the end of the belt
                conv_end_x = self.conveyor_position[0] + self.conveyor_width/2
                if new_x > conv_end_x + 0.2:
                    p.removeBody(obj_id)
                    self.objects.remove(obj_data)
                    continue
                
                # New position
                new_position = [new_x, position[1], position[2]]
                
                # Update the position (only for objects that have not been picked)
                if not obj_data['picked']:
                    p.resetBasePositionAndOrientation(obj_id, new_position, orientation)
                
            except:
                # Remove the object from the list if it has been deleted
                if obj_data in self.objects:
                    self.objects.remove(obj_data)
    
    def detect_objects_in_zone(self, zone_x_min=None, zone_x_max=None):
        """
        Detect objects in a specific area (along the X axis).

        Args:
            zone_x_min: Minimum X coordinate of the area
            zone_x_max: Maximum X coordinate of the area
        
        Returns:
            List: Detected objects
        """
        
        if zone_x_min is None:
            # Area reachable by the robot (along the X axis)
            zone_x_min = self.conveyor_position[0] - 0.15 # Adjust these values to match the robot's reach
        
        if zone_x_max is None:
            zone_x_max = self.conveyor_position[0] + 0.15 # Adjust these values to match the robot's reach
        
        detected = []
        
        for obj_data in self.objects:
            if obj_data['picked']:
                continue
            
            obj_id = obj_data['id']
            
            try:
                position, _ = p.getBasePositionAndOrientation(obj_id)
                
                # Check the area (along the X axis)
                if zone_x_min <= position[0] <= zone_x_max:
                    # Is it on the conveyor along the Y axis?
                    conv_y_min = self.conveyor_position[1] - self.conveyor_width/2
                    conv_y_max = self.conveyor_position[1] + self.conveyor_width/2
                    
                    if conv_y_min <= position[1] <= conv_y_max:
                        detected.append({
                            'id': obj_id,
                            'position': position,
                            'data': obj_data
                        })
                        self.stats['detected'] += 1
            except:
                pass
        
        return detected
    
    # ==================== PICK & PLACE ====================
    
    def pick_object(self, target_position, approach_height=0.35):
        """
        Grasp an object.
        
        Args:
            target_position: Target position [x, y, z]
            approach_height: Approach height
        
        Returns:
            bool: Whether the operation succeeded
        """
        
        print(f"\n🎯 Starting grasp: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
        
        # 1. Open the gripper
        self.gripper_open()
        print("   → Gripper opened")
        
        # 2. Move to the approach position (above the object)
        approach_pos = list(target_position)
        approach_pos[2] = approach_height
        
        print("   → Moving to the approach position...")
        self.move_to_position(approach_pos, velocity=0.8)
        
        # 3. Move down toward the object
        print("   → Moving down toward the object...")
        grasp_pos = list(target_position)
        grasp_pos[2] = target_position[2]  # Grasp slightly from above
        
        self.move_to_position(grasp_pos, velocity=0.3)
        
        # 4. Close the gripper
        print("   → Closing the gripper...")
        self.gripper_close()
        
        # 5. Check the grasp
        has_object = self.check_gripper_grasp()
        
        if has_object:
            print("   ✅ Object grasped!")
            self.stats['picked'] += 1
            
            # 6. Lift upward
            lift_pos = list(grasp_pos)
            lift_pos[2] = approach_height
            self.move_to_position(lift_pos, velocity=0.5)
            
            return True
        else:
            print("   ❌ Failed to grasp the object!")
            self.stats['failed'] += 1
            
            # Open the gripper and retreat
            self.gripper_open()
            self.move_to_position(approach_pos, velocity=0.5)
            
            return False
    
    def place_object(self):
        """
        Place the object in the drop box.
        """
        
        print(f"\n📦 Pltargetng object @ [{self.drop_position[0]:.3f}, {self.drop_position[1]:.3f}, {self.drop_position[2]:.3f}]")
        
        # 1. Move to the drop position
        self.move_to_position(self.drop_position, velocity=0.8)
        
        # 2. Open the gripper
        self.gripper_open()
        
        self.stats['dropped'] += 1
        print("   ✅ Object placed!")
        
        time.sleep(0.3)
    
    def go_home(self):
        """Return to the home position."""
        print("\n🏠 Returning to the home position...")
        home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        self.move_to_joint_positions(home_position, max_velocity=0.8)
    
    # ==================== MAIN LOOP ====================
    
    def run_single_cycle(self):
        """Run one pick-and-place cycle."""
        
        print("\n" + "="*60)
        print("🔄 SINGLE CYCLE")
        print("="*60)
        
        # Move to the observation position
        observe_position = [0, -0.3, 0, -1.5, 0, 1.2, 0.785]
        self.move_to_joint_positions(observe_position)
        
        # Wait briefly for an object to approach
        print("⏳ Waiting for an object...")
        for _ in range(60):
            self.update_conveyor()
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Detect objects
        detected = self.detect_objects_in_zone()
        
        if not detected:
            print("❌ No object in the grasping area!")
            return False
        
        # Select the nearest object
        target = detected[0]
        print(f"🎯 Target: ID={target['id']} @ [{target['position'][0]:.3f}, {target['position'][1]:.3f}, {target['position'][2]:.3f}]")
        
        # Grasp the object
        success = self.pick_object(target['position'])
        
        if success:
            # Mark the object as picked
            target['data']['picked'] = True
            
            # Place the object
            self.place_object()
            
            # Return home
            self.go_home()
            
            return True
        else:
            # Return home
            self.go_home()
            return False
    
    def run_continuous(self, duration=60, spawn_interval=5):
        """
        Run a continuous pick-and-place loop.
        
        Args:
            duration: Total duration (seconds)
            spawn_interval: Object creation interval (seconds)
        """
        
        print("\n" + "="*70)
        print("🤖 FRANKA PANDA CONVEYOR PICK-AND-PLACE SYSTEM")
        print("="*70)
        print(f"Duration: {duration} seconds")
        print(f"Object creation interval: {spawn_interval} seconds")
        print(f"Belt position: {self.conveyor_position}")
        print(f"Robot position: {self.robot_base_position}")
        print()
        
        start_time = time.time()
        last_spawn_time = start_time
        last_pick_time = start_time
        
        # Move to the observation position
        observe_position = [0, -0.3, 0, -1.5, 0, 1.2, 0.785]
        self.move_to_joint_positions(observe_position)
        
        # Main loop
        step_count = 0
        while time.time() - start_time < duration:
            
            # Update the conveyor
            self.update_conveyor()
            
            # Create a new object
            if time.time() - last_spawn_time > spawn_interval:
                self.spawn_object()
                last_spawn_time = time.time()
            
            # Perform pick-and-place (every 10 seconds)
            if time.time() - last_pick_time > 10:
                detected = self.detect_objects_in_zone()
                
                if detected:
                    target = detected[0]
                    print(f"\n🎯 Target selected: ID={target['id']}")
                    
                    success = self.pick_object(target['position'])
                    
                    if success:
                        target['data']['picked'] = True
                        self.place_object()
                    
                    self.go_home()
                    
                    # Return to the observation position
                    self.move_to_joint_positions(observe_position)
                
                last_pick_time = time.time()
            
            # Advance the simulation
            p.stepSimulation()
            time.sleep(1./240.)
            step_count += 1
            
            # Print a status message every 240 steps (1 second)
            if step_count % 240 == 0:
                elapsed = time.time() - start_time
                remaining = duration - elapsed
                print(f"⏱️  Elapsed: {elapsed:.1f}s | Remaining: {remaining:.1f}s | Total objects: {len(self.objects)}")
        
        # Show statistics
        self.print_statistics()
        
        print("\n" + "="*70)
        print("✅ SIMULATION COMPLETE!")
        print("="*70)
    
    def print_statistics(self):
        """Print statistics."""
        
        print("\n" + "="*60)
        print("📊 STATISTICS")
        print("="*60)
        print(f"   Objects created: {self.stats['spawned']}")
        print(f"   Objects detected: {self.stats['detected']}")
        print(f"   Objects grasped: {self.stats['picked']}")
        print(f"   Objects placed: {self.stats['dropped']}")
        print(f"   Failed attempts: {self.stats['failed']}")
        
        if self.stats['picked'] + self.stats['failed'] > 0:
            success_rate = self.stats['picked'] / (self.stats['picked'] + self.stats['failed']) * 100
            print(f"   Success rate: {success_rate:.1f}%")
        print("="*60)
    
    def cleanup(self):
        """Close the simulation."""
        p.disconnect()
        print("\n✅ PyBullet connection closed")


# ==================== DEMO AND TEST ====================

def demo_basic():
    """Basic demo - manual test."""
    
    print("\n🎮 BASIC DEMO - Manual object grasping\n")
    
    # Start the system
    system = PyBulletConveyorSystem(gui=True)
    
    print("\n📦 Creating 3 objects...")
    
    # Create several objects at different positions on the conveyor
    for i in range(3):
        x_pos = system.conveyor_position[0] - system.conveyor_length/2 + 0.3 + i * 0.3
        y_pos = system.conveyor_position[1]
        z_pos = system.conveyor_position[2] + 0.15
        system.spawn_object([x_pos, y_pos, z_pos])
    
    print("\n⏳ Waiting for the objects to settle...")
    for _ in range(120):
        p.stepSimulation()
        time.sleep(1./240.)
    
    # Grasp each object in sequence
    for cycle in range(3):
        print(f"\n{'='*60}")
        print(f"CYCLE {cycle + 1}/3")
        print(f"{'='*60}")
        
        success = system.run_single_cycle()
        
        if not success:
            print("⚠️  Skipping this cycle and continuing...")
        
        time.sleep(1)
    
    # Statistics
    system.print_statistics()
    
    print("\n✅ Demo complete!")
    print("Press ENTER to close the window...")
    input()
    
    system.cleanup()


def demo_continuous():
    """Continuous loop demo - automated system."""
    
    print("\n🎮 CONTINUOUS LOOP DEMO - Automated conveyor system\n")
    
    # Start the system
    system = PyBulletConveyorSystem(gui=True)
    
    try:
        # Run for 60 seconds
        system.run_continuous(duration=60, spawn_interval=6)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Stopped by user (Ctrl+C)")
        system.print_statistics()
    
    finally:
        print("\nPress ENTER to close the window...")
        input()
        system.cleanup()


def demo_interactive():
    """Interactive demo - user control."""
    
    print("\n🎮 INTERACTIVE DEMO\n")
    
    system = PyBulletConveyorSystem(gui=True)
    
    print("\nKomutlar:")
    print("  s - Create an object (spawn)")
    print("  p - Run one pick-and-place cycle")
    print("  h - Move to the home position")
    print("  o - Open the gripper")
    print("  c - Close the gripper")
    print("  q - Quit")
    print()
    
    try:
        while True:
            cmd = input("Enter a command: ").lower().strip()
            
            if cmd == 's':
                system.spawn_object()
            
            elif cmd == 'p':
                # Run the conveyor briefly
                print("⏳ Conveyor running...")
                for _ in range(120):
                    system.update_conveyor()
                    p.stepSimulation()
                    time.sleep(1./240.)
                
                system.run_single_cycle()
            
            elif cmd == 'h':
                system.go_home()
            
            elif cmd == 'o':
                system.gripper_open()
            
            elif cmd == 'c':
                system.gripper_close()
            
            elif cmd == 'q':
                break
            
            else:
                print("❌ Invalid command!")
            
            # Advance the simulation briefly
            for _ in range(10):
                system.update_conveyor()
                p.stepSimulation()
                time.sleep(1./240.)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Stopped by user")
    
    finally:
        system.print_statistics()
        system.cleanup()


def main():
    """Main program."""
    
    print("\n" + "="*70)
    print("🤖 FRANKA PANDA CONVEYOR PICK-AND-PLACE SYSTEM")
    print("="*70)
    print("\nIMPROVEMENTS:")
    print("  ✓ Franka Panda robot (real gripper!)")
    print("  ✓ Conveyor belt beside the robot (no collision!)")
    print("  ✓ Better IK and motion control")
    print("  ✓ Different shapes (box, cylinder, sphere)")
    print("  ✓ Improved gripper control")
    print()
    print("DEMO OPTIONS:")
    print("1. Basic demo (manual test with 3 objects)")
    print("2. Continuous loop (60 seconds, automated)")
    print("3. Interactive mode (manual control)")
    print("4. Exit")
    print("="*70)
    
    choice = input("\nYour choice (1-4): ")
    
    if choice == '1':
        demo_basic()
    elif choice == '2':
        demo_continuous()
    elif choice == '3':
        demo_interactive()
    elif choice == '4':
        print("👋 Goodbye!")
    else:
        print("❌ Invalid choice!")


if __name__ == '__main__':
    main()