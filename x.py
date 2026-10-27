import pybullet as p 
import pybullet_data
import time 
import math 
import numpy as np 

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.2])
table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.4],
                                   rgbaColor=[0.7, 0.5, 0.3, 1])
table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=table_collision,
                          baseVisualShapeIndex=table_visual,
                          basePosition=[0.5, 0, 0.2])

panda = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

num_joints = p.getNumJoints(panda)

arm_joints = [0, 1, 2, 3, 4, 5, 6]
gripper_joints = [9, 10]

for i in range(num_joints):
    info = p.getJointInfo(panda, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]

    type_str = {0: "REVOLUTE", 1: "PRISMATIC", 4: "FIXED"}.get(joint_type, "?")

    if i in arm_joints:
        print(f"  {i:2d}: {joint_name:30s} {type_str:10s} ← ARM ✓")
    elif i in gripper_joints:
        print(f"  {i:2d}: {joint_name:30s} {type_str:10s} ← GRIPPER ✋")
    else:
        print(f"  {i:2d}: {joint_name:30s} {type_str:10s}")
    
end_effector_index = 11

home_angle = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

for i, angle in enumerate(home_angle):
    p.resetJointState(panda, arm_joints[i], angle)

for joint in gripper_joints:
    p.resetJointState(panda, joint, 0.04)

for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

def move_arm(target_pos, target_orn=None, sure=2.0):
    if target_orn is None: 
        target_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
    
    ik_result = p.calculateInverseKinematics(
        bodyUniqueId=panda,
        endEffectorLinkIndex=end_effector_index,
        targetPosition=target_pos,
        targetOrientation=target_orn,
        maxNumIterations=100,
        residualThreshold=0.001
    )

    target_angle = [ik_result[i] for i in arm_joints]

    step_counter = int(sure * 240)
    for step in range(step_counter):
        for i, joint_idx in enumerate(arm_joints):
            p.setJoinMotorControl2(
                bodyUniqueId=panda,
                jointIndex=joint_idx,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_angle[i],
                force=87,
                maxVelocity=1.0
            )

        p.stepSimulation()
        time.sleep(1./240.)

def gripper_control(command, force=50):
    if command == "open":
        target = 0.04
    elif command == "close":
        target = 0.0
    else:
        return
    
    for joint in gripper_joints:
        p.setJointMotorControl2(
            panda, joint,
            p.POSITION_CONTROL,
            targetPosition=target,
            force=force
        )
    
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)

def end_effector_pos():
    state = p.getLinkState(panda, end_effector_index)
    return state[0]

def create_object(type, position, color, size=0.04):
    if type == "box":
        collision = p.createCollisionShape(p.GEOM_BOX,
                                           halfExtents=[size/2,size/2, size/2])
        visual = p.createVisualShape(p.GEOM_BOX,
                                     halfExtents=[size/2,size/2, size/2],
                                     rgbaColor=color)
    
    elif type == "sphere":
        collision = p.createCollisionShape(p.GEOM_SPHERE, radius=size/2)
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=size/2, rgbaColor=color)
    
    object = p.createMultiBody(
        baseMass=0.05,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=position
    )

    return object

def show_target(pos, color=[0, 1, 0, 0.5]):
    image = p.createVisualShape(p.GEOM_SPHERE, radius=0.3, rgbaColor=color)
    marker = p. createMultiBod(0, baseVisualShapeIndex=image, basePosition=pos)
    return marker

test_pos = end_effector_pos()

show_target(test_pos, [1, 0, 0, 0.7])
move_arm(test_pos, sure=3.0)

real_pos = end_effector_pos()
error  = math.sqrt(sum([(test_pos[i]-real_pos[i])**2 for i in range(3)]))

p.disconnect()
