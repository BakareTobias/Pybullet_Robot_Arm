# 🤖 PyBullet Robot Simulation Course

<div align="center">

![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=for-the-badge&logo=python&logoColor=white)
![PyBullet](https://img.shields.io/badge/PyBullet-3.x-FF6B35?style=for-the-badge&logo=python&logoColor=white)
![NumPy](https://img.shields.io/badge/NumPy-1.21%2B-013243?style=for-the-badge&logo=numpy&logoColor=white)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-22C55E?style=for-the-badge)

**A comprehensive step-by-step Python course on robot simulation.**  
From physics basics to computer vision — all in one place.

[Getting Started](#-getting-started) · [Lessons](#-lesson-contents) · [Projects](#-projects) · [Installation](#-installation)

</div>

---

## 📖 About

This repository is a structured robotics curriculum built on top of the PyBullet physics engine. Starting from zero prior knowledge, you'll progress through **Forward/Inverse Kinematics**, **Trajectory Planning**, **Collision Avoidance**, and **Computer Vision** integration — step by step.

Each lesson is self-contained: concept explanation → mathematical foundation → PyBullet implementation.

---

## ✨ Features

- 🏗️ **18 lessons** — from physics basics to advanced real-world projects
- 🤖 **Multiple robots** — Kuka IIWA and Franka Panda
- 📐 **Kinematics from scratch** — hand-derived formulas with full explanations
- 🎯 **Pick-and-Place** — real-world manipulation scenarios
- 📷 **Computer Vision** — virtual camera integration with color segmentation
- ⚡ **Optimization techniques** — IK caching, smart trajectory planning (~35% speedup)
- 🛡️ **Collision Avoidance** — proactive obstacle detection and avoidance

---

## 📚 Lesson Contents

### 🟢 Beginner

| File | Topic | Description |
|------|-------|-------------|
| `1_cube_creation.py` | PyBullet Setup | First simulation, physics engine test |
| `2_move_one_joint.py` | Single Joint Control | First movement with `POSITION_CONTROL` |
| `3_move_two_joint.py` | Multi-Joint Control | Coordinated movement, sinusoidal animation |
| `4_read_joint_value.py` | Reading Joint States | Real-time monitoring with `getJointState` |

### 🟡 Intermediate — Kinematics

| File | Topic | Description |
|------|-------|-------------|
| `5_forward_kinematic.py` | Forward Kinematics | End-effector position via `getLinkState` |
| `6_forward_kinematics_math.py` | FK Mathematics | Hand-derived formulas for a 2-link robot, workspace visualization |
| `6.5_kukaiiwa_fk.py` | Kuka FK Application | FK validation using a real robot model |
| `7_inverse_kinematics.py` | Inverse Kinematics | Target tracking with `calculateInverseKinematics` |
| `8_ik_read_joint_angle.py` | IK + State Reading | Real-time IK tracking, circular trajectory |
| `9_joint_position_graph.py` | Data Visualization | Joint position graphs with Matplotlib |

### 🔴 Advanced

| File | Topic | Description |
|------|-------|-------------|
| `10_jocabian_matris.py` | Jacobian Matrix | Hand-derived Jacobian, iterative IK algorithm |
| `11_trajectory_planning.py` | Trajectory Planning | Linear vs Cubic interpolation, waypoint following |
| `12_collisian_avoidance.py` | Collision Avoidance | `getClosestPoints`, safety margins, obstacle avoidance |

### 🤖 Robot Applications

| File | Topic | Description |
|------|-------|-------------|
| `13_pick_object_with_kukaiiwa.py` | Kuka Pick-and-Place | Constraint-based grasping mechanism |
| `14_pick_object_with_frankapanda.py` | Franka Panda | Real gripper control, corrected IK pipeline |
| `18_conveyor_pick_place.py` | Conveyor System | OOP design, continuous pick-place loop |

### 🚀 Projects

| File | Project | Description |
|------|---------|-------------|
| `15_color_recognition_and_classification.py` | Color Sorting | Automatic classification via RGB distance |
| `16_opt_object_classification.py` | Optimized Sorting | IK caching, smart trajectory — **35% faster** |
| `17_computer_vision_integration.py` | Computer Vision | Virtual camera, color segmentation, pixel→world transform |

---

## 🛠️ Installation

### Requirements

```
Python 3.8+
```

### Install Dependencies

```bash
pip install pybullet numpy matplotlib pillow opencv-python
```

### Clone the Repository

```bash
git clone https://github.com/your-username/pybullet-robot-course.git
cd pybullet-robot-course
```

### Run Your First Lesson

```bash
python 1_cube_creation.py
```

---

## 🚀 Getting Started

If you're completely new to robotics, follow the lessons in order:

```
1_cube_creation.py
    ↓
2_move_one_joint.py
    ↓
3_move_two_joint.py
    ...
```

If you're already familiar with kinematics theory, you can jump directly to `11_trajectory_planning.py` or `17_computer_vision_integration.py`.

---

## 🧠 Learning Path

```
Simulation Basics
      ↓
Joint Control
      ↓
Forward Kinematics ──→ Jacobian Matrix
      ↓
Inverse Kinematics
      ↓
Trajectory Planning ──→ Collision Avoidance
      ↓
Pick-and-Place (Kuka) ──→ Pick-and-Place (Franka)
      ↓
Color Classification ──→ Computer Vision
      ↓
Conveyor System (OOP)
```

---

## 🔑 Core Concepts

<details>
<summary><strong>Forward Kinematics</strong></summary>

Joint angles → End-effector position

```python
# 2-link robot (hand-derived)
x = L1*cos(θ1) + L2*cos(θ1 + θ2)
y = L1*sin(θ1) + L2*sin(θ1 + θ2)

# With PyBullet
state = p.getLinkState(robot, end_effector_link)
position = state[0]  # (x, y, z)
```
</details>

<details>
<summary><strong>Inverse Kinematics</strong></summary>

Target position → Joint angles

```python
joint_angles = p.calculateInverseKinematics(
    bodyUniqueId=robot,
    endEffectorLinkIndex=6,
    targetPosition=[x, y, z],
    maxNumIterations=100
)
```
</details>

<details>
<summary><strong>Trajectory Planning</strong></summary>

```python
# Linear interpolation
p(t) = p_start + (p_end - p_start) * t

# Cubic interpolation (smooth start/stop)
s = 3*t**2 - 2*t**3
p(t) = p_start + (p_end - p_start) * s
```
</details>

<details>
<summary><strong>Jacobian-Based IK</strong></summary>

```python
# Compute Jacobian
J = jacobian(theta1, theta2)

# Update joint angles iteratively
delta_theta = np.linalg.pinv(J) @ error
theta += alpha * delta_theta
```
</details>

<details>
<summary><strong>Collision Avoidance</strong></summary>

```python
# Proactive detection (before collision happens)
closest_points = p.getClosestPoints(
    bodyA=robot,
    bodyB=obstacle,
    distance=safety_margin
)

if len(closest_points) > 0:
    min_dist = min([pt[8] for pt in closest_points])
    # Re-plan trajectory if too close
```
</details>

---

## 📊 Project Results

### Color Classification (Project 1)
- 8 randomly placed cubes sorted by color into target bins
- **100% color recognition accuracy** using RGB Euclidean distance
- Optimized version is **~35% faster** via IK caching and nearest-first ordering

### Computer Vision (Project 2)
- 640×480 virtual overhead camera
- Real-time object detection via color segmentation
- Full pixel-to-world coordinate transformation pipeline

### Conveyor System (Project 3)
- OOP-based modular architecture (`PyBulletConveyorSystem` class)
- Continuous spawn → detect → pick → place loop
- Franka Panda with realistic gripper physics

---

## 📂 Project Structure

```
pybullet-robot-course/
├── 1_cube_creation.py                          # Basic simulation
├── 2_move_one_joint.py                         # Single joint control
├── 3_move_two_joint.py                         # Multi-joint control
├── 4_read_joint_value.py                       # State reading
├── 5_forward_kinematic.py                      # FK with PyBullet
├── 6_forward_kinematics_math.py                # FK mathematics
├── 6.5_kukaiiwa_fk.py                         # FK on Kuka robot
├── 7_inverse_kinematics.py                     # Basic IK
├── 8_ik_read_joint_angle.py                    # Advanced IK + state reading
├── 9_joint_position_graph.py                   # Data visualization
├── 10_jocabian_matris.py                       # Jacobian matrix & iterative IK
├── 11_trajectory_planning.py                   # Trajectory planning
├── 12_collisian_avoidance.py                   # Collision avoidance
├── 13_pick_object_with_kukaiiwa.py             # Kuka pick-and-place
├── 14_pick_object_with_frankapanda.py          # Franka Panda pick-and-place
├── 15_color_recognition_and_classification.py  # Color sorting project
├── 16_opt_object_classification.py             # Optimized sorting
├── 17_computer_vision_integration.py           # Computer vision project
├── 18_conveyor_pick_place.py                   # Conveyor belt system
└── pybullet_folder_path.py                     # Utility: list available models
```

---

## 🔧 Tech Stack

| Technology | Usage |
|------------|-------|
| **PyBullet** | Physics simulation, robot control, IK solver |
| **NumPy** | Matrix operations, kinematic calculations |
| **Matplotlib** | Graphs, workspace visualization |
| **OpenCV** | Image processing, object detection |
| **Pillow** | Image manipulation |

---

## 🌱 What's Next

After completing this course, consider exploring:

- **ROS 2** — real robot middleware and ecosystem
- **MoveIt 2** — professional motion planning framework
- **Real Hardware** — UR5, Kuka, or a DIY robot arm
- **Reinforcement Learning** — PyBullet + OpenAI Gym environments
- **Depth Camera Fusion** — multi-view 3D reconstruction

---

## 🤝 Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you'd like to change.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/new-lesson`)
3. Commit your changes (`git commit -m 'Add lesson: Topic X'`)
4. Push to the branch (`git push origin feature/new-lesson`)
5. Open a Pull Request

---

## 📄 License

Distributed under the MIT License. See `LICENSE` for more information.

---

<div align="center">

**Good luck on your robotics journey! 🚀**

Feel free to open an issue if you have any questions.

</div>
