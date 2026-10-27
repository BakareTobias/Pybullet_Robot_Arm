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
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def eklem_durumunu_oku(eklem_no):
    durum = p.getJointState(robot, eklem_no)
    return durum[0], durum[1]  # angle, velocity

def simule_et(adim_sayisi):
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# NEW FUNCTION: Read the End-Effector Position
# =====================================================
def el_pozisyonunu_oku():
    """
    Read the actual position of the robot hand (end-effector).

    Returns:
        (x, y, z): 3D position (in meters)
    """
    # getLinkState: Read the state of a link (part)
    # Link 6 = the Kuka robot's hand (last link)
    durum = p.getLinkState(robot, 6)

    # durum[0] = position (x, y, z)
    # durum[1] = orientation (rotation, quaternion)

    pozisyon = durum[0]  # (x, y, z) tuple

    return pozisyon

# =====================================================
# NEW: Visual Marker
# =====================================================
isaretleyici = None

def isaretleyici_ciz(x, y, z, renk=[0, 1, 0, 0.8]):
    """
    Draw a colored sphere at the specified position.

    Parameters:
        x, y, z: Position
        renk: [R, G, B, Alpha] (from 0 to 1)
    """
    global isaretleyici

    # Remove the old marker
    if isaretleyici is not None:
        try:
            p.removeBody(isaretleyici)
        except:
            pass

    # Create a new sphere
    gorsel = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=renk
    )

    isaretleyici = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=gorsel,
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
    eklem_hareket_ettir(i, 0)
simule_et(300)

# Read the hand position
x, y, z = el_pozisyonunu_oku()
print(f"\n✓ Robot hand position:")
print(f"   X = {x:.3f} meters")
print(f"   Y = {y:.3f} meters")
print(f"   Z = {z:.3f} meters")

# Mark with a green sphere
isaretleyici_ciz(x, y, z, renk=[0, 1, 0, 0.8])

input("\nPress ENTER to continue...")

# =====================================================
# TEST 2: Forward Kinematics - Single Joint
# =====================================================
print("\n" + "="*60)
print("TEST 2: Single Joint Movement and Position Change")
print("="*60)

print("\n→ Rotate joint 0 by 45°...")
eklem_hareket_ettir(0, math.radians(45))
simule_et(400)

x, y, z = el_pozisyonunu_oku()
print(f"\n✓ New hand position:")
print(f"   X = {x:.3f} meters")
print(f"   Y = {y:.3f} meters")
print(f"   Z = {z:.3f} meters")

# Mark with a red sphere
isaretleyici_ciz(x, y, z, renk=[1, 0, 0, 0.8])

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
    eklem_hareket_ettir(i, 0)
simule_et(300)

print("→ Joint 0: 30°, Joint 1: 45°")
eklem_hareket_ettir(0, math.radians(30))
eklem_hareket_ettir(1, math.radians(45))
simule_et(400)

x, y, z = el_pozisyonunu_oku()
print(f"\n✓ Hand position:")
print(f"   X = {x:.3f} m")
print(f"   Y = {y:.3f} m")
print(f"   Z = {z:.3f} m")

isaretleyici_ciz(x, y, z, renk=[0, 0, 1, 0.8])

input("\nPress ENTER to continue...")

# =====================================================
# TEST 4: Different Angles = Different Positions
# =====================================================
print("\n" + "="*60)
print("TEST 4: Various Positions")
print("="*60)
print("The robot will move to 5 different positions")
print("Each position will be marked with a different color\n")

pozisyonlar = [
    ([0, 0], [0, 1, 0, 0.7]),           # Green
    ([45, 0], [1, 0, 0, 0.7]),          # Red
    ([0, 45], [0, 0, 1, 0.7]),          # Blue
    ([45, 45], [1, 1, 0, 0.7]),         # Yellow
    ([-30, 60], [1, 0, 1, 0.7])         # Purple
]

for idx, (acilar, renk) in enumerate(pozisyonlar, 1):
    aci0, aci1 = acilar

    print(f"Position {idx}: Joint 0={aci0:3d}°, Joint 1={aci1:3d}°")

    eklem_hareket_ettir(0, math.radians(aci0))
    eklem_hareket_ettir(1, math.radians(aci1))
    simule_et(300)

    x, y, z = el_pozisyonunu_oku()
    print(f"   → Position: ({x:.3f}, {y:.3f}, {z:.3f})")

    # Create a persistent marker (do not remove it)
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=renk)
    p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

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

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:
    # Sinusoidal movement
    aci0 = 0.5 * math.sin(t)
    aci1 = 0.4 * math.cos(t * 1.2)

    eklem_hareket_ettir(0, aci0)
    eklem_hareket_ettir(1, aci1)

    # Read the hand position
    x, y, z = el_pozisyonunu_oku()

    # Update the marker
    isaretleyici_ciz(x, y, z)

    # Print to the screen
    print(f"\rHand Position: X={x:6.3f} Y={y:6.3f} Z={z:6.3f} | Time: {time.time()-baslangic:.1f}s", end="")

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
