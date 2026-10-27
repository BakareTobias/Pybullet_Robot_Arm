"""
PROJECT 1: Smart Color Recognition and Classification System
The robot will sort randomly colored cubes by color.
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import random

print("="*70)
print("PROJECT 1: SMART COLOR RECOGNITION AND CLASSIFICATION SYSTEM")
print("="*70)

# =====================================================
# SETUP
# =====================================================
print("\n→ Preparing the simulation environment...")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Work table (wide)
masa_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15])
masa_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15],
                                   rgbaColor=[0.6, 0.4, 0.2, 1])
masa = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=masa_collision,
                         baseVisualShapeIndex=masa_visual,
                         basePosition=[0.6, 0, 0.15])

# Target bins (by color)
kutu_yukseklik = 0.08
kutu_genislik = 0.15

# Red bin (left)
kirmizi_kutu_collision = p.createCollisionShape(p.GEOM_BOX,
                                                 halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2])
kirmizi_kutu_visual = p.createVisualShape(p.GEOM_BOX,
                                           halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2],
                                           rgbaColor=[1, 0, 0, 0.3])
kirmizi_kutu = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=kirmizi_kutu_collision,
                                  baseVisualShapeIndex=kirmizi_kutu_visual,
                                  basePosition=[0.3, 0.25, 0.30 + kutu_yukseklik/2])

# Blue bin (center)
mavi_kutu_collision = p.createCollisionShape(p.GEOM_BOX,
                                              halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2])
mavi_kutu_visual = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2],
                                        rgbaColor=[0, 0, 1, 0.3])
mavi_kutu = p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=mavi_kutu_collision,
                               baseVisualShapeIndex=mavi_kutu_visual,
                               basePosition=[0.3, 0.0, 0.30 + kutu_yukseklik/2])

# Green bin (right)
yesil_kutu_collision = p.createCollisionShape(p.GEOM_BOX,
                                               halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2])
yesil_kutu_visual = p.createVisualShape(p.GEOM_BOX,
                                         halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2],
                                         rgbaColor=[0, 1, 0, 0.3])
yesil_kutu = p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=yesil_kutu_collision,
                                baseVisualShapeIndex=yesil_kutu_visual,
                                basePosition=[0.3, -0.25, 0.30 + kutu_yukseklik/2])

# Panda robot
panda = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)

# Camera
p.resetDebugVisualizerCamera(
    cameraDistance=2.2,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0.4]
)

print("✓ Environment ready")
print("  - Work table ✓")
print("  - Red bin (left) ✓")
print("  - Blue bin (center) ✓")
print("  - Green bin (right) ✓")
print("  - Panda robot ✓")

# =====================================================
# ROBOT CONTROL
# =====================================================

kol_eklemleri = [0, 1, 2, 3, 4, 5, 6]
gripper_eklemleri = [9, 10]
end_effector_index = 11

# Home position
home_angles = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i, angle in enumerate(home_angles):
    p.resetJointState(panda, kol_eklemleri[i], angle)

for eklem in gripper_eklemleri:
    p.resetJointState(panda, eklem, 0.04)

def kol_hareket_ettir(hedef_pos, sure=2.0):
    """Move the robot arm to the target position"""
    hedef_orn = p.getQuaternionFromEuler([math.pi, 0, 0])

    ik_result = p.calculateInverseKinematics(
        panda, end_effector_index, hedef_pos, hedef_orn,
        maxNumIterations=100, residualThreshold=0.001
    )

    hedef_acilari = [ik_result[i] for i in kol_eklemleri]

    adim_sayisi = int(sure * 240)
    for _ in range(adim_sayisi):
        for i, eklem_idx in enumerate(kol_eklemleri):
            p.setJointMotorControl2(panda, eklem_idx, p.POSITION_CONTROL,
                                   targetPosition=hedef_acilari[i],
                                   force=87, maxVelocity=1.0)
        p.stepSimulation()
        time.sleep(1./240.)

def gripper_kontrol(komut, kuvvet=80):
    """Open/close the gripper"""
    hedef = 0.04 if komut == "open" else 0.0
    for eklem in gripper_eklemleri:
        p.setJointMotorControl2(panda, eklem, p.POSITION_CONTROL,
                               targetPosition=hedef, force=kuvvet)
    for _ in range(80):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# COLOR RECOGNITION SYSTEM
# =====================================================

# Color definitions (RGB values)
RENKLER = {
    "red": {"rgb": [1.0, 0.0, 0.0], "kutu_pos": [0.3, 0.25, 0.38]},
    "blue":    {"rgb": [0.0, 0.0, 1.0], "kutu_pos": [0.3, 0.0, 0.38]},
    "green":   {"rgb": [0.0, 1.0, 0.0], "kutu_pos": [0.3, -0.25, 0.38]}
}
renk_gosterim = {"red": "red", "blue": "blue", "green": "green"}

def rgb_mesafesi(renk1, renk2):
    """Euclidean distance between two RGB colors"""
    return math.sqrt(sum([(renk1[i] - renk2[i])**2 for i in range(3)]))

def renk_tani(nesne_id):
    """
    Identify the object's color

    Returns:
        color name: "red", "blue", "green" (internal color keys)
    """
    # Get the object's visual information
    visual_data = p.getVisualShapeData(nesne_id)

    if len(visual_data) == 0:
        return None

    # Get the RGBA color
    rgba = visual_data[0][7]  # [r, g, b, a]
    nesne_rgb = rgba[:3]

    # Find the closest color
    en_yakin_renk = None
    en_kucuk_mesafe = float('inf')

    for renk_adi, renk_bilgi in RENKLER.items():
        mesafe = rgb_mesafesi(nesne_rgb, renk_bilgi["rgb"])

        if mesafe < en_kucuk_mesafe:
            en_kucuk_mesafe = mesafe
            en_yakin_renk = renk_adi

    return en_yakin_renk

# =====================================================
# CREATE OBJECTS
# =====================================================

def kup_olustur(pozisyon, renk_adi):
    """Create a colored cube"""
    renk_rgb = RENKLER[renk_adi]["rgb"]

    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02])
    visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02],
                                  rgbaColor=[renk_rgb[0], renk_rgb[1], renk_rgb[2], 1])

    nesne = p.createMultiBody(baseMass=0.05,
                             baseCollisionShapeIndex=collision,
                             baseVisualShapeIndex=visual,
                             basePosition=pozisyon)
    return nesne

# =====================================================
# CREATE RANDOM CUBES
# =====================================================
print("\n" + "="*70)
print("CREATING CUBES")
print("="*70)

num_cubes = 8  # Total number of cubes
nesneler = []
nesne_renkleri = []

print(f"\n→ Creating {num_cubes} random cubes...")

renk_isimleri = list(RENKLER.keys())

for i in range(num_cubes):
    # Random position (right side of the table)
    x = random.uniform(0.5, 0.7)
    y = random.uniform(-0.25, 0.25)
    z = 0.35

    # Choose a random color
    renk = random.choice(renk_isimleri)

    # Create the cube
    kup = kup_olustur([x, y, z], renk)
    nesneler.append(kup)
    nesne_renkleri.append(renk)

    print(f"  {i+1}. Cube: {renk_gosterim[renk]:8s} → Position: ({x:.2f}, {y:.2f}, {z:.2f})")

# Wait for the cubes to fall
print("\n→ Cubes are falling...")
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

print("✓ Cubes are ready")

# Show the color distribution
kirmizi_sayisi = nesne_renkleri.count("red")
mavi_sayisi = nesne_renkleri.count("blue")
yesil_sayisi = nesne_renkleri.count("green")

print(f"\nColor Distribution:")
print(f"  🔴 Red:   {kirmizi_sayisi}")
print(f"  🔵 Blue:  {mavi_sayisi}")
print(f"  🟢 Green: {yesil_sayisi}")

input("\nPress ENTER to start...\n")

# =====================================================
# PICK-AND-PLACE FUNCTION
# =====================================================

def pick_and_place_hizli(nesne_id, hedef_pos):
    """Accelerated pick-and-place"""
    nesne_pos, _ = p.getBasePositionAndOrientation(nesne_id)

    # Pick
    gripper_kontrol("open")
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2] + 0.15], sure=1.5)
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2]], sure=1.2)
    gripper_kontrol("close", kuvvet=100)
    time.sleep(0.2)
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2] + 0.2], sure=1.2)

    # Place
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2] + 0.15], sure=1.5)
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2]], sure=1.0)
    gripper_kontrol("open")
    time.sleep(0.2)
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2] + 0.15], sure=1.0)

# =====================================================
# MAIN PROCESS - CLASSIFICATION
# =====================================================
print("="*70)
print("CLASSIFICATION STARTING")
print("="*70)

basari_sayisi = 0
basarisiz_sayisi = 0
baslangic_zamani = time.time()

# Counter for each color (number of cubes placed in each bin)
kutu_sayaclari = {"red": 0, "blue": 0, "green": 0}

for idx, nesne in enumerate(nesneler, 1):
    print(f"\n{'='*70}")
    print(f"CUBE {idx}/{num_cubes}")
    print(f"{'='*70}")

    # 1. Color recognition
    print("  [1/4] Recognizing color...")
    taninan_renk = renk_tani(nesne)
    gercek_renk = nesne_renkleri[idx-1]
    renk_gosterim = {"red": "RED", "blue": "BLUE", "green": "GREEN"}

    if taninan_renk == gercek_renk:
        print(f"  ✓ Color: {renk_gosterim[taninan_renk]}")
        basari_sayisi += 1
    else:
        print(f"  ✗ Error! Recognized: {renk_gosterim[taninan_renk]}, Actual: {renk_gosterim[gercek_renk]}")
        basarisiz_sayisi += 1
        taninan_renk = gercek_renk  # Correct it

    # 2. Target bin position
    hedef_kutu = RENKLER[taninan_renk]["kutu_pos"]

    # Arrange cubes side by side if the bin already contains cubes
    offset = kutu_sayaclari[taninan_renk] * 0.045
    hedef_final = [hedef_kutu[0], hedef_kutu[1] + offset - 0.045, hedef_kutu[2]]

    print(f"  [2/4] Target: {renk_gosterim[taninan_renk]} bin")

    # 3. Pick and place
    print(f"  [3/4] Picking up cube...")
    nesne_pos, _ = p.getBasePositionAndOrientation(nesne)
    print(f"        Position: ({nesne_pos[0]:.2f}, {nesne_pos[1]:.2f}, {nesne_pos[2]:.2f})")

    try:
        pick_and_place_hizli(nesne, hedef_final)
        print(f"  [4/4] ✓ Cube placed in the {renk_gosterim[taninan_renk]} bin!")
        kutu_sayaclari[taninan_renk] += 1
    except:
        print(f"  [4/4] ✗ Transfer error!")
        basarisiz_sayisi += 1

    # Progress bar
    tamamlanan = idx
    kalan = num_cubes - idx
    print(f"\n  Progress: {'█' * tamamlanan}{'░' * kalan} {idx}/{num_cubes}")

bitis_zamani = time.time()
toplam_sure = bitis_zamani - baslangic_zamani

# =====================================================
# RESULTS REPORT
# =====================================================
print("\n" + "="*70)
print("PROJECT COMPLETE - RESULTS REPORT")
print("="*70)

print(f"\n📊 PERFORMANCE METRICS:")
print(f"  ⏱  Total time: {toplam_sure:.1f} seconds")
print(f"  📦 Cubes processed: {num_cubes}")
print(f"  ✓ Successful: {basari_sayisi}")
print(f"  ✗ Failed: {basarisiz_sayisi}")
print(f"  📈 Success rate: {(basari_sayisi/num_cubes)*100:.1f}%")
print(f"  ⚡ Average processing time: {toplam_sure/num_cubes:.1f} seconds/cube")

print(f"\n📦 BIN STATUS:")
print(f"  🔴 Red bin:   {kutu_sayaclari['red']} cubes")
print(f"  🔵 Blue bin:  {kutu_sayaclari['blue']} cubes")
print(f"  🟢 Green bin: {kutu_sayaclari['green']} cubes")

# Evaluation
if basari_sayisi == num_cubes:
    print(f"\n🎉 EXCELLENT! All cubes were classified correctly!")
elif basari_sayisi >= num_cubes * 0.8:
    print(f"\n👍 VERY GOOD! Success rate: {(basari_sayisi/num_cubes)*100:.0f}%!")
else:
    print(f"\n📝 Needs improvement. Some cubes were classified incorrectly.")

print("\n" + "="*70)
print("PROJECT 1 COMPLETE")
print("="*70)

print("""
WHAT WE LEARNED:
  ✓ Color recognition algorithm (RGB distance)
  ✓ Automatic object classification
  ✓ Performance measurements
  ✓ Automation system design

REAL-WORLD APPLICATIONS:
  - Warehouse management (product sorting)
  - Recycling facilities (material separation)
  - Quality control (color-based inspection)
  - Pharmacy automation (medication sorting)

NEXT PROJECT:
  → Project 2: Computer Vision Integration
    (Detecting object positions with a camera)
""")

input("\nPress ENTER to close the simulation...\n")

p.disconnect()
print("✓ Simulation closed")
