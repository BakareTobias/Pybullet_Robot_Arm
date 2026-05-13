"""
2-Eklemli Robot - HAZIR ROBOT KULLANIMI
Kuka robot kullanarak forward kinematics öğreneceğiz
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("PyBullet ile Robot Kontrol - HAZIR ROBOT")
print("="*60)

# Simülasyon başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Zemin
p.loadURDF("plane.urdf")
print("✓ Zemin yüklendi")

# Kamerayı ayarla
p.resetDebugVisualizerCamera(
    cameraDistance=2.5,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.5]
)
print("✓ Kamera ayarlandı")

# KUKA IIWA robot yükle (7-DOF endüstriyel robot)
# Biz sadece ilk 2 eklemi kullanacağız
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print(f"✓ Kuka robot yüklendi (ID: {robot})")

# Eklem bilgilerini öğren
num_joints = p.getNumJoints(robot)
print(f"✓ Toplam eklem sayısı: {num_joints}")
print("\nEklem detayları:")

# Sadece hareketli eklemleri bul
movable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]

    # Type 0 = REVOLUTE (döner eklem)
    # Type 4 = FIXED (sabit bağlantı)
    if joint_type == p.JOINT_REVOLUTE:
        movable_joints.append(i)
        print(f"  Eklem {i}: {joint_name} (Hareketli)")

print(f"\n✓ Hareketli eklem sayısı: {len(movable_joints)}")
print("  İlk 2 eklemi kullanacağız (2-DOF için)")

# Kullanacağımız eklemler
joint1_idx = movable_joints[0]
joint2_idx = movable_joints[1]

def set_joint_position(joint_idx, angle, max_force=200):
    """
    Bir eklemi belirtilen açıya götür

    Args:
        joint_idx: Eklem indeksi
        angle: Hedef açı (radyan)
        max_force: Maksimum motor kuvveti
    """
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_idx,
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle,
        force=max_force,
        maxVelocity=1.0,
        positionGain=0.3,  # PD kontrolcü - P kazancı
        velocityGain=1.0    # PD kontrolcü - D kazancı
    )

def get_joint_position(joint_idx):
    """Eklemin gerçek açısını al"""
    state = p.getJointState(robot, joint_idx)
    return state[0]  # Pozisyon (radyan)

def get_end_effector_position():
    """
    End-effector'ın pozisyonunu al
    (Son linkin pozisyonu)
    """
    # Link state: [pos, orn, local_inertial_pos, ...]
    # Link 6 = end-effector (Kuka'da 7 link var, 0-6)
    state = p.getLinkState(robot, 6)
    return state[0]  # (x, y, z) pozisyon

def reset_robot():
    """Robotu başlangıç pozisyonuna getir"""
    print("\n→ Robot sıfırlanıyor (tüm eklemler 0°)...")
    for i in movable_joints:
        set_joint_position(i, 0, max_force=200)

    # Hareketin tamamlanmasını bekle
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)
    print("  ✓ Robot sıfırlandı")

# Marker (yeşil küre) için global değişken
marker_id = None

def visualize_target(x, y, z, color=[0, 1, 0, 0.7]):
    """
    Hedef pozisyonu görselleştir

    Args:
        x, y, z: Hedef pozisyon
        color: RGBA renk [r, g, b, a]
    """
    global marker_id

    # Eski marker'ı sil
    if marker_id is not None:
        try:
            p.removeBody(marker_id)
        except:
            pass

    # Yeni marker oluştur
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=color
    )

    marker_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x, y, z]
    )

print("\n" + "="*60)
print("FORWARD KİNEMATİK TESTLERİ")
print("="*60)

# Robot'u sıfırla
reset_robot()

# TEST 1: Her iki eklem 0°
print("\n[TEST 1] Her iki eklem 0° (Varsayılan duruş)")
target_j1 = 0.0
target_j2 = 0.0

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Hareket ediliyor...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

# Sonuçları göster
real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Hedef: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Gerçek: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector pozisyonu: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2])

input("\n→ Devam için ENTER'a basın...")

# TEST 2: İlk eklem 45°
print("\n[TEST 2] İlk eklem 45°, ikinci eklem 0°")
target_j1 = math.radians(45)  # 45 derece
target_j2 = 0.0

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Hareket ediliyor...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Hedef: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Gerçek: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector pozisyonu: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [1, 0, 0, 0.7])  # Kırmızı

input("\n→ Devam için ENTER'a basın...")

# TEST 3: Her iki eklem 45°
print("\n[TEST 3] Her iki eklem 45°")
target_j1 = math.radians(45)
target_j2 = math.radians(45)

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Hareket ediliyor...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Hedef: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Gerçek: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector pozisyonu: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [0, 0, 1, 0.7])  # Mavi

input("\n→ Devam için ENTER'a basın...")

# TEST 4: Negatif açılar
print("\n[TEST 4] İlk eklem -45°, ikinci eklem 30°")
target_j1 = math.radians(-45)
target_j2 = math.radians(30)

set_joint_position(joint1_idx, target_j1)
set_joint_position(joint2_idx, target_j2)

print("  Hareket ediliyor...", end="", flush=True)
for i in range(300):
    p.stepSimulation()
    time.sleep(1./240.)
    if i % 60 == 0:
        print(".", end="", flush=True)
print(" ✓")

real_j1 = get_joint_position(joint1_idx)
real_j2 = get_joint_position(joint2_idx)
ee_pos = get_end_effector_position()

print(f"  Hedef: θ1={math.degrees(target_j1):.1f}°, θ2={math.degrees(target_j2):.1f}°")
print(f"  Gerçek: θ1={math.degrees(real_j1):.1f}°, θ2={math.degrees(real_j2):.1f}°")
print(f"  End-effector pozisyonu: ({ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f})")

visualize_target(ee_pos[0], ee_pos[1], ee_pos[2], [1, 1, 0, 0.7])  # Sarı

input("\n→ Animasyona geçmek için ENTER'a basın...")

# SÜREKLI ANİMASYON
print("\n" + "="*60)
print("SÜREKLI ANİMASYON")
print("Robot düzgün bir şekilde sallanacak")
print("Kapatmak için CTRL+C yapın")
print("="*60)

t = 0
try:
    while True:
        # Sinüzoidal hareket (yumuşak)
        theta1 = 0.5 * math.sin(t * 0.5)      # Yavaş sallanma
        theta2 = 0.3 * math.cos(t * 0.7)      # Farklı frekansta

        set_joint_position(joint1_idx, theta1)
        set_joint_position(joint2_idx, theta2)

        # End-effector pozisyonunu göster
        ee_pos = get_end_effector_position()
        visualize_target(ee_pos[0], ee_pos[1], ee_pos[2])

        p.stepSimulation()
        time.sleep(1./240.)
        t += 0.02

except KeyboardInterrupt:
    print("\n\n✓ Simülasyon durduruldu")

p.disconnect()
print("✓ PyBullet kapatıldı")
print("\n" + "="*60)
print("ÖZET")
print("="*60)
print("✓ Forward Kinematics başarıyla test edildi")
print("✓ Robot kontrol sistemi çalışıyor")
print("✓ Sonraki ders: Inverse Kinematics")
print("="*60)
