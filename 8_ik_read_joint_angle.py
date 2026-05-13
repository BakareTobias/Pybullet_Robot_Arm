"""
DERS 4: Inverse Kinematics (Ters Kinematik)
Hedef pozisyondan eklem açılarını hesaplama
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

# Simülasyonu başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Kamerayı ayarla
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.5]
)

print("✓ Simülasyon başladı")

# Robot yükle
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot yüklendi\n")

# =====================================================
# YARDIMCI FONKSİYONLAR
# =====================================================
def eklem_hareket_ettir(eklem_no, aci, max_force=200, max_velocity=1.0):
    """Bir eklemi belirtilen açıya götürür"""
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci,
        force=max_force,
        maxVelocity=max_velocity,
        positionGain=0.3,
        velocityGain=1.0
    )

def eklem_durumunu_oku(eklem_no):
    """Eklemin durumunu okur"""
    durum = p.getJointState(robot, eklem_no)
    return durum[0]  # Açı

def end_effector_pozisyonu():
    """End-effector'ın pozisyonunu döndürür"""
    # Kuka IIWA'da son link (end-effector) genellikle link 6 veya 7
    num_links = p.getNumJoints(robot)
    end_effector_link = num_links - 1  # Son link
    
    link_state = p.getLinkState(robot, end_effector_link)
    return link_state[0]  # (x, y, z) pozisyon

def simule_et(adim_sayisi):
    """Simülasyonu çalıştır"""
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

def marker_olustur(x, y, z, renk=[1, 0, 0, 0.7]):
    """Hedef pozisyonu görselleştirmek için marker oluştur"""
    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=renk
    )
    marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape,
        basePosition=[x, y, z]
    )
    return marker

# =====================================================
# INVERSE KINEMATICS FONKSİYONU
# =====================================================
def inverse_kinematics(hedef_pozisyon, hedef_orientasyon=None):
    """
    Hedef pozisyona ulaşmak için gerekli eklem açılarını hesaplar
    
    Args:
        hedef_pozisyon: (x, y, z) hedef pozisyon
        hedef_orientasyon: (x, y, z, w) quaternion (opsiyonel)
    
    Returns:
        eklem_acilari: List of joint angles (radyan)
    """
    num_joints = p.getNumJoints(robot)
    end_effector_link = num_joints - 1
    
    # PyBullet'in built-in IK çözücüsünü kullan
    if hedef_orientasyon is None:
        # Sadece pozisyon hedefi (orientasyon önemli değil)
        hedef_orientasyon = [0, 0, 0, 1]  # Varsayılan quaternion
    
    # IK çözücü parametreleri
    ik_solver = 0  # 0 = ikSolver, 1 = ikSolverPosOnly
    max_iterations = 20
    residual_threshold = 0.01
    
    # Hareketli eklemleri bul
    movable_joints = []
    for i in range(num_joints):
        info = p.getJointInfo(robot, i)
        if info[2] == p.JOINT_REVOLUTE:
            movable_joints.append(i)
    
    # Mevcut eklem pozisyonlarını al (başlangıç değerleri)
    current_positions = []
    for joint_idx in movable_joints:
        current_positions.append(eklem_durumunu_oku(joint_idx))
    
    # IK hesapla
    ik_result = p.calculateInverseKinematics(
        bodyUniqueId=robot,
        endEffectorLinkIndex=end_effector_link,
        targetPosition=hedef_pozisyon,
        targetOrientation=hedef_orientasyon,
        lowerLimits=[-math.pi] * len(movable_joints),
        upperLimits=[math.pi] * len(movable_joints),
        jointRanges=[2*math.pi] * len(movable_joints),
        restPoses=current_positions,
        maxNumIterations=max_iterations,
        residualThreshold=residual_threshold
    )
    
    return list(ik_result)

# =====================================================
# TEST 1: Basit IK - Tek Hedef Pozisyon
# =====================================================
print("="*60)
print("TEST 1: Basit Inverse Kinematics")
print("="*60)

# Hedef pozisyon
hedef_x, hedef_y, hedef_z = 0.4, 0.3, 0.6
print(f"\nHedef pozisyon: ({hedef_x:.2f}, {hedef_y:.2f}, {hedef_z:.2f})")

# Marker oluştur
marker = marker_olustur(hedef_x, hedef_y, hedef_z, [1, 0, 0, 0.7])
print("✓ Kırmızı marker hedef pozisyonu gösteriyor")

# IK hesapla
print("\nInverse Kinematics hesaplanıyor...")
eklem_acilari = inverse_kinematics([hedef_x, hedef_y, hedef_z])

# Hareketli eklemleri bul
num_joints = p.getNumJoints(robot)
movable_joints = []
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    if info[2] == p.JOINT_REVOLUTE:
        movable_joints.append(i)

# Eklemleri hesaplanan açılara götür
print(f"\nHesaplanan eklem açıları:")
for idx, joint_idx in enumerate(movable_joints[:7]):  # İlk 7 eklem
    aci = eklem_acilari[idx]
    info = p.getJointInfo(robot, joint_idx)
    eklem_adi = info[1].decode('utf-8')
    print(f"  Eklem {joint_idx} ({eklem_adi}): {math.degrees(aci):.2f}°")
    eklem_hareket_ettir(joint_idx, aci)

# Hareketi bekle
print("\nRobot hedefe hareket ediyor...")
simule_et(500)

# Sonuçları kontrol et
gercek_pozisyon = end_effector_pozisyonu()
mesafe = math.sqrt(
    (gercek_pozisyon[0] - hedef_x)**2 +
    (gercek_pozisyon[1] - hedef_y)**2 +
    (gercek_pozisyon[2] - hedef_z)**2
)

print(f"\n✓ Hareket tamamlandı!")
print(f"  Hedef pozisyon: ({hedef_x:.3f}, {hedef_y:.3f}, {hedef_z:.3f})")
print(f"  Gerçek pozisyon: ({gercek_pozisyon[0]:.3f}, {gercek_pozisyon[1]:.3f}, {gercek_pozisyon[2]:.3f})")
print(f"  Mesafe hatası: {mesafe*1000:.2f} mm")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 2: Birden Fazla Hedef Pozisyon
# =====================================================
print("\n" + "="*60)
print("TEST 2: Birden Fazla Hedef Pozisyon")
print("="*60)

# Farklı renklerde markerlar
hedefler = [
    ([0.5, 0.2, 0.5], [1, 0, 0, 0.7]),  # Kırmızı
    ([0.3, 0.4, 0.6], [0, 1, 0, 0.7]),  # Yeşil
    ([0.4, -0.3, 0.5], [0, 0, 1, 0.7]),  # Mavi
    ([0.2, 0.0, 0.7], [1, 1, 0, 0.7]),  # Sarı
]

print(f"\n{len(hedefler)} farklı hedef pozisyon test edilecek\n")

for i, (hedef_pos, renk) in enumerate(hedefler):
    print(f"[Hedef {i+1}] Pozisyon: ({hedef_pos[0]:.2f}, {hedef_pos[1]:.2f}, {hedef_pos[2]:.2f})")
    
    # Marker oluştur
    marker = marker_olustur(hedef_pos[0], hedef_pos[1], hedef_pos[2], renk)
    
    # IK hesapla
    eklem_acilari = inverse_kinematics(hedef_pos)
    
    # Eklemleri hareket ettir
    for idx, joint_idx in enumerate(movable_joints[:7]):
        eklem_hareket_ettir(joint_idx, eklem_acilari[idx])
    
    # Hareketi bekle
    simule_et(400)
    
    # Sonuç kontrolü
    gercek_pos = end_effector_pozisyonu()
    mesafe = math.sqrt(
        (gercek_pos[0] - hedef_pos[0])**2 +
        (gercek_pos[1] - hedef_pos[1])**2 +
        (gercek_pos[2] - hedef_pos[2])**2
    )
    
    print(f"  → Gerçek: ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")
    print(f"  → Hata: {mesafe*1000:.2f} mm\n")
    
    time.sleep(0.5)  # Kısa bekleme

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 3: Trajectory Following (Yörünge Takibi)
# =====================================================
print("\n" + "="*60)
print("TEST 3: Yörünge Takibi (Trajectory Following)")
print("="*60)
print("Robot bir daire çizecek\n")

# Daire parametreleri
merkez = [0.4, 0.0, 0.6]
yaricap = 0.15
nokta_sayisi = 20

print(f"Daire merkezi: {merkez}")
print(f"Yarıçap: {yaricap}m")
print(f"Nokta sayısı: {nokta_sayisi}\n")

# Daire üzerindeki noktaları hesapla
noktalar = []
for i in range(nokta_sayisi):
    aci = 2 * math.pi * i / nokta_sayisi
    x = merkez[0] + yaricap * math.cos(aci)
    y = merkez[1] + yaricap * math.sin(aci)
    z = merkez[2]
    noktalar.append([x, y, z])

print("Robot daireyi çiziyor...")
for i, nokta in enumerate(noktalar):
    # IK hesapla
    eklem_acilari = inverse_kinematics(nokta)
    
    # Eklemleri hareket ettir
    for idx, joint_idx in enumerate(movable_joints[:7]):
        eklem_hareket_ettir(joint_idx, eklem_acilari[idx])
    
    # Her noktada kısa simülasyon
    simule_et(50)
    
    # İlerlemeyi göster
    if i % 5 == 0:
        gercek_pos = end_effector_pozisyonu()
        print(f"  Nokta {i+1}/{nokta_sayisi}: "
              f"({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f})")

print("\n✓ Yörünge takibi tamamlandı!")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 4: Gerçek Zamanlı IK Takibi
# =====================================================
print("\n" + "="*60)
print("TEST 4: Gerçek Zamanlı IK Takibi")
print("="*60)
print("10 saniye boyunca robot hareketli hedefi takip edecek\n")

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:
    # Hareketli hedef (sinüzoidal)
    hedef_x = 0.4 + 0.1 * math.sin(t)
    hedef_y = 0.0 + 0.15 * math.cos(t * 0.8)
    hedef_z = 0.6 + 0.05 * math.sin(t * 1.2)
    
    # IK hesapla
    eklem_acilari = inverse_kinematics([hedef_x, hedef_y, hedef_z])
    
    # Eklemleri hareket ettir
    for idx, joint_idx in enumerate(movable_joints[:7]):
        eklem_hareket_ettir(joint_idx, eklem_acilari[idx])
    
    # Gerçek pozisyonu oku
    gercek_pos = end_effector_pozisyonu()
    mesafe = math.sqrt(
        (gercek_pos[0] - hedef_x)**2 +
        (gercek_pos[1] - hedef_y)**2 +
        (gercek_pos[2] - hedef_z)**2
    )
    
    # Ekrana yazdır
    print(f"\rHedef: ({hedef_x:.3f}, {hedef_y:.3f}, {hedef_z:.3f}) | "
          f"Gerçek: ({gercek_pos[0]:.3f}, {gercek_pos[1]:.3f}, {gercek_pos[2]:.3f}) | "
          f"Hata: {mesafe*1000:.1f}mm | "
          f"Süre: {time.time()-baslangic:.1f}s", end="")
    
    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Gerçek zamanlı takip tamamlandı!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 4 ÖZET: Inverse Kinematics")
print("="*60)
print("✓ PyBullet'in IK çözücüsünü kullandık")
print("✓ Hedef pozisyondan eklem açılarını hesapladık")
print("✓ Birden fazla hedef pozisyonu test ettik")
print("✓ Yörünge takibi yaptık")
print("✓ Gerçek zamanlı hedef takibi yaptık")
print("\nSonraki ders: Collision Detection (Çarpışma Tespiti)")
print("="*60)

p.disconnect()
print("\n✓ Simülasyon kapatıldı")

