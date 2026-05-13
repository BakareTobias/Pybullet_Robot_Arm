"""
DERS 10: Gripper Kontrolü ve Nesne Yönetimi
Pick-and-place için temel beceriler
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("DERS 10: GRIPPER VE NESNE YÖNETİMİ")
print("="*60)

# =====================================================
# BÖLÜM 1: Gripper Nedir?
# =====================================================
print("\nBÖLÜM 1: Gripper (Tutucu) Kavramı")
print("-"*60)
print("""
Gripper: Robot elinin ucundaki tutma mekanizması

Çeşitleri:
  1. Paralel Gripper (En yaygın)
     └─┬─┘  Kapalı
     └───┘  Açık

  2. Vakumlu Gripper (Emme ile)
  3. Çok parmaklı (İnsansı el)
  4. Mıknatıslı (Metal nesneler için)

Kuka robotun gripper'ı:
  - 2 parmak (paralel)
  - Açma/kapama hareketi
  - Kuvvet kontrolü
""")

input("Devam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 2: Simülasyon Kurulumu
# =====================================================
print("="*60)
print("BÖLÜM 2: Ortamı Hazırlama")
print("="*60)

# Simülasyon başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")
print("✓ Simülasyon başladı")

# Robot yükle
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Kuka robot yüklendi")

# Kamera
p.resetDebugVisualizerCamera(
    cameraDistance=1.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0.3, 0, 0.3]
)

# Robot eklemlerini tanı
num_joints = p.getNumJoints(robot)
print(f"✓ Robot'un {num_joints} eklemi var")

print("\nEklem bilgileri:")
for i in range(num_joints):
    info = p.getJointInfo(robot, i)
    joint_name = info[1].decode('utf-8')
    joint_type = info[2]
    print(f"  {i}: {joint_name} (Tip: {joint_type})")

# Gripper eklemlerini bul (genelde son 2 eklem)
# Kuka'da gripper yok ama end-effector var
# Basit bir gripper mekanizması simüle edeceğiz

print("\nNot: Kuka modeli gripper içermiyor.")
print("     Basit bir tutma mekanizması simüle edeceğiz.")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 3: Nesne Oluşturma
# =====================================================
print("="*60)
print("BÖLÜM 3: Simülasyonda Nesne Oluşturma")
print("="*60)

print("\n3 farklı nesne türü oluşturacağız:")
print("  1. Küp (Box)")
print("  2. Küre (Sphere)")
print("  3. Silindir (Cylinder)\n")

# Yardımcı fonksiyon: Nesne oluştur
def nesne_olustur(tip, pozisyon, renk, boyut=0.05):
    """
    Simülasyonda nesne oluştur

    Parametreler:
        tip: "box", "sphere", "cylinder"
        pozisyon: [x, y, z]
        renk: [r, g, b, a]
        boyut: Nesne boyutu (metre)

    Döndürür:
        nesne_id: PyBullet nesne ID'si
    """
    if tip == "box":
        # Küp
        collision_shape = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[boyut/2, boyut/2, boyut/2]
        )
        visual_shape = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[boyut/2, boyut/2, boyut/2],
            rgbaColor=renk
        )

    elif tip == "sphere":
        # Küre
        collision_shape = p.createCollisionShape(
            p.GEOM_SPHERE,
            radius=boyut/2
        )
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=boyut/2,
            rgbaColor=renk
        )

    elif tip == "cylinder":
        # Silindir
        collision_shape = p.createCollisionShape(
            p.GEOM_CYLINDER,
            radius=boyut/2,
            height=boyut
        )
        visual_shape = p.createVisualShape(
            p.GEOM_CYLINDER,
            radius=boyut/2,
            length=boyut,
            rgbaColor=renk
        )

    # Nesneyi oluştur
    nesne_id = p.createMultiBody(
        baseMass=0.05,  # Hafif (50 gram)
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=pozisyon
    )

    return nesne_id

# Test: 3 farklı nesne oluştur
print("→ Nesneler oluşturuluyor...")

kutu1 = nesne_olustur("box", [0.4, 0.2, 0.3], [1, 0, 0, 1], boyut=0.05)
print(f"  ✓ Kırmızı küp oluşturuldu (ID: {kutu1})")

kutu2 = nesne_olustur("sphere", [0.4, 0.0, 0.3], [0, 0, 1, 1], boyut=0.05)
print(f"  ✓ Mavi küre oluşturuldu (ID: {kutu2})")

kutu3 = nesne_olustur("cylinder", [0.4, -0.2, 0.3], [0, 1, 0, 1], boyut=0.05)
print(f"  ✓ Yeşil silindir oluşturuldu (ID: {kutu3})")

# Düşmeleri bekle
print("\n→ Nesneler düşüyor...")
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

print("✓ Nesneler zemine düştü")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 4: Nesne Bilgilerini Okuma
# =====================================================
print("="*60)
print("BÖLÜM 4: Nesne Pozisyonunu Okuma")
print("="*60)

def nesne_pozisyonu_oku(nesne_id):
    """Nesnenin pozisyonunu ve oryantasyonunu oku"""
    pos, orn = p.getBasePositionAndOrientation(nesne_id)
    return pos, orn

print("\nMevcut nesne pozisyonları:")
for idx, nesne in enumerate([kutu1, kutu2, kutu3], 1):
    pos, _ = nesne_pozisyonu_oku(nesne)
    print(f"  Nesne {idx}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 5: Basit "Tutma" Mekanizması
# =====================================================
print("="*60)
print("BÖLÜM 5: Basit Tutma Mekanizması")
print("="*60)

print("""
Gerçek gripper simülasyonu karmaşıktır. Basit bir yöntem:

CONSTRAINT (Kısıtlama) Kullanma:
  - Robot end-effector'a çok yaklaşınca
  - Nesneyi end-effector'a "yapıştır" (sanal bağlantı)
  - Böylece robot hareket edince nesne de hareket eder
  - Bırakmak için: Bağlantıyı kaldır

Bu yönteme "Kinematic Grasping" denir.
""")

# Yardımcı fonksiyonlar
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        robot, eklem_no,
        p.POSITION_CONTROL,
        targetPosition=aci,
        force=200
    )

def ik_hesapla(x, y, z):
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

def robot_hareket_ettir(hedef_pos, sure=2.0):
    """Robot'u hedef pozisyona götür (trajectory ile)"""
    eklem_acilari = ik_hesapla(*hedef_pos)

    adim_sayisi = int(sure * 240)
    for _ in range(adim_sayisi):
        for i in range(7):
            eklem_hareket_ettir(i, eklem_acilari[i])
        p.stepSimulation()
        time.sleep(1./240.)

def end_effector_pozisyonu():
    """End-effector'ın mevcut pozisyonu"""
    durum = p.getLinkState(robot, 6)
    return durum[0]

# Tutma/Bırakma için global değişken
tutulan_nesne = None
constraint_id = None

def nesne_tut(nesne_id):
    """
    Nesneyi tut (constraint oluştur)
    """
    global tutulan_nesne, constraint_id

    if tutulan_nesne is not None:
        print("  ⚠ Zaten bir nesne tutuyorsunuz!")
        return False

    # End-effector pozisyonu
    ee_pos = end_effector_pozisyonu()

    # Nesne pozisyonu
    nesne_pos, _ = nesne_pozisyonu_oku(nesne_id)

    # Mesafe kontrolü
    mesafe = math.sqrt(
        (ee_pos[0]-nesne_pos[0])**2 +
        (ee_pos[1]-nesne_pos[1])**2 +
        (ee_pos[2]-nesne_pos[2])**2
    )

    if mesafe > 0.15:  # 15 cm'den uzaksa tutamaz
        print(f"  ✗ Nesne çok uzak! Mesafe: {mesafe*100:.1f}cm")
        return False

    # Constraint (bağlantı) oluştur
    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot,
        parentLinkIndex=6,  # End-effector
        childBodyUniqueId=nesne_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,  # Sabit bağlantı
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )

    tutulan_nesne = nesne_id
    print(f"  ✓ Nesne tutuldu! (Constraint ID: {constraint_id})")
    return True

def nesne_birak():
    """
    Tutulan nesneyi bırak (constraint kaldır)
    """
    global tutulan_nesne, constraint_id

    if tutulan_nesne is None:
        print("  ⚠ Elinizde nesne yok!")
        return False

    # Constraint'i kaldır
    p.removeConstraint(constraint_id)

    print(f"  ✓ Nesne bırakıldı!")
    tutulan_nesne = None
    constraint_id = None
    return True

input("Devam için ENTER'a basın...\n")

# =====================================================
# TEST 1: Basit Pick and Place
# =====================================================
print("="*60)
print("TEST 1: Basit Pick and Place")
print("="*60)

# Kırmızı küpü al
print("\n→ ADIM 1: Kırmızı küpün üstüne git")
kutu1_pos, _ = nesne_pozisyonu_oku(kutu1)
hedef_ustte = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.2]
robot_hareket_ettir(hedef_ustte, sure=2.0)
print("  ✓ Üstte")

print("\n→ ADIM 2: Küpün seviyesine in")
hedef_yakin = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.05]
robot_hareket_ettir(hedef_yakin, sure=1.5)
print("  ✓ Küpe yaklaştı")

print("\n→ ADIM 3: Küpü tut")
nesne_tut(kutu1)
time.sleep(0.5)

print("\n→ ADIM 4: Yukarı kaldır")
hedef_yukari = [kutu1_pos[0], kutu1_pos[1], kutu1_pos[2] + 0.3]
robot_hareket_ettir(hedef_yukari, sure=1.5)
print("  ✓ Küp havada!")

print("\n→ ADIM 5: Yeni pozisyona taşı")
yeni_pos = [0.2, -0.3, 0.3]
robot_hareket_ettir(yeni_pos, sure=2.0)
print("  ✓ Yeni pozisyonda")

print("\n→ ADIM 6: Küpü bırak")
nesne_birak()

# Biraz düşmesi için bekle
for _ in range(120):
    p.stepSimulation()
    time.sleep(1./240.)

print("  ✓ Küp bırakıldı ve düştü")

# Final pozisyonu kontrol et
final_pos, _ = nesne_pozisyonu_oku(kutu1)
print(f"\nKırmızı küp taşındı:")
print(f"  Başlangıç: ({kutu1_pos[0]:.2f}, {kutu1_pos[1]:.2f}, {kutu1_pos[2]:.2f})")
print(f"  Son      : ({final_pos[0]:.2f}, {final_pos[1]:.2f}, {final_pos[2]:.2f})")

input("\n✓ TEST 1 tamamlandı! Devam için ENTER...\n")

# =====================================================
# TEST 2: Tüm Nesneleri Toplama
# =====================================================
print("="*60)
print("TEST 2: Tüm Nesneleri Bir Araya Toplama")
print("="*60)

hedef_alan = [0.0, 0.0, 0.05]  # Merkeze topla

nesneler = [kutu1, kutu2, kutu3]
renkler = ["Kırmızı", "Mavi", "Yeşil"]

for idx, (nesne, renk) in enumerate(zip(nesneler, renkler)):
    print(f"\n→ {renk} nesneyi topluyorum...")

    # Nesne pozisyonu
    pos, _ = nesne_pozisyonu_oku(nesne)

    # Üste git
    print("  - Üste gidiliyor...")
    robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.25], sure=1.5)

    # İn
    print("  - İniliyor...")
    robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.05], sure=1.0)

    # Tut
    print("  - Tutuluyor...")
    if nesne_tut(nesne):
        # Kaldır
        print("  - Kaldırılıyor...")
        robot_hareket_ettir([pos[0], pos[1], pos[2] + 0.3], sure=1.0)

        # Hedef alana taşı
        print("  - Hedefe taşınıyor...")
        offset_x = idx * 0.08  # Yan yana diz
        robot_hareket_ettir([hedef_alan[0] + offset_x, hedef_alan[1], 0.3], sure=1.5)

        # Bırak
        print("  - Bırakılıyor...")
        robot_hareket_ettir([hedef_alan[0] + offset_x, hedef_alan[1], 0.15], sure=0.8)
        nesne_birak()

        # Düşmesi için bekle
        for _ in range(60):
            p.stepSimulation()
            time.sleep(1./240.)

        print(f"  ✓ {renk} nesne toplandı!")
    else:
        print(f"  ✗ {renk} nesne tutulamadı")

print("\n✓ Tüm nesneler toplandı!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 10 ÖZET")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Gripper kavramı
✓ Nesne oluşturma (createCollisionShape + createVisualShape)
✓ Nesne pozisyonu okuma (getBasePositionAndOrientation)
✓ Constraint ile tutma (createConstraint)
✓ Constraint kaldırarak bırakma (removeConstraint)
✓ Basit pick-and-place işlemi

ÖNEMLİ FONKSİYONLAR:
  p.createCollisionShape()  - Fizik şekli
  p.createVisualShape()     - Görsel şekli
  p.createMultiBody()       - Nesne oluştur
  p.createConstraint()      - Bağlantı oluştur
  p.removeConstraint()      - Bağlantıyı kaldır

CONSTRAINT TÜRLERİ:
  - JOINT_FIXED: Sabit bağlantı (gripper için)
  - JOINT_POINT2POINT: Nokta bağlantısı
  - JOINT_HINGE: Menteşe (döner)
  - JOINT_PRISMATIC: Kayar

GERÇEK GRIPPER:
  Gerçek robotlarda gripper'ın kendi eklemleri var
  Örnek: Robotiq 2F-85
    - 2 parmak
    - Her parmak motor kontrollü
    - Kuvvet sensörü

Bizim yöntem (kinematic grasping):
  ✓ Basit
  ✓ Hızlı
  ✓ Eğitim için yeterli
  ✗ Gerçekçi fizik yok

SONRAKI ADIM:
→ PROJE 1: Gelişmiş Pick-and-Place
  - Çoklu nesne
  - Renk tanıma
  - Hedef kutulara yerleştirme
""")

input("\nDers bitti! ENTER'a basın...\n")

p.disconnect()
print("✓ PyBullet kapatıldı")
