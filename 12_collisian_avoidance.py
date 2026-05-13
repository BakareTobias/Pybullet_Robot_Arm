"""
DERS 8: Collision Avoidance (Çarpışma Önleme)
Robot engellerden kaçarak hedefe ulaşacak
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

print("="*60)
print("DERS 8: COLLISION AVOIDANCE")
print("="*60)

# =====================================================
# BÖLÜM 1: Çarpışma Tespiti
# =====================================================
print("\nBÖLÜM 1: Çarpışma Tespiti Nasıl Çalışır?")
print("-"*60)
print("""
PyBullet'te 2 çarpışma kontrol yöntemi var:

1. getContactPoints(): Fiziksel temas oldu mu?
   - Gerçek zamanlı
   - Çarpışma SONRASI tespit
   - "Ups, çarptık!" ✗

2. getClosestPoints(): En yakın mesafe ne?
   - Önceden kontrol
   - Çarpışma ÖNCESİ tespit
   - "Yaklaşıyoruz, dön!" ✓

Biz 2. yöntemi kullanacağız (proaktif yaklaşım)
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
print("✓ Simülasyon başladı")

# Zemin
p.loadURDF("plane.urdf")

# Robot
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot yüklendi")

# Kamera ayarla
p.resetDebugVisualizerCamera(
    cameraDistance=2.5,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

# Engeller ekleyelim
print("\n→ Engeller ekleniyor...")

engeller = []

# Engel 1: Kırmızı küp
engel1_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.3])
engel1_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.3],
                                     rgbaColor=[1, 0, 0, 0.7])
engel1 = p.createMultiBody(
    baseMass=0,  # Sabit (hareket etmez)
    baseCollisionShapeIndex=engel1_collision,
    baseVisualShapeIndex=engel1_visual,
    basePosition=[0.4, 0.1, 0.3]
)
engeller.append(engel1)
print(f"  ✓ Engel 1 (Kırmızı küp): [0.4, 0.1, 0.3]")

# Engel 2: Mavi küp
engel2_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, 0.2])
engel2_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.15, 0.2],
                                     rgbaColor=[0, 0, 1, 0.7])
engel2 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=engel2_collision,
    baseVisualShapeIndex=engel2_visual,
    basePosition=[0.2, -0.2, 0.2]
)
engeller.append(engel2)
print(f"  ✓ Engel 2 (Mavi küp): [0.2, -0.2, 0.2]")

# Engel 3: Yeşil silindir
engel3_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.4)
engel3_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.08, length=0.4,
                                     rgbaColor=[0, 1, 0, 0.7])
engel3 = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=engel3_collision,
    baseVisualShapeIndex=engel3_visual,
    basePosition=[0.3, 0.3, 0.2]
)
engeller.append(engel3)
print(f"  ✓ Engel 3 (Yeşil silindir): [0.3, 0.3, 0.2]")

print(f"\n✓ Toplam {len(engeller)} engel eklendi")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# YARDIMCI FONKSİYONLAR
# =====================================================
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci,
        force=200,
        maxVelocity=2.0
    )

def ik_hesapla(x, y, z):
    return p.calculateInverseKinematics(robot, 6, [x, y, z])

marker = None
def marker_goster(x, y, z, renk=[0, 1, 0, 0.8]):
    global marker
    if marker is not None:
        try:
            p.removeBody(marker)
        except:
            pass
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=renk)
    marker = p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

def carpisma_var_mi(guvenlik_mesafesi=0.05):
    """
    Robot herhangi bir engele çok yakın mı?

    Parametreler:
        guvenlik_mesafesi: Minimum güvenli mesafe (metre)

    Döndürür:
        True: Çarpışma riski var
        False: Güvenli
    """
    for engel in engeller:
        # Robot'un tüm linkleriyle engel arasındaki en yakın mesafe
        yakin_noktalar = p.getClosestPoints(
            bodyA=robot,
            bodyB=engel,
            distance=guvenlik_mesafesi  # Bu mesafeden yakınsa raporla
        )

        if len(yakin_noktalar) > 0:
            # En yakın mesafeyi bul
            min_mesafe = min([nokta[8] for nokta in yakin_noktalar])

            if min_mesafe < guvenlik_mesafesi:
                return True, min_mesafe

    return False, float('inf')

# =====================================================
# BÖLÜM 3: TEST - Çarpışma Olmadan Hareket
# =====================================================
print("="*60)
print("BÖLÜM 3: Güvenli Bölgede Hareket")
print("="*60)

print("\n→ Robot güvenli bölgede hareket edecek (engellere uzak)")

guvenli_hedefler = [
    [0.5, 0.5, 0.6],   # Engellerin dışında
    [0.6, -0.4, 0.5],  # Engellerin dışında
]

for idx, hedef in enumerate(guvenli_hedefler, 1):
    print(f"\n  Hedef {idx}: {hedef}")

    marker_goster(*hedef, renk=[0, 1, 0, 0.8])
    eklem_acilari = ik_hesapla(*hedef)

    for i in range(7):
        eklem_hareket_ettir(i, eklem_acilari[i])

    # Hareket ederken çarpışma kontrolü
    for _ in range(300):
        p.stepSimulation()
        time.sleep(1./240.)

        carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.1)
        if carpti:
            print(f"    ⚠ Engele çok yakın! Mesafe: {mesafe*100:.1f} cm")

    print(f"    ✓ Hedefe ulaşıldı (güvenli)")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 4: TEST - Engele Çarpma Denemesi
# =====================================================
print("="*60)
print("BÖLÜM 4: Tehlikeli Hedef (Çarpışma Riski!)")
print("="*60)

tehlikeli_hedef = [0.4, 0.1, 0.3]  # Engel 1'in tam içinde!
print(f"\n⚠ Tehlikeli hedef: {tehlikeli_hedef}")
print("  (Bu hedef Engel 1'in içinde!)")

marker_goster(*tehlikeli_hedef, renk=[1, 0, 0, 0.8])

print("\n→ Robot tehlikeli hedefe gidecek...")
eklem_acilari = ik_hesapla(*tehlikeli_hedef)

for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])

# Hareket sırasında takip et
carpti_mi = False
for adim in range(400):
    p.stepSimulation()
    time.sleep(1./240.)

    carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.1)

    if carpti and not carpti_mi:
        print(f"  ⚠⚠⚠ UYARI: Engele {mesafe*100:.1f} cm yaklaştı!")
        carpti_mi = True

    # Her 50 adımda durum raporu
    if adim % 50 == 0 and carpti:
        print(f"    Adım {adim}: Engele mesafe = {mesafe*100:.1f} cm")

if carpti_mi:
    print("\n  ✗ Çarpışma riski tespit edildi!")
else:
    print("\n  ✓ Garip, çarpışma olmadı (engel küçük mü?)")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 5: Basit Kaçınma Stratejisi
# =====================================================
print("="*60)
print("BÖLÜM 5: Akıllı Kaçınma - Yüksekten Git!")
print("="*60)

print("""
Strateji: Engellerin ÜZERİNDEN geç
  1. Başlangıç → Yukarı çık (güvenli yükseklik)
  2. Yukarıda → Hedefe doğru hareket et
  3. Hedef üstünde → Aşağı in
""")

baslangic = [0.5, 0.5, 0.5]
hedef_son = [0.3, -0.3, 0.4]  # Engellerin yakınında

guvenli_yukseklik = 0.9  # Engellerin üstünde

print(f"\nBaşlangıç: {baslangic}")
print(f"Hedef    : {hedef_son}")
print(f"Güvenli yükseklik: {guvenli_yukseklik}m")

# Waypoint'ler oluştur
waypoints = [
    np.array(baslangic),                                    # 1. Başlangıç
    np.array([baslangic[0], baslangic[1], guvenli_yukseklik]),  # 2. Yukarı çık
    np.array([hedef_son[0], hedef_son[1], guvenli_yukseklik]),  # 3. Hedefin üstüne git
    np.array(hedef_son)                                     # 4. Aşağı in
]

print("\nWaypoint'ler:")
for idx, wp in enumerate(waypoints, 1):
    print(f"  {idx}. {wp}")

# Waypoint'leri göster
for wp in waypoints:
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=[1, 1, 0, 0.9])
    p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=wp.tolist())

print("\n→ Robot waypoint'lerden geçerek güvenli şekilde gidecek...")

# Her waypoint'e git
for idx in range(len(waypoints) - 1):
    baslangic_wp = waypoints[idx]
    bitis_wp = waypoints[idx + 1]

    print(f"\n  Segment {idx+1}/{len(waypoints)-1}")

    # Cubic interpolation ile yumuşak hareket
    adim_sayisi = 200
    for adim in range(adim_sayisi):
        t = adim / adim_sayisi
        s = 3 * t**2 - 2 * t**3  # Cubic

        hedef_anlik = baslangic_wp + (bitis_wp - baslangic_wp) * s

        marker_goster(*hedef_anlik, renk=[0, 1, 1, 0.8])
        eklem_acilari = ik_hesapla(*hedef_anlik)

        for i in range(7):
            eklem_hareket_ettir(i, eklem_acilari[i])

        p.stepSimulation()
        time.sleep(1./240.)

        # Çarpışma kontrolü
        carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.15)
        if carpti and adim % 50 == 0:
            print(f"    ⚠ Engele yakınlık: {mesafe*100:.1f} cm (ama güvenli yükseklikte!)")

    print(f"    ✓ Waypoint {idx+2}'ye ulaşıldı")

print("\n✓ Robot tüm waypoint'lerden geçti!")
print("✓ Engellerden başarıyla kaçıldı!")

input("\nDevam için ENTER'a basın...\n")

# =====================================================
# BÖLÜM 6: Gelişmiş - Dinamik Kaçınma
# =====================================================
print("="*60)
print("BÖLÜM 6: Dinamik Kaçınma (Bonus)")
print("="*60)

print("""
Gelişmiş teknik: Gerçek zamanlı kaçınma
  - Her adımda çarpışma kontrolü
  - Çok yaklaşırsa yönünü değiştir

Bu basit bir demo, gerçek uygulamalar daha karmaşık:
  - RRT (Rapidly-exploring Random Tree)
  - A* algoritması
  - Potential Field yöntemleri
""")

print("\n→ Robot engellere çok yaklaşırsa duracak...")

hedef_dinamik = [0.35, 0.0, 0.4]
marker_goster(*hedef_dinamik, renk=[1, 0.5, 0, 0.8])

eklem_acilari = ik_hesapla(*hedef_dinamik)

for i in range(7):
    eklem_hareket_ettir(i, eklem_acilari[i])

for adim in range(500):
    carpti, mesafe = carpisma_var_mi(guvenlik_mesafesi=0.12)

    if carpti:
        if mesafe < 0.08:  # Çok yakın!
            print(f"\n  🛑 DUR! Engele {mesafe*100:.1f} cm - Çok yakın!")
            print("     Gerçek uygulamada alternatif yol aranır.")
            break
        elif adim % 50 == 0:
            print(f"  ⚠ Dikkat: {mesafe*100:.1f} cm mesafede engel var")

    p.stepSimulation()
    time.sleep(1./240.)

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*60)
print("DERS 8 ÖZET")
print("="*60)
print("""
ÖĞRENDİKLERİMİZ:
✓ Çarpışma tespiti (getClosestPoints)
✓ Güvenlik mesafesi kavramı
✓ Engel oluşturma (collision & visual shapes)
✓ Basit kaçınma stratejisi (yüksekten geç)
✓ Waypoint tabanlı güvenli hareket

ÖNEMLİ FONKSİYONLAR:
  p.getClosestPoints(robot, engel, distance)
  p.createCollisionShape(...)
  p.createVisualShape(...)

GERÇEK DÜNYA:
  - Otonom araçlar
  - Endüstriyel robotlar (insan yakınında)
  - Drone'lar
  - Mobil robotlar (yer temizlik, depo)

GELİŞMİŞ TEKNİKLER (İleri seviye):
  - RRT (Rapidly-exploring Random Tree)
  - A* path planning
  - Potential Fields
  - Dynamic Window Approach

SONRAKI DERS:
→ İleri seviye IK (Jacobian yöntemi)
  Veya siz başka bir konu önerebilirsiniz!
""")

p.disconnect()
print("\n✓ PyBullet kapatıldı")
