"""
DERS 4: Forward Kinematics (İleri Kinematik)
Eklem açılarından robot elinin pozisyonunu hesaplama
"""

import pybullet as p
import pybullet_data
import time
import math

# Simülasyonu başlat
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
print("✓ Simülasyon başladı")

# Robot yükle
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot yüklendi")

# Zemin ekle (referans için)
p.loadURDF("plane.urdf")

# =====================================================
# YARDIMCI FONKSİYONLAR
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
    return durum[0], durum[1]  # açı, hız

def simule_et(adim_sayisi):
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# YENİ FONKSİYON: End-Effector Pozisyonunu Oku
# =====================================================
def el_pozisyonunu_oku():
    """
    Robot elinin (end-effector) gerçek pozisyonunu okur

    Döndürür:
        (x, y, z): 3 boyutlu pozisyon (metre cinsinden)
    """
    # getLinkState: Bir linkin (parçanın) durumunu okur
    # Link 6 = Kuka robotun eli (son link)
    durum = p.getLinkState(robot, 6)

    # durum[0] = pozisyon (x, y, z)
    # durum[1] = oryantasyon (dönüş, quaternion)

    pozisyon = durum[0]  # (x, y, z) tuple

    return pozisyon

# =====================================================
# YENİ: Görsel İşaretleyici (Marker)
# =====================================================
isaretleyici = None

def isaretleyici_ciz(x, y, z, renk=[0, 1, 0, 0.8]):
    """
    Belirtilen pozisyona renkli küre çizer

    Parametreler:
        x, y, z: Pozisyon
        renk: [R, G, B, Alpha] (0-1 arası)
    """
    global isaretleyici

    # Eski işaretleyiciyi sil
    if isaretleyici is not None:
        try:
            p.removeBody(isaretleyici)
        except:
            pass

    # Yeni küre oluştur
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
# TEST 1: Başlangıç Pozisyonu
# =====================================================
print("\n" + "="*60)
print("TEST 1: Robot Elinin Başlangıç Pozisyonu")
print("="*60)

# Tüm eklemleri sıfırla
print("\n→ Robot sıfırlanıyor (tüm eklemler 0°)...")
for i in range(7):
    eklem_hareket_ettir(i, 0)
simule_et(300)

# El pozisyonunu oku
x, y, z = el_pozisyonunu_oku()
print(f"\n✓ Robot eli pozisyonu:")
print(f"   X = {x:.3f} metre")
print(f"   Y = {y:.3f} metre")
print(f"   Z = {z:.3f} metre")

# Yeşil küre ile işaretle
isaretleyici_ciz(x, y, z, renk=[0, 1, 0, 0.8])

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 2: Forward Kinematics - Tek Eklem
# =====================================================
print("\n" + "="*60)
print("TEST 2: Tek Eklem Hareketi ve Pozisyon Değişimi")
print("="*60)

print("\n→ Eklem 0'ı 45° döndür...")
eklem_hareket_ettir(0, math.radians(45))
simule_et(400)

x, y, z = el_pozisyonunu_oku()
print(f"\n✓ Yeni el pozisyonu:")
print(f"   X = {x:.3f} metre")
print(f"   Y = {y:.3f} metre")
print(f"   Z = {z:.3f} metre")

# Kırmızı küre ile işaretle
isaretleyici_ciz(x, y, z, renk=[1, 0, 0, 0.8])

print("\nGözlem:")
print("  Eklem 0 Z ekseni etrafında döner")
print("  → X ve Y koordinatları değişir, Z sabit kalır")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 3: İki Eklem Hareketi
# =====================================================
print("\n" + "="*60)
print("TEST 3: İki Eklem Birlikte Hareket")
print("="*60)

# Sıfırla
print("\n→ Robot sıfırlanıyor...")
for i in range(7):
    eklem_hareket_ettir(i, 0)
simule_et(300)

print("→ Eklem 0: 30°, Eklem 1: 45°")
eklem_hareket_ettir(0, math.radians(30))
eklem_hareket_ettir(1, math.radians(45))
simule_et(400)

x, y, z = el_pozisyonunu_oku()
print(f"\n✓ El pozisyonu:")
print(f"   X = {x:.3f} m")
print(f"   Y = {y:.3f} m")
print(f"   Z = {z:.3f} m")

isaretleyici_ciz(x, y, z, renk=[0, 0, 1, 0.8])

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 4: Farklı Açılar = Farklı Pozisyonlar
# =====================================================
print("\n" + "="*60)
print("TEST 4: Çeşitli Pozisyonlar")
print("="*60)
print("Robot 5 farklı pozisyona gidecek")
print("Her pozisyon farklı renkte işaretlenecek\n")

pozisyonlar = [
    ([0, 0], [0, 1, 0, 0.7]),           # Yeşil
    ([45, 0], [1, 0, 0, 0.7]),          # Kırmızı
    ([0, 45], [0, 0, 1, 0.7]),          # Mavi
    ([45, 45], [1, 1, 0, 0.7]),         # Sarı
    ([-30, 60], [1, 0, 1, 0.7])         # Mor
]

for idx, (acilar, renk) in enumerate(pozisyonlar, 1):
    aci0, aci1 = acilar

    print(f"Pozisyon {idx}: Eklem 0={aci0:3d}°, Eklem 1={aci1:3d}°")

    eklem_hareket_ettir(0, math.radians(aci0))
    eklem_hareket_ettir(1, math.radians(aci1))
    simule_et(300)

    x, y, z = el_pozisyonunu_oku()
    print(f"   → Pozisyon: ({x:.3f}, {y:.3f}, {z:.3f})")

    # Kalıcı işaretleyici oluştur (silinmeden)
    gorsel = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=renk)
    p.createMultiBody(0, baseVisualShapeIndex=gorsel, basePosition=[x, y, z])

    time.sleep(0.5)

print("\n✓ 5 pozisyon işaretlendi (renkli küreler)")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 5: Canlı İzleme - Hareket Eden El
# =====================================================
print("\n" + "="*60)
print("TEST 5: Canlı İzleme - Robot Sallanırken El Pozisyonu")
print("="*60)
print("10 saniye boyunca robot sallanacak")
print("El pozisyonu gerçek zamanlı gösterilecek\n")

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:
    # Sinüzoidal hareket
    aci0 = 0.5 * math.sin(t)
    aci1 = 0.4 * math.cos(t * 1.2)

    eklem_hareket_ettir(0, aci0)
    eklem_hareket_ettir(1, aci1)

    # El pozisyonunu oku
    x, y, z = el_pozisyonunu_oku()

    # İşaretleyici güncelle
    isaretleyici_ciz(x, y, z)

    # Ekrana yazdır
    print(f"\rEl Pozisyonu: X={x:6.3f} Y={y:6.3f} Z={z:6.3f} | Süre: {time.time()-baslangic:.1f}s", end="")

    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Canlı izleme tamamlandı!")

# =====================================================
# ÖZET ve AÇIKLAMA
# =====================================================
print("\n" + "="*60)
print("FORWARD KİNEMATİK NEDİR?")
print("="*60)
print("""
Forward Kinematics (İleri Kinematik):
  Eklem açıları → El pozisyonu

Örnek:
  Girdi : Eklem 0 = 45°, Eklem 1 = 30°
  Çıktı : El pozisyonu = (X, Y, Z)

Matematiksel olarak:
  - Her eklem bir transformasyon matrisi oluşturur
  - Bu matrisler çarpılır
  - Sonuç: El'in 3D koordinatları

PyBullet bunu bizim için hesaplıyor:
  p.getLinkState(robot, link_index)

Gelecek derste:
  Kendi FK fonksiyonumuzu yazacağız (2-link basit robot için)
""")

print("="*60)
print("DERS 4 TAMAMLANDI!")
print("="*60)

p.disconnect()
