"""
DERS 2: İki Eklem Hareketi
İki eklemi hareket ettireceğiz - Basit robot kontrolü
"""

import pybullet as p
import pybullet_data
import time

# ADIM 1: Simülasyonu başlat (pencere aç)
p.connect(p.GUI)
print("✓ Pencere açıldı")

# ADIM 1.5: PyBullet'in robot klasörünü ekle (ÖNEMLİ!)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print("✓ Robot klasörü eklendi")

# ADIM 2: Kuka robotunu yükle
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
print("✓ Robot yüklendi")

# ADIM 3: Yerçekimini aç
p.setGravity(0, 0, -9.81)
print("✓ Yerçekimi ayarlandı")

# ADIM 4: İlk eklemi 45 dereceye götür
def eklem_hareket_ettir(eklem_no, aci):
    """
    Bir eklemi belirtilen açıya götürür

    eklem_no: Hangi eklem (0, 1, 2, ...)
    aci: Hedef açı (radyan cinsinden)
    """
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def simule_et(adim_sayisi):
    """
    Simülasyonu belirtilen adım kadar çalıştırır

    adim_sayisi: Kaç adım (örn: 500 = ~2 saniye)
    """
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: İki eklemi sırayla hareket ettir
# =====================================================
print("\n[TEST 1] Eklemleri sırayla hareket ettir")

print("  → Eklem 0'ı 45 dereceye götür...")
eklem_hareket_ettir(eklem_no=0, aci=0.785)
simule_et(500)

print("  → Eklem 1'i 30 dereceye götür...")
eklem_hareket_ettir(eklem_no=1, aci=0.524)  # 30 derece
simule_et(500)

print("  ✓ Test 1 tamamlandı\n")
input("Devam için ENTER'a basın...")

# =====================================================
# TEST 2: İki eklemi AYNI ANDA hareket ettir
# =====================================================

print("\n[TEST 2] İki eklemi aynı anda hareket ettir")

print("  → Her iki eklemi 0 dereceye getir (sıfırla)...")
eklem_hareket_ettir(0, 0)
eklem_hareket_ettir(1, 0)
simule_et(1000)

print("  ✓ Test 2 tamamlandı\n")
input("Devam için ENTER'a basın...")

# =====================================================
# TEST 3: Sürekli hareket (animasyon)
# =====================================================
print("\n[TEST 3] Sürekli hareket - Robotun sallanması")
print("Kapatmak için CTRL+C yapın\n")

import math

t = 0
try:
    while True:
        # Sinüs dalgası ile yumuşak hareket
        aci_0 = 0.5 * math.sin(t)
        aci_1 = 0.3 * math.cos(t * 1.5)

        eklem_hareket_ettir(0, aci_0)
        eklem_hareket_ettir(1, aci_1)

        p.stepSimulation()
        time.sleep(1./240.)

        t += 0.05 # Zamanı ilerlet

except KeyboardInterrupt:
    print("\n✓ Simülasyon durduruldu")

p.disconnect()
print("✓ PyBullet kapatıldı")
