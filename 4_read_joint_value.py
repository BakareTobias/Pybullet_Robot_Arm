"""
DERS 3: Eklem Durumunu Okuma
Robotun gerçek pozisyonunu ve hızını okuyacağız
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
print("✓ Robot yüklendi\n")

# =====================================================
# YENİ FONKSİYON: Eklem durumunu oku
# =====================================================
def eklem_durumunu_oku(eklem_no):
    """
    Bir eklemin gerçek durumunu okur

    Döndürür:
        aci: Eklemin mevcut açısı (radyan)
        hiz: Eklemin dönüş hızı (radyan/saniye)
    """
    durum = p.getJointState(robot, eklem_no)

    # durum bir tuple'dır (çoklu değer):
    # durum[0] = pozisyon (açı)
    # durum[1] = hız
    # durum[2] = reaksiyon kuvvetleri (şimdilik kullanmayız)
    # durum[3] = uygulanan tork

    aci = durum[0]
    hiz = durum[1]

    return aci, hiz

# =====================================================
# Eski fonksiyonlar
# =====================================================
def eklem_hareket_ettir(eklem_no, aci):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=eklem_no,
        controlMode=p.POSITION_CONTROL,
        targetPosition=aci
    )

def simule_et(adim_sayisi):
    for i in range(adim_sayisi):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# TEST 1: Başlangıç pozisyonunu oku
# =====================================================
print("="*50)
print("TEST 1: Başlangıç Pozisyonu")
print("="*50)

# İlk 3 eklemin durumunu oku
for eklem in range(4):
    aci, hiz = eklem_durumunu_oku(eklem)
    print(f"Eklem {eklem}:")
    print(f"  Açı: {aci:.3f} radyan = {math.degrees(aci):.1f} derece")
    print(f"  Hız: {hiz:.3f} rad/s")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 2: Hedef vs Gerçek Karşılaştırma
# =====================================================
print("\n" + "="*50)
print("TEST 2: Hedef vs Gerçek Açı")
print("="*50)

hedef_aci = 1.0  # ~57 derece

print(f"\nHedef: Eklem 0'ı {hedef_aci:.3f} radyana götür ({math.degrees(hedef_aci):.1f}°)")
eklem_hareket_ettir(0, hedef_aci)

# Hareket sırasında her 100 adımda durumu göster
print("\nHareket ediyor...")
for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    # Her 100 adımda bir rapor ver
    if adim % 100 == 0:
        aci, hiz = eklem_durumunu_oku(0)
        print(f"  Adım {adim:3d}: Açı={math.degrees(aci):6.2f}°, Hız={hiz:6.3f} rad/s")

# Final durumu
aci_final, hiz_final = eklem_durumunu_oku(0)
print(f"\n✓ Hareket tamamlandı!")
print(f"  Hedef açı : {math.degrees(hedef_aci):.2f}°")
print(f"  Gerçek açı: {math.degrees(aci_final):.2f}°")
print(f"  Fark      : {math.degrees(aci_final - hedef_aci):.2f}°")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 3: İki Eklemin Koordineli Hareketi
# =====================================================
print("\n" + "="*50)
print("TEST 3: İki Eklem Koordineli Hareket")
print("="*50)

# Hedefler
hedef_0 = 0.8   # ~46 derece
hedef_1 = -0.6  # ~-34 derece
hedef_2 = 1.57
hedef_3 = -1.048

print(f"\nHedefler:")
print(f"  Eklem 0: {math.degrees(hedef_0):.1f}°")
print(f"  Eklem 1: {math.degrees(hedef_1):.1f}°")

eklem_hareket_ettir(0, hedef_0)
eklem_hareket_ettir(1, hedef_1)
eklem_hareket_ettir(2, hedef_2)
eklem_hareket_ettir(3, hedef_3)

print("\nHareket ediyor...")
for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    if adim % 100 == 0:
        aci_0, hiz_0 = eklem_durumunu_oku(0)
        aci_1, hiz_1 = eklem_durumunu_oku(1)
        aci_2, hiz_2 = eklem_durumunu_oku(2)
        aci_3, hiz_3 = eklem_durumunu_oku(3)
        print(f"  Adım {adim:3d}:")
        print(f"    Eklem 0: {math.degrees(aci_0):6.2f}°")
        print(f"    Eklem 1: {math.degrees(aci_1):6.2f}°")
        print(f"    Eklem 2: {math.degrees(aci_2):6.2f}°")
        print(f"    Eklem 2: {math.degrees(aci_3):6.2f}°")

print("\n✓ Koordineli hareket tamamlandı!")

input("\nDevam için ENTER'a basın...")

# =====================================================
# TEST 4: Açıları Sıfırlama
# =====================================================
print("\n" + "="*50)
print("TEST 4: Açıları Sıfırlama")
print("="*50)

eklem_hareket_ettir(0, 0)
eklem_hareket_ettir(1, 0)
eklem_hareket_ettir(2, 0)
eklem_hareket_ettir(3, 0)

for adim in range(500):
    p.stepSimulation()
    time.sleep(1./240.)

    if adim % 100 == 0:
        aci_0, hiz_0 = eklem_durumunu_oku(0)
        aci_1, hiz_1 = eklem_durumunu_oku(1)
        aci_2, hiz_2 = eklem_durumunu_oku(2)
        aci_3, hiz_3 = eklem_durumunu_oku(3)
        print(f"  Adım {adim:3d}:")
        print(f"    Eklem 0: {math.degrees(aci_0):6.2f}°")
        print(f"    Eklem 1: {math.degrees(aci_1):6.2f}°")
        print(f"    Eklem 2: {math.degrees(aci_2):6.2f}°")
        print(f"    Eklem 2: {math.degrees(aci_3):6.2f}°")

print("\n✓ Açılar sıfırlandı tamamlandı!")

input("\nDevam için ENTER'a basın...")
# =====================================================
# TEST 5: Canlı İzleme (Gerçek Zamanlı)
# =====================================================
print("\n" + "="*50)
print("TEST 5: Canlı İzleme - Robot Sallanırken")
print("="*50)
print("10 saniye boyunca robot sallanacak")
print("Eklem durumları gerçek zamanlı gösterilecek\n")

baslangic = time.time()
t = 0

while time.time() - baslangic < 10:  # 10 saniye
    # Sinüzoidal hareket
    hedef_0 = 0.6 * math.sin(t)
    hedef_1 = 0.4 * math.cos(t * 1.3)

    eklem_hareket_ettir(0, hedef_0)
    eklem_hareket_ettir(1, hedef_1)

    # Gerçek durumu oku
    gercek_0, hiz_0 = eklem_durumunu_oku(0)
    gercek_1, hiz_1 = eklem_durumunu_oku(1)

    # Ekrana yazdır (aynı satırı güncelle)
    print(f"\rEklem 0: {math.degrees(gercek_0):6.1f}° | "
          f"Eklem 1: {math.degrees(gercek_1):6.1f}° | "
          f"Süre: {time.time()-baslangic:.1f}s", end="")

    p.stepSimulation()
    time.sleep(1./240.)
    t += 0.02

print("\n\n✓ Canlı izleme tamamlandı!")

# =====================================================
# ÖZET
# =====================================================
print("\n" + "="*50)
print("DERS 3 ÖZET")
print("="*50)
print("✓ Eklem durumunu okumayı öğrendik")
print("✓ Hedef vs gerçek açıları karşılaştırdık")
print("✓ Gerçek zamanlı izleme yaptık")
print("\nSonraki ders: Forward Kinematics")
print("(Eklem açılarından robot elinin pozisyonunu hesaplama)")
print("="*50)

p.disconnect()
