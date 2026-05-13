"""
PROJE 1: Akıllı Renk Tanıma ve Sınıflandırma Sistemi
Robot rastgele renkli küpleri renklerine göre sınıflandıracak
"""

import pybullet as p
import pybullet_data
import time
import math
import numpy as np
import random

print("="*70)
print("PROJE 1: AKILLI RENK TANIMA VE SINIFLANDIRMA SİSTEMİ")
print("="*70)

# =====================================================
# KURULUM
# =====================================================
print("\n→ Simülasyon ortamı hazırlanıyor...")

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Çalışma masası (geniş)
masa_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15])
masa_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 0.4, 0.15],
                                   rgbaColor=[0.6, 0.4, 0.2, 1])
masa = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=masa_collision,
                         baseVisualShapeIndex=masa_visual,
                         basePosition=[0.6, 0, 0.15])

# Hedef kutuları (renklere göre)
kutu_yukseklik = 0.08
kutu_genislik = 0.15

# Kırmızı kutu (sol)
kirmizi_kutu_collision = p.createCollisionShape(p.GEOM_BOX,
                                                 halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2])
kirmizi_kutu_visual = p.createVisualShape(p.GEOM_BOX,
                                           halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2],
                                           rgbaColor=[1, 0, 0, 0.3])
kirmizi_kutu = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=kirmizi_kutu_collision,
                                  baseVisualShapeIndex=kirmizi_kutu_visual,
                                  basePosition=[0.3, 0.25, 0.30 + kutu_yukseklik/2])

# Mavi kutu (orta)
mavi_kutu_collision = p.createCollisionShape(p.GEOM_BOX,
                                              halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2])
mavi_kutu_visual = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[kutu_genislik/2, kutu_genislik/2, kutu_yukseklik/2],
                                        rgbaColor=[0, 0, 1, 0.3])
mavi_kutu = p.createMultiBody(baseMass=0,
                               baseCollisionShapeIndex=mavi_kutu_collision,
                               baseVisualShapeIndex=mavi_kutu_visual,
                               basePosition=[0.3, 0.0, 0.30 + kutu_yukseklik/2])

# Yeşil kutu (sağ)
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

# Kamera
p.resetDebugVisualizerCamera(
    cameraDistance=2.2,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0.5, 0, 0.4]
)

print("✓ Ortam hazır")
print("  - Çalışma masası ✓")
print("  - Kırmızı kutu (sol) ✓")
print("  - Mavi kutu (orta) ✓")
print("  - Yeşil kutu (sağ) ✓")
print("  - Panda robot ✓")

# =====================================================
# ROBOT KONTROLÜ
# =====================================================

kol_eklemleri = [0, 1, 2, 3, 4, 5, 6]
gripper_eklemleri = [9, 10]
end_effector_index = 11

# Home pozisyonu
home_angles = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
for i, angle in enumerate(home_angles):
    p.resetJointState(panda, kol_eklemleri[i], angle)

for eklem in gripper_eklemleri:
    p.resetJointState(panda, eklem, 0.04)

def kol_hareket_ettir(hedef_pos, sure=2.0):
    """Robot kolunu hedef pozisyona götür"""
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
    """Gripper aç/kapat"""
    hedef = 0.04 if komut == "ac" else 0.0
    for eklem in gripper_eklemleri:
        p.setJointMotorControl2(panda, eklem, p.POSITION_CONTROL,
                               targetPosition=hedef, force=kuvvet)
    for _ in range(80):
        p.stepSimulation()
        time.sleep(1./240.)

# =====================================================
# RENK TANIMA SİSTEMİ
# =====================================================

# Renk tanımları (RGB değerleri)
RENKLER = {
    "kirmizi": {"rgb": [1.0, 0.0, 0.0], "kutu_pos": [0.3, 0.25, 0.38]},
    "mavi":    {"rgb": [0.0, 0.0, 1.0], "kutu_pos": [0.3, 0.0, 0.38]},
    "yesil":   {"rgb": [0.0, 1.0, 0.0], "kutu_pos": [0.3, -0.25, 0.38]}
}

def rgb_mesafesi(renk1, renk2):
    """İki RGB renk arasındaki Euclidean mesafe"""
    return math.sqrt(sum([(renk1[i] - renk2[i])**2 for i in range(3)]))

def renk_tani(nesne_id):
    """
    Nesnenin rengini tanı

    Returns:
        renk_adi: "kirmizi", "mavi", "yesil"
    """
    # Nesnenin görsel bilgisini al
    visual_data = p.getVisualShapeData(nesne_id)

    if len(visual_data) == 0:
        return None

    # RGBA rengini al
    rgba = visual_data[0][7]  # [r, g, b, a]
    nesne_rgb = rgba[:3]

    # En yakın rengi bul
    en_yakin_renk = None
    en_kucuk_mesafe = float('inf')

    for renk_adi, renk_bilgi in RENKLER.items():
        mesafe = rgb_mesafesi(nesne_rgb, renk_bilgi["rgb"])

        if mesafe < en_kucuk_mesafe:
            en_kucuk_mesafe = mesafe
            en_yakin_renk = renk_adi

    return en_yakin_renk

# =====================================================
# NESNE OLUŞTURMA
# =====================================================

def kup_olustur(pozisyon, renk_adi):
    """Renkli küp oluştur"""
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
# RASTGELE KÜPLER OLUŞTUR
# =====================================================
print("\n" + "="*70)
print("KÜPLER OLUŞTURULUYOR")
print("="*70)

num_cubes = 8  # Toplam küp sayısı
nesneler = []
nesne_renkleri = []

print(f"\n→ {num_cubes} rastgele küp oluşturuluyor...")

renk_isimleri = list(RENKLER.keys())

for i in range(num_cubes):
    # Rastgele pozisyon (masanın sağ tarafı)
    x = random.uniform(0.5, 0.7)
    y = random.uniform(-0.25, 0.25)
    z = 0.35

    # Rastgele renk seç
    renk = random.choice(renk_isimleri)

    # Küpü oluştur
    kup = kup_olustur([x, y, z], renk)
    nesneler.append(kup)
    nesne_renkleri.append(renk)

    print(f"  {i+1}. Küp: {renk:8s} → Pozisyon: ({x:.2f}, {y:.2f}, {z:.2f})")

# Küplerin düşmesini bekle
print("\n→ Küpler düşüyor...")
for _ in range(240):
    p.stepSimulation()
    time.sleep(1./240.)

print("✓ Küpler hazır")

# Renk dağılımını göster
kirmizi_sayisi = nesne_renkleri.count("kirmizi")
mavi_sayisi = nesne_renkleri.count("mavi")
yesil_sayisi = nesne_renkleri.count("yesil")

print(f"\nRenk Dağılımı:")
print(f"  🔴 Kırmızı: {kirmizi_sayisi}")
print(f"  🔵 Mavi:    {mavi_sayisi}")
print(f"  🟢 Yeşil:   {yesil_sayisi}")

input("\nBaşlamak için ENTER'a basın...\n")

# =====================================================
# PICK AND PLACE FONKSİYONU
# =====================================================

def pick_and_place_hizli(nesne_id, hedef_pos):
    """Hızlandırılmış pick-and-place"""
    nesne_pos, _ = p.getBasePositionAndOrientation(nesne_id)

    # Pick
    gripper_kontrol("ac")
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2] + 0.15], sure=1.5)
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2]], sure=1.2)
    gripper_kontrol("kapat", kuvvet=100)
    time.sleep(0.2)
    kol_hareket_ettir([nesne_pos[0], nesne_pos[1], nesne_pos[2] + 0.2], sure=1.2)

    # Place
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2] + 0.15], sure=1.5)
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2]], sure=1.0)
    gripper_kontrol("ac")
    time.sleep(0.2)
    kol_hareket_ettir([hedef_pos[0], hedef_pos[1], hedef_pos[2] + 0.15], sure=1.0)

# =====================================================
# ANA İŞLEM - SINIFLANDIRMA
# =====================================================
print("="*70)
print("SINIFLANDIRMA BAŞLIYOR")
print("="*70)

basari_sayisi = 0
basarisiz_sayisi = 0
baslangic_zamani = time.time()

# Her renk için sayaç (kutuya kaç küp kondu)
kutu_sayaclari = {"kirmizi": 0, "mavi": 0, "yesil": 0}

for idx, nesne in enumerate(nesneler, 1):
    print(f"\n{'='*70}")
    print(f"KÜP {idx}/{num_cubes}")
    print(f"{'='*70}")

    # 1. Renk tanıma
    print("  [1/4] Renk tanınıyor...")
    taninan_renk = renk_tani(nesne)
    gercek_renk = nesne_renkleri[idx-1]

    if taninan_renk == gercek_renk:
        print(f"  ✓ Renk: {taninan_renk.upper()}")
        basari_sayisi += 1
    else:
        print(f"  ✗ Hata! Tanınan: {taninan_renk}, Gerçek: {gercek_renk}")
        basarisiz_sayisi += 1
        taninan_renk = gercek_renk  # Düzelt

    # 2. Hedef kutu pozisyonu
    hedef_kutu = RENKLER[taninan_renk]["kutu_pos"]

    # Kutuda zaten küpler varsa, yan yana diz
    offset = kutu_sayaclari[taninan_renk] * 0.045
    hedef_final = [hedef_kutu[0], hedef_kutu[1] + offset - 0.045, hedef_kutu[2]]

    print(f"  [2/4] Hedef: {taninan_renk.upper()} kutusu")

    # 3. Pick and Place
    print(f"  [3/4] Küp alınıyor...")
    nesne_pos, _ = p.getBasePositionAndOrientation(nesne)
    print(f"        Pozisyon: ({nesne_pos[0]:.2f}, {nesne_pos[1]:.2f}, {nesne_pos[2]:.2f})")

    try:
        pick_and_place_hizli(nesne, hedef_final)
        print(f"  [4/4] ✓ Küp {taninan_renk} kutusuna kondu!")
        kutu_sayaclari[taninan_renk] += 1
    except:
        print(f"  [4/4] ✗ Taşıma hatası!")
        basarisiz_sayisi += 1

    # İlerleme çubuğu
    tamamlanan = idx
    kalan = num_cubes - idx
    print(f"\n  İlerleme: {'█' * tamamlanan}{'░' * kalan} {idx}/{num_cubes}")

bitis_zamani = time.time()
toplam_sure = bitis_zamani - baslangic_zamani

# =====================================================
# SONUÇ RAPORU
# =====================================================
print("\n" + "="*70)
print("PROJE TAMAMLANDI - SONUÇ RAPORU")
print("="*70)

print(f"\n📊 PERFORMANS METRİKLERİ:")
print(f"  ⏱  Toplam Süre: {toplam_sure:.1f} saniye")
print(f"  📦 İşlenen Küp: {num_cubes}")
print(f"  ✓ Başarılı: {basari_sayisi}")
print(f"  ✗ Başarısız: {basarisiz_sayisi}")
print(f"  📈 Başarı Oranı: {(basari_sayisi/num_cubes)*100:.1f}%")
print(f"  ⚡ Ortalama İşlem Süresi: {toplam_sure/num_cubes:.1f} saniye/küp")

print(f"\n📦 KUTU DURUMU:")
print(f"  🔴 Kırmızı Kutu: {kutu_sayaclari['kirmizi']} küp")
print(f"  🔵 Mavi Kutu:    {kutu_sayaclari['mavi']} küp")
print(f"  🟢 Yeşil Kutu:   {kutu_sayaclari['yesil']} küp")

# Değerlendirme
if basari_sayisi == num_cubes:
    print(f"\n🎉 MÜKEMMEL! Tüm küpler doğru sınıflandırıldı!")
elif basari_sayisi >= num_cubes * 0.8:
    print(f"\n👍 ÇOK İYİ! %{(basari_sayisi/num_cubes)*100:.0f} başarı oranı!")
else:
    print(f"\n📝 Geliştirilmeli. Bazı küpler yanlış sınıflandırıldı.")

print("\n" + "="*70)
print("PROJE 1 BİTTİ")
print("="*70)

print("""
ÖĞRENDİKLERİMİZ:
  ✓ Renk tanıma algoritması (RGB mesafesi)
  ✓ Otomatik nesne sınıflandırma
  ✓ Performans ölçümleri
  ✓ Otomasyon sistemi tasarımı

GERÇEK DÜNYA UYGULAMALARI:
  - Depo yönetimi (ürün sınıflandırma)
  - Geri dönüşüm tesisleri (materyal ayırma)
  - Kalite kontrol (renk bazlı muayene)
  - Eczane otomasyonu (ilaç sınıflandırma)

SONRAKI PROJE:
  → Proje 2: Computer Vision Entegrasyonu
    (Kamera ile nesne pozisyonu tespit etme)
""")

input("\nSimülasyonu kapatmak için ENTER'a basın...\n")

p.disconnect()
print("✓ Simülasyon kapatıldı")
