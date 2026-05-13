"""
DERS 1: Tek Eklem Hareketi
Sadece 1 eklemi hareket ettireceğiz - En basit robot kontrolü
"""

import pybullet as p
import pybullet_data
import time

# ADIM 1: Simülasyonu başlat (pencere aç)
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
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
print("\nRobot ilk eklemini 45 dereceye hareket ettiriyor...")

# 45 derece = 0.785 radyan
target_angle = 0.785

# Eklem 0'ı (ilk eklem) kontrol et
p.setJointMotorControl2(
    bodyUniqueId=robot,      # Hangi robot
    jointIndex=1,            # Hangi eklem (0 = ilk)
    controlMode=p.POSITION_CONTROL,  # "Bu açıya git" modu
    targetPosition=target_angle  # Hedef açı
)

# ADIM 5: Simülasyonu çalıştır (robot hareket edecek)
for i in range(1000):
    p.stepSimulation()   # Fiziği 1 adım ilerlet
    time.sleep(1./240.)  # Yavaş görmek için bekle

print("✓ Hareket tamamlandı!")
print("\nPencereyi kapatmak için CTRL+C yapın")

# Pencereyi açık tut
while True:
    p.stepSimulation()
    time.sleep(1./240.)
