"""
PyBullet Kurulum Test Kodu
Bu kod basitçe bir pencere açar ve bir küp düşürür.
Amaç: PyBullet'in doğru kurulduğunu görmek
"""

import pybullet as p
import pybullet_data
import time

# ADIM 1: Simülasyon penceresini aç (GUI = Graphical User Interface)
# p.DIRECT kullanırsanız pencere açılmaz, sadece hesaplamalar yapılır
physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # Görüntüyü basitleştirir
print("PyBullet başarıyla başlatıldı!")

# ADIM 2: PyBullet'in hazır model klasörünü ekle
# Bu klasörde zemin, arabalar, robotlar gibi hazır modeller var
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# ADIM 3: Yerçekimini ayarla (m/s^2)
# Dünya'da z ekseni -9.81 (aşağı doğru)
p.setGravity(0, 0, -2)

# ADIM 4: Zemin ekle (düz bir platform)
planeId = p.loadURDF("plane.urdf")
print("Zemin eklendi")

# ADIM 5: Bir küp ekle (başlangıç pozisyonu: x=0, y=0, z=1)
# URDF = Unified Robot Description Format (robot tanım dosyası)
cubeStartPos = [0, 0, 5]  # 1 metre yükseklikte
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Dönüş yok
boxId = p.loadURDF("cube.urdf", cubeStartPos, cubeStartOrientation)
print("Küp eklendi, düşecek...")

# ADIM 6: Simülasyonu çalıştır (240 adım = yaklaşık 1 saniye)
# Her stepSimulation() çağrısı fiziği 1/240 saniye ilerletir
for i in range(240):
    p.stepSimulation()  # Fiziği bir adım ilerlet
    time.sleep(1./240.)  # Gerçek zamanlı görmek için bekle

# ADIM 7: Küpün son pozisyonunu al
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Küpün son pozisyonu: {cubePos}")
print("Zemine düştü! (z koordinatı ~0.5 olmalı, küpün yüksekliği 0.5m)")

# Pencereyi açık tut
print("\nPencereyi kapatmak için CTRL+C yapın")
while True:
    p.stepSimulation()
    time.sleep(1./240.)
