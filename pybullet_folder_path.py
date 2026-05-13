import pybullet_data
import os

# PyBullet data yolunu al
data_path = pybullet_data.getDataPath()
print(f"PyBullet data path: {data_path}")

# Tüm dosya ve klasörleri listele
print("\nTüm robotlar ve dosyalar:")
for item in os.listdir(data_path):
    item_path = os.path.join(data_path, item)
    if os.path.isdir(item_path):
        print(f"📁 {item}")
    else:
        print(f"📄 {item}")
