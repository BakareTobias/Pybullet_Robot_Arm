import pybullet_data
import os

# Get the PyBullet data path
data_path = pybullet_data.getDataPath()
print(f"PyBullet data path: {data_path}")

# List all files and directories
print("\nAll robots and files:")
for item in os.listdir(data_path):
    item_path = os.path.join(data_path, item)
    if os.path.isdir(item_path):
        print(f"📁 {item}")
    else:
        print(f"📄 {item}")
