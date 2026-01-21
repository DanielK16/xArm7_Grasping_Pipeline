import numpy as np


# File Paths
grasp_path = '/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_data/predicted_grasps.npy'

# Step 1: Load predicted grasps
grasp_array = np.load(grasp_path)  # shape: (N, 18)

# Step 2: Define camera intrinsics color camera
fx = 912.0184936523438
fy = 911.9789428710938
cx = 652.2272338867188
cy = 377.44488525390625

# Step 2: Define camera intrinsics depth camera
# fx = 641.1640625
# fy = 641.1640625
# cx = 642.5853271484575
# cy = 359.3363037109375

def camera_xyz_to_pixel_uv(x, y, z):
    """Convert camera 3D coordinates to pixel coordinates."""
    u = (x * fx / z) + cx
    v = (y * fy / z) + cy
    return int(round(u)), int(round(v))

# Step 3: Append (u, v) to each grasp
uv_list = []
for grasp in grasp_array:
    x, y, z = grasp[13:16]  # Translation vector
    u, v = camera_xyz_to_pixel_uv(x, y, z) # Pixel coordinates
    uv_list.append([u, v])

uv_array = np.array(uv_list)
grasp_with_uv = np.hstack((grasp_array, uv_array))  # shape: (N, 20)


# Save or use as needed
np.save(grasp_path, grasp_with_uv)
reloaded = np.load(grasp_path)
print(reloaded.shape)  # Should be (N, 21)

