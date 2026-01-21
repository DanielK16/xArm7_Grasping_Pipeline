import numpy as np
import cv2

# File Paths
grasp_path = 'scene_data/predicted_grasps.npy'
mask_path = 'scene_data/mask.png'


# Step 1: Load predicted grasps
grasp_array = np.load(grasp_path)  # shape: (N, 20)


# Step 2: Read the black & white mask image
mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)  # White=255, Black=0
height, width = mask.shape

# Step 3: Extract uv coords (columns 18,19 since 0-indexed)
uv_array = grasp_array[:, 18:20].astype(int)

# Step 4: Check each (u, v) against the mask
mask_flags = []
for u, v in uv_array:
    if 0 <= v < height and 0 <= u < width:
        mask_flags.append(1 if mask[v, u] == 255 else 0)
    else:
        mask_flags.append(0)
        
mask_flags = np.array(mask_flags).reshape(-1, 1)

# Step 5: Append mask flag as new column
grasp_with_mask_filter = np.hstack((grasp_array, mask_flags))  # shape (N,21)

# Save or use as needed
np.save(grasp_path, grasp_with_mask_filter)
reloaded = np.load(grasp_path)
print(reloaded.shape)  # Should be (N, 21)

