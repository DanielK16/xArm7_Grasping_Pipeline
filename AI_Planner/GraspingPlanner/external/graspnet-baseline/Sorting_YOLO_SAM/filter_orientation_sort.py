import numpy as np


# File Paths
grasp_path = '/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_data/predicted_grasps.npy'

# Step 1: Load predicted grasps
grasp_array = np.load(grasp_path)  # shape: (N, 17)


def calculate_tilt_score(rotation_matrix):
    """
    Calculate the tilt score based on the rotation matrix.
    The third column of the rotation matrix represents the gripper's approach direction (grasp normal).
    """
    upright_vector = np.array([0, 0, 1])  # The upright vector (z-axis)

    grasp_normal = rotation_matrix[:, 0]  # Z-axis was swapped with X-axis, so first column was taken

    grasp_normal = grasp_normal / np.linalg.norm(grasp_normal)

    # Compute the tilt score (dot product between grasp normal and upright vector)
    tilt_score = np.dot(grasp_normal, upright_vector)

    return tilt_score
    
    
# Step 2: Filter grasps based on their tilt score
tilt_flags = []
tilt_threshold = 0.85 # Tilt threshold for keeping upright grasps (-1 to 1)


for grasp in grasp_array:
    # Extract rotation matrix (reshape from flat 9 values)
    R_flat = grasp[4:13]
    R = np.array(R_flat).reshape(3, 3)

    # Calculate the tilt score using the rotation matrix
    tilt_score = calculate_tilt_score(R)

    # Apply the tilt threshold
    if tilt_score >= tilt_threshold:
        tilt_flags.append([1])  # Keep the grasp
    else:
        tilt_flags.append([0])  # Discard the grasp

tilt_flags = np.array(tilt_flags)


# Step 3: Final appended array
grasp_with_orientation_filter = np.hstack((grasp_array, tilt_flags))  # shape: (N, 18)

# Save or use as needed
np.save(grasp_path, grasp_with_orientation_filter)
reloaded = np.load(grasp_path)
print(reloaded.shape)  # Should be (N, 21)

