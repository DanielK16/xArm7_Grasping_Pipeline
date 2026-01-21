import numpy as np
from graspnetAPI import GraspGroup

# Load predicted grasps
grasps = np.load('/home/rric/GRASP/CLEAN_PROCESS/scene_data/predicted_grasps.npy', allow_pickle=True)

# Convert to GraspGroup object
gg = GraspGroup(grasps)

# Apply NMS and sort
gg.nms()
gg.sort_by_score()

# Top grasp
top_grasp = gg[0]

# Access the 21st element (Python is 0-indexed)
mask20_grasp = grasps[19]

print("Content of line 20:")
print(mask20_grasp)

print("Top Grasp Position:", top_grasp.translation)

# Correct way to get rotation
if hasattr(top_grasp, 'rotation_matrix'):
    print("Top Grasp Rotation Matrix:\n", top_grasp.rotation_matrix)
else:
    # fallback: use get_pose() which returns 4x4 homogeneous matrix
    pose = top_grasp.get_pose()
    print("Top Grasp Pose (homogeneous 4x4):\n", pose)

print("Top Grasp Width:", top_grasp.width)
print("Top Grasp Score:", top_grasp.score)
