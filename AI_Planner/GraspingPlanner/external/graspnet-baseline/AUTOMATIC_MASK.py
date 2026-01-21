#Mask automatically
#Choose the point with the highest condidence score from predicted_grasps.npy

from segment_anything import SamPredictor, sam_model_registry
import numpy as np
import matplotlib.pyplot as plt
import cv2
import torch
from screeninfo import get_monitors  # <-- added import
import matplotlib.pyplot as plt

# --- Load and prepare image ---
image_path="/home/rric/GRASP/graspnet-baseline/scene_images/color.png"
image = cv2.imread(image_path)
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# --- Load SAM model ---
sam_checkpoint = "/home/rric/segment-anything/checkpoint/sam_vit_b.pth"
model_type = "vit_b"
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
sam.to(device=device)

predictor = SamPredictor(sam)
predictor.set_image(image)

#Choose Top Grasp Point:
grasp_array = np.load("/home/rric/GRASP/graspnet-baseline/scene_data/predicted_grasps.npy")
#Filter after highest score and Orientation Filter [17]=1
top_grasp = None
for grasp in grasp_array:
    if grasp[17] == 1:  # Orientation filter ok
        u, v = int(grasp[18]), int(grasp[19])
        if 20 < u < 1260 and 20 < v < 700:  # Inside safe image area
            top_grasp = grasp
            break  # first valid grasp
        else:
            print(f"Skipped grasp at ({u},{v}) - outside image region")

if top_grasp is None:
    raise ValueError("No valid grasp found within the allowed region!")


# Pixel-Koordinaten im Bild
u, v = int(top_grasp[18]), int(top_grasp[19])
input_point = np.array([[u, v]])
input_label = np.array([1])  # 1 = foreground

#Show Choosen Point:
plt.imshow(image)
plt.scatter(u, v, color='red', s=50)
plt.show(block=False)
plt.pause(3)         
plt.close()            

#Generate Mask
masks, scores, logits = predictor.predict(
    point_coords=input_point,
    point_labels=input_label,
    multimask_output=True
)

# pick the mask with the highest confidence
best_idx = np.argmax(scores)  
mask = masks[best_idx]

# --- Save mask ---
mask_uint8 = (mask * 255).astype('uint8')
# Expand mask outline outward by 10 pixels
kernel_size = 35 #normally 20
kernel = np.ones((kernel_size, kernel_size), np.uint8)
cv2.imwrite('/home/rric/GRASP/graspnet-baseline/scene_data/normal_mask.png', mask_uint8)
expanded_mask = cv2.dilate(mask_uint8, kernel, iterations=2)
cv2.imwrite('/home/rric/GRASP/graspnet-baseline/scene_data/mask.png', expanded_mask)
