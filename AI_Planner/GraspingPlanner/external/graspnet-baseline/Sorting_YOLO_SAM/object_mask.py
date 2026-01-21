from segment_anything import SamPredictor, sam_model_registry
import cv2
import torch
import numpy as np
import json

# --- Load image ---
image_path = "/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_image/color.png"
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

# --- YOLO bounding box (replace with actual YOLO output) ---
with open("/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/cup_box.json", "r") as f:
    bbox = json.load(f)

x1, y1, x2, y2 = bbox["x1"], bbox["y1"], bbox["x2"], bbox["y2"]

# --- Prepare SAM input from bounding box ---
# SAM expects center point + box size; here we can just use center as "foreground" point
center_x = (x1 + x2) // 2
center_y = (y1 + y2) // 2
input_point = np.array([[center_x, center_y]])
input_label = np.array([1])  # 1 = foreground

# --- Predict mask with SAM ---
masks, scores, logits = predictor.predict(
    point_coords=input_point,
    point_labels=input_label,
    multimask_output=True
)

# --- Pick the highest score mask ---
best_idx = np.argmax(scores)
mask = masks[best_idx]

# --- Save mask ---
mask_uint8 = (mask * 255).astype(np.uint8)
cv2.imwrite("/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_image/mask.png", mask_uint8)
print("Mask saved as 'mask.png'")
