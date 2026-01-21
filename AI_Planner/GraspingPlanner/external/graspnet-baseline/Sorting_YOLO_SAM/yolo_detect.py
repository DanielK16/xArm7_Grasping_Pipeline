from ultralytics import YOLO
import cv2
import numpy as np
import json
import os

# Load pretrained YOLOv8 model
model = YOLO("/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/yolo11n.pt")  

# Load your image
image_path = "/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_image/color.png"
frame = cv2.imread(image_path)
if frame is None:
    raise FileNotFoundError(f"Image not found at {image_path}")


# Run YOLO detection
class_names = model.names
results = model(frame)

# Collect detections
highest_confidence = 0
best_box = None
best_class = None

for r in results:
    boxes = r.boxes.xyxy.cpu().numpy()
    scores = r.boxes.conf.cpu().numpy()
    classes = r.boxes.cls.cpu().numpy()

    for box, score, cls_id in zip(boxes, scores, classes):
        label_name = class_names[int(cls_id)]
        if label_name.lower() != "cup":  # filter for cups only
            continue
        if score > highest_confidence:
            highest_confidence = score
            best_box = box
            best_class = label_name

if best_box is None:
    raise ValueError("No cup detected in the image.")

x1, y1, x2, y2 = map(int, best_box)
print(f"Best detection: {best_class} with confidence {highest_confidence:.2f}")

# --- Draw bounding box on image ---
cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
cv2.putText(frame, f"{best_class}: {highest_confidence:.2f}", 
            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

import json

# Example bounding box from YOLO
bbox = {
    "x1": x1,
    "y1": y1,
    "x2": x2,
    "y2": y2,
    "label": best_class,
    "confidence": float(highest_confidence)
}

# Save to JSON
json_path = "/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/cup_box.json"
with open(json_path, "w") as f:
    json.dump(bbox, f)

print(f"Bounding box saved to {json_path}")

# --- Save and display image ---
output_path = "/home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/Scene_image/scene_detected.jpg"
cv2.imwrite(output_path, frame)
print(f"Detection saved to {output_path}")

cv2.imshow("YOLO Detection", frame)
cv2.waitKey(100000)
cv2.destroyAllWindows()
