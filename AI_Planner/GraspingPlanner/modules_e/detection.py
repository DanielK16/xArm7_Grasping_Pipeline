import numpy as np
import os
from ultralytics import YOLOWorld

class YoloDetector:
    def __init__(self, cfg):
        print(f">> [YOLO] Loading Modell: {cfg.YOLO_PATH}")
        self.model = YOLOWorld(cfg.YOLO_PATH)
        self.cfg = cfg
        
        self.prompts = [
            #Chopsticks
            "a pair of long thin orange sticks used for eating food",
            "a pair of long thin green sticks used for eating food",
            "one long thin green chopstick",
            "one long thin orange chopstick",
            "thin green rod",
            "thin orange rod",
            "green plastic stick",
            "orange plastic stick",
            # Cup
            "a cup with handle",
            "a tilted cup or mug",
            "a cup lying at an angle",
            "an elliptical rim of a cup",
            "a cylindrical drinking vessel seen from the side",
            "the bottom of a cup",
            "cup with a utensil inside",
            #Spoon
            "orange spoon",
            "a short orange utensil with a rounded end for scooping food",
            "glossy orange plastic spoon",
            "orange spoon inside a container",
            # Bowl
            "circular plastic green bowl",
            "circular plastic orange bowl",
            # Apple
            "red apple",
            "green apple",
            # False Negatives
            "a tray used for eating food",
        ]
        self.model.set_classes(self.prompts)
        
        self.prompt_to_label = {
            #Chopsticks
            "a pair of long thin orange sticks used for eating food": "chopstick",
            "a pair of long thin green sticks used for eating food": "chopstick",
            "one long thin green chopstick": "chopstick",
            "one long thin orange chopstick": "chopstick",
            "thin green rod": "chopstick",
            "thin orange rod": "chopstick",
            "green plastic stick": "chopstick",
            "orange plastic stick": "chopstick",
            #Cup
            "a cup with handle": "cup",
            "a tilted cup or mug": "cup",
            "a cup lying at an angle": "cup",
            "an elliptical rim of a cup": "cup",
            "a cylindrical drinking vessel seen from the side": "cup",
            "the bottom of a cup": "cup",
            "cup with a utensil inside": "cup",
            # Spoon
            "orange spoon": "spoon_orange",
            "a short orange utensil with a rounded end for scooping food": "spoon_orange",
            "glossy orange plastic spoon": "spoon_orange",
            "orange spoon inside a container": "spoon_orange",
            #Bowl
            "circular plastic green bowl": "bowl_green",
            "circular plastic orange bowl": "bowl_orange",
            #Apple
            "red apple": "apple",
            "green apple": "apple",
            # False Negatives
            "a tray used for eating food": None, # ignore tray
        }
        
        self.class_colors = {
            "chopstick": (0, 165, 255),
            "bowl_orange": (0, 69, 255),
            "bowl_green": (0, 200, 0),
            "spoon_orange": (0, 0, 255),
            "cup": (255, 0, 0)
        }

    def detect(self, image):
        results = self.model.predict(
            source=image,
            conf=self.cfg.CONF_THRESH,
            iou=self.cfg.IOU_THRESHOLD, 
            imgsz=self.cfg.IMGSZ,
            device='cuda:0',
            half=self.cfg.HALF,
            verbose=self.cfg.VERBOSE,
            augment=self.cfg.AUGMENT,
            agnostic_nms=self.cfg.AGNOSTIC_NMS
        )
        return results[0].boxes

    def parse(self, boxes, img_width, img_height):
        raw_objects = []
        for box in boxes:
            cls_idx = int(box.cls[0].item())
            prompt = self.prompts[cls_idx]
            label = self.prompt_to_label.get(prompt, prompt)
            
            if label is None: continue 

            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            norm_bbox = [
                round(x1 / img_width, 4), round(y1 / img_height, 4),
                round(x2 / img_width, 4), round(y2 / img_height, 4)
            ]

            obj_data = {
                "label": label,
                "confidence": float(box.conf[0].item()),
                "bbox_pixel": [x1, y1, x2, y2],
                "bbox_norm": norm_bbox,
                "center_pixel": [cx, cy],
                "color": self.class_colors.get(label, (128, 128, 128)),
                "id": 0
            }
            raw_objects.append(obj_data)

        # Spatial NMS 
        if not raw_objects: return []
        
        raw_objects.sort(key=lambda x: x['confidence'], reverse=True)
        filtered = []
        for obj in raw_objects:
            is_dup = False
            for existing in filtered:
                dist = np.linalg.norm(np.array(obj['center_pixel']) - np.array(existing['center_pixel']))
                if dist < self.cfg.DUPLICATE_DIST:
                    is_dup = True
                    break
            if not is_dup:
                filtered.append(obj)
        
        for i, obj in enumerate(filtered): 
            obj['id'] = i
            
        return filtered
