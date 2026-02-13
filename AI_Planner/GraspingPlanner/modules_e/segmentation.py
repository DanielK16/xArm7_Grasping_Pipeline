import cv2
import numpy as np
import os
from ultralytics import SAM

class SamAnalyzer:
    def __init__(self, cfg):
        """
        Preload the model 
        """
        print(f">> [SAM] Loading Modell: {cfg.SAM_PATH}...")
        self.cfg = cfg
        self.container_labels = getattr(cfg, 'CONTAINER_LABELS', [])
        
        # Define output path of masks
        self.output_dir = os.path.join(cfg.OUTPUT_DIR, "masks")
        if not os.path.exists(self.output_dir): 
            os.makedirs(self.output_dir)

        try:
            self.model = SAM(cfg.SAM_PATH)
        except Exception as e:
            print(f"!! [SAM] error: {e}")
            self.model = None

    def _create_ring_mask(self, mask_uint8):
        """
        Creates a ring mask by eroding the filled contour.
        Useful for bowls and plates to target the rim.
        """
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return mask_uint8
        
        filled_base = np.zeros_like(mask_uint8)
        for cnt in contours:
            cv2.drawContours(filled_base, [cnt], -1, 255, thickness=-1)

        kernel_size = max(1, int(mask_uint8.shape[0] * 0.06))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        inner_mask = cv2.erode(filled_base, kernel)

        return cv2.subtract(filled_base, inner_mask)

    def _make_cup_bigger(self, mask_uint8):
        """
        Dilates the mask to make it slightly larger.
        Useful for small cups or thin edges.
        """
        kernel_size = max(1, int(mask_uint8.shape[0] * 0.03))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        bigger_mask = cv2.dilate(mask_uint8, kernel, iterations=1)
        return bigger_mask

    def _get_depth_stats(self, mask, depth_img):
            """
            Computes median depth of object
            """
            if depth_img is None: return 9999
            mask_bool = mask.astype(bool)
            valid_depths = depth_img[np.logical_and(mask_bool, depth_img > 0)]

            if valid_depths.size == 0: return 9999

            # Percentile filter: Remove outliers
            low = np.percentile(valid_depths, 5)
            high = np.percentile(valid_depths, 95)
            filtered = valid_depths[(valid_depths >= low) & (valid_depths <= high)]
        
            return np.median(filtered) if filtered.size > 0 else 9999

    def _check_stacking_order(self, objects, facts):
            """
            Check which objects are on top based on median depth of masks
            """
            for cont in objects:
                contains_list = cont.get('contains', [])
                if len(contains_list) > 1:
                    items_inside = []
                    for item_id in contains_list:
                        obj = next((o for o in objects if o['id'] == item_id), None)
                        if obj: items_inside.append(obj)
                    
                    valid_items = [o for o in items_inside if 0 < o.get('depth_median', 9999) < 9000]
                    
                    if len(valid_items) >= 2:
                        valid_items.sort(key=lambda x: x['depth_median'])
                        top_item = valid_items[0]
                        bottom_item = valid_items[1]
                        delta_mm = bottom_item['depth_median'] - top_item['depth_median']
                        
                        # ignore differences below 4mm
                        if delta_mm > 4:
                            fact = f"- STACKING: {top_item['label']} (ID {top_item['id']}) is ON TOP OF {bottom_item['label']} (ID {bottom_item['id']}) (Delta: {int(delta_mm)}mm)."
                            facts.append(fact)
                            print(f"   [GEO] {fact}") 

            return facts

    def _fill_holes_convex(self, mask_uint8):
        """
        fill holes convex (used for logic check 'is inside')
        """
        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours: return np.zeros_like(mask_uint8)
        all_points = np.vstack(contours)
        hull = cv2.convexHull(all_points)
        filled = np.zeros_like(mask_uint8)
        cv2.drawContours(filled, [hull], -1, 255, -1)
        return filled

    def _calc_overlap(self, inner, outer):
        """
        Computes percentage of overlap using bitwise AND operation
        """
        intersection = np.logical_and(inner, outer)
        return np.count_nonzero(intersection) / np.count_nonzero(inner) if np.count_nonzero(inner) > 0 else 0

    def analyze_scene(self, image, depth_img, objects): 

        if not self.model or not objects: return objects, "No SAM model or objects loaded."
            
        print(">> [SAM] Generating Masks and analysing geometry...")
        # 1. Create Mask from Bounding Box
        bboxes = [obj['bbox_pixel'] for obj in objects]
        results = self.model(image, bboxes=bboxes, verbose=False)
        
        # Safety Check
        if not results or results[0].masks is None: 
            return objects, "SAM failed, No masks created"
            
        # convert masks to numpy and load it into cpu
        masks_data = results[0].masks.data.cpu().numpy()
        h, w = image.shape[:2]
        
        processed = []
            
        # 2. Process Masks
        for i, raw in enumerate(masks_data):
            # convert raw masks into uint8 mask
            mask_u8 = (raw * 255).astype(np.uint8)
            if mask_u8.shape[:2] != (h, w):
                mask_u8 = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_NEAREST)
                
            label = objects[i]['label']
            label_lower = label.lower()
            is_container = any(k in label_lower for k in self.container_labels)

            # Debug Save First Mask
            if getattr(self.cfg, 'DEBUG_SAVE_MASKS', False):
                cv2.imwrite(os.path.join(self.output_dir, f"{label}_{i}_first.png"), mask_u8)

            # 2.1 FIRST: Create the FILLED mask (the logical base)
            if is_container:
                mask_filled_u8 = self._fill_holes_convex(mask_u8)
            else:
                mask_filled_u8 = mask_u8

            # 2.2 SECOND: Customize the mask for grasping
            # Check for Cups/Bottles -> Dilate
            if any(x in label_lower for x in getattr(self.cfg, 'GRASP_DILATE_LABELS', ["cup"])):
                mask_grasp_u8 = self._make_cup_bigger(mask_filled_u8)
                
            # Check for Bowls/Plates -> Ring Mask
            elif any(x in label_lower for x in getattr(self.cfg, 'GRASP_RING_LABELS', ["bowl", "plate"])):
                mask_grasp_u8 = self._create_ring_mask(mask_filled_u8)
            
            else:
                mask_grasp_u8 = mask_filled_u8

            # Save processed masks only in DEBUG mode
            if getattr(self.cfg, 'DEBUG_SAVE_MASKS', False):
                cv2.imwrite(os.path.join(self.output_dir, f"{label}_{i}_orig.png"), mask_grasp_u8)
                if is_container:
                    cv2.imwrite(os.path.join(self.output_dir, f"{label}_{i}_filled.png"), mask_filled_u8)
              
            # Convert to boolean for logic processing
            mask_orig_bool = mask_grasp_u8 > 0
            mask_filled_bool = mask_filled_u8 > 0
                
            # 2.3 Compute median depth
            median_z = self._get_depth_stats(mask_grasp_u8, depth_img)
            objects[i]['depth_median'] = median_z

            processed.append({'orig': mask_orig_bool, 'filled': mask_filled_bool, 'is_cont': is_container})
            objects[i]['relation'] = "ON TABLE"
            objects[i]['contains'] = []

        # 3. Compute Relations
        facts = []
        for i, inner in enumerate(objects):
            inner_logic_mask = processed[i]['filled']
            inner_area = np.count_nonzero(inner_logic_mask) 
                
            candidates = []

            for j, outer in enumerate(objects):
                if i == j or not processed[j]['is_cont']: continue
                    
                outer_filled_mask = processed[j]['filled']
                outer_area = np.count_nonzero(outer_filled_mask)
                  
                if inner_area >= outer_area: continue 

                ratio = self._calc_overlap(inner_logic_mask, outer_filled_mask)
                    
                if ratio > self.cfg.OVERLAP_THRESH:
                    candidates.append((j, ratio, outer_area))

            if candidates:
                candidates.sort(key=lambda x: x[2]) 
                best_idx = candidates[0][0]
                cont = objects[best_idx]
                    
                objects[best_idx]['contains'].append(inner['id'])
                    
                rel = f"INSIDE ID {cont['id']} ({cont['label']})"
                if len(candidates) > 1: rel += " (+others)"
                   
                inner['relation'] = rel
                facts.append(f"- {inner['label']} (ID {inner['id']}) is INSIDE {cont['label']} (ID {cont['id']}).")

        # 4. Compute Stacking order
        facts = self._check_stacking_order(objects, facts)

        return objects, "\n".join(facts) if facts else "No nested objects."
