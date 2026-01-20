import os
import cv2
import numpy as np
import json
from math import atan2, degrees
from scipy.spatial.transform import Rotation as R

class VerticalGraspDetector:
    """
    VerticalGraspDetector
    ---------------------
    A robust solver for top-down grasping using RGB-D data. 
    """

    def __init__(self, config):
        print(">> [VerticalGrasp] Initializing Geometric Vertical Solver...")
        self.cfg = config
        self.output_dir = self.cfg.OUTPUT_DIR
        
        # Intrinsics for 2D -> 3D Projection
        self.fx = self.cfg.FX
        self.fy = self.cfg.FY
        self.cx = self.cfg.CX
        self.cy = self.cfg.CY
        self.factor_depth = self.cfg.DEPTH_FACTOR

    def calculate(self, color, depth, mask, label):
        """
        Calculates 6-DOF grasp candidates. Returns True if successful.
        """
        if mask is None or np.sum(mask) == 0:
            print(f"   !! [Error] Mask for '{label}' is empty or None.")
            return False

        # 1. Calculate Primary Orientation via PCA
        center, main_axis, side_axis = self._get_orientation_pca(mask)
        yaw_angle = self._compute_yaw(main_axis)
        pitch_angle = 0.0
        
        # Ensure target_center is actually ON the object mask.
        target_center = self._snap_to_mask(mask, center) 
        
        # Debug storage for visualization
        debug_points = []

        # ==============================================================================
        # OBJECT-SPECIFIC LOGIC
        # ==============================================================================
        if label == "spoon_orange" or label == "spoon":
            tip_plus = self._find_mask_boundary(mask, target_center, main_axis)
            tip_minus = self._find_mask_boundary(mask, target_center, -main_axis)
            
            width_plus = self._sample_average_width(mask, target_center, tip_plus, side_axis)
            width_minus = self._sample_average_width(mask, target_center, tip_minus, side_axis)
            
            if width_plus < width_minus:
                t_pos = target_center + (tip_plus - target_center) * 0.6
                target_center = self._snap_to_mask(mask, t_pos)
                yaw_angle += 180 
            else:
                t_pos = target_center + (tip_minus - target_center) * 0.6
                target_center = self._snap_to_mask(mask, t_pos)

            debug_points = [tuple(tip_plus.astype(int)), tuple(tip_minus.astype(int))]

        elif label == "chopstick":
            pitch_angle = self._compute_pitch_chopstick(depth, mask, target_center, main_axis)
            
        # -----------------------------------------------------------
        # MULTI-CANDIDATE GENERATION
        # -----------------------------------------------------------
        pos_offsets = [0, 20, -20]
        yaw_jitters = [0, 3, -3]
        pitch_jitters = [0, 5, -5]
        
        if label == "spoon_orange":
            yaw_jitters.extend([180, 183, 177])

        grasp_candidates = []

        for p_off in pos_offsets:
            for y_jit in yaw_jitters:
                for p_jit in pitch_jitters:
                    test_pixel = np.array(target_center) + main_axis * p_off
                    valid_pixel = self._snap_to_mask(mask, test_pixel)
                    x3d, y3d, z3d, final_uv = self._pixel_to_3d_robust(depth, valid_pixel, main_axis, mask)
                    
                    if z3d > 0:
                        candidate_yaw = (yaw_angle + y_jit) % 360
                        candidate_pitch = pitch_angle + p_jit
                        
                        candidate = {
                            "rank": len(grasp_candidates) + 1,
                            "score": round(1.0 - (abs(p_off)*0.01) - (abs(y_jit % 180)*0.01) - (abs(p_jit)*0.02), 3),
                            "label": label,
                            "pixel": {"x": int(final_uv[0]), "y": int(final_uv[1])},
                            "camera_coords": {"x": x3d, "y": y3d, "z": z3d},
                            "orientation_coords": {
                                "roll_deg": 180.0,
                                "pitch_deg": float(round(candidate_pitch, 2)),
                                "yaw_deg": float(round(candidate_yaw, 2))
                            }
                        }
                        grasp_candidates.append(candidate)
                        if len(grasp_candidates) >= 25: break
                if len(grasp_candidates) >= 25: break
            if len(grasp_candidates) >= 25: break

        if not grasp_candidates:
            print(f"   !! [Failure] No valid depth found for '{label}' candidates.")
            return False
            
        # Updated Visualization call with side_axis and mask for boundary drawing
        self._visualize_results(color, mask, center, target_center, main_axis, side_axis, label, debug_points)
        self._save_output(grasp_candidates)
        return True

    # -------------------------------------------------------------------------
    # SNAP-TO-MASK LOGIC 
    # -------------------------------------------------------------------------

    def _snap_to_mask(self, mask, point, search_radius=30):
        u, v = int(round(point[0])), int(round(point[1]))
        h, w = mask.shape
        if 0 <= u < w and 0 <= v < h and mask[v, u] > 0:
            return np.array([u, v])
        for r in range(1, search_radius):
            for du in range(-r, r + 1):
                for dv in [-r, r]:
                    nu, nv = u + du, v + dv
                    if 0 <= nu < w and 0 <= nv < h and mask[nv, nu] > 0:
                        return np.array([nu, nv])
                for dv in range(-r + 1, r):
                    nu, nv = u + r, v + dv
                    if 0 <= nu < w and 0 <= nv < h and mask[nv, nu] > 0:
                        return np.array([nu, nv])
                    nu, nv = u - r, v + dv
                    if 0 <= nu < w and 0 <= nv < h and mask[nv, nu] > 0:
                        return np.array([nu, nv])
        return point

    # -------------------------------------------------------------------------
    # DEPTH & PROJECTION HELPERS
    # -------------------------------------------------------------------------

    def _get_median_depth(self, depth_img, u, v, window=5):
        h, w = depth_img.shape
        u_m, v_m = int(u), int(v)
        patch = depth_img[max(0,v_m-2):min(h,v_m+3), max(0,u_m-2):min(w,u_m+3)]
        valid = patch[patch > 0]
        return float(np.median(valid)) if valid.size > 0 else 0.0

    def _pixel_to_3d_simple(self, depth_img, point):
        u, v = int(point[0]), int(point[1])
        if not (0 <= u < self.cfg.CAM_WIDTH and 0 <= v < self.cfg.CAM_HEIGHT):
            return 0, 0, 0
        z_raw = self._get_median_depth(depth_img, u, v)
        z = z_raw / self.factor_depth
        if z <= 0: return 0, 0, 0
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return round(x, 4), round(y, 4), round(z, 4)

    def _pixel_to_3d_robust(self, depth_img, center, main_axis, mask):
        x, y, z = self._pixel_to_3d_simple(depth_img, center)
        if z > 0: return x, y, z, center
        for dist in range(5, 150, 5):
            for direction in [1, -1]:
                test_pos = center + main_axis * (dist * direction)
                u, v = int(test_pos[0]), int(test_pos[1])
                if 0 <= u < mask.shape[1] and 0 <= v < mask.shape[0] and mask[v, u] > 0:
                    _, _, z_found = self._pixel_to_3d_simple(depth_img, test_pos)
                    if z_found > 0:
                        final_x = (center[0] - self.cx) * z_found / self.fx
                        final_y = (center[1] - self.cy) * z_found / self.fy
                        return round(final_x, 4), round(final_y, 4), round(z_found, 4), center
        return 0, 0, 0, center

    # -------------------------------------------------------------------------
    # GEOMETRIC PROBING HELPERS
    # -------------------------------------------------------------------------

    def _find_mask_boundary(self, mask, start_pt, direction):
        h, w = mask.shape
        curr = np.array(start_pt, dtype=float)
        last_valid = curr.copy()
        direction = direction / np.linalg.norm(direction)
        for _ in range(800):
            curr += direction * 2.0
            u, v = int(curr[0]), int(curr[1])
            if not (0 <= u < w and 0 <= v < h) or mask[v, u] == 0: return last_valid
            last_valid = curr.copy()
        return last_valid

    def _sample_average_width(self, mask, start, end, side_axis):
        widths = [self._get_width_at_pixel(mask, start + (end - start) * a, side_axis) for a in [0.25, 0.5, 0.75]]
        return np.mean(widths)

    def _get_width_at_pixel(self, mask, point, side_axis):
        h, w = mask.shape
        side_vec = side_axis / np.linalg.norm(side_axis)
        def reach_edge(direction):
            curr = np.array(point, dtype=float)
            for _ in range(200):
                curr += direction
                u, v = int(np.round(curr[0])), int(np.round(curr[1]))
                if not (0 <= u < w and 0 <= v < h) or mask[v, u] == 0: return curr
            return curr
        edge1 = reach_edge(side_vec)
        edge2 = reach_edge(-side_vec)
        return np.linalg.norm(edge1 - edge2)

    def _get_orientation_pca(self, mask):
        coords = np.column_stack(np.where(mask > 0))
        pts = np.array([[x, y] for y, x in coords], dtype=np.float32)
        mean, eigenvectors, _ = cv2.PCACompute2(pts, mean=None)
        return mean[0], eigenvectors[0], eigenvectors[1]

    def _compute_yaw(self, axis):
        return degrees(atan2(axis[1], axis[0]))

    def _compute_pitch_chopstick(self, depth_img, mask, center, main_axis, max_steps=120):
        main_axis = main_axis / np.linalg.norm(main_axis)
        h, w = mask.shape
        def find_edge(direction):
            pos = np.array(center, dtype=float)
            last_valid = pos.copy()
            for _ in range(max_steps):
                x, y = np.round(pos).astype(int)
                if x < 0 or x >= w or y < 0 or y >= h or mask[y, x] == 0: break
                last_valid = pos.copy()
                pos += direction
            return (int(last_valid[0]), int(last_valid[1]))
        p1 = find_edge(main_axis)
        p2 = find_edge(-main_axis)
        x1, y1, z1 = self._pixel_to_3d_simple(depth_img, p1)
        x2, y2, z2 = self._pixel_to_3d_simple(depth_img, p2)
        if z1 == 0 or z2 == 0: return 0.0
        dz = z1 - z2
        dist_2d = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        pitch = degrees(atan2(dz, dist_2d))
        return round(-pitch, 4)

    # -------------------------------------------------------------------------
    # OUTPUT & VISUALIZATION
    # -------------------------------------------------------------------------

    def _save_output(self, data):
        path = os.path.join(self.output_dir, "grasp_points.json")
        with open(path, 'w') as f: json.dump(data, f, indent=4)

    def _visualize_results(self, color, mask, pca_center, target_center, main_axis, side_axis, label, debug_pts):
        """
        Enhanced visualization including Main Axis, Side Axis, Boundary, and Targets.
        """
        vis = color.copy()
        
        # 1. Draw Mask Boundary (Contours)
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (255, 255, 255), 1)

        # 2. Draw Principal Axes (Green = Main, Red = Side) at PCA Center
        def draw_axis_line(img, origin, vec, color_val, length=100):
            p1 = (int(origin[0] - vec[0] * length), int(origin[1] - vec[1] * length))
            p2 = (int(origin[0] + vec[0] * length), int(origin[1] + vec[1] * length))
            cv2.line(img, p1, p2, color_val, 2)

        draw_axis_line(vis, pca_center, main_axis, (0, 255, 0), length=150)
        draw_axis_line(vis, pca_center, side_axis, (0, 0, 255), length=60)

        # 3. Draw original PCA Center (Blue)
        cv2.circle(vis, (int(pca_center[0]), int(pca_center[1])), 6, (255, 0, 0), -1)

        # 4. Draw Probing/Boundary endpoints (Magenta Crosses)
        for pt in debug_pts:
            cv2.drawMarker(vis, (int(pt[0]), int(pt[1])), (255, 0, 255), cv2.MARKER_CROSS, 15, 2)

        # 5. Draw the Final Grasp Target (Yellow with border)
        cv2.circle(vis, (int(target_center[0]), int(target_center[1])), 8, (0, 255, 255), -1)
        cv2.circle(vis, (int(target_center[0]), int(target_center[1])), 9, (0, 0, 0), 2)

        # 6. Information Text
        cv2.putText(vis, f"Object: {label}", (int(target_center[0]) + 20, int(target_center[1]) - 20), 
                    cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 255), 2)

        # 7. Legend
        overlay = vis.copy()
        cv2.rectangle(overlay, (10, 10), (300, 190), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, vis, 0.4, 0, vis)

        legend_items = [
            ((255, 0, 0), "PCA Center"),
            ((0, 255, 255), "Grasp Target "),
            ((255, 255, 255), "Mask Boundary (Contour)"),
            ((0, 255, 0), "Main Axis"),
            ((0, 0, 255), "Side Axis"),
            ((255, 0, 255), "Boundary Endpoints")
        ]

        for i, (color_val, text) in enumerate(legend_items):
            y_pos = 35 + (i * 25)
            if "Axis" in text:
                cv2.line(vis, (20, y_pos - 5), (45, y_pos - 5), color_val, 2)
            elif "Boundary" in text and "Contour" in text:
                cv2.rectangle(vis, (20, y_pos - 12), (45, y_pos), color_val, 1)
            else:
                cv2.circle(vis, (32, y_pos - 6), 5, color_val, -1)
            cv2.putText(vis, text, (60, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imwrite(os.path.join(self.output_dir, "grasp_analysis_debug.png"), vis)