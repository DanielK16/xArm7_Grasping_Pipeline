import os
import sys
import numpy as np
import torch
import cv2
import json
import open3d as o3d
from scipy.spatial.transform import Rotation as R
try:
    from screeninfo import get_monitors
except ImportError:
    get_monitors = None


# =========================================================================
# PATH SETUP
# =========================================================================
BASELINE_ROOT = '/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/external/graspnet-baseline'
API_ROOT = '/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/external/graspnetAPI'

def setup_paths():
    if API_ROOT not in sys.path: 
        sys.path.append(API_ROOT)
    models_path = os.path.join(BASELINE_ROOT, 'models')
    if models_path not in sys.path: 
        sys.path.insert(0, models_path)
    sys.path.append(os.path.join(BASELINE_ROOT, 'dataset'))
    sys.path.append(os.path.join(BASELINE_ROOT, 'utils'))

setup_paths()

try:
    import graspnet
    from geometry_msgs.msg import PoseStamped
    from graspnetAPI import GraspGroup
    from graspnet import GraspNet, pred_decode
    from collision_detector import ModelFreeCollisionDetector
    from data_utils import CameraInfo, create_point_cloud_from_depth_image
except ImportError as e:
    raise ImportError(f"[ERROR] GraspNet Dependencies missing: {e}")

# =========================================================================
# GRASPNET CLASS
# =========================================================================

class GraspNetGrasp:
    def __init__(self, config):
        """
        Inititalise GraspNet
        """
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        print(f">> [GraspNet] Initializing on {self.device}...")
        
        self.cfg = config 
        self.output_dir = self.cfg.OUTPUT_DIR

        # --- Parameters ---
        self.num_point = self.cfg.NUM_POINT
        self.num_view = 300
        self.voxel_size = self.cfg.VOXEL_SIZE
        self.approach_dist = self.cfg.APPROACH_DIST
        self.collision_thresh = self.cfg.COLLISON_THRESH

        # --- Axis Swap Matrix ---
        self.swap_matrix = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])

        # --- Intrinsics ---
        self.width = self.cfg.CAM_WIDTH
        self.height = self.cfg.CAM_HEIGHT
        self.fx = self.cfg.FX
        self.fy = self.cfg.FY
        self.cx = self.cfg.CX
        self.cy = self.cfg.CY
        self.factor_depth = self.cfg.DEPTH_FACTOR
        
        self.min_depth = self.cfg.WORKSPACE_MIN_Z
        self.max_depth = self.cfg.WORKSPACE_MAX_Z

        self.camera_info = CameraInfo(
            self.width, self.height, self.fx, self.fy, self.cx, self.cy, self.factor_depth
        )

        # --- Load Model ---
        print(">> [GraspNet] Building Network...")
        self.net = GraspNet(input_feature_dim=0, num_view=self.num_view, num_angle=12, num_depth=4,
                            cylinder_radius=self.cfg.CYLINDER_RADIUS, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        self.net.to(self.device)
        
        ckpt_path = self.cfg.GRASPNET_CHECKPOINT 
        if not os.path.exists(ckpt_path):
            raise FileNotFoundError(f"Checkpoint not found at: {ckpt_path}")
            
        checkpoint = torch.load(ckpt_path, map_location=self.device)
        self.net.load_state_dict(checkpoint['model_state_dict'])
        self.net.eval() 
        print(">> [GraspNet] Model loaded.")

    def calculate(self, rgb, depth, mask=None, label=None):
        """
        Computes and Filters Grasps
        """
        # set max grasp width (important for grasp success: idk exactly why)
        max_width = 0.08
        if label == "bowl_orange" or label == "bowl_green":
            max_width = self.cfg.BOWL_MAX_GRASP_WIDTH
        elif label == "cup":
            max_width = self.cfg.CUP_MAX_GRASP_WIDTH
        elif label == "plate":
            max_width = self.cfg.PLATE_MAX_GRASP_WIDTH
        graspnet.GRASP_MAX_WIDTH = max_width

        # Choose Tilt Orientation for label
        tilt = (0.5,1.0)    #default tilt
        if label == "bowl_orange" or label == "bowl_green" or label== "bowl":
            tilt= (self.cfg.ORIENTATION_BOWL_MIN, self.cfg.ORIENTATION_BOWL_MAX)
        elif label == "cup":
            tilt = (self.cfg.ORIENTATION_CUP_MIN, self.cfg.ORIENTATION_CUP_MAX)
        elif label == "plate":
            tilt = (self.cfg.ORIENTATION_PLATE_MIN, self.cfg.ORIENTATION_PLATE_MAX)
        tilt_min ,tilt_max = tilt


        # 1. Preprocessing
        color = rgb.astype(np.float32) / 255.0
        depth_m = depth.astype(np.float32) / self.factor_depth
        depth_mask = (depth_m > self.min_depth) & (depth_m < self.max_depth)
        # 1.1 Create point Cloud
        cloud = create_point_cloud_from_depth_image(depth, self.camera_info, organized=True)
        cloud_masked = cloud[depth_mask]
        color_masked = color[depth_mask]

        if len(cloud_masked) == 0: return [], [], []

        if len(cloud_masked) >= self.num_point:
            idxs = np.random.choice(len(cloud_masked), self.num_point, replace=False)
        else:
            idxs = np.concatenate([np.arange(len(cloud_masked)), np.random.choice(len(cloud_masked), self.num_point - len(cloud_masked), replace=True)])
        
        cloud_sampled = cloud_masked[idxs]
        color_sampled = color_masked[idxs]
        cloud_tensor = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32)).to(self.device)
        end_points = {'point_clouds': cloud_tensor, 'cloud_colors': color_sampled}

        # 2. Inference
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)

        # 3. Collision
        if self.collision_thresh > 0:
            cloud_o3d = o3d.geometry.PointCloud()
            cloud_o3d.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
            mfcdetector = ModelFreeCollisionDetector(np.asarray(cloud_o3d.points), voxel_size=self.voxel_size)
            collision_mask = mfcdetector.detect(gg, approach_dist=self.approach_dist, collision_thresh=self.collision_thresh)
            gg = gg[~collision_mask]

        gg = gg.sort_by_score()
        
        # --- INFO PRINT: INITIAL COUNT ---
        print(f"   [GraspNet] Initial Raw Grasps found (Collision Checked): {len(gg)}")

        # 4.1 Orientation Flag
        tilt_flags = self._compute_tilt_flags(gg, tilt_min, tilt_max)
        # 4.2 UV Coords
        uv_coords  = self._compute_uv_coords(gg)
        # 4.3 Mask flag
        mask_flags = self._compute_mask_flags(uv_coords, mask)

        # 5. Save Internal Array
        final_data = self._save_predictions(gg, tilt_flags, uv_coords, mask_flags)

        # 6. Return ROS Poses
        poses, scores = self._to_ros_poses(gg)
        
        return poses, scores, final_data

    
    def process_and_save_grasps(self, final_data, label, filter_mode='orientation_mask'):
        """
        takes final_data, and saves all valid grasps inside a list
        Filtering of grasps
        """
        print(f">> [Post-Process] Filtering all grasps with mode: {filter_mode}")
        
        # --- DEBUG STATS CALCULATION ---
        total_input = final_data.shape[0]
        # column 17: orientation
        passed_orientation = np.sum(final_data[:, 17] == 1)
        # column 20: mask
        passed_mask = np.sum(final_data[:, 20] == 1)
        
        # 1. Filter
        # final_data: [0-16 Raw | 17 TiltFlag | 18 U | 19 V | 20 MaskFlag]
        
        filtered = []
        if filter_mode == 'raw':
            filtered = final_data
        elif filter_mode == 'orientation_only':
            filtered = final_data[final_data[:, 17] == 1]
        elif filter_mode == 'mask_only':
            filtered = final_data[final_data[:, 20] == 1]
        elif filter_mode == 'orientation_mask':
            filtered = final_data[(final_data[:, 17] == 1) & (final_data[:, 20] == 1)]
        else:
            filtered = final_data
            
        # --- INFO PRINT: DETAILED FILTER STATS ---
        count_after = filtered.shape[0]
        
        print(f"\n" + "-"*40)
        print(f"   [Filter Debug Stats] Label: {label}")
        print(f"   Input Grasps:        {total_input}")
        print(f"   Passed Orientation:  {passed_orientation}  (Removed: {total_input - passed_orientation})")
        print(f"   Passed SAM Mask:     {passed_mask}  (Removed: {total_input - passed_mask})")
        print(f"   Combined (Final):    {count_after}")
        print("-"*40 + "\n")

        if filtered.shape[0] == 0:
            print(f"   !! [Post-Process] No valid grasps found after filtering!")
            return False

        # 2. list of all valid grasps
        saved_grasps = []
        
        # iterate throug hall grasps
        for i in range(filtered.shape[0]):
            grasp = filtered[i]
            
            score = grasp[0]
            translation = grasp[13:16]  # XYZ
            rotation_flat = grasp[4:13] # Rot Matrix flach
            rotation_matrix = rotation_flat.reshape(3, 3)
            pixel_x = int(grasp[18])
            pixel_y = int(grasp[19])

            # 3. Transformations
            # Axis Swap
            rotation_matrix = rotation_matrix @ self.swap_matrix

            # Euler Angles
            r = R.from_matrix(rotation_matrix)
            roll_deg, pitch_deg, yaw_deg = r.as_euler('xyz', degrees=True)

            # 4. Dictionary 
            grasp_entry = {
                "rank": i + 1, # rank based on score
                "score": float(score),
                "label": label,
                "pixel": {"x": pixel_x, "y": pixel_y},
                "camera_coords": {
                    "x": float(translation[0]),
                    "y": float(translation[1]),
                    "z": float(translation[2])
                },
                "orientation_coords": {
                    "roll_deg": float(roll_deg),
                    "pitch_deg": float(pitch_deg),
                    "yaw_deg": float(yaw_deg)
                }
            }
            saved_grasps.append(grasp_entry)

        # 5. Save as json 
        json_path = os.path.join(self.output_dir, "grasp_points.json") # Plural, da Liste
        with open(json_path, 'w') as f:
            json.dump(saved_grasps, f, indent=4)

        print(f"   >> [Post-Process] Success! Saved {len(saved_grasps)} grasps to {json_path}")
        return True

    # -----------------------------------------------------------------------------------
    # INTERNE HELPER
    # -----------------------------------------------------------------------------------
    
    def _compute_tilt_flags(self, gg, t_min, t_max):
        rots = gg.rotation_matrices
        tilt_scores = rots[:, 2, 0] 
        flags = np.where((tilt_scores >= t_min) & (tilt_scores <= t_max), 1, 0)
        return flags.reshape(-1, 1)

    def _compute_uv_coords(self, gg):
        trans = gg.grasp_group_array[:, 13:16]
        x, y, z = trans[:, 0], trans[:, 1], trans[:, 2]
        z[z == 0] = 0.001
        u = (x * self.fx / z) + self.cx
        v = (y * self.fy / z) + self.cy
        uv = np.vstack((u, v)).T
        return np.rint(uv).astype(np.int32)

    def _compute_mask_flags(self, uv_coords, mask):
        if mask is None: return np.ones((len(uv_coords), 1))
        height, width = mask.shape
        flags = []
        for u, v in uv_coords:
            if 0 <= v < height and 0 <= u < width:
                flags.append(1 if mask[v, u] > 0 else 0)
            else:
                flags.append(0)
        return np.array(flags).reshape(-1, 1)

    def _save_predictions(self, gg, tilt_flags, uv_coords, mask_flags):
        if not os.path.exists(self.output_dir): os.makedirs(self.output_dir)
        save_path = os.path.join(self.output_dir, 'predicted_grasps.npy')
        raw_data = gg.grasp_group_array
        final_data = np.hstack((raw_data, tilt_flags, uv_coords, mask_flags))
        np.save(save_path, final_data)
        return final_data

    def _to_ros_poses(self, gg):
        poses, scores = [], []
        for i in range(len(gg)):
            g = gg[i]
            r = R.from_matrix(g.rotation_matrix)
            q = r.as_quat()
            p = PoseStamped()
            p.header.frame_id = "camera_link"
            p.pose.position.x = float(g.translation[0])
            p.pose.position.y = float(g.translation[1])
            p.pose.position.z = float(g.translation[2])
            p.pose.orientation.x = float(q[0])
            p.pose.orientation.y = float(q[1])
            p.pose.orientation.z = float(q[2])
            p.pose.orientation.w = float(q[3])
            poses.append(p)
            scores.append(float(g.score))
        return poses, scores

    def visualize_results(self, color, depth, final_data, filter_mode='orientation_mask'):
            """
            """
            if not self.cfg.VISUALIZE_GRASPS:
                return

            print(f">> [VIS] Preparing 3D Visualization (Mode: {filter_mode})...")
            print("   >> Close the window to continue...")

            # 1. Filter Logic
            if filter_mode == 'orientation_only':
                filtered = final_data[final_data[:, 17] == 1]
            elif filter_mode == 'mask_only':
                filtered = final_data[final_data[:, 20] == 1]
            elif filter_mode == 'orientation_mask':
                filtered = final_data[(final_data[:, 17] == 1) & (final_data[:, 20] == 1)]
            elif filter_mode == 'raw':
                # Zeigt alles an, was GraspNet generiert hat
                filtered = final_data
                print(f"   [VIS] Showing ALL {len(filtered)} grasps.")
            else:
                # Fallback
                filtered = final_data
                print(f"   [VIS] Unknown mode '{filter_mode}', showing all.")

            if len(filtered) == 0:
                print("   [VIS] No grasps found for this filter.")
                return

            # 2. Point Cloud erstellen (direkt aus RAM)
            c_norm = color.astype(np.float32) / 255.0
            cloud_npy = create_point_cloud_from_depth_image(depth, self.camera_info, organized=True)
            
            d_m = depth.astype(np.float32) / self.factor_depth
            mask = (d_m > self.min_depth) & (d_m < self.max_depth)
            
            points = cloud_npy[mask]
            colors = c_norm[mask]

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points.astype(np.float32))
            pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32))

            # 3. Grasp Geometries erstellen
            gg = GraspGroup(filtered)
            gg.sort_by_score()
            
            # Display top grasps number x
            gg = gg[:self.cfg.SHOW_TOP_GRASP_NUMBER]

            # Sicherheitsbegrenzung für Performance (Top 500)
            if len(gg) > 500:
                print(f"   [VIS] Limiting visualization to top 500 of {len(gg)} grasps for performance.")
                gg_vis = gg[:500]
            else:
                gg_vis = gg
            
            # Transformation für Visualisierung (Flip Z)
            flip_z = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            pcd.transform(flip_z)
            
            grippers = gg_vis.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(flip_z)

            # 4. Fenster anzeigen
            geometries = [pcd, *grippers]
            window_name = f"GraspNet Debug - Mode: {filter_mode}"
            
            width, height = 1280, 720
            if get_monitors:
                m = get_monitors()[0]
                width, height = m.width // 2, m.height // 2

            vis = o3d.visualization.Visualizer()
            vis.create_window(window_name=window_name, width=width, height=height, left=0, top=0)
            
            for geom in geometries:
                vis.add_geometry(geom)
                
            opt = vis.get_render_option()
            opt.background_color = np.asarray([0.05, 0.05, 0.05]) # Fast Schwarz
            opt.point_size = 1.0 # Kleinere Punkte für bessere Sicht auf Gripper

            vis.run()
            vis.destroy_window()
            print(">> [VIS] Visualization closed.")
