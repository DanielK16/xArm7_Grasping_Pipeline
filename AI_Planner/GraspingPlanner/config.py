import os
from dataclasses import dataclass, field
from typing import List

@dataclass
class Config:
    # --- CAMERA Settings---
    CAM_HEIGHT: float = 720.0
    CAM_WIDTH: float = 1280.0
    FX: float = 909.7272338867188
    FY: float = 909.4821166992188
    CX: float = 637.5324096679688
    CY: float = 350.2189636230469
    DEPTH_FACTOR: float = 1000.0
    
    # --- DETECTION Settings YOLO  --- (detection.py)
    CONF_THRESH: float = 0.15     		# minimum confidence score 	 
    IOU_THRESHOLD: float = 0.6			# Intersection over Union Threshold for NMS
    IMGSZ = 640   #960 #1280			# input image size the model was trained on
    VERBOSE = False				# Show detailed console output
    HALF = True					# Enables FP16 (half precision) inference
    AUGMENT = False				# Enables Test Time Augmentation
    AGNOSTIC_NMS = True				# Prevents multiple boxes from different classes
    DUPLICATE_DIST: int = 0  

    # --- SEGMENTATION Settings --- (segmentation.py)
    OVERLAP_THRESH: float = 0.1   		# Intersection over Area threshold
    	# Lables that should be treated as container
    CONTAINER_LABELS = ["bowl", "cup", "bowl_orange", "bowl_green"]		
	# Labels for "Dilation" (makes the mask slightly larger for better grasping)
    GRASP_DILATE_LABELS = ["cup"]
	# Labels for "Ring Mask" (targets the rim/edge for grasping)
    GRASP_RING_LABELS = ["bowl", "bowl_orange", "bowl_green", "plate", "tray"]
	# Set to True to save masks for debugging
    DEBUG_SAVE_MASKS = True
	
    # --- MODELS PATHS --- (downloaded automatically and put inside models folder)
    YOLO_PATH: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/models/yolov8x-worldv2.pt")
    SAM_PATH: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/models/sam2.1_l.pt")
    OLLAMA_MODEL: str = "ministral-3:14b"
    
    # --- PATHS ---
    OUTPUT_DIR: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data")
    PROMPT_PATH: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/prompts/save_long_prompt.txt")
    
    # --- GRASPNET ---
    GRASPNET_CHECKPOINT: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/external/graspnet-baseline/logs/log_kn/checkpoint-rs.tar")
    WORKSPACE_MASK_PATH: str = os.path.expanduser("~/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlaner/scene_data/workspace_mask/full_workspace_mask.png")
    WORKSPACE_MIN_Z: float = 0.15
    WORKSPACE_MAX_Z: float = 0.80
    CYLINDER_RADIUS: float = 0.05
    ORIENTATION_BOWL_MAX: float = 0.88
    ORIENTATION_BOWL_MIN: float = 0.51
    ORIENTATION_CUP_MAX: float = 0.88
    ORIENTATION_CUP_MIN: float = 0.55
    ORIENTATION_PLATE_MAX: float = 0.88
    ORIENTATION_PLATE_MIN: float = 0.6

    VISUALIZE_GRASPS = True
    VISUALIZE_MODE: str = "orientation_mask"    #orientation_mask orientation_only mask_only raw
    SHOW_TOP_GRASP_NUMBER: int = 5          	# number of top grasps to visualize

    BOWL_MAX_GRASP_WIDTH: float = 0.02
    CUP_MAX_GRASP_WIDTH: float = 0.085
    PLATE_MAX_GRASP_WIDTH: float = 0.02
    COLLISON_THRESH: float = 0.007
    APPROACH_DIST: float = 0.1
    VOXEL_SIZE: float = 0.005
    NUM_POINT: float = 20000
    
    # CLASSES
    GRASPNET_CLASSES: List[str] = field(
    default_factory=lambda: [
        'bowl_orange', 
        'bowl_green', 
        'cup', 
        'bowl',
        'apple',])
    
    VERTICAL_CLASSES: List[str] = field(
    default_factory=lambda: [
        'spoon_orange', 
        'chopstick'])
    
