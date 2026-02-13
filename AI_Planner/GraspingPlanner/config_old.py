import os
from dataclasses import dataclass, field
from typing import List

@dataclass
class Config:
    # --- CAMERA ---
    CAM_WIDTH: int = 1280
    CAM_HEIGHT: int = 720
    CAM_FPS: int = 15
    LASER_POWER: int = 250
    SHOW_RES = False
    FX: float = 909.7272338867188
    FY: float = 909.4821166992188
    CX: float = 637.5324096679688
    CY: float = 350.2189636230469
    # FX: float = 912.0184936523438
    # FY: float = 911.9789428710938
    # CX: float = 652.2272338867188
    # CY: float = 377.44488525390625
    DEPTH_FACTOR: float = 1000.0
    
    # Pre Processing Filters for depth image
	# SPATIAL FILTER 
    FILTER_SPATIAL_MAGNITUDE: float = 2.0     # Default: 2.0
    FILTER_SPATIAL_ALPHA: float = 0.5         # Default: 0.5
    FILTER_SPATIAL_DELTA: int = 20            # Default: 20
    	# TEMPORAL FILTER
    FILTER_TEMPORAL_ALPHA: float = 0.4        # Default: 0.4
    FILTER_TEMPORAL_DELTA: int = 20           # Default: 20
    	# HOLE FILLING FILTER
    # Modus: 0=Disabled, 1=Farthest from camera, 2=Nearest from camera
    FILTER_HOLE_FILL_MODE: int = 1            # Default: 1    
    
    # --- DETECTION ---
    CONF_THRESH: float = 0.15      
    IOU_THRESHOLD: float = 0.6
    IMGSZ = 640   #960 #1280
    VERBOSE = False
    HALF = True

    # --- SEGMENTATION ---
    OVERLAP_THRESH: float = 0.1   # Ab wann ist etwas "Inside"?
    DUPLICATE_DIST: int = 0       # Pixel Abstand für Duplikate

    # --- MODELS ---
    YOLO_PATH: str = os.path.expanduser("~/AI_Planner/GraspingPlanner/models/yolov8x-worldv2.pt")
    SAM_PATH: str = os.path.expanduser("~/AI_Planner/GraspingPlanner/models/sam2.1_l.pt")
    OLLAMA_MODEL: str = "ministral-3:14b"
    
    # --- PFADE ---
    OUTPUT_DIR: str = os.path.expanduser("~/AI_Planner/GraspingPlanner/scene_data")
    PROMPT_PATH: str = "/home/rric/AI_Planner/GraspingPlanner/prompts/save_long_prompt.txt"
    
    # --- GRASPNET ---
    GRASPNET_CHECKPOINT = "/home/rric/AI_Planner/GraspingPlanner/external/graspnet-baseline/logs/log_kn/checkpoint-rs.tar"
    WORKSPACE_MASK_PATH = "home/rric/AI_Planner/GraspingPlaner/scene_data/workspace_mask/full_workspace_mask.png"
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
    
    SHOW_TOP_GRASP_NUMBER: int = 5          # number of top grasps to visualize

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
    
