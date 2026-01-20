import cv2
import json
import os
import time
import signal

# Imports from modules
from config import Config
#from modules_e.camera import RealSenseCam
from modules_e.camera_ros import RealSenseCam
from modules_e.detection import YoloDetector
from modules_e.segmentation import SamAnalyzer
# --- UPDATED: Use Deterministic Logic instead of AI ---
from modules_e.deterministc_planner import DeterministicPlanner 
from modules_e.visualization import draw_annotations
from modules_e.graspnet import GraspNetGrasp
from modules_e.vertical import VerticalGraspDetector
# subprocess for calivration of tray
import subprocess

# Signal Handler for Bash file
start_pipeline = False
def signal_handler(signum, frame):
    global start_pipeline
    start_pipeline = True

class RobotApp:
    def __init__(self):
        # Load config
        self.cfg = Config()
        
        # Initalise modules
        self.cam = RealSenseCam(self.cfg)
        self.yolo = YoloDetector(self.cfg)
        self.sam = SamAnalyzer(self.cfg)
        
        # Initialize Deterministic Planner 
        self.planner = DeterministicPlanner()
        
        self.running = True
        self.graspnet = GraspNetGrasp(self.cfg)
        self.vertical = VerticalGraspDetector(self.cfg)
        
        # Show possible Camera Resolutions
        self.cam.get_camera_info()


    def save_snapshot(self, color, depth, annotated, objects):
        # save color, depth, annotated image and Yolo Detections
        output_dir = self.cfg.OUTPUT_DIR
        os.makedirs(output_dir, exist_ok=True)
        cv2.imwrite(f"{output_dir}/color.png", color)
        cv2.imwrite(f"{output_dir}/depth.png", depth)
        cv2.imwrite(f"{output_dir}/annotated.png", annotated)
        
        # Save Detections in Json
        clean = [{k:v for k,v in o.items() if k not in ['color']} for o in objects]
        with open(f"{output_dir}/data.json", "w") as f: json.dump(clean, f, indent=4)
        print(f">> Saved: color.png, depth.png and annotated.png and data.json")

    def save_final_grasp(self,target_id):
        #extracts final grasp from VLM model output
        output_dir=self.cfg.OUTPUT_DIR
        os.makedirs(output_dir,exist_ok=True)
        with open(f"{output_dir}/data.json", "r") as f:
            data=json.load(f)

        target_obj = next((obj for obj in data if obj['id'] == target_id), None)

        if target_obj:
            with open(f"{output_dir}/selected_grasp.json", "w") as f:
                json.dump(target_obj, f, indent=4)
            print(f">> Final Grasp Target (ID {target_id}) saved to selected_grasp.json")
        else:
            print(f"!! Warnung: ID {target_id} not found in data.json")


    def run(self):
        global start_pipeline
        try:
            # Register Signal
            signal.signal(signal.SIGUSR1,signal_handler)

            # Start Camera
            self.cam.start()
            print("\n=== Camera started ===")

            # Calbiration Tray Start
            # print('Press in Image Frame:')
            # print("[ENTER] = Finish Setup, [q] = Stop, [c] = Calibration Tray")
            # ready_to_start = False
            # while not ready_to_start and self.running:
            #     color, _ = self.cam.get_frames()
            #     if color is None: continue

            #     cv2.putText(color, "SETUP: [c] Calibrate | [Enter] Start", (20, 50), 
            #                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
            #     cv2.imshow("Grasping Planner", color)
                
            #     key = cv2.waitKey(1) & 0xFF

            #     if key == ord('c'):
            #         print("\n>> [USER] Starting Calibration...")
            #         cv2.destroyAllWindows() 
            #         try:
            #             cmd = ["ros2", "run", "ai_planner_ros", "calibrate_tray"]
            #             subprocess.run(cmd, check=True)
            #             print(">> [SUCCESS] Calibration done.")
            #         except subprocess.CalledProcessError:
            #             print("!! [ERROR] Calibration failed.")

            #     elif key == 13:
            #         print(">> [USER] Starting Main Pipeline...")
            #         ready_to_start = True 
            #     elif key == ord('q'):
            #         self.running = False
            #         return 
                # Calibration Tray End

            while self.running:
                color, depth = self.cam.get_frames()
                if color is None: continue
                h, w = color.shape[:2]

                # A. YOLO 
                boxes = self.yolo.detect(color)
                objects = self.yolo.parse(boxes, w, h) #objects contains list with labels, confidence scores and 2D coordinates of bbox
                # Debug Print
                #print(boxes)
                #print(objects)

                # B. Add annotaions
                disp = draw_annotations(color, objects)
                cv2.imshow("Grasping Planner", disp)

                # C. Input
                key = cv2.waitKey(1)

                # D. No Detections
                if not objects:
                    print("No objects detected! Waiting for objects")
                    continue
                
                if key == ord('q'): self.running = False
                #elif key == 13 or start_pipeline == True: # ENTER or Signal PID
                elif start_pipeline == True: #Signal PID
                    start_pipeline = False
                    
                    # RETRY LOOP FOR INVALID TARGETS
                    max_attempts = 3
                    attempt = 0
                    success_pipeline = False

                    while attempt < max_attempts and not success_pipeline:
                        attempt += 1
                        print("\n" + "="*40)
                        if not objects:
                            print(">> [INFO] No objects detected.")
                            break
                        
                        print(f">> Pipeline Attempt {attempt}/{max_attempts}")

                        #########################
                        ### MASKING AND GEOMETRY ANALYSIS AND SAVING OF IMAGES
                        #########################
                        start_mask = time.time()
                        print("\nSegmentation and Spatial Analyzis:")
                        objects, facts = self.sam.analyze_scene(color, depth, objects)
                        #print(objects)
                        #print(facts)
                        
                        final_view = draw_annotations(color, objects)
                        self.save_snapshot(color, depth, final_view, objects)
                        end_mask = time.time()
                        print(f"Duration for Masking and Analysis: {end_mask - start_mask}")

                        #########################
                        ###     LOGIC Planner 
                        #########################   
                        start_planner_time = time.time()
                        print("\nPLANNING:")
                        # return target_id from deterministic decision
                        target_id = self.planner.get_decision(json_path=None, objects=objects)
                        target_obj = None

                        for obj in objects:
                            if obj["id"] == target_id:
                                target_obj = obj
                                self.save_final_grasp(target_id)
                                break
                        if target_id is None or target_obj is None:
                            print(" No valid target found")
                            continue  # Jump to next attempt
                        

                        
                        print(f"Duration for Planner: {time.time() - start_planner_time}")

                        ####################################################
                        ###     COMPUTE GRASPS WITH GRASPNET OR VERTICAL
                        ####################################################
                        if target_obj:
                            start_grasp = time.time()
                            label = target_obj['label']
                            obj_id = target_obj['id']
                            
                            mask_filename = f"{label}_{obj_id}_orig.png"
                            mask_path = os.path.join(self.cfg.OUTPUT_DIR, "masks", mask_filename)
                            obj_mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)

                            success = False
                            if label in self.cfg.GRASPNET_CLASSES:
                                print(f">> Strategy: GraspNetGrasp for '{label}'")
                                poses, scores, data = self.graspnet.calculate(color, depth, mask=obj_mask, label=label)
                                self.graspnet.visualize_results(color, depth, data, filter_mode=self.cfg.VISUALIZE_MODE)
                                success = self.graspnet.process_and_save_grasps(data, label=label, filter_mode='orientation_mask')
                            elif label in self.cfg.VERTICAL_CLASSES:
                                print(f">> Strategy: Vertical Grasp for '{label}'")
                                success = self.vertical.calculate(color, depth, obj_mask, label)
                            
                            if success:
                                success_pipeline = True # Exit retry loop
                                print(f"Duration Grasp Generation: {time.time() - start_grasp}")
                            else:
                                print(f"!! Grasping failed for {label}. Retrying attempt...")

                    if not success_pipeline:
                        print("!! All attempts failed. Writing fallback.")
                        fallback_path = os.path.join(self.cfg.OUTPUT_DIR, "grasp_points.json")
                        with open(fallback_path, "w") as f: json.dump([], f)

        finally:
            self.cam.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    app = RobotApp()
    app.run()