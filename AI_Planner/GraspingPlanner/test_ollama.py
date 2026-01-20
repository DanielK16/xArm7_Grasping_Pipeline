import cv2
import json
import os
import time
from types import SimpleNamespace

# Nur die notwendigen Imports
from config import Config
from modules_e.planner import OllamaPlanner
from modules_e.camera import RealSenseCam 

class PromptTester:
    def __init__(self):
        self.cfg = Config()
        self.cam = RealSenseCam(self.cfg)
        self.planner = OllamaPlanner(self.cfg)
        self.running = True

    def run(self):
        try:
            self.cam.start()
            print("\n=== Prompt Test Mode ===")

            while self.running:
                color, _ = self.cam.get_frames()
                if color is None: continue

                # Einfaches Live-Bild anzeigen
                cv2.imshow("VLM Prompt Tester", color)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.running = False
                elif key == 13: # ENTER
                    print("\n" + "="*50)
                    print(f">> Triggering AI with Model: {self.cfg.OLLAMA_MODEL}")
                    
                    test_objects = [
                        {"id": 0, "label": "bleach_bottle", "bbox_norm": [0.1, 0.2, 0.3, 0.4]},
                        {"id": 1, "label": "plastic_cup", "bbox_norm": [0.5, 0.5, 0.6, 0.7]}
                    ]
                    test_facts = "The table is flat. Sunlight is coming from the left."

                    start_time = time.time()
                    
                    # AI aufrufen
                    decision = self.planner.get_action(color, test_objects, test_facts)
                    
                    duration = time.time() - start_time
                    
                    if decision:
                        print(f">> AI Response ({duration:.2f}s):\n{json.dumps(decision, indent=2)}")
                    else:
                        print("!! AI hat keine gültige Antwort geliefert.")
                    
                    print("="*50 + "\n")

        finally:
            self.cam.stop()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    app = PromptTester()
    app.run()
