# LOGIK FOR DECIDING WHAT OBJECT TO GRASP
import json

class DeterministicPlanner:
    def __init__(self):

        print("[PLANNER] Initializing Deterministic Planner")
        # Define Object Categories
        self.BULKY_ITEMS = {"bowl", "bowl_orange", "bowl_green", "cup"}
        self.SMALL_ITEMS = {"spoon", "spoon_orange", "chopstick"}

        # Define Tiers: IDs get assigned to it
        self.TIER_1 = []   # INSIDE OTHERS AND NOT BLOCKED
        self.TIER_2 = []   # EMPTY CONTAINER AND NOT BLOCKED
        self.TIER_3 = []   # SMALL ITEMS ON TABLE AND NOT BLOCKED
        self.TIER_4 = []   # ALL OTHER OBJECTS
        self.BLOCKED = []  # NOT ALLOWED TO GRASP
        self.INSIDE = []   # OBJECTS WHICH ARE INSIDE ANOTHER

        self.TARGET_ID = None

    def load_data(self, json_path=None, objects=None):
        """
        Load Data either from Json Path or a List
        """
        # Check which data is given
        if json_path is not None:
            with open(json_path, "r") as f:
                self.objects = json.load(f)
            print(f"Data loaded from {json_path}")
        elif objects is not None:
            self.objects = objects
            print("Using objects parameter for planning")
        else:
            print("No data loaded please check!")

    def available_objects(self):
        print("#"*50)
        print("Sorting following Objects:")
        for obj in self.objects:
            print(f"\tObject label: {obj['label']} ID: {obj['id']}")

    def check_if_blocked(self):
        """
        Goes through self.objects
        An object is blocked when:
            - contains another ID
            - is stacked on top of another
        """
        # Check for each label if it contains another ID
        for obj in self.objects:
            content = obj.get('contains',[])
            if len(content) > 0:
                self.BLOCKED.append(obj["id"])

        # Compare median depth for each object
        # get all objects inside another
        # Extra rule: if spoon and cup inside a container select cup!
        for obj in self.objects:
            content = obj.get('contains',[])
            if len(content) > 0:
                objects_inside = []
                for inside_id in content:
                    for search_obj in self.objects:
                        if search_obj["id"] == inside_id:
                            objects_inside.append(search_obj)
                if objects_inside:
                    labels_in_container = [item["label"] for item in objects_inside]
                    if "cup" in labels_in_container and ("spoon" in labels_in_container or "chopstick" in labels_in_container):
                        top_obj = next(item for item in objects_inside if item["label"] == "cup")
                    else:
                        top_obj = min(objects_inside, key=lambda x: x["depth_median"])
                    for item in objects_inside:
                        if item["id"] not in self.INSIDE:
                            self.INSIDE.append(item["id"])
                        if item["id"] != top_obj["id"]:
                            if item["id"] not in self.BLOCKED and item["label"] != "cup":
                                self.BLOCKED.append(item["id"])
                                print(f"{item['id']} is blocked because it is below {top_obj['id']}")

        #print(f"Blocked Objects: {self.BLOCKED}")
        #print(f"Objects inside: {self.INSIDE}")

    def sort_into_tiers(self):
        """
        sort object ids into tiers
        """    
        # if object is in INSIDE and NOT BLOCKED +> TIER 1
        for obj in self.objects:
            if obj["id"] in self.INSIDE and obj["id"] not in self.BLOCKED:
                self.TIER_1.append(obj["id"])

        # if object is in BULKY and NOT BLOCKED TIER_2
            elif obj["label"] in self.BULKY_ITEMS and obj["id"] not in self.BLOCKED:
                self.TIER_2.append(obj["id"]) 
        
        # if object is in SMALL and NOT BLOCKED
            elif obj["label"] in self.SMALL_ITEMS and obj["id"]not in self.INSIDE and obj["id"] not in self.BLOCKED:
                self.TIER_3.append(obj["id"])
        
        # if no tier is suitable put it into Tier_4
            elif obj["id"] not in self.BLOCKED:
                self.TIER_4.append(obj["id"])

        #print(f"Objects IDs in TIER_1: {self.TIER_1}")
        #print(f"Objects IDs in TIER_2: {self.TIER_2}") 
        #print(f"Objects IDs in TIER_3: {self.TIER_3}") 
        #print(f"Objects IDs in TIER_4: {self.TIER_4}") 

    def target_id(self):
        """
        Choose Target_id based on TIER levels
        """
        if self.TIER_1:
            #check if an cup is in Tier_1
            cups_in_tier_1 = [obj for obj in self.objects if obj["id"] in self.TIER_1 and obj["label"] == "cup"]
            if cups_in_tier_1:
                self.TARGET_ID = cups_in_tier_1[0]["id"]
            else: 
                self.TARGET_ID = self.TIER_1[0]
        elif self.TIER_2:
            self.TARGET_ID = self.TIER_2[0]
        elif self.TIER_3:
            self.TARGET_ID = self.TIER_3[0]
        elif self.TIER_4:
            self.TARGET_ID = self.TIER_4[0]
        else:
            self.TARGET_ID = None
            print("All TIERS are empty!")
        #print(f"Target Id: {self.TARGET_ID}")

    def overview(self):
        print("#"*50)
        print("OVERVIEW OBJECTS")
        print(f"{'ID':<4} | {'LABEL':<16} | {'STATUS':<10} |")
        for obj in self.objects:
            id = obj["id"]
            label = obj["label"]
            if obj["id"] in self.BLOCKED:
                status = "BLOCKED"
            elif obj["id"] in self.TIER_1:
                status = "TIER_1"
            elif obj["id"] in self.TIER_2:
                status = "TIER_2"
            elif obj["id"] in self.TIER_3:
                status = "TIER_3"
            elif obj["id"] in self.TIER_4:
                status = "TIER_4"

            print(f"{id:<4} | {label:<16} | {status:<10} |")
        print(f"\nFinal Target_ID: {self.TARGET_ID}")
        print("#"*50)

    def reset(self):
        """
        resets Dictionarys for next Planning Iteration
        """
        self.TIER_1 = []   
        self.TIER_2 = []  
        self.TIER_3 = []  
        self.TIER_4 = [] 
        self.BLOCKED = [] 
        self.INSIDE = []   

    def get_decision(self,json_path=None,objects=None):
        """
        used for main_deterministic.py
        """
        self.load_data(json_path,objects)
        # Show all available objects
        self.available_objects()
        # First Step: Identify Blocked Objects
        self.check_if_blocked()
        #Second Step: Categorize objects into Tiers
        self.sort_into_tiers()
        #Select target_id()
        self.target_id()
        #Print Overview
        self.overview()
        #Return Target_ID
        returned_target_id = self.TARGET_ID
        self.reset()
        return returned_target_id


if __name__ == "__main__":
    planner = DeterministicPlanner()
    planner.load_data(json_path="/home/rric/xArm7_Grasping_Pipeline/AI_Planner/GraspingPlanner/scene_data/data.json")
    # Show all available objects
    planner.available_objects()
    # First Step: Identify Blocked Objects
    planner.check_if_blocked()
    #Second Step: Categorize objects into Tiers
    planner.sort_into_tiers()
    #Select target_id()
    planner.target_id()
    #Print Overview
    planner.overview()
    #Reset Tiers for next run
    planner.reset()
