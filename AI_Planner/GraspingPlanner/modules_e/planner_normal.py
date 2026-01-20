import re

class DeterministicPlanner:
    """
    A deterministic logic for robotic grasping.
    strict rule-based execution.
    
    STRATEGY:
    1. Clear contents of containers or stacks first (Tier 1).
    2. Clear large empty containers (Tier 2).
    3. Clear remaining small items on table (Tier 3).
    """

    def __init__(self, cfg):
        print(f">> [LOGIC] Loading Deterministic Planner...")
        self.cfg = cfg
        
        # --- CONFIGURATION: Object Categorie Classes ---
        # BULKY ITEMS:
        self.BULKY_CLASSES = [
            'bowl', 'cup', 'mug', 'box', 'plate', 'pan', 'tray',
            'bowl_orange', 'bowl_green', 'can', 'block', 'apple', 'orange'
        ]
        
        # THIN ITEMS:
        self.THIN_CLASSES = [
            'spoon', 'chopstick', 'fork', 'knife', 'pen', 
            'spoon_orange', 'marker', 'pencil'
        ]

    def _get_area(self, obj):
        """
        Calculates the 2D area of the object based on its normalized BBox.
        Used to determine which object is 'largest'.
        """
        # bbox format usually: [x1, y1, x2, y2]
        bbox = obj.get('bbox_norm', [0,0,0,0])
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        return width * height

    def _parse_stacking_facts(self, facts):
        """
        Parses the text output from segmentation.py to identify physical stacking.
        
        Returns:
            blocked_map (dict): Maps bottom_id -> Reason string
            top_ids (set): IDs of objects sitting on top of others
        """
        blocked_map = {}
        top_ids = set()
        
        # Regex to capture: "ID 0 ... ON TOP OF ... ID 1"
        pattern = r"ID (\d+).*?ON TOP OF.*?ID (\d+)"
        
        if facts:
            for line in facts.split('\n'):
                if "ON TOP OF" in line:
                    match = re.search(pattern, line)
                    if match:
                        top_id = int(match.group(1))
                        bottom_id = int(match.group(2))
                        
                        # The bottom object is blocked by the top one
                        blocked_map[bottom_id] = f"Stacked UNDER ID {top_id}"
                        top_ids.add(top_id)
                        
                        # Debug print to console
                        print(f"   [LOGIC-PARSE] Detected Stack: ID {top_id} is on top of ID {bottom_id}")
        
        return blocked_map, top_ids

    def get_action(self, image, objects, facts):
        """
        Main execution pipeline.
        1. Identify Blocked Objects
        2. Categorize into Priority Tiers.
        3. Select target based on Hierarchy + Semantic Rules.
        """
        if not objects: return None

        # Print Header for Debugging
        print("\n" + "="*65)
        print(f"   LOGIC PLANNER - DECISION MATRIX")
        print("="*65)

        # =========================================================
        # Rule 1: Identify Blocked Objects
        # =========================================================
        blocked_reasons = {}  # Dictionary {id: "Reason string"}
        
        # 1a. Block Containers that are not empty
        # 'contains' list is populated by segmentation.py
        for obj in objects:
            contains_list = obj.get('contains', [])
            if len(contains_list) > 0:
                blocked_reasons[obj['id']] = f"Contains IDs {contains_list}"

        # 1b. Block Objects at the bottom of a stack
        stack_blocked_map, stacked_top_ids = self._parse_stacking_facts(facts)
        blocked_reasons.update(stack_blocked_map)

        # =========================================================
        # STEP 2: CATEGORIZATION (Assign Priority Tiers)
        # =========================================================
        tier_1_complex = []     # High Priority: Inside Container or Top of Stack
        tier_2_containers = []  # Medium Priority: Empty Bulky Containers
        tier_3_table_items = [] # Low Priority: Small Items on Table
        
        object_status = {} # For Debug Table

        for obj in objects:
            oid = obj['id']
            label = obj['label']
            
            # --- Check if Object is Blocked ---
            if oid in blocked_reasons:
                object_status[oid] = ("BLOCKED", blocked_reasons[oid])
                continue

            # --- Analyze Properties ---
            relation = obj.get('relation', '')
            is_inside = "INSIDE" in relation
            is_stacked_top = oid in stacked_top_ids
            is_bulky = any(k in label.lower() for k in self.BULKY_CLASSES)

            # --- Assign to Tiers ---
            if is_inside:
                tier_1_complex.append(obj)
                object_status[oid] = ("VALID", "Tier 1 (Inside Container)")
            
            elif is_stacked_top:
                tier_1_complex.append(obj)
                object_status[oid] = ("VALID", "Tier 1 (Top of Stack)")
            
            elif is_bulky:
                tier_2_containers.append(obj)
                object_status[oid] = ("VALID", "Tier 2 (Empty Container)")
            
            else:
                tier_3_table_items.append(obj)
                object_status[oid] = ("VALID", "Tier 3 (Table Item)")

        # =========================================================
        # DEBUG: PRINT STATUS TABLE
        # =========================================================
        print(f"{'ID':<4} | {'LABEL':<16} | {'STATUS':<10} | {'DETAIL / REASON'}")
        print("-" * 70)
        for obj in objects:
            oid = obj['id']
            lbl = obj['label']
            stat, reason = object_status.get(oid, ("UNKNOWN", "Error"))
            print(f"{oid:<4} | {lbl:<16} | {stat:<10} | {reason}")
        print("-" * 70)

        # =========================================================
        # STEP 3: SELECTION TOURNAMENT (Decision Logic)
        # =========================================================
        target_obj = None
        decision_log = ""

        # Internal Helper Function for Selection
        def select_best(candidates, tier_name, prioritize_bulky=False):
            """
            Selects the best object from a list of candidates.
            Args:
                prioritize_bulky (bool): If True, a Cup will ALWAYS win against a Spoon.
            """
            print(f">> Analyzing {tier_name} with {len(candidates)} candidates...")
            
            # 1. Default Sort: Largest Area first
            candidates.sort(key=self._get_area, reverse=True)
            
            # 2. Apply Semantic Priority (Cup > Spoon)
            if prioritize_bulky:
                # Filter out all Bulky items (Cups, Bowls, etc.)
                bulky_items = [o for o in candidates if any(b in o['label'].lower() for b in self.BULKY_CLASSES)]
                
                # CRITICAL RULE: If any bulky item exists, pick it immediately.
                if bulky_items:
                    winner = bulky_items[0] # Pick the largest bulky item
                    
                    # Log if we overrode a thin item
                    thin_present = any(o for o in candidates if o not in bulky_items)
                    if thin_present:
                        print(f"   -> PRIORITY RULE ACTIVE: {winner['label']} (Bulky) wins against Spoon/Thin items!")
                    else:
                        print(f"   -> Selected largest Bulky item: {winner['label']}")
                        
                    return winner
            
            # 3. Fallback: Just pick the largest object by area
            winner = candidates[0]
            print(f"   -> Selected largest item: {winner['label']}")
            return winner

        # --- Execute Hierarchy ---
        if tier_1_complex:
            # We enforce prioritize_bulky=True here to solve the Cup-Spoon-Dilemma inside bowls
            target_obj = select_best(tier_1_complex, "TIER 1 (Complex)", prioritize_bulky=True)
            decision_log = "Tier 1 (Complex) - Cleared Content/Stack"
        
        elif tier_2_containers:
            target_obj = select_best(tier_2_containers, "TIER 2 (Empty)", prioritize_bulky=False)
            decision_log = "Tier 2 (Empty Container) - Cleared Largest"
        
        elif tier_3_table_items:
            target_obj = select_best(tier_3_table_items, "TIER 3 (Table)", prioritize_bulky=False)
            decision_log = "Tier 3 (Table Item) - Cleared Largest"
        
        else:
            print("!! NO ACTION POSSIBLE (Deadlock or Empty Scene)")
            print("="*65 + "\n")
            return None

        print(f">> FINAL DECISION: Grasp {target_obj['label']} (ID {target_obj['id']})")
        print("="*65 + "\n")

        # Return JSON format compatible with Main Loop
        return {
            "check": decision_log,
            "target_id": target_obj['id']
        }