import cv2
import numpy as np
import json

#load mask
mask = cv2.imread("/home/rric/GRASP/graspnet-baseline/scene_data/normal_mask.png", cv2.IMREAD_GRAYSCALE)
#load predicted_grasps
grasp_array = np.load("/home/rric/GRASP/graspnet-baseline/scene_data/predicted_grasps.npy")

if mask is None:
    raise ValueError("Mask not loaded")
if grasp_array is None:
    raise ValueError("Predicted Grasps not loaded")

#Compute Area
area_pixels = np.sum(mask > 0)
#print(f"Area in Pixel: {area_pixels}")

# compute Center Mask
moments = cv2.moments(mask)
if moments['m00'] == 0:
    raise ValueError("Mask empty")
cx = int(moments['m10'] / moments['m00'])
cy = int(moments['m01'] / moments['m00'])
#print(f"Center of Mask: x={cx}, y={cy}")

#Compute Top Grasp pose
top_grasp = None
for grasp in grasp_array:
    if grasp[17] == 1:  # Orientation filter ok
        u, v = int(grasp[18]), int(grasp[19])
        if 20 < u < 1260 and 20 < v < 700:  # Inside safe image area
            top_grasp = grasp
            break  # first valid grasp
        else:
            print(f"Skipped grasp at ({u},{v}) - outside image region")

if top_grasp is None:
    raise ValueError("No valid grasp found within the allowed region!")

# Pixel-coords in Image
u, v = int(top_grasp[18]), int(top_grasp[19])
#print(f"center GRASP: x={u}, y={v}")

#Compute angle
vec_x = u - cx
vec_y = v - cy
angle_rad = np.arctan2(vec_y, vec_x)  
angle_deg = np.degrees(angle_rad)     
#print("Winkel in rad:",angle_deg)

#Compute side of  grasp
if vec_x > 0:
    side = True
elif vec_x < 0:
    side = False

is_bowl=False
is_apple=False
is_trash=False

#Classifikation
is_bowl = area_pixels > 32000 and area_pixels <35500
is_apple = area_pixels > 16000 and area_pixels < 20000
if not is_bowl and not is_apple:
    is_trash=True

# Save Values
data_to_save = {
    "area_pixels": int(area_pixels),  
    "angle_deg": float(angle_deg),   
    "grasp_side": bool(side),
    "is_bowl": bool(is_bowl),
    "is_apple": bool(is_apple),
    "is_trash": bool(is_trash)
}

with open("/home/rric/GRASP/graspnet-baseline/scene_data/grasp_info.json", "w") as f:
    json.dump(data_to_save, f, indent=4)  

#print("Area und Winkel gespeichert in grasp_info.json")


