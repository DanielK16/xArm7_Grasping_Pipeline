#!/bin/bash
source ~/miniforge3/etc/profile.d/conda.sh

conda activate graspnet-baseline
python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/publish_grasp_sort.py --mode orientation_mask &  # static TF publisher
TF_PUBLISHER_PID=$!
sleep 3

# loop main grasp pipeline
while true; do
  #read -p "Press Enter to start new grasp, or Ctrl+C to cancel..."

  echo "Starting new grasp..." 

  conda activate graspnet-baseline

  echo "=======Start Scene Capture======="
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/scene_cap_sort.py &&
  echo "========End Scene Capture========"

  echo "=====Start Grasp Generation======"
  CUDA_VISIBLE_DEVICES=0 python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/demo_sorting.py --checkpoint_path /home/rric/GRASP/graspnet-baseline/logs/log_kn/checkpoint-rs.tar&&
  echo "====Filter and Visualization===="
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/filter_orientation_sort.py && # discard grasps that are too horizontal
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/append_uv_sorting.py && # convert 3d real-world coordinates (x,y,z) to 2d pixel coordinates (u,v)

  echo "====YOLO Detection==============="
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/yolo_detect.py &&
  echo "====END YOLO DETECTION====="
  
  conda activate sam

  echo "======Segment Target Object======"
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/object_mask.py && 
  echo "====End Segment Target Object===="

  conda activate graspnet-baseline

  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/filter_mask_sorting.py && # match mask area with grasp candidates
  
 # python3 visualizegrasp.py --mode orientation_mask --view all && # visualize filtered grasp candidates that match mask
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/visualizegrasp_sort.py --mode orientation_mask --view top & # visualize filtered grasp candidates that match mask 

  #echo "Grasp frame should be updated automatically by persistent TF publisher."
  sleep 0.5

  # Confirmation before moving arm
  echo "======Starting Arm Movement======"
  read -p "Press Enter to start arm movement script, or Ctrl+C to cancel..."

  # Fully deactivate all conda envs before arm movement script
  while [[ "$CONDA_DEFAULT_ENV" != "" ]]; do conda deactivate; done
  python3 /home/rric/GRASP/graspnet-baseline/Sorting_YOLO_SAM/move_to_grasp_sort.py
# Kill any previous preview windows quietly
  pkill -f visualizegrasp.py >/dev/null 2>&1 || true
  sleep 0.5
  echo "=====Arm Movement Completed======"

done

# Optionally handle shutdown
trap "echo 'Stopping TF publisher'; kill $TF_PUBLISHER_PID; exit" SIGINT SIGTERM