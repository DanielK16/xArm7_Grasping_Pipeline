#!/bin/bash

source ~/miniforge3/etc/profile.d/conda.sh
echo "===Activate static TF Publisher==="
python3 PUBLISH_GRASP.py --mode orientation_mask &  # static TF publisher
TF_PUBLISHER_PID=$!

conda activate graspnet-baseline

# loop main grasp pipeline
while true; do
  #read -p "Press Enter to start new grasp, or Ctrl+C to cancel..."

  echo "Starting new grasp..." 

  conda activate graspnet-baseline

  echo "=======Start Scene Capture======="
  python3 scene_capture.py &&
  echo "========End Scene Capture========"

  echo "=====Start Grasp Generation======"
  CUDA_VISIBLE_DEVICES=0 python3 demo.py --checkpoint_path logs/log_kn/checkpoint-rs.tar &&
  echo "======End Grasp Generation======="

  echo "====Filter Orientation and Append_UV==="
  python3 FILTER_ORIENTATION.py && # discard grasps that are too horizontal
  python3 APPEND_UV.py && # convert 3d real-world coordinates (x,y,z) to 2d pixel coordinates (u,v)
  echo "===END FILTER Orientation and APPEND_UV==="
  
  conda activate sam

  echo "======Segment Target Object Automatically======"
  python3 /home/rric/GRASP/graspnet-baseline/AUTOMATIC_MASK.py

  conda activate graspnet-baseline
  
  echo "===Filter_mask==="
  python3 FILTER_MASK.py && # match mask area with grasp candidates
  
  #echo "===Visualize Top Grasps==="
  #python3 visualizegrasp.py --mode orientation_mask --view all && # visualize filtered grasp candidates that match mask
  #python3 visualizegrasp.py --mode orientation_mask --view top & # visualize filtered grasp candidates that match mask 

  echo "===Properties_Mask==="
  python /home/rric/GRASP/graspnet-baseline/PROPERTIES_MASK.py


  echo "======Starting Arm Movement======"
  #read -p "Press Enter to start arm movement script, or Ctrl+C to cancel..."

  # Fully deactivate all conda envs before arm movement script
  while [[ "$CONDA_DEFAULT_ENV" != "" ]]; do conda deactivate; done
 
  python3 MOVE_TO_GRASP_SORT.py

  # Kill any previous preview windows quietly
  pkill -f visualizegrasp.py >/dev/null 2>&1 || true
  sleep 0.5
  echo "=====Arm Movement Completed======"

done

# Optionally handle shutdown
trap "echo 'Stopping TF publisher'; kill $TF_PUBLISHER_PID; exit" SIGINT SIGTERM
