#!/bin/bash


source ~/miniforge3/etc/profile.d/conda.sh


conda activate graspnet-baseline
python3 PUBLISH_GRASP.py --mode orientation_mask &  # static TF publisher
TF_PUBLISHER_PID=$!
sleep 3

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

  echo "======Visualize All Grasps======="
  python3 visualizegrasp.py --mode raw --view all && # visualize unfiltered grasp candidates
  echo "====End Visualize All Grasps====="

  conda activate sam

  echo "======Segment Target Object======"
  python3 ~/segment-anything/sgmt_interactive.py && # choose mask area
  #python3 /home/rric/GRASP/graspnet-baseline/AUTOMATIC_MASK.py
  echo "====End Segment Target Object===="

  conda activate graspnet-baseline

  echo "====Filter and Visualization===="
  python3 FILTER_ORIENTATION.py && # discard grasps that are too horizontal
  python3 APPEND_UV.py && # convert 3d real-world coordinates (x,y,z) to 2d pixel coordinates (u,v)
  python3 FILTER_MASK.py && # match mask area with grasp candidates
  
  #python3 visualizegrasp.py --mode orientation_mask --view all && # visualize filtered grasp candidates that match mask
  python3 visualizegrasp.py --mode orientation_mask --view top & # visualize filtered grasp candidates that match mask 

  #echo "Grasp frame should be updated automatically by persistent TF publisher."
  sleep 0.5
  # Confirmation before moving arm
  echo "======Starting Arm Movement======"
  read -p "Press Enter to start arm movement script, or Ctrl+C to cancel..."

  # Fully deactivate all conda envs before arm movement script
  while [[ "$CONDA_DEFAULT_ENV" != "" ]]; do conda deactivate; done
  python3 MOVE_TO_GRASP.py
# Kill any previous preview windows quietly
  pkill -f visualizegrasp.py >/dev/null 2>&1 || true
  sleep 0.5
  echo "=====Arm Movement Completed======"

done

# Optionally handle shutdown
trap "echo 'Stopping TF publisher'; kill $TF_PUBLISHER_PID; exit" SIGINT SIGTERM
