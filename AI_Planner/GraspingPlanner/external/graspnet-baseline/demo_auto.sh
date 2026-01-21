#!/bin/bash

source ~/miniforge3/etc/profile.d/conda.sh

# Start the persistent TF publisher in the background (only once)
conda activate graspnet-baseline
python3 PUBLISH_GRASP.py --mode orientation_only &  # Active grasp publisher
TF_PUBLISHER_PID=$!
sleep 3

# Now run your main grasp loop
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

  conda activate graspnet-baseline

  echo "====Filter and Visualization===="
  python3 FILTER_ORIENTATION.py && # match mask area with grasp candidates
  python3 visualizegrasp.py --mode orientation_only --view top & # visualize filtered grasp candidates that match mask 

  #echo "Grasp frame should be updated automatically by persistent TF publisher."
  sleep 1

  # Confirmation before moving arm
  echo "======Starting Arm Movement======"
  #read -p "Press Enter to start arm movement script, or Ctrl+C to cancel..."

  # Fully deactivate all conda envs before arm movement script
  while [[ "$CONDA_DEFAULT_ENV" != "" ]]; do conda deactivate; done
  python3 MOVE_TO_GRASP.py
# Kill any previous preview windows quietly
  pkill -f visualizegrasp.py >/dev/null 2>&1 || true
  #sleep 0.5
  echo "=====Arm Movement Completed======"

done

# Optionally handle shutdown
trap "echo 'Stopping TF publisher'; kill $TF_PUBLISHER_PID; exit" SIGINT SIGTERM
