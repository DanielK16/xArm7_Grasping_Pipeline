# xArm7_Grasping_Pipeline
Grasping Pipeline using Ufactory xArm7. Picking up utensils and sorting them automatic. 

The Goal of this project is to enhance a robotic grasping system. The target deployment is for example a hawker center, where the robot assists in clearing trays for sorting utensils for the washing process.
The operation involves heavily cluttered scenes, where multiple utensils may overlap or be stacked inside bowls and plates.

# Overview System Pick and Place
The system is divided into three functional stages: Detection, Planning, and Execution.
![System_Overview](https://github.com/DanielK16/xArm7_Grasping_Pipeline/blob/main/images/System_Overview.jpeg)


# System Environment
This project was developed on the following hardware and software:

| Component         | Details                                         | Command to Check                |
| :---------------- | :---------------------------------------------- | :------------------------------ |
| **Hardware** | Jetson AGX Orin Development Kit 64GB                 |                                 |
| **OS Version** | Ubuntu 22.04.5 LTS (Jammy)                         | `lsb_release -a`                |
| **JetPack** | 6.1                                                   | `cat /etc/nv_tegra_release`     |
| **L4T Release** | R36.4.7                                           | `cat /etc/nv_tegra_release`     |
| **CUDA Version** | 12.6.68                                          | `nvcc --version`                |
| **Driver Version**| 540.4.0                                         | `nvidia-smi`                    |
| **ROS 2 Distro** | Humble                                           | `printenv ROS_DISTRO`           |

# AI_Planner (conda env python=3.10)
Python Programm handling Object Detection, Segmentation, Planning and Grasp Generation using following modules_e:
Use DEMO_BASH_DETERMINISTIC for planning what to grasp deterministic.
Use DEMO_BASH_LLM for planning what to grasp with Ollama.
- camera.py: starting realsense camera with python realsense sdk
- camera_ros.py: getting data from realsense camera
- detection.py: Utilizes YOLO World to identify and localize various utensils such as cups, bowls, spoons, and chopsticks with high confidence.
- deterministic_planner: Implements a rule-based hierarchy to prioritize grasping operations. It uses a four-tier system to ensure complex cluttered arrangements (e.g., clearing a cup's contents) are resolved before simpler surface-level tasks.
- graspnet.py: Computes complex grasp candidates for volumetric items like bowls using GraspNet-baseline.
- planner.py: Implements an Ollama framework to use an VLM model for Decision what to grasp next.
- planner_normal.py: #OLD VERSION
- segmentation.py: Employs the Segment Anything Model (SAM) to generate precise object masks and analyze spatial relationships, identifying if objects are stacked or contained within others.
- vertical.py: A geometric solver for robust top-down grasping of thin items like spoons or chopsticks using RGB-D data.

# Install conda env (on JETSON AGX ORIN with Cuda 12.6):
conda create -n AI_Planner python==3.10
conda activate AI_Planner
conda env update --file requirements.yml
#install correct torch version for cuda system!
pip install --force-reinstall --no-cache-dir -U torch==2.8.0 torchvision==0.23.0 torchaudio==2.8.0 --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
pip install numpy==1.23.4
pip install opencv-python==4.11.0.86

# ros2_ws
ai_planner_ros: Acts as the bridge between AI perception and motion planning.
  - moveit_pub.py: Publishes grasp candidates from JSON files as ROS 2 PoseArray messages.
  - scene_spawner.py: Maintains the MoveIt2 planning scene with static collision objects like tables and safety boundaries.
ai_robot_control: Manages the physical execution.
  - moveit_run_node.cpp:Executes the grasping sequence (Pre-grasp, Grasp, Lift, and Drop) using the Pilz Industrial Motion Planner for precise linear and PTP movements.
# Credits
* **GraspNet**
@article{fang2023robust,
  title={Robust grasping across diverse sensor qualities: The GraspNet-1Billion dataset},
  author={Fang, Hao-Shu and Gou, Minghao and Wang, Chenxi and Lu, Cewu},
  journal={The International Journal of Robotics Research},
  year={2023},
  publisher={SAGE Publications Sage UK: London, England}
}
* **GraspNet**
@inproceedings{fang2020graspnet,
  title={GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping},
  author={Fang, Hao-Shu and Wang, Chenxi and Gou, Minghao and Lu, Cewu},
  booktitle={Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition(CVPR)},
  pages={11444--11453},
  year={2020}
}
* **SAM 2**
@article{ravi2024sam2,
  title={SAM 2: Segment Anything in Images and Videos},
  author={Ravi, Nikhila and Gabeur, Valentin and Hu, Yuan-Ting and Hu, Ronghang and Ryali, Chaitanya and Ma, Tengyu and Khedr, Haitham and R{\"a}dle, Roman and Rolland, Chloe and Gustafson, Laura and Mintun, Eric and Pan, Junting and Alwala, Kalyan Vasudev and Carion, Nicolas and Wu, Chao-Yuan and Girshick, Ross and Doll{\'a}r, Piotr and Feichtenhofer, Christoph},
  journal={arXiv preprint arXiv:2408.00714},
  url={https://arxiv.org/abs/2408.00714},
  year={2024}





