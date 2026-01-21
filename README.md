# xArm7_Grasping_Pipeline
Grasping Pipeline using Ufactory xArm7. Picking up utensils and sorting them automatic. 

The Goal of this project is to enhance a robotic grasping system. The target deployment is for example a hawker center, where the robot assists in clearing trays for sorting utensils for the washing process.
The operation involves heavily cluttered scenes, where multiple utensils may overlap or be stacked inside bowls and plates.

# Overview System
[system_overview]: ![System_Overview](https://github.com/user-attachments/assets/03287d5c-d121-4554-9fdd-f9edc027b774)


# AI_Planner (conda env python=3.10)
Python Programm handling Object Detection, Segmentation, Planning and Grasp Generation using following modules_e:
Use DEMO_BASH_DETERMINISTIC for planning what to grasp deterministic.
Use DEMO_BASH_LLM for planning what to grasp with Ollama.
- camera.py: starting realsense camera with python sdk
- camera_ros.py: getting data from ros2
- detection.py: used for object detection using yolo world
- deterministic_planner: 
- graspnet.py: compute 6DOF grasps using graspnet-baseline
- planner.py: 
- planner_normal.py
- segmentation.py
- vertical.py
- visualization.py

# ros2_ws
ai_planner_ros: Used for publishing scene_data from AI_Planner. Creates planning scene for MoveIt2.
ai_robot_control: Used for execution with MoveIt2.
# Credits
* **GraspNet**
@article{fang2023robust,
  title={Robust grasping across diverse sensor qualities: The GraspNet-1Billion dataset},
  author={Fang, Hao-Shu and Gou, Minghao and Wang, Chenxi and Lu, Cewu},
  journal={The International Journal of Robotics Research},
  year={2023},
  publisher={SAGE Publications Sage UK: London, England}
}

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





