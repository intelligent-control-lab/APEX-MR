# APEX-MR

<div>
<a href="https://intelligent-control-lab.github.io/APEX-MR/"><img src="https://img.shields.io/badge/Project_Page-Website-green?logo=googlechrome&logoColor=white" alt="Project Page" height=22px></a>
<a href="https://arxiv.org/abs/2503.15836" target="_blank"><img src=https://img.shields.io/badge/ArXiv-Paper-b5212f.svg?logo=arxiv alt="ArXiv" height=22px></a>
</div>


***Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly***<br>
[Philip Huang*](https://philip-huang.github.io/),
[Ruixuan Liu*](https://waynekyrie.github.io/),
[Shobhit Aggarwal](https://engineering.cmu.edu/mfi/directory/bios/aggarwal-shobhit.html),
[Changliu Liu](http://icontrol.ri.cmu.edu/people/changliu.html),
[Jiaoyang Li](https://jiaoyangli.me/)<br>
*Carnegie Mellon University*<br>
RSS 2025

---
This is the code repository  for APEX-MR, our planning and execution framework for dual-arm LEGO assembly.

## Installation

### Prerequisites
**Gurobi (optional):** 
  If you would like to search over all possible LEGO grasp/support poses (i.e. $P = 28$), you need 
  a [Gurobi WLS license](https://support.gurobi.com/hc/en-us/articles/13232844297489-How-do-I-set-up-a-Web-License-Service-WLS-license) for stability estimation. Academics may request a free license from the
  Gurobi website [here](https://www.gurobi.com/features/academic-wls-license/); after obtaining the license,
  place it in your *home directory* or
  another [recommended location](https://support.gurobi.com/hc/en-us/articles/360013417211-Where-do-I-place-the-Gurobi-license-file-gurobi-lic).
    - If you do not have access to Gurobi, you can still run the code with the default LEGO grasp poses (i.e. $P = 1$) computed by our automated assembly sequence planner

### Docker Installation
Build the docker image and run it inside docker
```
cd docker && bash build.sh
```

### Native Installtaion with ROS
If you are not using the docker file, the following setup has been tested on Ubuntu 20.04 with ROS Noetic. You may need to install some system dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [moveit](https://moveit.ai/install/)
- [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
- [rviz tools](http://wiki.ros.org/rviz_visual_tools)
- [moveit visual tools](http://wiki.ros.org/moveit_visual_tools)

Once you have download ros, and other system deps, under your ```catkin_ws/src```, download the robot model to your workspace
- [gp4 digital twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin/tree/apexmr-release) Checkout to the ``apexmr-release`` tag!

Under the ```catkin_ws/src``` workspace, doanload the code. Then use ```catkin build``` to compile the code.

## Run APEX-MR
### Task Planning
To generate the task assignment for lego task, run
```
roslaunch apex_mr lego_assign.launch task:=cliff
```
You should see a message saying lego assignment success.

### Motion Planning and TPG construction
To compute the motion plan for lego assembly and build the corresponding TPG for asynchronous execution, run
```
roslaunch apex_mr lego.launch task:=cliff
```
You should see a lego planning success
#### Planning params
```vmax:=X``` to set maximum velocity scale, default ```X = 1``` <br>
```adg_shortcut_time:=X``` to set the time for TPG shortcutting, default ```X = 1s``` <br>
```sync_plan:=true``` to run the sync planning baseline <br>
```sync_shortcut_time:=X``` to set the time for RRT-Connect,shortcutting, default ```X = 0.1s``` <br>

Target LEGO assembly are specified under ```config/lego_tasks/assembly_tasks```. <br>
Description of the LEGO assembly plate are specified under ```config/env_setup/assembly_tasks```

### Visualizing in Gazebo
By default, APEX-MR uses the fake hardware in Moveit for collision checks and Rviz for visualization. Optinoally it is also possible to visualize the Lego assembly environment in Gazebo.

To use Gazebo, first launch the simulator
```
roslaunch robot_digital_twin dual_gp4.launch 
```
This may take some time. Then run the motion planning command earlier and wait for the TPG to be generated. Once the TPG is computed, the robots should be both in Rviz and Gazebo at the same time.

### Running the experiments in paper
We provide a script to run all 9 LEGO assemblies from the paper in simulation
```
python3 scripts.py/benchmark.py
```

Note that the exact results may be slightly different from the numbers reported in our paper.

## Citation

If you find this repository useful for your research, please cite the following work.

```bibtex
@inproceedings{huang2025apexmr,
              title = {APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly},
              author = {Huang, Philip and Liu, Ruixuan and Aggarwal, Shobhit and Liu, Changliu and Li, Jiaoyang},
              year = {2025},
              info = {https://intelligent-control-lab.github.io/APEX-MR/},
              booktitle = {Robotics: Science and Systems},
              url = {https://arxiv.org/abs/2503.15836}
            }
```
