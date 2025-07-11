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
Build the Docker image and run it inside Docker
```
cd docker && bash build.sh
```

### Native Installation with ROS
If you are not using the Docker file, the following setup has been tested on Ubuntu 20.04 with ROS Noetic. You may need to install some system dependencies
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [moveit](https://moveit.ai/install/)
- [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
- [rviz tools](http://wiki.ros.org/rviz_visual_tools)
- [moveit visual tools](http://wiki.ros.org/moveit_visual_tools)

Once you have downloaded ROS, and other system dependencies, under your ```catkin_ws/src```, download the robot model to your workspace
- [gp4 digital twin](https://github.com/intelligent-control-lab/Robot_Digital_Twin/tree/apexmr-release) Checkout to the ``apexmr-release`` tag!

Under the ```catkin_ws/src``` workspace, download this repo. Then use ```catkin build``` to compile.

## Run APEX-MR
### Task Planning
To generate the task assignment for lego task, run
```
roslaunch apex_mr lego_assign.launch task:=cliff
```
You should see a message saying lego assignment success.

### Motion Planning and TPG construction
To compute the motion plan for LEGO assembly and build the corresponding TPG for asynchronous execution, run
```
roslaunch apex_mr lego.launch task:=cliff
```
You should see a LEGO planning success
#### Planning params
```vmax:=X``` to set maximum velocity scale, default ```X = 1``` <br>
```adg_shortcut_time:=X``` to set the time for TPG shortcutting, default ```X = 1s``` <br>
```sync_plan:=true``` to run the sync planning baseline <br>
```sync_shortcut_time:=X``` to set the time for RRT-Connect,shortcutting, default ```X = 0.1s``` <br>

Target LEGO assembly is specified under ```config/lego_tasks/assembly_tasks```. <br>
Description of the LEGO assembly plate is specified under ```config/env_setup/assembly_tasks```

### Visualizing in Gazebo
By default, APEX-MR uses the fake hardware in Moveit for collision checks and Rviz for visualization. Optionally, it is also possible to visualize the LEGO assembly environment in Gazebo.

To use Gazebo, first launch the simulator
```
roslaunch robot_digital_twin dual_gp4.launch 
```
This may take some time. Then you can run the motion planning command earlier and wait for the TPG to be generated. Once the TPG is computed, the robots should be displayed in both Rviz and Gazebo at the same time.

### Running the experiments in the paper
We provide a script to run all 9 LEGO assemblies from the paper in simulation
```
python3 scripts.py/benchmark.py
```

Note that the exact results may be slightly different from the numbers reported in our paper.


## Core Documentation

1. **[Configuration and Input Files](docs/01-configuration.md)** - JSON configuration system for tasks, environments, and robot properties
2. **[Task Assignment and Integer Linear Programming](docs/02-task-assignment.md)** - How `lego_assign` performs  task allocation
3. **[Motion Planning and TPG Construction](docs/03-motion-planning.md)** - How `lego_node` builds Temporal Plan Graphs
4. **[Execution Framework](docs/04-execution.md)** - Lego policies and asynchronous coordination

## System Overview

APEX-MR implements a complete pipeline for multi-robot cooperative assembly:

1. **Input Processing**: Task descriptions, environment setup, and robot calibration from JSON files
2. **Task Assignment**: Integer Linear Programming to optimally assign assembly steps to robots
3. **Motion Planning**: Sequential motion planning with collision avoidance and temporal constraints
4. **TPG Construction**: Building Temporal Plan Graphs for asynchronous execution
5. **Execution**: Real-time coordination using motion policies and force feedback


```
APEX-MR Code Architecture

┌─────────────────────────────────────────────────────────────────┐
│                    Configuration                                │
│  JSON configs, LEGO library, robot properties, calibration      │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                    Task Assignment                              │
│  Integer Linear Programming, pose optimization, stability       │
│  Classes: TaskAssignment, stability_node.py, task_assignment.py │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                    Motion Planning                              │
│  Sequential planning, collision checking, trajectory generation │
│  Classes: DualArmPlanner                                        │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                    TPG Construction                             │
│  Temporal Plan Graphs building, shortcutting, optimization      │
│  Classes: TPG, ADG, ShortcutSampler, ActivityGraph              │
└─────────────────────────────────────────────────────────────────┘
                                │
┌─────────────────────────────────────────────────────────────────┐
│                    TPG Execution                                │
│  Asynchronous coordination, motion policies, force control      │
│  Classes: LegoPolicy                                            |
└─────────────────────────────────────────────────────────────────┘
```

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
