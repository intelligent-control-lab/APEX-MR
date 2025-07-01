# Task Assignment and Integer Linear Programming

This document explains how the `lego_assign` executable performs optimal task allocation using Integer Linear Programming (ILP) to assign assembly steps to robots while minimizing execution time and ensuring feasibility.

## Overview

The task assignment phase takes as input:
- Assembly task descriptions (target LEGO structure)
- Environment setup (initial brick placements)
- Robot kinematics and capabilities

And produces:
- Optimal robot assignments for each assembly step
- Goal poses for each robot and each task

## Algorithm

The task assignment is formulated as an Integer Linear Programming problem that optimizes:

1. **Makespan Minimization**: Minimize total sequential assembly time
2. **Feasibility Constraints**: Ensure all assignments are kinematically feasible and collision free
3. **Stability Constraints**: Respect stability requirements
4. **Resource Allocation**: Load balance between robots

## Implementation in `lego_assignment.cpp`

### Main Class: `TaskAssignment`

```cpp
class TaskAssignment {
public:
    TaskAssignment(const std::string &output_dir,
                  const std::string &task_name,
                  const std::vector<std::string> &group_names,
                  const std::vector<std::string> &eof_names,
                  bool motion_plan_cost,
                  bool check_stability,
                  bool optimize_poses,
                  bool print_debug);
```

**Key Parameters**:
- `motion_plan_cost`: Include motion planning cost in optimization
- `check_stability`: Use Gurobi for stability analysis (requires license)
- `optimize_poses`: Search over multiple grasp poses (P=28 vs P=1)
- `print_debug`: Output detailed optimization information

## ILP Solver Integration

The system uses two optimization approaches:

#### 1. ROS Service Integration
```cpp
apex_mr::TaskAssignment assignment_srv;
assignment_srv.request.task_name = task_name_;
assignment_srv.request.optimize_poses = optimize_poses_;
assignment_srv.request.check_stability = check_stability_;

if (assignment_client_.call(assignment_srv)) {
    // Process optimization results
}
```

#### 2. Direct Gurobi Integration (when available)
When Gurobi is available with a license, the system can perform detailed stability analysis and pose optimization over all 28 possible grasp poses per LEGO brick.

## Motion Planning Integration

### Feasibility Checking

For each potential assignment, the system:

1. **Computes Inverse Kinematics** for target poses
2. **Checks Collision-Free Paths** between configurations
3. **Estimates Motion Cost** for trajectory optimization

### Pose Generation

The system generates multiple candidate poses for each assembly operation based on different approach angles and grasp strategies. When `optimize_poses=true`, it searches over all possible poses; otherwise uses P=1 default pose generated from our assembly planner.

## Stability Analysis

When Gurobi is available, the system performs stability analysis using the [StableLego](https://github.com/intelligent-control-lab/StableLego) tool

## Output Generation

After optimization, the system generates two types of output:

### 1. Sequence Assignment (`*_seq.json`)

Enhanced assembly task with robot assignments:

```json
{
    "1": {
        "x": 24, "y": 28, "z": 1,    // Same as input task
        "brick_id": 2, "ori": 0,
        // ... other fields from assembly task ...
        "robot_id": 1,               // Assigned robot (0 or 1)
        "sup_robot_id": 0           // Supporting robot (if needed)
    }
}
```

### 2. Motion Waypoints (`*_steps.csv`)

Detailed motion trajectories with robot poses and joint angles for each waypoint.

## Usage

Run task assignment:

```bash
roslaunch apex_mr lego_assign.launch task:=cliff
```

Parameters:
- `task`: Task name (e.g., cliff, faucet, fish_high)
- `optimize_poses`: Enable pose optimization (default: false)
- `check_stability`: Enable Gurobi stability analysis (default: true)
- `motion_plan_cost`: Include motion planning in optimization (default: false)

Next: [Motion Planning and TPG Construction](03-motion-planning.md)
