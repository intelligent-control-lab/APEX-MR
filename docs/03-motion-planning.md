# Motion Planning and TPG Construction

This document explains how the `lego_node` executable performs sequential motion planning and constructs Temporal Plan Graphs (TPGs) for asynchronous execution. This implements the core algorithmic contribution from the APEX-MR paper.

## Overview

The motion planning phase takes the task assignment output and:

1. **Sequential Motion Planning**: Plans collision-free motions for each robot sequentially
2. **TPG Construction**: Builds a Temporal Plan Graphs encoding temporal dependencies
3. **Shortcutting**: Optimizes the TPG to reduce makespan while maintaining safety
4. **Serialization**: Saves the TPG to disk for offline execution

#### Implementation in `lego_node.cpp`

## Sequential Motion Planning

### Planning Pipeline

The system plans motions sequentially while maintaining a shared collision environment. Each assembly step is planned in sequence, with the planning scene updated after each step to reflect the new object positions.

### Motion Planning with MoveIt

The system uses MoveIt for motion planning, setting start states from the current planning scene and goal constraints based on the assembly operation type (pick, place, etc.). Each planned trajectory is validated for safety before being stored for TPG construction.

### Collision Environment Management

The system maintains a dynamic collision environment that is updated as objects are picked and placed during the assembly sequence. Objects are attached to robots when picked and detached when placed at their final positions.


## Temporal Plan Graphs Construction

### TPG Data Structure

The TPG is implemented in `include/tpg.h` and `include/adg.h`. It represents the temporal dependency structure for asynchronous execution. Each node represents a robot action, and edges represent dependencies between actions. The ADG class is the multi-modal extension of TPG, which includes a more complete task graph and kinematic switches informations. 

### Building the TPG

The TPG construction process creates nodes for each motion segment and adds dependencies based on:

1. **Task Dependencies**: Structural constraints requiring certain bricks to be placed before others, and coordination requirements where one robot must support while another places
2. **Motion Dependencies**:  constraints between robots to avoid conflicts

### Node Creation from Trajectories

Motion trajectories are decomposed into TPG nodes representing approach transition, manipulation actions, and transition back to home poses. Each node contains the robot configuration.

## TPG Serialization

The TPG is serialized using Boost for offline storage, allowing the motion planning phase to be separated from the execution phase. The serialized TPG contains all necessary information for asynchronous execution.

## Usage

Run motion planning and TPG construction:

```bash
roslaunch apex_mr lego.launch task:=cliff
```

Parameters:
- `task`: Task name
- `vmax`: Velocity scaling (default: 1.0)
- `sync_shortcut_time`: individual path shortcutting for each single-agent(or synchronized dual-arm) segment (default: 0.1s)
- `adg_shortcut_time`: Shortcutting time limit (default: 1s)
- `sync_plan`: Use synchronous baseline instead of TPG (default: false)

The generated TPG serves as input to the execution system, enabling offline planning with online asynchronous execution.

Next: [Task Execution Framework](04-execution.md)
