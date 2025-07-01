# Task Execution Framework

This document explains the runtime system that coordinates asynchronous multi-robot assembly using the Temporal Plan Graph (TPG) and Lego manipulation policies.

## Overview

The execution framework provides:

1. **Asynchronous Coordination**: Robots execute independently while respecting TPG constraints
2. **Lego Policies**: Learned motion primitives for LEGO assembly operations
3. **Force Feedback**: Force/torque sensing for LEGO manipulation

## Policy Framework

### Base Policy Class

The policy system is defined in `include/policy.h` and provides an abstract interface for executing different types of robot actions.

### LEGO-Specific Policies

The LEGO assembly policies are implemented in `include/lego_policy.h` and provide force-controlled manipulation primitives specifically designed for LEGO assembly operations.

We implement pick, place, support, and handover primities with force control integration. The press down pose are dynamically determined, based on a force control threshold.


## TPG Execution Coordination

### TPG Executor

The TPG executor coordinates asynchronous execution by managing node dependencies, checking resource conflicts, and ensuring proper sequencing of operations across multiple robots.

### Asynchronous Execution Loop

Each robot runs in its own execution thread, waiting for dependencies to be satisfied before executing its assigned TPG nodes. This enables parallel execution while maintaining safety constraints.

### Dependency Checking

The system continuously checks that all predecessor nodes are completed  before allowing a node to execute.


## Integration with Robot Hardware

### MFI Robot Integration

When available, the system integrates with MFI robot hardware at Mill 19 using position controller given joint trajectories or single joint pose

### Simulation Interface

For testing and development, the system provides a simulation interface using MoveIt for trajectory execution and planning scene updates.

## Usage

Execute a task with the generated TPG:

```bash
# For simulation digital twin
roslaunch robot_digital_twin dual_gp4.launch
# For execution at on real hardware
roslaunch apex_mr lego.launch task:=cliff load_adg:=true async:=true mfi:=true
```

Parameters:
- `task`: Task name
- `async`: Use asynchronous execution (each robot running in its own thread) (default: false)
- `mfi`: Use MFI robot interface (default: false)
