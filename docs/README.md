# APEX-MR Documentation

Welcome to the comprehensive documentation for APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly.

This documentation explains the technical implementation details corresponding to the RSS 2025 paper "APEX-MR: Multi-Robot Asynchronous Planning and Execution for Cooperative Assembly". Each section explains how the theoretical concepts from the paper are implemented in code. This doucmentaiton is generated with the help of Github Copilot.

## Core Documentation

1. **[Configuration and Input Files](01-configuration.md)** - JSON configuration system for tasks, environments, and robot properties
2. **[Task Assignment and Integer Linear Programming](02-task-assignment.md)** - How `lego_assign` performs optimal task allocation
3. **[Motion Planning and TPG Construction](03-motion-planning.md)** - How `lego_node` builds Temporal Plan Graphss
4. **[Task Execution Framework](04-execution.md)** - Lego policies and asynchronous coordination

## System Overview

APEX-MR implements a complete pipeline for multi-robot cooperative assembly:

1. **Input Processing**: Task descriptions, environment setup, and robot calibration from JSON files
2. **Task Assignment**: Integer Linear Programming to optimally assign assembly steps to robots
3. **Motion Planning**: Sequential motion planning with collision avoidance and temporal constraints
4. **TPG Construction**: Building Temporal Plan Graphss for asynchronous execution
5. **Execution**: Real-time coordination using motion policies and force feedback

## High-Level Architecture

```
APEX-MR System Architecture

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

## Quick Start

For quick setup and usage, see the main [README.md](../README.md) in the root directory.

---

*Last updated: July 2025*
