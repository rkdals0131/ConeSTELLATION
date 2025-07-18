# ConeSTELLATION: Cone-based STructural ELement Layout for Autonomous NavigaTION

A modular cone-based Graph SLAM system for Formula Student Driverless, inspired by GLIM's architecture.

## Overview

ConeSTELLATION processes cone detection instances from LiDAR clustering with YOLO-based color classification to build a consistent map and estimate vehicle pose in real-time. Unlike traditional point cloud SLAM, it works directly with detected cone objects, making it more efficient and suitable for structured environments like racing tracks.

## Features

- **Modular Architecture**: Plugin-based system allowing easy algorithm swapping
- **Real-time Performance**: Asynchronous processing pipeline
- **Robust Data Association**: Color-aware cone matching with outlier rejection
- **Graph Optimization**: GTSAM-based backend for global consistency
- **ROS2 Integration**: Native ROS2 support with standard message types
- **Extensible Design**: Easy to add new modules and algorithms

## Architecture

The system follows GLIM's proven modular design:

```
Cone Detections → Preprocessing → Association → Local Mapping → Global Mapping
                       ↓              ↓              ↓               ↓
                   Callbacks      Callbacks      Callbacks       Callbacks
```

## Building

```bash
cd ~/ROS2_Workspace/Symforce_ws
colcon build --packages-select cone_stellation --symlink-install
```

## Documentation

- Development Plan: [DEVELOPMENT_PLAN.md](DEVELOPMENT_PLAN.md)
- Architecture Guide: [CLAUDE.md](CLAUDE.md)
- Debug Log: [debug_log.md](debug_log.md)

## Status

Currently in initial development phase. See DEVELOPMENT_PLAN.md for roadmap.
