# src/ Directory

This directory contains implementation files for the cone_stellation SLAM system.

## Structure

- **cone_stellation/**: Main source directory
  - **ros/**: ROS2 specific implementations
    - `cone_slam_node.cpp`: Main ROS2 node that orchestrates SLAM
  - **common/**: Common utilities implementation (if needed)
  - **factors/**: Factor implementations (currently header-only)
    - `inter_landmark_factors.cpp`: Inter-landmark factor implementations
  - **preprocessing/**: Preprocessing implementations
    - `cone_preprocessor.cpp`: Cone preprocessing logic
  - **mapping/**: Mapping module implementations (currently header-only)
  - **util/**: Utility implementations (if needed)

## Implementation Notes

- Most logic is in headers (header-only design)
- Source files mainly for:
  - ROS2 node implementations
  - Complex algorithms that benefit from separate compilation
  - Reducing compilation time for frequently used headers

## Current Status

- Basic ROS2 node implemented
- Minimal source files (most in headers)
- Need to add odometry and submap implementations