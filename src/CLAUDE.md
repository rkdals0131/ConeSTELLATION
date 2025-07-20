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

## Current Status (2025-07-20)

### Recent Updates
- **cone_slam_node.cpp**: 
  - ✅ Integrated cone-based odometry module (AsyncConeOdometry)
  - ✅ Fixed data flow: Cones → Odometry → Mapping
  - ✅ Removed dependency on ground truth TF
  - ✅ Added odometry publisher (/slam/odometry)
  - ✅ Uses estimated poses from cone matching
  - Added support for SimpleConeMapping mode
  - Separated visualization to viewer module (SLAMVisualizer)

### Fixed Issues
- ✅ **Critical**: Now using cone-based odometry estimation
- ✅ Data flow corrected to match GLIM architecture
- ✅ No longer relies on external TF for pose estimation

### Next Steps
- Build and test the complete system
- Fine-tune odometry parameters
- Enable inter-landmark factors
- Add loop closure detection