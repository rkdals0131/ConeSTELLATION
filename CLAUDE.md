# Path: /home/user1/ROS2_Workspace/Symforce_ws/src/cone_stellation/CLAUDE.md

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this directory.

## Package Overview

ConeSTELLATION (Cone-based STructural ELement Layout for Autonomous NavigaTION) is a cone-based Graph SLAM system inspired by GLIM's modular architecture. It processes cone detection instances from LiDAR clustering with YOLO-based color classification instead of raw point clouds.

## Current Status

- **Created**: 2025-07-18
- **Updated**: 2025-07-20
- **Status**: Implementing cone-based odometry and data association
- **Architecture**: Based on GLIM's proven modular design with novel inter-landmark factors
- **Latest Updates**: 
  - ✅ Topic unified to `/fused_sorted_cones_ukf_sim`
  - ✅ Implemented observation buffering system (TentativeLandmark)
  - ✅ Color voting and track ID hysteresis
  - ✅ Fixed color definitions to match track layout
  - ✅ Updated SimpleConeMapping to track colors
  - ✅ Separated visualization to viewer module
  - ✅ Implemented cone-based odometry module
  - ✅ Fixed GTSAM type construction issues
  - ❌ No data association - landmarks duplicated
  - ❌ Blue cones appear intermittently
  - ⏳ Need to implement proper data association
  - ⏳ Need to test and debug odometry estimation

## Project Structure (Current)

```
cone_stellation/
├── include/cone_stellation/    # Public headers (header-only design)
│   ├── common/                 # Core data structures (cone.hpp, estimation_frame.hpp)
│   ├── preprocessing/          # Cone data preprocessing (cone_preprocessor.hpp)
│   ├── mapping/               # SLAM mapping with inter-landmark factors (cone_mapping.hpp)
│   ├── factors/               # Custom GTSAM factors (inter_landmark_factors.hpp)
│   └── util/                  # ROS2 utilities (ros_utils.hpp)
├── src/                       # Implementation files
│   └── cone_slam_node.cpp    # Main ROS2 node
├── config/                    # YAML configuration files
│   ├── slam_config.yaml      # SLAM parameters
│   └── dummy_publisher_config.yaml # Simulation parameters
├── scripts/                   # Python simulation scripts
│   └── dummy_publisher_node.py # Dummy cone publisher for testing
└── launch/                    # ROS2 launch files
    ├── cone_slam_launch.py
    └── dummy_publisher_launch.py
```

## Key Design Decisions

1. **Modular Architecture**: Following GLIM's plugin-based system
2. **Data Flow**: Cone Detection → Preprocessing → Association → Mapping → Optimization
3. **Base Classes**: Abstract interfaces for each module type
4. **Configuration**: JSON-based hierarchical configuration
5. **Threading**: Asynchronous wrappers for real-time performance
6. **Tentative Landmark System**: Observation buffering before landmark creation
   - Prevents false positives from noise
   - Ensures landmarks are well-constrained
   - Color voting for robust classification
   - Track ID hysteresis for occlusion handling

## Development Phases

### Completed Phases ✅
1. **Basic Infrastructure**: Core data structures, simulation, visualization
2. **Core Modules**: ConePreprocessor, ConeMapping with ISAM2
3. **GTSAM Integration**: Custom factors (inter-landmark, observation)
4. **ROS2 Integration**: Basic node, TF, config, visualization
5. **Tentative Landmarks**: Observation buffering, color voting, promotion criteria

### Current Development (Detailed in docs/)
- **Phase 1-4**: Odometry/mapping separation (see `odometry_mapping_separation.md`)
- **Phase 5**: Fixed-lag smoother implementation (see `DEVELOPMENT_PLAN.md`)
- **Phase 6**: Enhanced color voting and track management
- **Phase 7**: Loop closure with cone constellations
- **Phase 8-10**: IMU/GPS integration

### GLIM Features Integration
See `glim_features_integration.md` for detailed roadmap including:
- Multi-threading architecture
- Memory management strategies
- Robust optimization techniques
- Configuration management
- Serialization and recovery

### Implementation Status
See `implementation_status.md` for:
- ✅ Already implemented features
- ❌ Not yet implemented features
- Performance targets and metrics
- Testing requirements

## Dependencies

- **Required**: Eigen, GTSAM, gtsam_points, spdlog, Boost
- **ROS2**: geometry_msgs, nav_msgs, sensor_msgs, tf2
- **Optional**: OpenCV (for visualization), CUDA (future GPU acceleration)

## Key Differences from cc_slam_sym

1. **No SymForce**: Direct GTSAM integration without intermediate code generation
2. **Novel Inter-landmark Factors**: Key innovation for sparse cone observations
   - ConeDistanceFactor: Maintains relative distances between co-observed cones
   - ConeLineFactor: Enforces collinearity for cones on track boundaries
   - Pattern-based factors: Leverages geometric patterns in cone layouts
3. **Header-only Design**: Following GLIM's approach for flexibility
4. **Proven Architecture**: Based on GLIM's successful design patterns

## References

- GLIM architecture: `/home/user1/ROS2_Workspace/GLIM_ws/src/glim/`
- Development plan: `DEVELOPMENT_PLAN.md`