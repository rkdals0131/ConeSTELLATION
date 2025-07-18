# Path: /home/user1/ROS2_Workspace/Symforce_ws/src/cone_stellation/CLAUDE.md

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this directory.

## Package Overview

ConeSTELLATION (Cone-based STructural ELement Layout for Autonomous NavigaTION) is a cone-based Graph SLAM system inspired by GLIM's modular architecture. It processes cone detection instances from LiDAR clustering with YOLO-based color classification instead of raw point clouds.

## Current Status

- **Created**: 2025-07-18
- **Status**: Initial planning phase
- **Architecture**: Based on GLIM's proven modular design

## Project Structure (Planned)

```
cone_stellation/
├── include/cone_stellation/    # Public headers
│   ├── common/                 # Core data structures
│   ├── preprocessing/          # Cone data preprocessing
│   ├── odometry/              # Cone-based pose estimation (includes association)
│   ├── mapping/               # Local and global mapping (includes loop closure)
│   ├── util/                  # Utilities and infrastructure
│   └── viewer/                # Visualization
├── src/cone_stellation/       # Implementation files
├── config/                    # JSON configuration files
├── modules/                   # Dynamic loadable modules
├── scripts/                   # Simulation scripts (from cc_slam_sym)
├── test/                      # Unit and integration tests
└── launch/                    # ROS2 launch files
```

## Key Design Decisions

1. **Modular Architecture**: Following GLIM's plugin-based system
2. **Data Flow**: Cone Detection → Preprocessing → Association → Mapping → Optimization
3. **Base Classes**: Abstract interfaces for each module type
4. **Configuration**: JSON-based hierarchical configuration
5. **Threading**: Asynchronous wrappers for real-time performance

## Development Phases

1. **Phase 1**: Basic infrastructure and data structures (Current)
   - ✅ Simulation integration complete
   - ✅ Enhanced visualization module created
2. **Phase 2**: Core module implementations
3. **Phase 3**: GTSAM integration
4. **Phase 4**: Asynchronous processing
5. **Phase 5**: ROS2 integration
6. **Phase 6**: Visualization and debugging tools

## Dependencies

- **Required**: Eigen, GTSAM, gtsam_points, spdlog, Boost
- **ROS2**: geometry_msgs, nav_msgs, sensor_msgs, tf2
- **Optional**: OpenCV (for visualization), CUDA (future GPU acceleration)

## Key Differences from cc_slam_sym

1. **No SymForce**: Direct GTSAM integration without intermediate code generation
2. **Modular Design**: Plugin-based architecture for easy algorithm swapping
3. **Proven Architecture**: Based on GLIM's successful design patterns
4. **Clear Separation**: Core library separate from ROS2 wrapper

## References

- GLIM architecture: `/home/user1/ROS2_Workspace/GLIM_ws/src/glim/`
- Development plan: `DEVELOPMENT_PLAN.md`