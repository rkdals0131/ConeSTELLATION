# include/ Directory

This directory contains all public headers for the cone_stellation SLAM system, following a header-only design pattern inspired by GLIM.

## Structure

- **cone_stellation/**: Main namespace directory containing all headers
  - **common/**: Core data structures (cone.hpp, estimation_frame.hpp, tentative_landmark.hpp)
  - **factors/**: GTSAM custom factors (cone_observation_factor.hpp, inter_landmark_factors.hpp)
  - **mapping/**: Mapping and optimization modules
    - cone_mapping.hpp: Main mapping module with ISAM2
    - cone_mapping_safe.hpp: Thread-safe wrapper
    - data_association.hpp: Cone-landmark matching
    - loop_closure_detector.hpp: Loop detection
    - simple_cone_mapping.hpp: Basic mapping implementation
  - **odometry/**: Odometry estimation modules
    - cone_odometry_base.hpp: Abstract base class
    - cone_odometry_2d.hpp: 2D odometry implementation
    - async_cone_odometry.hpp: Asynchronous wrapper
  - **preprocessing/**: Cone data preprocessing (cone_preprocessor.hpp)
  - **util/**: ROS2 utilities
    - ros_utils.hpp: ROS2 helper functions
    - drift_correction_manager.hpp: Map-odom transform calculation
  - **viewer/**: Visualization components
    - viewer_base.hpp: Base visualization class
    - viewer_manager.hpp: Manages multiple viewers
    - cone_viewer.hpp, pose_viewer.hpp, track_viewer.hpp: Specific viewers
    - optimization_viewer.hpp: Factor graph visualization
    - loop_closure_viewer.hpp: Loop closure visualization
    - slam_visualizer.hpp, slam_visualizer_improved.hpp: Complete SLAM visualization
    - visualization_utils.hpp: Helper utilities

## Design Philosophy

Following GLIM's approach, most implementations are header-only for:
- Template flexibility
- Easier integration
- No need for complex linking
- Inline optimization opportunities

## Current Status (2025-07-20)

### Recent Updates
- **cone.hpp**: ✅ FIXED - Co-observation tracking now properly counts observations
  - Added co_observation_counts_ map for actual counting
  - Fixed bug where co_observation_count() only returned 0 or 1
- **mapping/**: ✅ Inter-landmark factors NOW WORKING!
  - Distance factors created between co-observed landmarks
  - Helps maintain track shape (especially curves)
  - Visualization shows red lines for inter-landmark constraints
- **data_association.hpp**: Basic data association module
  - Nearest neighbor matching with color constraints
  - Track ID support integrated
- **odometry/**: Cone-based odometry modules
  - cone_odometry_2d.hpp: 2D implementation using GTSAM
  - async_cone_odometry.hpp: Asynchronous wrapper
- **viewer/**: Separated visualization modules following GLIM architecture
- **util/drift_correction_manager.hpp**: Calculates map->odom transform

### Current Architecture
- Cone observations → Odometry estimation → Mapping with inter-landmark factors
- Drift correction properly calculates map->odom transform
- Real-time optimization with ISAM2
- Track ID utilized for robust data association

### Next Steps
- Implement pattern detection (line, curve, parallel lines)
- Add IMU/GPS integration for high-rate odometry
- Implement loop closure detection
- Performance optimization for larger maps