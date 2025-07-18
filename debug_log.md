# ConeSTELLATION Debug Log

## 2025-07-18 - Initial Project Setup
- **Issue**: Starting fresh cone-based SLAM implementation
- **Problem**: Previous cc_slam_sym implementation with SymForce had divergence issues
- **Solution**: Adopting GLIM's proven modular architecture for cone-based SLAM
- **Result**: Created development plan and project structure based on GLIM's design patterns

## 2025-07-18 - Simulation Integration Complete
- **Issue**: Need to integrate existing cc_slam_sym simulation for testing
- **Problem**: Visualization dependencies and import paths needed adjustment
- **Solution**: 
  - Copied simulation scripts from cc_slam_sym to cone_stellation/scripts/
  - Created VisualizationAdapter to replace missing visualization utilities
  - Updated CMakeLists.txt and package.xml for Python support
  - Fixed import paths for standalone execution
- **Result**: Simulation infrastructure ready for use, can run dummy_publisher_node.py

## 2025-07-18 23:15:00 - Visualization Module Creation
- **Issue**: Need visualization utilities for cone_stellation with factor graph support
- **Problem**: Original visualization.py from cc_slam_sym lacks factor graph edge visualization
- **Solution**: Created enhanced visualization module with:
  - FactorGraphVisualizer class for edge rendering
  - Performance optimizations (visibility culling, edge decimation)
  - Batch rendering by edge type and cone color
  - SLAMVisualizationManager for high-level control
- **Result**: New visualization system ready for factor graph SLAM with performance considerations

## 2025-07-18 23:30:00 - TF Transform Analysis
- **Issue**: User asked about map->odom->base_link TF chain not being published
- **Problem**: Understanding if the TF structure is correct for SLAM
- **Solution**: Analyzed dummy_publisher TF publishing:
  - odom->base_link: Published by dummy_publisher (noisy odometry)
  - map->odom: Intentionally NOT published (SLAM node's responsibility)
  - Cone detections correctly transformed from map to base_link frame
- **Result**: Confirmed user's understanding is correct - this is proper ROS2 SLAM architecture

## 2025-07-18 23:45:00 - Dummy Publisher Debug
- **Issue**: TF transforms and marker topics not being published
- **Problem**: Multiple issues with cone_stellation dummy_publisher adaptation
- **Solution**: 
  - Fixed VisualizationHelper -> VisualizationAdapter references
  - Added missing create_roi_marker and create_path_marker methods
  - Fixed sensor_sim initialization with vehicle_state
  - Added sensor_sim.update_vehicle_state() in update_motion
  - Added comprehensive debug logging
- **Result**: Node should now publish TF and markers properly - need to re-run to verify

## 2025-07-18 23:55:00 - Critical Data Format Fix
- **Issue**: Dummy publisher still failing after initial fixes
- **Problem**: VisualizationAdapter's publish_cone_array expected dict but received list
- **Solution**: 
  - Modified publish_ground_truth_cones to convert dict to list (matching original)
  - Updated VisualizationAdapter.publish_cone_array to process list of dicts
  - Changed cone_data['pos'] to cone['pos'] references
  - Changed cone_id to cone.get('id', idx) for proper list handling
- **Result**: Data format mismatch resolved - ready for testing

## 2025-07-19 00:00:00 - TF Tree Completion for Standalone Testing
- **Issue**: TF tree broken when running dummy_publisher without SLAM
- **Problem**: map->odom transform missing, causing visualization issues in RViz
- **Solution**: 
  - Added publish_map_to_odom parameter (default: true)
  - Publishes identity map->odom transform when SLAM not running
  - Warns user to disable when running with SLAM to avoid conflicts
  - Added parameter to config file for easy toggling
- **Result**: Complete TF tree for standalone testing, configurable for SLAM integration

## 2025-07-19 00:10:00 - Marker Visualization Fixes
- **Issue**: Detected cones showing as opaque cylinders instead of semi-transparent spheres
- **Problem**: Wrong marker type and alpha value in publish_detected_cones
- **Solution**: 
  - Changed marker.type from CYLINDER to SPHERE
  - Changed marker.color.a from 1.0 to 0.4 (semi-transparent)
  - Added debug logging for publish_map_to_odom parameter
- **Result**: Detected cones now properly display as semi-transparent spheres