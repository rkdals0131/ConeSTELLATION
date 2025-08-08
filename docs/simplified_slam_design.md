# Simplified SLAM Design (No Data Association)

## Overview
A simplified SLAM implementation that bypasses complex data association logic and directly adds all incoming observations to the factor graph. This approach is designed to work with external odometry from EKF and cone observations without matching.

## Current System Analysis

### Launch Files
1. **slam_only_launch.py**
   - Launches cone_slam_node
   - Loads slam_config.yaml
   - Expects external odometry

2. **ekf_fusion_nocheon.launch.py**
   - Runs EKF fusion node (ekf_fusion_node)
   - Publishes `/odometry/filtered` at 50Hz
   - Manages TF tree: map → odom → base_link
   - Runs cone_frame_transformer for `/fused_sorted_cones_ukf_map`

### Current Data Flow
```
/odometry/filtered (50Hz) → ConeSLAMNode::odom_callback
/fused_sorted_cones_ukf_map → ConeSLAMNode::cone_callback
                                    ↓
                            ConePreprocessor
                                    ↓
                            Data Association (Complex!)
                                    ↓
                            ConeMapping::add_keyframe
                                    ↓
                            GTSAM Factor Graph
```

## Simplified Design

### Key Changes
1. **Remove Data Association**: No matching between observations and landmarks
2. **Direct Landmark Creation**: Every cone observation creates a new landmark
3. **Simple Factor Graph**: Only odometry and observation factors
4. **Reduced Processing Rate**: Downsample to 20Hz or less

### Simplified Data Flow
```
/odometry/filtered (50Hz) → Buffer/Downsample → Add Pose Node (20Hz)
/fused_sorted_cones_ukf_map → Direct Processing → Add Landmark Nodes
                                                        ↓
                                                  Add Observation Edges
                                                        ↓
                                                  GTSAM Optimization
                                                        ↓
                                                  RViz Visualization
```

## Implementation Plan

### Step 1: Create SimplifiedSLAMNode
Create a new simplified node that:
- Subscribes to `/odometry/filtered` and `/fused_sorted_cones_ukf_map`
- Downsamples odometry to 20Hz
- Creates pose nodes for each odometry update
- Creates landmark nodes for each cone observation
- No data association or tentative landmarks

### Step 2: Factor Graph Structure
```cpp
// Pose nodes: x0, x1, x2, ...
// Landmark nodes: l0, l1, l2, ...

// Factor types:
1. Prior factor on x0 (first pose)
2. Odometry factors between consecutive poses (xi → xi+1)
3. Observation factors from poses to landmarks (xi → lj)
```

### Step 3: Key Simplifications
```cpp
class SimplifiedSLAMNode : public rclcpp::Node {
  void odom_callback(nav_msgs::msg::Odometry) {
    // Downsample to 20Hz
    if (time_since_last_pose < 0.05) return;
    
    // Add pose node
    Symbol pose_key('x', next_pose_id_++);
    
    // Add odometry factor from previous pose
    if (next_pose_id_ > 0) {
      add_odometry_factor(prev_pose, current_pose);
    }
    
    last_pose_key_ = pose_key;
  }
  
  void cone_callback(TrackedConeArray) {
    // No data association!
    for (const auto& cone : msg->cones) {
      // Create new landmark for each observation
      Symbol landmark_key('l', next_landmark_id_++);
      
      // Transform cone to world frame using current pose
      Vector2d world_pos = transform_to_world(cone, current_pose);
      
      // Add landmark to graph
      initial_values_.insert(landmark_key, Point2(world_pos));
      
      // Add observation factor
      add_observation_factor(last_pose_key_, landmark_key, cone);
    }
  }
};
```

## TF Tree Management

### Current Issues
- Complex TF lookups cause timing problems
- map → odom transform conflicts between EKF and SLAM

### Simplified Approach
1. **EKF manages**: odom → base_link (from `/odometry/filtered`)
2. **SLAM publishes**: map → odom (identity initially, then drift correction)
3. **Static TFs**: base_link → sensors (from launch file)

## Configuration

### Simplified slam_config.yaml
```yaml
slam:
  # Downsampling
  pose_rate_hz: 20.0  # Downsample odometry to this rate
  
  # GTSAM parameters
  isam2_relinearize_threshold: 0.1
  isam2_relinearize_skip: 10
  
  # Noise models (simplified)
  odometry_noise: [0.1, 0.1, 0.05]  # x, y, theta
  observation_noise: [0.3, 0.3]      # range, bearing or x, y
  
  # Optimization
  optimize_every_n_poses: 10
  
  # Visualization
  publish_rate_hz: 10.0
  max_landmarks_visualized: 1000
```

## Benefits

### Advantages
1. **Simplicity**: No complex data association logic
2. **Robustness**: Can't fail on association errors
3. **Fast Development**: Quick to implement and test
4. **Debugging**: Easy to understand factor graph structure

### Disadvantages
1. **Memory**: Unbounded landmark growth
2. **Accuracy**: No loop closure or landmark merging
3. **Optimization Cost**: Graph grows quickly

## Testing Strategy

### Phase 1: Basic Functionality
- Verify pose nodes are created at 20Hz
- Confirm landmark nodes are created for each cone
- Check factor graph connectivity
- Validate RViz visualization

### Phase 2: Integration Testing
```bash
# Terminal 1: Launch EKF
ros2 launch gps_imu_fusion ekf_fusion_nocheon.launch.py

# Terminal 2: Launch simplified SLAM
ros2 launch cone_stellation simplified_slam_launch.py

# Terminal 3: Play bag file
ros2 bag play your_bag_file.bag --clock
```

### Expected Outputs
- `/slam/pose`: Current robot pose estimate
- `/slam/landmarks`: All landmark positions
- `/slam/path`: Robot trajectory
- `/slam/visualization_markers`: RViz markers for debugging

## Migration Path

### From Simplified to Full SLAM
Once TF issues are resolved:
1. Add data association module
2. Implement tentative landmarks
3. Enable inter-landmark factors
4. Add loop closure detection

## File Modifications Needed

### New Files
- `include/cone_stellation/mapping/simplified_slam_node.hpp`
- `src/cone_stellation/ros/simplified_slam_node.cpp`
- `launch/simplified_slam_launch.py`
- `config/simplified_slam_config.yaml`

### Modified Files
- `CMakeLists.txt`: Add new executable
- `package.xml`: No changes needed

## Summary

This simplified approach removes the complexity of data association and focuses on:
1. Reliable odometry integration (50Hz → 20Hz)
2. Direct landmark creation from observations
3. Simple factor graph with only odometry and observation factors
4. Clear visualization in RViz

The system can be progressively enhanced once basic functionality is verified.