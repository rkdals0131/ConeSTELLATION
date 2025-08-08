# Drift Correction Fix - Using Latest Keyframe Instead of Optimized Pose

## Problem
SLAM uses only OPTIMIZED poses from GTSAM for drift correction, which only update when `optimize()` is called.
This causes base_link to appear stuck in map frame between optimizations.

## Root Cause
```cpp
// Current problematic code in visualization_callback:
auto values = mapping_->get_current_estimate();  // Returns OPTIMIZED values only!
// These only update when optimize() is called
```

## Solution Options

### Option 1: Use Latest Keyframe Pose (Recommended)
Track the latest keyframe pose separately and use it for drift correction:

```cpp
class ConeSLAMNode {
  // Add member variable
  Eigen::Isometry3d latest_slam_pose_;  // Always updated with new keyframes
  
  void cone_callback() {
    if (is_keyframe) {
      // ... add keyframe ...
      
      // Always update latest SLAM pose
      latest_slam_pose_ = sensor_pose;  // Use current pose, not optimized
    }
  }
  
  void visualization_callback() {
    // Use latest keyframe pose for drift correction
    drift_manager_->update_slam_pose(current_time, latest_slam_pose_);
    
    // Only use optimized values for visualization of historical poses
    auto values = mapping_->get_current_estimate();
    // ... visualize optimized trajectory ...
  }
}
```

### Option 2: Continuous Drift Interpolation
Interpolate drift correction between optimizations:

```cpp
class DriftCorrectionManager {
  void interpolate_drift(double alpha) {
    // Smoothly interpolate between last optimized and current odometry
    T_map_odom_interpolated = (1-alpha) * T_map_odom_optimized + alpha * T_map_odom_current;
  }
}
```

### Option 3: Return to optimize_every_n_frames: 1
Simple but computationally expensive. Not recommended for real-time operation.

### Option 4: Separate Visualization from Optimization
Run optimization in separate thread and always use latest poses for visualization:

```cpp
class AsyncConeMapping : public ConeMapping {
  // Latest poses (always updated)
  std::map<int, Eigen::Isometry3d> latest_poses_;
  
  // Optimized poses (updated asynchronously)
  std::map<int, Eigen::Isometry3d> optimized_poses_;
  
  void add_keyframe(frame) {
    latest_poses_[frame->id] = frame->T_world_sensor;
    optimization_queue_.push(frame);
  }
  
  Eigen::Isometry3d get_latest_pose(int id) {
    return latest_poses_[id];  // Always returns most recent
  }
}
```

## Immediate Fix (Option 1 Implementation)

1. In `cone_slam_node.cpp`, add member variable:
```cpp
Eigen::Isometry3d latest_slam_pose_ = Eigen::Isometry3d::Identity();
```

2. In `cone_callback`, after adding keyframe:
```cpp
if (is_keyframe) {
  // ... existing keyframe code ...
  latest_slam_pose_ = sensor_pose;  // Track latest keyframe pose
}
```

3. In `visualization_callback`, replace drift correction update:
```cpp
// Instead of using optimized pose from GTSAM
// drift_manager_->update_slam_pose(current_time, T_map_base_from_optimization);

// Use latest keyframe pose
drift_manager_->update_slam_pose(current_time, latest_slam_pose_);
```

This ensures map->odom transform updates with every keyframe, not just optimizations.

## Testing
After applying fix:
1. Set `optimize_every_n_frames: 10` (or higher)
2. Run SLAM with EKF
3. base_link should move smoothly at 50Hz
4. Keyframes should update drift correction immediately
5. Optimization should refine poses without causing jumps