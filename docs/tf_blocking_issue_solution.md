# TF Blocking Issue - Root Cause and Solution

## Problem Description
When SLAM creates keyframes, `base_link` stops moving smoothly and jumps/freezes instead of continuous 50Hz movement.

## Root Cause
**GTSAM optimization blocks the main thread during keyframe processing**

```yaml
# The culprit in slam_config.yaml:
optimize_every_n_frames: 1  # This was causing optimization on EVERY keyframe!
```

### What was happening:
1. Keyframe created → SLAM calls `ConeMapping::add_keyframe()`
2. Every frame, `optimize()` is called (because `optimize_every_n_frames: 1`)
3. GTSAM ISAM2 optimization runs on main thread
4. During optimization, entire ROS2 executor is blocked
5. EKF cannot publish `odom → base_link` transform
6. Result: `base_link` freezes until optimization completes

## Immediate Solution (Applied)
Changed `optimize_every_n_frames` from 1 to 10:
```yaml
optimize_every_n_frames: 10  # Now optimizes every 10 keyframes
```

This reduces blocking frequency by 10x while still maintaining map quality.

## Long-term Solutions

### 1. Async Optimization (Recommended)
Implement asynchronous optimization similar to `AsyncConeOdometry`:
```cpp
class AsyncConeMapping : public ConeMapping {
  std::thread optimization_thread_;
  std::queue<OptimizationRequest> request_queue_;
  
  void optimize_async() {
    // Run optimization in separate thread
    // Publish results when complete
  }
};
```

### 2. Multi-threaded Executor
Use ROS2 multi-threaded executor to prevent blocking:
```cpp
// In main()
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(slam_node);
executor.spin();
```

### 3. Separate Optimization Node
Create dedicated optimization node that receives factor graph updates via ROS2 topics.

### 4. Fixed-rate Optimization Timer
Instead of frame-based triggering, use time-based:
```cpp
optimization_timer_ = create_wall_timer(
  1.0s,  // Optimize every 1 second
  [this]() { this->optimize_if_needed(); }
);
```

## Testing
After applying the fix:
```bash
# Terminal 1
ros2 launch gps_imu_fusion ekf_fusion_nocheon.launch.py

# Terminal 2  
ros2 launch cone_stellation slam_only_launch.py

# Terminal 3 - Monitor TF
ros2 run tf2_ros tf2_echo odom base_link

# Should see continuous updates at ~50Hz even during keyframe creation
```

## Performance Considerations
- `optimize_every_n_frames: 10` is a good balance
- Can increase to 20-30 for smoother operation
- Decrease to 5 if map drifts too much between optimizations

## Verification
Monitor optimization timing:
```bash
ros2 topic echo /rosout | grep "Running optimization"
```

Should see optimization messages every 10 keyframes, not every frame.