# Debugging TF Freeze Issue

## Problem
When SLAM processes keyframes, `odom → base_link` TF from EKF stops updating, even though they are separate processes.

## Debugging Steps

### 1. Monitor TF Publishing Rate
```bash
# Terminal 1: Check if EKF is actually publishing
ros2 topic hz /tf
ros2 run tf2_ros tf2_monitor odom base_link

# Terminal 2: Check CPU usage
htop  # Watch for 100% CPU during keyframe processing
```

### 2. Check for TF Conflicts
```bash
# See all TF publishers
ros2 run tf2_tools view_frames
# Check if multiple nodes publish same transform
```

### 3. Test Without Visualization
```bash
# Disable SLAM visualization to isolate issue
ros2 param set /cone_slam visualization.publish_rate 0.0

# Or run without RViz
```

### 4. Add Debug Logging
In `cone_slam_node.cpp`:
```cpp
void cone_callback() {
    auto start = std::chrono::high_resolution_clock::now();
    
    if (is_keyframe) {
        RCLCPP_INFO(this->get_logger(), "Keyframe processing START");
        mapping_->add_keyframe(frame);
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        RCLCPP_WARN(this->get_logger(), "Keyframe processing took %ld ms", duration.count());
    }
}
```

### 5. Profile Data Association
In `cone_mapping.hpp`:
```cpp
void process_cone_observations() {
    auto start = std::chrono::high_resolution_clock::now();
    // ... existing code ...
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    RCLCPP_INFO(get_logger(), "Data association took %ld ms", duration.count());
}
```

## Potential Solutions

### Solution 1: Async Keyframe Processing
```cpp
class ConeSLAMNode {
    std::thread keyframe_thread_;
    std::queue<EstimationFrame::Ptr> keyframe_queue_;
    
    void cone_callback() {
        if (is_keyframe) {
            keyframe_queue_.push(frame);  // Don't block main thread
        }
    }
    
    void keyframe_processor() {
        while (rclcpp::ok()) {
            if (!keyframe_queue_.empty()) {
                auto frame = keyframe_queue_.front();
                keyframe_queue_.pop();
                mapping_->add_keyframe(frame);  // Process in background
            }
        }
    }
};
```

### Solution 2: Reduce Visualization Load
```cpp
// Limit number of markers
const size_t MAX_MARKERS = 100;
if (markers.size() > MAX_MARKERS) {
    markers.resize(MAX_MARKERS);  
}
```

### Solution 3: Multi-threaded Executor
```cpp
int main() {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConeSLAMNode>();
    
    // Use multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
```

## Most Likely Cause
**Heavy computation in `process_cone_observations()` blocks visualization_callback from running, which prevents TF publishing.**

Even though EKF publishes `odom → base_link` at 50Hz, if RViz or the TF listener is blocked processing SLAM's heavy visualization, it appears frozen.