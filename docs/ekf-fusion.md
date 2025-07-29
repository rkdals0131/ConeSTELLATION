# EKF Fusion Implementation Documentation

## Overview

ConeSTELLATION employs a hybrid architecture that separates high-rate sensor fusion from SLAM optimization. This document details the Extended Kalman Filter (EKF) implementation using the `robot_localization` package for fusing IMU and GPS data at 100Hz, providing stable odometry for vehicle control while SLAM handles global consistency.

## Architecture Decision

### Why External EKF?

The decision to use an external EKF for odometry follows GLIM's proven architecture:

1. **Control Stability**: Vehicle control requires consistent 100Hz odometry
2. **Computational Efficiency**: Separates high-rate fusion from SLAM optimization
3. **Sensor Flexibility**: Easy integration of additional sensors
4. **Proven Approach**: Successfully used in GLIM and other production systems

### System Architecture

```
┌─────────────────────────────────────────────┐
│         External Sensors (100Hz)             │
│    IMU (100Hz) + RTK GPS (10Hz)            │
└────────────────┬────────────────────────────┘
                 │ 
                 ↓
┌─────────────────────────────────────────────┐
│      robot_localization EKF (100Hz)         │
│   - Fuses IMU angular velocity/acceleration │
│   - Integrates GPS position/velocity        │
│   - Publishes odom→base_link transform      │
└────────────────┬────────────────────────────┘
                 │ Fused Odometry
                 ↓
┌─────────────────────────────────────────────┐
│      ConeSTELLATION SLAM (10-30Hz)         │
│   - Cone-based mapping                      │
│   - Global optimization                     │
│   - Publishes map→odom drift correction    │
└─────────────────────────────────────────────┘
```

## EKF Configuration

### State Vector

The EKF estimates a 15-dimensional state vector:

```
X = [x, y, z,                    # Position (m)
     roll, pitch, yaw,           # Orientation (rad)
     vx, vy, vz,                 # Linear velocity (m/s)
     vroll, vpitch, vyaw,        # Angular velocity (rad/s)
     ax, ay, az]                 # Linear acceleration (m/s²)
```

### Sensor Configuration

#### IMU Configuration (100Hz)
```yaml
imu0: /ouster/imu
imu0_config: [false, false, false,  # position (not used)
              true,  true,  true,   # orientation (roll, pitch, yaw)
              false, false, false,  # linear velocity (not directly measured)
              true,  true,  true,   # angular velocity
              true,  true,  true]   # linear acceleration
imu0_differential: false
imu0_relative: false
imu0_remove_gravitational_acceleration: true
```

#### GPS Position Configuration (10Hz)
```yaml
pose0: /gps/pose
pose0_config: [true,  true,  true,   # position (x, y, z from UTM)
               false, false, false,  # orientation (not from GPS)
               false, false, false,  # velocity (separate topic)
               false, false, false,  # angular velocity
               false, false, false]  # acceleration
pose0_differential: false
pose0_relative: false
```

#### GPS Velocity Configuration (10Hz)
```yaml
twist0: /ublox_gps_node/fix_velocity
twist0_config: [false, false, false,  # position
                false, false, false,  # orientation
                true,  true,  true,   # linear velocity
                false, false, false,  # angular velocity
                false, false, false]  # acceleration
```

### Process Noise Covariance

The process noise represents uncertainty in the motion model:

```yaml
# Diagonal values for each state component
process_noise_covariance:
  x, y:        0.05   # Position noise (m²)
  z:           0.06   # Vertical position noise (m²)
  roll, pitch: 0.03   # Attitude noise (rad²)
  yaw:         0.06   # Heading noise (rad²)
  vx, vy:      0.025  # Horizontal velocity noise (m²/s²)
  vz:          0.04   # Vertical velocity noise (m²/s²)
  vroll, vpitch: 0.01 # Angular velocity noise (rad²/s²)
  vyaw:        0.02   # Yaw rate noise (rad²/s²)
  ax, ay:      0.01   # Horizontal acceleration noise (m²/s⁴)
  az:          0.015  # Vertical acceleration noise (m²/s⁴)
```

## GPS to Local Frame Conversion

### UTM Projection

GPS lat/lon coordinates are converted to local Cartesian coordinates:

```python
class GPSToCartesianConverter:
    def __init__(self):
        self.origin_set = False
        self.utm_zone = None
        self.origin_easting = None
        self.origin_northing = None
        
    def gps_callback(self, msg: NavSatFix):
        # Convert to UTM
        easting, northing, zone_number, zone_letter = utm.from_latlon(
            msg.latitude, msg.longitude)
        
        # Set origin on first fix
        if not self.origin_set and msg.status.status >= 0:
            self.origin_easting = easting
            self.origin_northing = northing
            self.utm_zone = (zone_number, zone_letter)
            self.origin_set = True
            
        # Compute local coordinates
        if self.origin_set:
            x = easting - self.origin_easting
            y = northing - self.origin_northing
            z = msg.altitude - self.origin_altitude
            
            # Publish as PoseWithCovarianceStamped
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.pose.pose.position = Point(x=x, y=y, z=z)
            
            # Set covariance based on fix type
            cov = self.compute_covariance(msg.status, msg.position_covariance)
            pose_msg.pose.covariance = cov
```

### Covariance Computation

GPS covariance adapts based on RTK fix status:

```python
def compute_covariance(self, status, gps_covariance):
    # Base covariance from GPS
    if gps_covariance[0] > 0:  # Valid covariance from GPS
        cov = np.diag([gps_covariance[0], gps_covariance[4], 
                       gps_covariance[8]])
    else:
        # Default based on fix type
        if status.status == NavSatStatus.STATUS_FIX:
            if status.service & NavSatStatus.SERVICE_COMPASS:  # RTK Fix
                cov = np.diag([0.02**2, 0.02**2, 0.04**2])
            else:  # RTK Float
                cov = np.diag([0.3**2, 0.3**2, 0.5**2])
        else:  # Single or No Fix
            cov = np.diag([2.0**2, 2.0**2, 5.0**2])
    
    # Convert to 6x6 pose covariance (position only)
    pose_cov = np.zeros((6, 6))
    pose_cov[:3, :3] = cov
    
    return pose_cov.flatten().tolist()
```

## Frame Transformations

### Coordinate Frames

The system maintains several coordinate frames:

1. **map**: Global fixed frame (aligned with UTM grid)
2. **odom**: Odometry frame (drifts over time)
3. **base_link**: Vehicle body frame
4. **imu_link**: IMU sensor frame
5. **gps_link**: GPS antenna frame

### Transform Tree

```
map
 └── odom (published by SLAM drift correction)
      └── base_link (published by EKF)
           ├── imu_link (static transform)
           └── gps_link (static transform)
```

### Static Transforms

```xml
<!-- In URDF or static transform publisher -->
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="0.0 0.0 0.1 0 0 0 base_link imu_link"/>
      
<node pkg="tf2_ros" exec="static_transform_publisher"
      args="0.5 0.0 0.2 0 0 0 base_link gps_link"/>
```

## Integration with SLAM

### Odometry Usage in SLAM

```cpp
void ConeSLAMNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Convert odometry to pose
    Eigen::Isometry3d T_odom_base = ros_utils::to_eigen(msg->pose.pose);
    
    // Store for SLAM processing
    current_frame->T_odom_base = T_odom_base;
    current_frame->timestamp = msg->header.stamp;
    
    // Use velocity for prediction
    current_frame->linear_velocity = Eigen::Vector3d(
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
}
```

### Drift Correction

SLAM publishes map→odom transform to correct drift:

```cpp
void DriftCorrectionManager::updateDriftCorrection(
    const Eigen::Isometry3d& T_map_base_optimized,
    const Eigen::Isometry3d& T_odom_base_current,
    double timestamp) {
    
    // Compute drift correction transform
    Eigen::Isometry3d T_map_odom = T_map_base_optimized * T_odom_base_current.inverse();
    
    // Store with timestamp for interpolation
    transform_history_.push_back({timestamp, T_map_odom});
    
    // Publish to TF
    publishTransform(T_map_odom, timestamp);
}
```

## Launch Configuration

### Complete System Launch

```python
def generate_launch_description():
    return LaunchDescription([
        # GPS to Cartesian converter
        Node(
            package='cone_stellation',
            executable='gps_to_cartesian.py',
            name='gps_converter',
            parameters=[{
                'use_sim_time': True,
                'publish_rate': 10.0
            }]
        ),
        
        # Robot Localization EKF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(
                get_package_share_directory('cone_stellation'),
                'config', 'ekf_config.yaml'
            )],
            remappings=[
                ('odometry/filtered', '/ekf/odometry'),
            ]
        ),
        
        # SLAM node
        Node(
            package='cone_stellation',
            executable='cone_slam_node',
            name='cone_slam',
            parameters=[{
                'use_sim_time': True,
                'odometry_topic': '/ekf/odometry'
            }]
        )
    ])
```

## Performance Tuning

### EKF Optimization

1. **Prediction Rate**: Run at sensor rate (100Hz) for smooth output
2. **Queue Sizes**: Keep small (10) to avoid processing old data
3. **Timeout**: Set to 0.1s to detect sensor failures quickly

### Measurement Rejection

```yaml
# Mahalanobis distance threshold for outlier rejection
imu0_rejection_threshold: 3.0
pose0_rejection_threshold: 5.0
twist0_rejection_threshold: 3.0

# Only reject after N consecutive outliers
consecutive_outlier_threshold: 5
```

### Computational Considerations

- EKF update: ~0.5ms per iteration
- GPS conversion: ~0.1ms per message
- Transform publication: ~0.05ms
- Total CPU usage: <5% on modern processors

## Troubleshooting

### Common Issues

1. **GPS Origin Not Set**
   - Check GPS fix status
   - Verify at least one valid fix received
   - Check `/gps/pose` topic publishing

2. **IMU Orientation Incorrect**
   - Verify IMU mounting orientation
   - Check `imu_link` static transform
   - Enable `imu0_remove_gravitational_acceleration`

3. **Drift Not Corrected**
   - Verify SLAM is publishing map→odom
   - Check transform timestamps
   - Ensure time synchronization

### Debugging Tools

```bash
# Monitor EKF status
ros2 topic echo /diagnostics

# Check transform tree
ros2 run tf2_tools view_frames

# Visualize in RViz
ros2 launch cone_stellation imu_gps_ekf_launch.py rviz:=true

# Record for analysis
ros2 bag record /ekf/odometry /ouster/imu /gps/pose /tf
```

## Future Enhancements

### Planned Improvements

1. **Additional Sensors**
   - Wheel encoders for slip detection
   - Visual odometry integration
   - Dual GPS heading

2. **Advanced Filtering**
   - Adaptive noise models
   - Non-linear error state formulation
   - Multi-model filtering for fault tolerance

3. **GTSAM Integration**
   - IMU preintegration factors
   - GPS factors with RTK uncertainty
   - Tight coupling option for research

### Research Directions

1. **Learning-based Fusion**
   - Neural network for sensor weight adaptation
   - Terrain-aware noise models
   - Predictive sensor failure detection

2. **Distributed Fusion**
   - Multi-vehicle cooperative localization
   - Edge computing distribution
   - Resilient architecture