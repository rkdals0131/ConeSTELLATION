# ConeSTELLATION Development Plan

## 1. Introduction

ConeSTELLATION (Cone-based STructural ELement Layout for Autonomous NavigaTION) is a cone-based Graph SLAM system designed for Formula Student autonomous racing. It processes cone detection instances from LiDAR clustering with YOLO-based color classification, inspired by GLIM's modular architecture.

**Key Resources:**
- Sensor data formats: [input_topic_form.md](input_topic_form.md)
- ROS2 topic structure: [topic_structure.md](topic_structure.md)
- Debugging archive: [debug_log.md](debug_log.md)

## 2. Current Status (August 2025)

### ✅ Fully Operational Components
- **Core SLAM**: GTSAM-based factor graph with ISAM2 optimization
- **Visualization**: Comprehensive RViz display with performance optimization
- **Sensor Simulators**: Enhanced IMU/GPS with realistic noise models
- **IMU-GPS Integration**: Full EKF fusion
- **EKF Configuration**: External 50-100Hz odometry for vehicle control working perfectly
- **TF Tree Management**: All coordinate frame relationships resolved and broadcasting
- **Rosbag Compatibility**: Reliable operation with recorded data playback

### ⚠️ Working with Known Limitations
- **Stationary Yaw Drift**: System functional but experiences gradual orientation drift when stationary (no wheel encoders)
- **LiDAR Robustness**: Cone detection affected by vehicle vibrations in real-world conditions
- **Data Association**: Robust with color constraints and track ID support
- **Inter-landmark Factors**: Distance constraints between co-observed cones
- **Loop Closure**: Enhanced for sparse environments with constellation-based recognition
- **Drift Correction**: Dynamic map→odom transform calculation

### ❌ Not Yet Implemented
- GTSAM IMU preintegration factors (using robot_localization instead)
- RTK GPS position factors with adaptive weighting (using robot_localization)
- Multi-threaded architecture
- GLIM-inspired implicit loop closure (planned)
- Wheel encoder integration for improved stationary performance

## 3. System Architecture

### 3.1 Overall Design
```
┌─────────────────────────────────────────────┐
│         External Sensors (100Hz)             │
│    IMU + RTK GPS → robot_localization       │
└────────────────┬────────────────────────────┘
                 │ Fused Odometry (100Hz)
                 ↓
┌─────────────────────────────────────────────┐
│         ConeSTELLATION SLAM (10-30Hz)       │
│                                             │
│  Cone Detection → Data Association →        │
│  Factor Graph → Optimization → Map         │
│                                             │
│  Output: map→odom drift correction         │
└─────────────────────────────────────────────┘
```

### 3.2 Architectural Decisions

**Multi-rate Hybrid Architecture:**
- **Control Layer** (100Hz): IMU+GPS fusion via robot_localization for stable vehicle control
- **SLAM Layer** (10-30Hz): Mapping and drift correction
- **Rationale**: Control stability matters more than global accuracy during racing

**Why External Odometry (like GLIM)?**
- Fixed-lag smoother inappropriate for landmark SLAM (requires continuous features)
- External EKF handles high-rate sensor fusion efficiently
- SLAM focuses on global consistency and drift correction
- Proven approach in GLIM for similar reasons

### 3.3 Module Structure
```
cone_stellation/
├── include/cone_stellation/
│   ├── common/              # Core data structures
│   ├── preprocessing/       # Cone data preprocessing
│   ├── mapping/            # SLAM mapping with inter-landmark
│   ├── factors/            # Custom GTSAM factors
│   └── util/               # ROS2 utilities
├── src/                    # Implementation files
├── config/                 # YAML configuration
├── scripts/                # Python simulators
└── launch/                 # ROS2 launch files
```

## 4. Implementation Details

### 4.1 Inter-landmark Factors

**Innovation**: Pairwise distance constraints between co-observed cones to handle sparse observations (2-10 cones/frame).

**Key Features:**
- Co-visibility tracking with configurable thresholds
- Clustering algorithm to avoid over-constraining
- Adaptive noise model based on distance
- Visualization as red lines in RViz

**Parameters:**
```yaml
inter_landmark:
  enabled: true
  min_covisibility_count: 3
  min_distance: 1.5
  max_distance: 15.0
  noise_model:
    base_stddev: 0.05
    distance_factor: 0.01
```

### 4.2 Loop Closure Implementation

**Current Status** (2025-07-28):
- **Previous Approach Failed**: Complex descriptor-based system proved unstable
- **Issues**: Memory leaks, race conditions, poor performance in sparse environments
- **Decision**: Replace with GLIM-inspired implicit loop closure

**New Approach - GLIM-Inspired Implicit Loop Closure**:
- **No Descriptors**: Spatial proximity and overlap-based detection
- **Submap Concept**: Group keyframes into local submaps
- **Simple Matching**: Distance threshold + landmark overlap
- **Geometric Validation**: Ensure consistent relative geometry
- **Robust Integration**: Huber kernel for outlier rejection

**Implementation Plan**:
- Phase 1: Basic submap generation and overlap detection (2 weeks)
- Phase 2: Geometric consistency and robustness (2 weeks)
- Phase 3: Optimization and memory management (1 week)

**Details**: See [glim_inspired_loop_closure.md](glim_inspired_loop_closure.md)

### 4.3 IMU-GPS Integration (Current Focus)

**Enhanced Simulators**:
- **IMU**: Allan variance noise, temperature drift, g-sensitivity
- **GPS**: RTK Fix/Float/Single modes, multipath effects

**Integration Plan**:
1. GTSAM IMU preintegration between keyframes
2. GPS position factors with RTK-aware covariance
3. Coordinate frame setup (map → odom → base_link → imu_link)
4. Robot_localization EKF configuration

## 5. Feature Integration Roadmap

### 5.1 GLIM Features to Integrate

**High Priority:**
- ✅ ISAM2 incremental optimization
- ✅ ROS2 parameter system
- ✅ Drift correction (map→odom)
- 🚧 Multi-threading architecture
- ⏳ Memory management
- ⏳ Serialization/recovery

**Medium Priority:**
- ⏳ Robust kernels (Huber, Cauchy)
- ⏳ Interactive viewer
- ⏳ Global registration
- ⏳ GPU acceleration

### 5.2 ConeSTELLATION-Specific Features

**Completed:**
- ✅ Inter-landmark factors
- ✅ Tentative landmark system
- ✅ Color-based data association
- ✅ Track ID utilization

**Planned:**
- ⏳ GLIM-inspired implicit loop closure
- ⏳ Submap-based mapping architecture
- ⏳ Spatial indexing with KD-trees
- ⏳ Geometric consistency validation

## 6. Development Phases

### Phase 1: Core Infrastructure ✅
- Basic data structures
- GTSAM integration
- ROS2 node setup
- Visualization

### Phase 2: Basic SLAM ✅
- Data association
- Factor graph construction
- ISAM2 optimization
- Drift correction

### Phase 3: Advanced Features ✅ (Completed)
- ✅ Inter-landmark factors
- ✅ IMU-GPS integration (via robot_localization EKF)
- ✅ TF tree management and broadcasting
- ✅ Rosbag compatibility and testing
- ❌ Enhanced loop closure (deprecated - replaced with simpler approach)
- ⏳ GLIM-inspired implicit loop closure (future work)
- ⏳ Multi-threading optimization

### Phase 4: Production Ready
- ⏳ Robust optimization
- ⏳ Error recovery
- ⏳ Performance optimization
- ⏳ Comprehensive testing

## 7. Testing Strategy

### Simulation Testing
- Enhanced sensor simulators with realistic noise
- Multiple motion profiles (straight, circular, figure-8)
- Ground truth comparison

### Real Data Testing
- Rosbag playback from actual races
- Performance metrics (accuracy, timing)
- Failure mode analysis

### Integration Testing
- End-to-end system validation
- Multi-sensor synchronization
- Real-time performance

## 8. Performance Targets

- **Odometry Rate**: 100Hz (external EKF)
- **SLAM Rate**: 10-30Hz
- **Accuracy**: < 0.5m drift over 1km track
- **Robustness**: Handle 50% cone occlusions
- **Latency**: < 50ms for drift correction

## 9. Dependencies

**Required:**
- Eigen3
- GTSAM 4.0+
- gtsam_points
- spdlog
- Boost
- robot_localization

**Optional:**
- OpenCV (visualization)
- CUDA (future GPU acceleration)

## 10. References

- GLIM architecture: `/home/user1/ROS2_Workspace/GLIM_ws/src/glim/`
- GLIM paper: "GLIM: 3D Range-Inertial Localization and Mapping with GPU-Accelerated Scan Matching Factors"
- Implicit Loop Closure Design: [glim_inspired_loop_closure.md](glim_inspired_loop_closure.md)
- Critical Issues Analysis: [critical-issues.md](critical-issues.md)
- Formula Student rules and track specifications