# ConeSTELLATION SLAM Architecture

## Overview

ConeSTELLATION is a cone-based Graph SLAM system that extends GLIM's architecture with novel inter-landmark factors. The key innovation is adding geometric constraints between cones observed from the same pose, extracting maximum information from sparse cone observations.

## Core Innovation: Inter-Landmark Factors

Traditional SLAM only creates factors between:
- Pose → Landmark (observation factors)
- Pose → Pose (odometry factors)

ConeSTELLATION adds:
- **Landmark → Landmark factors** for co-observed cones
- **Geometric pattern factors** (lines, curves, grids)
- **Relative distance/angle constraints** between cone pairs

### Why This Matters

In Formula Student:
- Only 2-10 cones visible per frame (very sparse)
- Cones form structured patterns (track boundaries)
- Relative positions are highly constrained
- Co-visibility implies geometric relationships

## System Architecture

### 1. Data Flow Pipeline

```
Cone Detections → Preprocessing → Odometry → Mapping → Optimization
     ↓                ↓              ↓          ↓           ↓
TrackedCones    AssociatedCones  LocalMap  GlobalMap  Optimized Map
```

### 2. Key Components

#### 2.1 Preprocessing Module
- Cone tracking and ID management
- Outlier rejection
- Uncertainty estimation
- Pattern detection (lines, curves)

#### 2.2 Odometry Module
- Fast local cone association
- Motion estimation from cone observations
- IMU integration (if available)
- Keyframe selection

#### 2.3 Mapping Module
- Global cone map management
- Inter-landmark factor creation
- Loop closure detection
- Map optimization triggers

#### 2.4 Factor Graph Structure

**Symbols:**
- `X(i)`: Vehicle pose at keyframe i
- `L(j)`: Cone landmark j position
- `V(i)`: Vehicle velocity (optional)
- `B(i)`: IMU bias (if IMU available)

**Factor Types:**

1. **Odometry Factors**
   - Between consecutive poses X(i) → X(i+1)
   - From wheel encoders or IMU

2. **Cone Observation Factors**
   - Pose to landmark X(i) → L(j)
   - Range-bearing or Cartesian

3. **Inter-Landmark Factors** (Novel)
   - Between co-observed cones L(j) → L(k)
   - Distance constraints
   - Angle constraints
   - Line/curve alignment

4. **Pattern Factors** (Novel)
   - Multiple cones forming geometric patterns
   - Straight line constraints
   - Circular arc constraints
   - Parallel line constraints

### 3. Inter-Landmark Factor Design

#### 3.1 Pairwise Distance Factor
For cones j,k observed from pose i:
```
residual = ||L(j) - L(k)|| - d_observed
```

#### 3.2 Relative Angle Factor
For cone triple (j,k,m):
```
residual = angle(L(j), L(k), L(m)) - θ_observed
```

#### 3.3 Line Alignment Factor
For cones on a detected line:
```
residual = distance_to_line(L(j), line_params)
```

#### 3.4 Track Boundary Factor
Enforces parallel track edges:
```
residual = parallelism(left_cones, right_cones)
```

## Implementation Plan

### Phase 1: Basic Framework
- [ ] Core data structures (Cone, ConeObservation)
- [ ] Basic factor graph with pose-landmark factors
- [ ] Simple data association
- [ ] Visualization infrastructure

### Phase 2: Inter-Landmark Factors
- [ ] Pairwise distance factors
- [ ] Co-visibility tracking
- [ ] Factor weight tuning
- [ ] Performance optimization

### Phase 3: Pattern Recognition
- [ ] Line detection from cone sets
- [ ] Curve fitting algorithms
- [ ] Pattern-based factors
- [ ] Track structure enforcement

### Phase 4: Advanced Features
- [ ] Multi-hypothesis data association
- [ ] Dynamic factor weighting
- [ ] Map management and cleanup
- [ ] Loop closure with inter-landmark validation

## Technical Details

### Coordinate Frames
- `map`: Global fixed frame
- `odom`: Drifting odometry frame
- `base_link`: Vehicle center
- `imu_link`: IMU sensor (optional)

### Cone Representation
```cpp
struct Cone {
    int id;                    // Unique landmark ID
    Eigen::Vector2d position;  // 2D position in map frame
    ConeColor color;          // YELLOW, BLUE, RED, UNKNOWN
    double confidence;        // Detection confidence
    std::set<int> co_observed; // IDs of cones seen together
};
```

### Factor Graph Update Strategy
1. Add new keyframe when:
   - Traveled > 1m or rotated > 10°
   - New cones detected
   - Loop closure candidate

2. Create inter-landmark factors when:
   - Multiple cones in single observation
   - Confidence above threshold
   - Consistent co-observation

3. Optimize when:
   - Every N keyframes (batch)
   - Loop closure detected
   - Large residuals detected

## Configuration Parameters

See `config/slam_config.yaml` for all parameters including:
- Factor weights
- Association thresholds  
- Pattern detection settings
- Optimization triggers

## Evaluation Metrics

- **Trajectory Error**: ATE, RPE
- **Map Consistency**: Cone position accuracy
- **Pattern Preservation**: Track structure quality
- **Computational Performance**: Real-time factor