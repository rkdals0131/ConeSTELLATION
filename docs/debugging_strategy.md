# SLAM Crash Debugging Strategy

## Current Issues
1. **IndeterminantLinearSystemException** on landmarks (l0, l8)
2. **"Pose no longer in graph"** warnings during optimization
3. **Segmentation fault** (exit code -11) during ISAM2 update

## Root Cause Analysis

### Pattern Analysis
The crashes follow this sequence:
1. Initial frames process successfully
2. Landmarks are created and added to the graph
3. During optimization, poses are somehow removed from the graph
4. Subsequent observation factors fail because they reference missing poses
5. The system becomes underconstrained, leading to IndeterminantLinearSystemException
6. Eventually results in segmentation fault

### Likely Causes
1. **Marginalization Issue**: ISAM2 might be marginalizing out poses too aggressively
2. **Factor Graph Inconsistency**: Factors referencing variables that don't exist
3. **Initialization Order**: Variables must be added before factors that reference them
4. **Memory Management**: Shared pointers or references becoming invalid

## Debugging Steps

### Step 1: Add Comprehensive Logging
```cpp
// In cone_mapping.hpp, add debug logging:
RCLCPP_INFO(logger, "=== FACTOR GRAPH STATE ===");
RCLCPP_INFO(logger, "Total factors: %zu", isam2_->getFactorsUnsafe().size());
RCLCPP_INFO(logger, "New factors to add: %zu", new_factors_.size());
RCLCPP_INFO(logger, "New values to add: %zu", initial_values_.size());

// List all variables in the graph
auto current_estimate = isam2_->calculateEstimate();
for (const auto& key_value : current_estimate) {
    RCLCPP_INFO(logger, "Variable %s exists in estimate", 
                gtsam::DefaultKeyFormatter(key_value.key).c_str());
}
```

### Step 2: Verify Variable Existence Before Factor Creation
```cpp
// Before adding any factor, verify all referenced variables exist:
bool all_variables_exist = true;
for (const auto& factor : new_factors_) {
    for (const auto& key : factor->keys()) {
        if (!current_estimate.exists(key) && !initial_values_.exists(key)) {
            RCLCPP_ERROR(logger, "Factor references non-existent variable: %s",
                        gtsam::DefaultKeyFormatter(key).c_str());
            all_variables_exist = false;
        }
    }
}
```

### Step 3: Test Minimal Configuration
Start with the absolute minimum and gradually add complexity:

1. **Level 0**: Only odometry (no landmarks)
2. **Level 1**: Add one landmark with strong prior
3. **Level 2**: Add multiple landmarks without inter-landmark factors
4. **Level 3**: Add inter-landmark factors
5. **Level 4**: Enable all features

### Step 4: ISAM2 Parameter Investigation
```cpp
// Try different ISAM2 configurations:
gtsam::ISAM2Params params;

// Configuration A: Disable marginalization
params.enablePartialRelinearizationCheck = false;
params.cacheLinearizedFactors = true;
params.findUnusedFactorSlots = false;

// Configuration B: Conservative relinearization
params.relinearizeThreshold = 0.001;  // Very low threshold
params.relinearizeSkip = 1;  // Relinearize every time

// Configuration C: Different factorization
params.factorization = gtsam::ISAM2Params::QR;  // Instead of CHOLESKY
```

### Step 5: Memory Validation
```cpp
// Add memory validation checks:
class MemoryValidator {
    std::set<gtsam::Key> added_variables_;
    std::set<gtsam::Key> marginalized_variables_;
    
    void validate_factor(const gtsam::NonlinearFactor::shared_ptr& factor) {
        for (const auto& key : factor->keys()) {
            if (marginalized_variables_.count(key) > 0) {
                throw std::runtime_error("Factor references marginalized variable!");
            }
            if (added_variables_.count(key) == 0) {
                throw std::runtime_error("Factor references non-existent variable!");
            }
        }
    }
};
```

## Minimal Test Implementation

### TestLevel0: Odometry Only
```cpp
class OdometryOnlySLAM {
    void add_frame(const EstimationFrame::Ptr& frame) {
        gtsam::Symbol pose_key('x', pose_count_);
        
        if (pose_count_ == 0) {
            // Add prior
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(0.001, 0.001, 0.001));
            new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(
                pose_key, gtsam::Pose2(0, 0, 0), prior_noise));
        } else {
            // Add odometry
            gtsam::Symbol prev_key('x', pose_count_ - 1);
            auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3(0.1, 0.1, 0.05));
            new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(
                prev_key, pose_key, gtsam::Pose2(1, 0, 0), odom_noise));
        }
        
        initial_values_.insert(pose_key, gtsam::Pose2(pose_count_, 0, 0));
        
        // Update every frame for testing
        isam2_->update(new_factors_, initial_values_);
        new_factors_.resize(0);
        initial_values_.clear();
        
        pose_count_++;
    }
};
```

### TestLevel1: Single Landmark
```cpp
class SingleLandmarkSLAM : public OdometryOnlySLAM {
    void add_frame(const EstimationFrame::Ptr& frame) {
        // Add odometry as before
        OdometryOnlySLAM::add_frame(frame);
        
        // Add single landmark on first frame only
        if (pose_count_ == 1 && !landmark_added_) {
            gtsam::Symbol landmark_key('l', 0);
            gtsam::Symbol pose_key('x', 0);
            
            // Add landmark with strong prior
            auto landmark_prior = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector2(0.01, 0.01));
            new_factors_.add(gtsam::PriorFactor<gtsam::Point2>(
                landmark_key, gtsam::Point2(5, 0), landmark_prior));
            
            // Add observation
            auto obs_noise = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector2(0.1, 0.1));
            new_factors_.add(ConeObservationFactor(
                pose_key, landmark_key, gtsam::Point2(5, 0), obs_noise));
            
            initial_values_.insert(landmark_key, gtsam::Point2(5, 0));
            landmark_added_ = true;
            
            // Update
            isam2_->update(new_factors_, initial_values_);
            new_factors_.resize(0);
            initial_values_.clear();
        }
    }
    
    bool landmark_added_ = false;
};
```

## Error-Specific Solutions

### For IndeterminantLinearSystemException
1. Ensure every landmark has at least 2 observations OR a prior
2. Check rank of the system before optimization
3. Add regularization term if needed

### For "Pose no longer in graph"
1. Never reference poses older than the fixed-lag window
2. Store which poses have been marginalized
3. Skip observations from marginalized poses

### For Segmentation Fault
1. Check all shared_ptr validity before use
2. Ensure no dangling references to GTSAM variables
3. Validate memory access patterns

## Testing Protocol

1. **Unit Test Each Component**:
   - Test factor creation independently
   - Test ISAM2 update with known good data
   - Test each optimization trigger

2. **Integration Test with Synthetic Data**:
   - Create deterministic test sequence
   - Verify each step produces expected result
   - Check for memory leaks

3. **Stress Test**:
   - Run with 1000+ frames
   - Inject noise and outliers
   - Verify graceful degradation

## Monitoring Metrics

Track these during debugging:
1. Number of variables in graph over time
2. Number of factors in graph over time
3. Optimization time per iteration
4. Memory usage
5. Condition number of the system

## Next Steps

1. Implement TestLevel0 (odometry only)
2. Verify it runs for 1000+ frames without crash
3. Gradually add complexity
4. Identify exact point of failure
5. Apply targeted fix

This systematic approach should help identify the exact cause of the crashes.