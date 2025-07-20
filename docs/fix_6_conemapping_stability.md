# Fix 6: ConeMapping Stability Improvements

## Problem
ConeMapping still crashed during optimization with 45 factors and 15 values.

## Solutions Applied

### 1. ISAM2 Parameters Aligned with SimpleConeMapping
```cpp
params.evaluateNonlinearError = false;  // Skip error evaluation for stability
params.factorization = gtsam::ISAM2Params::QR;  // More stable than CHOLESKY
```
- QR factorization is more numerically stable than Cholesky
- Skipping error evaluation reduces computation and potential numerical issues

### 2. Factor Validation Before Optimization
```cpp
// Validate factors before update
for (size_t i = 0; i < new_factors_.size(); ++i) {
  if (!new_factors_[i]) {
    RCLCPP_ERROR(rclcpp::get_logger("cone_mapping"), 
                "Null factor at index %zu", i);
    new_factors_.erase(new_factors_.begin() + i);
    --i;
  }
}
```

### 3. More Frequent Optimization
```yaml
optimize_every_n_frames: 1  # Optimize after every keyframe for stability
```
- Prevents accumulation of too many factors before optimization
- Smaller updates are more stable

### 4. Stricter Tentative Landmark Requirements
```yaml
min_observations: 5       # Increased from 3
min_time_span: 1.0       # Increased from 0.5
max_position_variance: 0.3  # Decreased from 0.5
min_color_confidence: 0.7   # Increased from 0.6
```
- Ensures only high-quality landmarks are promoted
- Reduces noise in the system

## Expected Benefits
- More stable optimization with smaller, frequent updates
- Better numerical stability with QR factorization
- Higher quality landmarks entering the system
- Null factor protection

## Testing
The system should now:
- Process keyframes one at a time
- Use more stable numerical methods
- Create fewer but higher quality landmarks