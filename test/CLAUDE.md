# test/ Directory

This directory contains unit tests for the cone_stellation SLAM system.

## Test Files

### test_inter_landmark_factors.cpp
Unit tests for custom GTSAM factors:
- ConeDistanceFactor: Tests pairwise distance constraints
- ConeLineFactor: Tests collinearity constraints
- ConeAngleFactor: Tests angle constraints between cone triples
- CompleteTrackScenario: Integration test with full track

## Test Results

✅ All factor tests passing
- Distance factor correctly maintains cone distances
- Line factor enforces collinearity
- Angle factor preserves geometric relationships
- Full track optimization converges

## Running Tests

```bash
# Build tests
colcon build --packages-select cone_stellation

# Run all tests
colcon test --packages-select cone_stellation

# See results
colcon test-result --verbose
```

## Future Tests Needed

- [ ] Cone preprocessing tests
- [ ] Data association tests
- [ ] Loop closure tests
- [ ] Performance benchmarks
- [ ] Multi-threaded safety tests
- [ ] ROS2 integration tests

## Current Status

- Basic factor tests implemented
- Need more comprehensive test coverage
- Missing performance benchmarks
- No integration tests yet