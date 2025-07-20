# Fix 1: Enable Inter-Landmark Factors

## Changes Made

1. **Modified `config/slam_config.yaml`**:
   ```yaml
   # Changed from:
   enable_inter_landmark_factors: false    # TEMPORARILY DISABLED FOR DEBUGGING
   use_simple_mapping: true                # Use SimpleConeMapping for debugging
   
   # To:
   enable_inter_landmark_factors: true     # Enable inter-landmark factors
   use_simple_mapping: false               # Use full ConeMapping for inter-landmark support
   ```

## What This Enables

1. **Inter-Landmark Distance Factors**: When two landmarks are co-observed multiple times, the system will create distance constraints between them using `ConeDistanceFactor`.

2. **Full ConeMapping Class**: Switches from SimpleConeMapping to the full implementation that includes:
   - Tentative landmark system
   - Co-observation tracking
   - Inter-landmark factor creation
   - Pattern-based factor support (though patterns are currently disabled in code)

## Expected Behavior

When you run the system now:
- Co-observed landmarks will have their relative distances constrained
- This should improve map consistency, especially for sparse observations
- The factor graph visualization should show additional edges between landmarks (not just from poses to landmarks)

## Testing

To verify this is working:
1. Run the SLAM system with dummy publisher
2. Check RViz for inter-landmark edges (should appear as lines between landmarks)
3. Monitor console output for messages about inter-landmark factor creation
4. Check that the map maintains better geometric consistency

## Notes

- Pattern detection is implemented but pattern-based factors are temporarily disabled in code (line 448 in cone_mapping.hpp)
- The system requires at least 2 co-observations before creating inter-landmark factors (configurable via `min_covisibility_count`)
- Maximum distance for inter-landmark factors is 10.0 meters (configurable)