# Fix 3: Inter-landmark Factor Crash Resolution

## Problem
When inter-landmark factors were enabled, the system crashed with a segmentation fault during optimization:
```
[INFO] Running optimization with 131 new factors and 16 new values
[ERROR] process has died [pid 257300, exit code -11]
```

## Root Cause Analysis
The crash likely occurred because:
1. Inter-landmark factors were being created between landmarks that weren't fully initialized in GTSAM
2. The observation-to-landmark mapping was being used incorrectly
3. Too many factors were being created at once (131 factors)

## Changes Made

### 1. Improved Safety Checks in `create_distance_factor()`
- Added try-catch block for exception handling
- Use landmark positions instead of observation positions for stability
- Added distance sanity checks (0.1m - max_landmark_distance)
- Better logging for debugging

### 2. Enhanced `should_create_inter_landmark_factor()`
- Added safety checks for landmark existence
- Use actual co-observation count instead of binary check
- Added minimum distance check (0.1m) to avoid degenerate cases

### 3. Updated `create_inter_landmark_factors()`
- Added safety check for minimum 2 landmarks
- Check landmark existence before accessing

### 4. Temporarily Disabled Inter-landmark Factors
```yaml
enable_inter_landmark_factors: false    # Temporarily disable to fix crash
use_simple_mapping: false               # Use full ConeMapping for track ID support
```

## Testing Strategy

1. **Test with inter-landmark factors disabled**:
   - Verify Track ID association works correctly
   - Ensure no crashes occur
   - Check that tentative landmark system works

2. **Gradually enable inter-landmark factors**:
   - Set `enable_inter_landmark_factors: true`
   - Increase `min_covisibility_count` to 3 or 4 (reduce factor creation)
   - Monitor console for inter-landmark factor creation messages

3. **Debug if crash recurs**:
   - Check which landmarks are involved
   - Verify they exist in GTSAM before factor creation
   - Consider adding factor creation limits per frame

## Next Steps
1. Build and test with inter-landmark factors disabled
2. Once stable, re-enable with higher co-visibility requirements
3. Add duplicate factor prevention mechanism
4. Consider batching factor creation across frames