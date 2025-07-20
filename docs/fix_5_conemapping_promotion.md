# Fix 5: ConeMapping Tentative Landmark Promotion Fix

## Problem
ConeMapping crashed during optimization while SimpleConeMapping worked fine. Analysis showed the crash occurred in `promote_tentative_landmarks()` function.

## Root Cause
The `promote_tentative_landmarks()` function was trying to:
1. Add observation factors from old frames that may have been marginalized from ISAM2
2. Access poses that no longer exist in the factor graph
3. Create an inconsistent state when no valid observations could be added

## Solution
Commented out the problematic section that adds old observation factors:

```cpp
// IMPORTANT: Skip adding old observation factors to avoid crashes
// Old frames may have been marginalized from ISAM2
// Let future observations create factors naturally

/* COMMENTED OUT - This causes crashes when referencing old frames
// Add observation factors from all frames that observed this tentative landmark
...
*/ // END OF COMMENTED OUT SECTION
```

Now the function:
1. Creates the landmark and adds it to GTSAM
2. Adds a prior if it's one of the first 3 landmarks
3. Does NOT add observation factors from past frames
4. Lets future observations naturally create factors

## Benefits
- Avoids accessing potentially deleted poses
- Simpler and more robust
- Landmarks still get properly constrained by future observations
- No risk of creating inconsistent factor graphs

## Testing
Re-enabled ConeMapping with:
```yaml
use_simple_mapping: false  # Use ConeMapping with fixes
```

The system should now work with:
- Tentative landmark system active
- Track ID support enabled
- No crashes during optimization