# Fix 4: Track ID Crash Resolution

## Problem
After adding track ID support to ConeLandmark, the system still crashes during optimization:
```
[INFO] Running optimization with 42 new factors and 15 new values
[ERROR] process has died [pid 258738, exit code -11]
```

## Root Cause Analysis
The crash was likely caused by:
1. Accessing uninitialized map entries in `update_track_id()` when `primary_track_id_` was -1
2. The tentative landmark system in ConeMapping adding complexity

## Changes Made

### 1. Fixed Track ID Update Logic
```cpp
void update_track_id(int id, float score = 1.0) {
  if (id < 0) return;  // Ignore invalid track IDs
  
  track_id_scores_[id] += score;
  
  // Update primary track ID if this one has higher score
  if (primary_track_id_ < 0 || 
      track_id_scores_[id] > track_id_scores_[primary_track_id_]) {
    primary_track_id_ = id;
  }
}
```

### 2. Fixed Track ID Setter
```cpp
void set_track_id(int id) { 
  if (id >= 0) {
    primary_track_id_ = id;
    track_id_scores_[id] = 1.0;  // Initialize score
  }
}
```

### 3. Switched to SimpleConeMapping
```yaml
use_simple_mapping: true  # Use SimpleConeMapping for stability
```

## Testing Strategy

1. **Build and test with SimpleConeMapping**:
   - This bypasses the tentative landmark system
   - Isolates whether the crash is from track ID or ConeMapping complexity

2. **If stable with SimpleConeMapping**:
   - The issue is likely in ConeMapping's tentative landmark system
   - Need to debug ConeMapping separately

3. **If still crashes**:
   - The issue is deeper than track ID
   - May need to revert all changes and debug incrementally

## Note
SimpleConeMapping doesn't use ConeLandmark, so track ID features won't be active. This is intentional to isolate the crash cause.