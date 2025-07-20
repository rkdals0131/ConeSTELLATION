# Fix 2: Add Track ID to Data Association

## Changes Made

### 1. Added Track ID to ConeLandmark (`common/cone.hpp`)

```cpp
// Added fields:
int primary_track_id_;          // Primary track ID from sensor
std::map<int, float> track_id_scores_; // Track ID observation history with scores

// Added methods:
int track_id() const { return primary_track_id_; }
void set_track_id(int id) { primary_track_id_ = id; }
void update_track_id(int id, float score = 1.0) {
  track_id_scores_[id] += score;
  // Update primary track ID if this one has higher score
  if (track_id_scores_[id] > track_id_scores_[primary_track_id_]) {
    primary_track_id_ = id;
  }
}
```

### 2. Updated Data Association (`mapping/data_association.hpp`)

```cpp
// Added to Config:
double track_id_weight;          // Weight for track ID matching (default: 5.0)
bool use_track_id;               // Enable track ID in association (default: true)

// Modified association logic:
// Now uses a scoring system instead of just distance
double score = distance;
// Track ID bonus (negative score for matching track IDs)
if (config_.use_track_id && obs.id >= 0 && landmark->track_id() >= 0) {
  if (obs.id == landmark->track_id()) {
    score -= config_.track_id_weight;  // Prioritize track ID matches
  }
}
```

### 3. Updated ConeMapping to Propagate Track IDs

- When creating new landmarks: Set initial track ID from observation
- When promoting tentative landmarks: Copy primary track ID
- When re-observing landmarks: Update track ID history with scoring

## How It Works

1. **Track ID Priority**: When an observation has the same track ID as a landmark, it gets a significant score bonus (default: -5.0), making it much more likely to be associated even if it's slightly farther away.

2. **Track ID History**: Each landmark maintains a history of all track IDs it has been observed with. The most frequent track ID becomes the primary track ID.

3. **Robustness**: The system still works if track IDs are missing (-1) or change due to occlusions. It just falls back to position-based matching.

## Expected Benefits

- **Reduced False Associations**: Track IDs help maintain correct associations even with noisy position measurements
- **Better Occlusion Handling**: When a cone is temporarily occluded and gets a new track ID, the position still allows correct association
- **Improved Consistency**: Landmarks maintain their identity better across frames

## Testing

To verify this is working:
1. Monitor the console for association messages showing track ID matches
2. Check that landmarks maintain consistent track IDs over time
3. Verify that false positive cones (with different track IDs) create separate landmarks
4. Test occlusion scenarios where track IDs might change

## Configuration

The track ID weight can be adjusted in the data association config:
- Higher weight (e.g., 10.0): Strong preference for track ID matches
- Lower weight (e.g., 2.0): Mild preference, position still dominates
- Set to 0.0 to disable track ID influence