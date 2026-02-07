# ConeSTELLATION System Review (Consolidated)

Created: 2025-12-21 01:29:26 KST
Last Updated: 2025-12-21 02:10:08 KST

## Scope
- This document supersedes `docs/cone_mapping_robustness_analysis.md` and the previous `docs/critical-issues.md`.
- Code reviewed: `src/cone_stellation/ros/cone_slam_node.cpp`,
  `include/cone_stellation/mapping/cone_mapping.hpp`,
  `src/loop_closure_detector.cpp`,
  `include/cone_stellation/mapping/loop_closure_detector.hpp`,
  `include/cone_stellation/util/drift_correction_manager.hpp`,
  `include/cone_stellation/preprocessing/cone_preprocessor.hpp`,
  `include/cone_stellation/mapping/data_association.hpp`.
- Runtime testing was not performed for this review.

## Working Protocol
- This file is a rolling exchange log. Questions will be batched in one place below.
- Please write answers directly in this file under "Open Questions (Batch N)".
- I will not ask ad-hoc questions in chat until a decision point is reached.

## Problem Definition and Current Reality
- Inputs: cone detections (with color + track ID) and EKF odometry.
- Graph state: Pose2 and Point2 with observation factors and inter-landmark distance factors.
- Output today: map and `map -> base_link_slam` TF; `map -> odom` drift correction is disabled in the node, so the system behaves like mapping-on-odom, not closed-loop SLAM.

## Findings (Ordered by Impact)

### 1) Loop Closure Pipeline Is Not Connected (Blocker)
- `LoopClosureDetector` exists but is never called in `ConeMapping` or the ROS node.
- No loop closure factors are added to the graph, so global consistency cannot be enforced.

### 2) Drift Correction Disabled (Blocker)
- `DriftCorrectionManager` is implemented and thread-safe, but `add_odometry_pose()` and `update_slam_pose()` calls are commented out.
- `map -> odom` is not published by this node, so SLAM corrections do not propagate to TF.

### 3) Inter-Landmark Distance Factors Are Self-Constraints (Blocker for Loop Closure)
- The measured distance is computed from the current landmark estimates, not from same-frame observations.
- This adds little new information and can resist map corrections when loop closure is introduced.

### 4) Backend Growth and Memory Management Missing (Major)
- Poses, landmarks, and factors grow without pruning or marginalization.
- Loop closure descriptors also grow without pruning if the detector is activated (`prune_old_descriptors()` is not called).

### 5) Data Association Is Under-Modeled (Major)
- `ConeMapping` uses nearest neighbor + color gate only; no covariance/Mahalanobis gating.
- Track IDs are not stabilized by the preprocessor; IDs are mostly incremental or fully upstream.
- Early direct landmark creation bypasses the tentative-landmark buffer and admits noisy points early.

### 6) Robustness to Outliers Is Weak (Major)
- Constant measurement noise; no robust kernels (Huber/Tukey).
- Observation noise is not scaled by range or detection confidence.

### 7) Concurrency Risks (Major)
- `MultiThreadedExecutor` runs callbacks and visualization concurrently without guarding shared state in `ConeMapping`.
- `ISAM2::calculateEstimate()` and graph updates are not protected from concurrent access.

### 8) Configuration Plumbing Gaps (Minor but Compounding)
- ISAM2 parameters exist in `ConeMapping::Config` but are not loaded from ROS params.
- Some YAML parameters are effectively ignored, which complicates tuning.

## Interpretation
- The approach is not fundamentally wrong. The system lacks the integration that turns the idea into SLAM: loop closure wiring, drift correction publishing, measurement-derived inter-landmark constraints, and bounded backend growth.

## Consequences If Unchanged
- Loop closure cannot succeed; even if enabled, it does not update the map.
- Long runs will grow memory and degrade performance.
- Map corrections will be resisted by self-constraints and lack of robust loss.

## User Input and Responses

### A) Goal and Scope
1) Target mode (choose one):
   - [ ] A. Mapping-only (map frame == odom; no loop closure)
   - [x] B. Full SLAM (map->odom drift correction + loop closure)

2) Expected run length:
   - [x] A. <10 min (Specifically, < 5 min as per user)
   - [ ] B. 10-60 min
   - [ ] C. >60 min

### B) Loop Closure Policy
3) Loop closure tolerance (choose one):
   - [ ] A. Aggressive (false positives acceptable)
   - [x] B. Balanced
   - [ ] C. Conservative (precision priority)

4) Acceptable loop-closure latency:
   - [ ] A. Immediate (<1 s)
   - [x] B. 1-5 s
   - [ ] C. 5-30 s

### C) Data Association Assumptions
5) Track ID reliability from frontend (choose one):
   - [x] A. High (can be trusted)
   - [ ] B. Medium (use as weak prior)
   - [ ] C. Low (ignore)

6) Cone color reliability:
   - [x] A. High
   - [ ] B. Medium
   - [ ] C. Low

### D) Backend Constraints
7) CPU budget for the SLAM node (free text):
   - Answer: No quantitative limit for now. Implement functionality first, then optimize.

8) Memory budget for the SLAM node (free text):
   - Answer: No quantitative limit for now. Implement functionality first, then optimize.

### E) Next Steps Preference (Order: 1 -> 2 -> 3)
9) Which to implement first:
   - [x] 1. Loop-closure pipeline wiring (detector -> validation -> graph factor -> map->odom)
   - [x] 2. Inter-landmark measurement redesign + robust loss + association gating
   - [x] 3. Backend memory/graph management (keyframe pruning/marginalization)

---

### F) Open Questions (Batch 1 - Pre-Decision)
(Write answers inline here. Do not reply in chat unless asked.)

1) TF ownership for `map -> odom` when SLAM is running:
   - [x] A. SLAM publishes `map -> odom` (disable EKF map->odom)
   - [ ] B. EKF publishes `map -> odom` (SLAM publishes only `map -> base_link_slam`)

2) Dummy publisher topic alignment for SLAM testing:
   - [x] A. Add remap in `launch/test_slam_launch.py` (`/cone/for_sim` -> `/cone/fused/ukf`)
   - [ ] B. Add input topic parameter in `cone_slam_node` (configurable source)

3) Loop closure insertion policy (default thresholds otherwise):
   - [x] A. Insert loop closure factors immediately on validation
   - [ ] B. Queue and insert only every N keyframes (to throttle)

## User Additional Unstructured Requirements

- **Dummy Publisher Check**: Re-verify the `dummy_publisher` structure before starting development.
- **Track ID Clarification**: The "Track ID" mentioned here is NOT the internal landmark ID assigned by SLAM. It refers to the track ID assigned by the perception frontend (sensor fusion and Kalman filter tracking stage) before it enters SLAM.
- **Tracking Behavior**: The Kalman filter tracks objects while they are in the field of view. If an object disappears and then reappears after a long period, it will be treated as a new object with a new Track ID. This behavior should be confirmed in the `dummy_publisher` structure.
- **Reference Package**: The `dummy_publisher` simulates the `calico` package (`~/ROS2_Workspace/ros2_ws/src/calico`). Please check the Kalman filtering track ID output in the `calico` package, as this is the direct input to the SLAM system.
- **Implementation Order**: Follow the priority sequence 1 -> 2 -> 3 strictly. Task 3 (Backend memory management) should be handled last.

## Exchange Log
- 2025-12-21 01:54:48 KST: Confirmed `dummy_publisher` track ID resets when cones leave FOV. Confirmed `calico` UKF path passes `track_id` from tracker to `/cone/fused/ukf`. Added Batch 1 questions to define TF ownership and dummy publisher alignment before loop-closure wiring.
- 2025-12-21 02:08:31 KST: Batch 1 answered. Proceeding with Step 1 implementation: loop-closure pipeline wiring + SLAM-owned `map -> odom` + dummy publisher remap in test launch.
- 2025-12-21 02:10:08 KST: Step 1 changes applied: loop-closure factors wired into `ConeMapping`, drift correction re-enabled with `map -> odom` publishing in `ConeSLAMNode`, and dummy publisher remap added in `launch/test_slam_launch.py`.


### todo
ros2 launch cone_stellation test 어쩌고 launch 해서 상태 봐보기. 