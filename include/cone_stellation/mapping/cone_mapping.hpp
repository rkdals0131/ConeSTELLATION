#pragma once

#include <memory>
#include <unordered_map>
#include <set>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
#include <rclcpp/rclcpp.hpp>

#include "cone_stellation/common/cone.hpp"
#include "cone_stellation/common/estimation_frame.hpp"
#include "cone_stellation/common/tentative_landmark.hpp"
#include "cone_stellation/factors/inter_landmark_factors.hpp"
#include "cone_stellation/factors/cone_observation_factor.hpp"

namespace cone_stellation {

/**
 * @brief Main mapping module handling factor graph construction and optimization
 * 
 * Key innovation: Adds inter-landmark factors between co-observed cones
 */
class ConeMapping {
public:
  using Ptr = std::shared_ptr<ConeMapping>;
  
  struct Config {
    // ISAM2 parameters
    double isam2_relinearize_threshold;
    int isam2_relinearize_skip;
    
    // Factor weights
    double odometry_noise;
    double cone_observation_noise;
    double inter_landmark_distance_noise;  // Novel factor
    double pattern_factor_noise;          // Novel factor
    
    // Inter-landmark factor creation
    bool enable_inter_landmark_factors;
    double min_covisibility_count;  // Min times seen together
    double max_landmark_distance; // Max distance for factor
    
    // Data association
    double max_association_distance; // Max distance for associating observations
    
    // Optimization triggers
    int optimize_every_n_frames;
    bool optimize_on_loop_closure;
    
    Config() : isam2_relinearize_threshold(0.1), isam2_relinearize_skip(10),
               odometry_noise(0.1), cone_observation_noise(0.5),
               inter_landmark_distance_noise(0.1), pattern_factor_noise(0.05),
               enable_inter_landmark_factors(true), min_covisibility_count(2),
               max_landmark_distance(10.0), max_association_distance(2.0),
               optimize_every_n_frames(10), optimize_on_loop_closure(true) {}
  };
  
  ConeMapping(const Config& config = Config()) 
    : config_(config), 
      next_pose_id_(0), 
      next_landmark_id_(0),
      next_tentative_id_(0),
      frames_since_optimization_(0) {
    
    // Initialize ISAM2
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = config_.isam2_relinearize_threshold;
    params.relinearizeSkip = config_.isam2_relinearize_skip;
    params.enableRelinearization = true;
    params.evaluateNonlinearError = false;  // Skip error evaluation for stability
    params.cacheLinearizedFactors = false;  // Don't cache to save memory
    params.factorization = gtsam::ISAM2Params::QR;  // More stable than CHOLESKY
    params.findUnusedFactorSlots = true;   // Clean up unused factor slots
    isam2_ = std::make_shared<gtsam::ISAM2>(params);
  }
  
  /**
   * @brief Add new keyframe to the map
   */
  void add_keyframe(const EstimationFrame::Ptr& frame) {
    // Create pose variable
    gtsam::Symbol pose_key('x', next_pose_id_);
    
    // Add odometry factor from previous pose
    if (next_pose_id_ > 0) {
      add_odometry_factor(pose_key, frame);
    } else {
      // Add prior for first pose
      add_prior_factor(pose_key, frame);
    }
    
    // Process cone observations
    if (frame->cone_observations) {
      process_cone_observations(frame, pose_key);
    }
    
    // Store frame
    frames_[next_pose_id_] = frame;
    next_pose_id_++;
    
    // Optimize if needed
    frames_since_optimization_++;
    if (frames_since_optimization_ >= config_.optimize_every_n_frames) {
      optimize();
    }
    
    RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                "Added keyframe %d with %zu observations, total factors: %zu, landmarks: %zu", 
                next_pose_id_ - 1,
                frame->cone_observations ? frame->cone_observations->cones.size() : 0,
                new_factors_.size(),
                landmarks_.size());
  }
  
  /**
   * @brief Get current estimate of all poses and landmarks
   */
  gtsam::Values get_current_estimate() const {
    return isam2_->calculateEstimate();
  }
  
  /**
   * @brief Get all tracked landmarks
   */
  std::unordered_map<int, ConeLandmark::Ptr> get_landmarks() const {
    return landmarks_;
  }
  
  /**
   * @brief Get the current factor graph for visualization
   */
  gtsam::NonlinearFactorGraph get_factor_graph() const {
    // Return the complete factor graph from ISAM2
    return isam2_->getFactorsUnsafe();
  }
  
  /**
   * @brief Get all keyframe poses for visualization
   */
  std::unordered_map<int, gtsam::Pose2> get_poses() const {
    std::unordered_map<int, gtsam::Pose2> poses;
    auto values = isam2_->calculateEstimate();
    
    for (const auto& [id, frame] : frames_) {
      gtsam::Symbol pose_key('x', id);
      if (values.exists(pose_key)) {
        poses[id] = values.at<gtsam::Pose2>(pose_key);
      }
    }
    return poses;
  }
  
  /**
   * @brief Get the next pose ID (for frame->id assignment)
   */
  int get_next_pose_id() const {
    return next_pose_id_;
  }

private:
  /**
   * @brief Add prior factor for first pose
   */
  void add_prior_factor(const gtsam::Symbol& pose_key, const EstimationFrame::Ptr& frame) {
    // Convert 3D pose to 2D for factor graph
    const auto& T = frame->T_world_sensor;
    double yaw = std::atan2(T.rotation()(1,0), T.rotation()(0,0));
    gtsam::Pose2 pose2d(T.translation().x(), T.translation().y(), yaw);
    
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
    new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(pose_key, pose2d, prior_noise));
    
    initial_values_.insert(pose_key, pose2d);
  }
  
  /**
   * @brief Add odometry factor between consecutive poses
   */
  void add_odometry_factor(const gtsam::Symbol& current_pose_key, 
                          const EstimationFrame::Ptr& current_frame) {
    gtsam::Symbol prev_pose_key('x', next_pose_id_ - 1);
    
    // Get previous frame
    const auto& prev_frame = frames_[next_pose_id_ - 1];
    
    // Calculate relative transformation
    Eigen::Isometry3d T_prev_current = prev_frame->T_world_sensor.inverse() * 
                                       current_frame->T_world_sensor;
    
    // Convert to 2D
    double dx = T_prev_current.translation().x();
    double dy = T_prev_current.translation().y();
    double dtheta = std::atan2(T_prev_current.rotation()(1,0), 
                               T_prev_current.rotation()(0,0));
    
    gtsam::Pose2 odom_delta(dx, dy, dtheta);
    
    // Noise model (could be made adaptive based on motion)
    auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(config_.odometry_noise, config_.odometry_noise, 
                      config_.odometry_noise * 0.5)); // Less noise on rotation
    
    new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
        prev_pose_key, current_pose_key, odom_delta, odom_noise);
    
    // Initial guess based on previous pose
    auto prev_values = isam2_->calculateEstimate();
    if (prev_values.exists(prev_pose_key)) {
      gtsam::Pose2 prev_pose = prev_values.at<gtsam::Pose2>(prev_pose_key);
      initial_values_.insert(current_pose_key, prev_pose * odom_delta);
    } else {
      // Fallback (should not happen)
      const auto& T = current_frame->T_world_sensor;
      double yaw = std::atan2(T.rotation()(1,0), T.rotation()(0,0));
      initial_values_.insert(current_pose_key, 
                           gtsam::Pose2(T.translation().x(), T.translation().y(), yaw));
    }
    
    RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                "Added odometry factor: %s -> %s, delta: (%.2f, %.2f, %.2f)", 
                gtsam::DefaultKeyFormatter(prev_pose_key).c_str(),
                gtsam::DefaultKeyFormatter(current_pose_key).c_str(),
                dx, dy, dtheta);
  }
  
  /**
   * @brief Process cone observations and create factors
   */
  void process_cone_observations(const EstimationFrame::Ptr& frame, 
                                const gtsam::Symbol& pose_key) {
    const auto& obs_set = *frame->cone_observations;
    std::vector<int> observed_landmark_ids;
    
    // Process each observation
    for (size_t i = 0; i < obs_set.cones.size(); i++) {
      const auto& obs = obs_set.cones[i];
      
      // Try to associate with confirmed landmarks first
      int landmark_id = associate_with_confirmed_landmark(obs, frame);
      
      if (landmark_id >= 0) {
        // Associated with confirmed landmark
        frame->observation_to_landmark[i] = landmark_id;
        add_observation_factor(pose_key, landmark_id, obs);
        observed_landmark_ids.push_back(landmark_id);
        landmarks_[landmark_id]->increment_observations();
        landmarks_[landmark_id]->update_track_id(obs.id);
        RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                    "Associated observation %zu with landmark %d", i, landmark_id);
      } else {
        // For now, create landmarks immediately if we have few landmarks
        // This ensures the graph is properly constrained from the start
        if (landmarks_.size() < 10) {
          // Create new landmark immediately
          Eigen::Vector2d world_pos = frame->transform_to_world(obs);
          int new_landmark_id = next_landmark_id_++;
          landmarks_[new_landmark_id] = std::make_shared<ConeLandmark>(new_landmark_id, world_pos, obs.color);
          landmarks_[new_landmark_id]->set_track_id(obs.id);
          
          // Add to GTSAM
          gtsam::Symbol landmark_key('l', new_landmark_id);
          initial_values_.insert(landmark_key, gtsam::Point2(world_pos));
          
          // Add prior for first few landmarks
          if (new_landmark_id < 3) {
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1));
            new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
                landmark_key, gtsam::Point2(world_pos), prior_noise);
            RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                        "Created landmark %d with prior at (%.2f, %.2f)", 
                        new_landmark_id, world_pos.x(), world_pos.y());
          }
          
          // Add observation factor
          frame->observation_to_landmark[i] = new_landmark_id;
          add_observation_factor(pose_key, new_landmark_id, obs);
          observed_landmark_ids.push_back(new_landmark_id);
        } else {
          // Use tentative landmarks for later landmarks
          associate_with_tentative_landmark(obs, frame);
        }
      }
    }
    
    // Only promote tentative landmarks after we have enough confirmed ones
    if (landmarks_.size() >= 10) {
      promote_tentative_landmarks();
    }
    
    // Create inter-landmark factors for co-observed confirmed landmarks
    if (config_.enable_inter_landmark_factors && observed_landmark_ids.size() >= 2) {
      create_inter_landmark_factors(observed_landmark_ids, frame);
    }
    
    // Create pattern-based factors
    for (const auto& pattern : obs_set.detected_patterns) {
      create_pattern_factors(pattern, frame);
    }
  }
  
  /**
   * @brief Associate observation with confirmed landmarks
   */
  int associate_with_confirmed_landmark(const ConeObservation& obs, const EstimationFrame::Ptr& frame) {
    // Transform observation to world frame
    Eigen::Vector2d world_pos = frame->transform_to_world(obs);
    
    // Find nearest landmark of same color
    int best_id = -1;
    double best_distance = 2.0; // Association threshold
    
    for (const auto& [id, landmark] : landmarks_) {
      if (landmark->color() != obs.color && obs.color != ConeColor::UNKNOWN) {
        continue;
      }
      
      double distance = (landmark->position() - world_pos).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_id = id;
      }
    }
    
    return best_id;  // Returns -1 if no association
  }
  
  /**
   * @brief Add cone observation factor
   */
  void add_observation_factor(const gtsam::Symbol& pose_key, int landmark_id,
                             const ConeObservation& obs) {
    gtsam::Symbol landmark_key('l', landmark_id);
    
    // Create noise model based on observation uncertainty
    auto obs_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector2(config_.cone_observation_noise, config_.cone_observation_noise));
    
    // Use our custom cone observation factor
    // The observation is already in vehicle frame
    new_factors_.emplace_shared<ConeObservationFactor>(
        pose_key, landmark_key, gtsam::Point2(obs.position.x(), obs.position.y()), obs_noise);
    
    RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                "Added observation factor: pose %s -> landmark %s", 
                gtsam::DefaultKeyFormatter(pose_key).c_str(),
                gtsam::DefaultKeyFormatter(landmark_key).c_str());
  }
  
  /**
   * @brief Create inter-landmark factors between co-observed cones
   * This is the key innovation!
   */
  void create_inter_landmark_factors(const std::vector<int>& landmark_ids,
                                    const EstimationFrame::Ptr& frame) {
    // Skip if too few landmarks
    if (landmark_ids.size() < 2) return;
    
    // Update co-observation tracking
    for (size_t i = 0; i < landmark_ids.size(); ++i) {
      for (size_t j = i + 1; j < landmark_ids.size(); ++j) {
        int id1 = landmark_ids[i];
        int id2 = landmark_ids[j];
        
        // Safety check
        if (landmarks_.find(id1) == landmarks_.end() || 
            landmarks_.find(id2) == landmarks_.end()) {
          continue;
        }
        
        landmarks_[id1]->add_co_observed(id2);
        landmarks_[id2]->add_co_observed(id1);
        
        // Create distance factor if cones are frequently co-observed
        if (should_create_inter_landmark_factor(id1, id2)) {
          create_distance_factor(id1, id2, frame);
        }
      }
    }
  }
  
  /**
   * @brief Check if we should create factor between two landmarks
   */
  bool should_create_inter_landmark_factor(int id1, int id2) const {
    // Safety check
    if (landmarks_.find(id1) == landmarks_.end() || landmarks_.find(id2) == landmarks_.end()) {
      return false;
    }
    
    const auto& lm1 = landmarks_.at(id1);
    const auto& lm2 = landmarks_.at(id2);
    
    // Check if they've been co-observed enough times
    int co_obs_count = lm1->co_observation_count(id2);
    if (co_obs_count < config_.min_covisibility_count) {
      return false;
    }
    
    // Check distance
    double distance = (lm1->position() - lm2->position()).norm();
    if (distance < 0.1 || distance > config_.max_landmark_distance) {
      return false;
    }
    
    // Avoid creating duplicate factors
    // (In a real implementation, we'd track which factors have been created)
    
    return true;
  }
  
  /**
   * @brief Create distance factor between two landmarks
   */
  void create_distance_factor(int id1, int id2, const EstimationFrame::Ptr& frame) {
    // Safety check: ensure landmarks exist
    if (landmarks_.find(id1) == landmarks_.end() || landmarks_.find(id2) == landmarks_.end()) {
      RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                  "Skipping inter-landmark factor - landmarks not in map");
      return;
    }
    
    gtsam::Symbol landmark1_key('l', id1);
    gtsam::Symbol landmark2_key('l', id2);
    
    // Check if both landmarks exist in GTSAM values
    try {
      auto current_estimate = isam2_->calculateEstimate();
      bool landmark1_exists = current_estimate.exists(landmark1_key) || initial_values_.exists(landmark1_key);
      bool landmark2_exists = current_estimate.exists(landmark2_key) || initial_values_.exists(landmark2_key);
      
      if (!landmark1_exists || !landmark2_exists) {
        RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                    "Skipping inter-landmark factor - landmarks not yet in graph (L%d: %s, L%d: %s)",
                    id1, landmark1_exists ? "yes" : "no", id2, landmark2_exists ? "yes" : "no");
        return;
      }
      
      // Use landmark positions instead of observations for more stable distance
      const auto& lm1 = landmarks_.at(id1);
      const auto& lm2 = landmarks_.at(id2);
      double measured_distance = (lm1->position() - lm2->position()).norm();
      
      // Sanity check on distance
      if (measured_distance < 0.1 || measured_distance > config_.max_landmark_distance) {
        RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                    "Skipping inter-landmark factor - invalid distance %.2f", measured_distance);
        return;
      }
      
      auto distance_noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector1(config_.inter_landmark_distance_noise));
      
      new_factors_.emplace_shared<ConeDistanceFactor>(
          landmark1_key, landmark2_key, measured_distance, distance_noise);
      
      RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                  "Created inter-landmark factor between L%d and L%d (dist: %.2f)", 
                  id1, id2, measured_distance);
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("cone_mapping"), 
                  "Exception creating inter-landmark factor: %s", e.what());
    }
  }
  
  /**
   * @brief Create factors from detected patterns
   */
  void create_pattern_factors(const ConePattern& pattern, const EstimationFrame::Ptr& frame) {
    // TEMPORARILY DISABLED for debugging
    return;
    
    if (pattern.type == ConePattern::LINE && pattern.cone_ids.size() >= 3) {
      // Create line factors for all triples
      for (size_t i = 0; i < pattern.cone_ids.size() - 2; ++i) {
        for (size_t j = i + 1; j < pattern.cone_ids.size() - 1; ++j) {
          for (size_t k = j + 1; k < pattern.cone_ids.size(); ++k) {
            // Map local IDs to global landmark IDs
            int lid1 = -1, lid2 = -1, lid3 = -1;
            if (frame->observation_to_landmark.count(pattern.cone_ids[i]))
              lid1 = frame->observation_to_landmark[pattern.cone_ids[i]];
            if (frame->observation_to_landmark.count(pattern.cone_ids[j]))
              lid2 = frame->observation_to_landmark[pattern.cone_ids[j]];
            if (frame->observation_to_landmark.count(pattern.cone_ids[k]))
              lid3 = frame->observation_to_landmark[pattern.cone_ids[k]];
            
            if (lid1 >= 0 && lid2 >= 0 && lid3 >= 0) {
              gtsam::Symbol l1('l', lid1);
              gtsam::Symbol l2('l', lid2);
              gtsam::Symbol l3('l', lid3);
              
              auto line_noise = gtsam::noiseModel::Diagonal::Sigmas(
                  gtsam::Vector1(config_.pattern_factor_noise));
              
              new_factors_.emplace_shared<ConeLineFactor>(l1, l2, l3, line_noise);
            }
          }
        }
      }
    }
  }
  
  /**
   * @brief Run optimization
   */
  void optimize() {
    if (new_factors_.empty() && initial_values_.empty()) {
      RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                  "Skipping optimization - no new factors or values");
      return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                "Running optimization with %zu new factors and %zu new values", 
                new_factors_.size(), initial_values_.size());
    
    try {
      // Log factor details before optimization
      RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                  "Factor graph details: %zu total factors in graph", 
                  isam2_->getFactorsUnsafe().size());
      
      // Validate factors before update
      for (size_t i = 0; i < new_factors_.size(); ++i) {
        if (!new_factors_[i]) {
          RCLCPP_ERROR(rclcpp::get_logger("cone_mapping"), 
                      "Null factor at index %zu", i);
          new_factors_.erase(new_factors_.begin() + i);
          --i;
        }
      }
      
      // Check for empty update
      if (new_factors_.empty() && initial_values_.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("cone_mapping"), 
                    "Empty update after validation");
        return;
      }
      
      // Perform ISAM2 update
      gtsam::ISAM2Result result = isam2_->update(new_factors_, initial_values_);
      
      RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                  "ISAM2 update result: %zu cliques updated, %zu variables re-eliminated", 
                  result.cliques, result.variablesReeliminated);
      
      // Skip additional optimization for now to reduce complexity
      // isam2_->update();
      
      // Clear for next iteration
      new_factors_.resize(0);
      initial_values_.clear();
      frames_since_optimization_ = 0;
      
      // Update landmark positions from optimized values
      auto current_estimate = isam2_->calculateEstimate();
      for (auto& [id, landmark] : landmarks_) {
        gtsam::Symbol landmark_key('l', id);
        if (current_estimate.exists(landmark_key)) {
          gtsam::Point2 optimized_pos = current_estimate.at<gtsam::Point2>(landmark_key);
          landmark->update_position(Eigen::Vector2d(optimized_pos.x(), optimized_pos.y()));
        }
      }
      
      RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                  "Optimization complete. Total poses: %d, Total landmarks: %zu", 
                  next_pose_id_, landmarks_.size());
                  
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("cone_mapping"), 
                  "Optimization failed: %s", e.what());
      // Don't clear factors/values so we can try again next time
    }
  }
  
  Config config_;
  
  // GTSAM components
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values initial_values_;
  
  // Map data
  std::unordered_map<int, EstimationFrame::Ptr> frames_;
  std::unordered_map<int, ConeLandmark::Ptr> landmarks_;
  std::unordered_map<int, TentativeLandmark::Ptr> tentative_landmarks_;
  
  // ID counters
  int next_pose_id_;
  int next_landmark_id_;
  int next_tentative_id_;
  int frames_since_optimization_;
  
  /**
   * @brief Associate observation with tentative landmarks
   */
  void associate_with_tentative_landmark(const ConeObservation& obs, const EstimationFrame::Ptr& frame) {
    // Transform observation to world frame
    Eigen::Vector2d world_pos = frame->transform_to_world(obs);
    
    // Create observation record
    LandmarkObservation landmark_obs;
    landmark_obs.world_position = world_pos;
    landmark_obs.sensor_position = obs.position;
    landmark_obs.color = obs.color;
    landmark_obs.track_id = obs.id;
    landmark_obs.timestamp = frame->timestamp;
    landmark_obs.confidence = obs.confidence;
    landmark_obs.frame_id = frame->id;
    
    // Find nearest tentative landmark
    int best_id = -1;
    double best_distance = config_.max_association_distance; // Use same threshold
    
    for (const auto& [id, tentative] : tentative_landmarks_) {
      // Check color compatibility
      if (tentative->get_primary_color() != ConeColor::UNKNOWN &&
          obs.color != ConeColor::UNKNOWN &&
          tentative->get_primary_color() != obs.color) {
        continue;
      }
      
      double distance = (tentative->get_mean_position() - world_pos).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_id = id;
      }
    }
    
    if (best_id >= 0) {
      // Add to existing tentative landmark
      tentative_landmarks_[best_id]->add_observation(landmark_obs);
    } else {
      // Create new tentative landmark
      auto tentative = std::make_shared<TentativeLandmark>(next_tentative_id_++);
      tentative->add_observation(landmark_obs);
      tentative_landmarks_[tentative->get_id()] = tentative;
    }
  }
  
  /**
   * @brief Promote ready tentative landmarks to confirmed landmarks
   */
  void promote_tentative_landmarks() {
    std::vector<int> promoted_ids;
    
    for (const auto& [id, tentative] : tentative_landmarks_) {
      if (tentative->is_ready_for_promotion()) {
        // Create confirmed landmark
        Eigen::Vector2d position = tentative->get_mean_position();
        ConeColor color = tentative->get_primary_color();
        
        int landmark_id = next_landmark_id_++;
        landmarks_[landmark_id] = std::make_shared<ConeLandmark>(landmark_id, position, color);
        landmarks_[landmark_id]->set_track_id(tentative->get_primary_track_id());
        
        // Add to GTSAM
        gtsam::Symbol landmark_key('l', landmark_id);
        initial_values_.insert(landmark_key, gtsam::Point2(position.x(), position.y()));
        
        // Add prior only for first few landmarks to prevent underconstrained system
        if (landmark_id < 3) {  // Only add strong priors to first 3 landmarks
          auto landmark_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
              gtsam::Vector2(0.1, 0.1)); // Strong prior for anchor landmarks
          new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Point2>>(
              landmark_key, gtsam::Point2(position.x(), position.y()), landmark_prior_noise);
          RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                      "Added prior to landmark %d as anchor", landmark_id);
        }
        
        // IMPORTANT: Skip adding old observation factors to avoid crashes
        // Old frames may have been marginalized from ISAM2
        // Let future observations create factors naturally
        
        /* COMMENTED OUT - This causes crashes when referencing old frames
        // Add observation factors from all frames that observed this tentative landmark
        auto observing_frames = tentative->get_observing_frames();
        int valid_observations = 0;
        for (int frame_id : observing_frames) {
          if (frames_.count(frame_id) > 0) {
            gtsam::Symbol pose_key('x', frame_id);
            
            // Check if this pose still exists in values (either current estimate or new values)
            bool pose_exists = false;
            try {
              auto current_estimate = isam2_->calculateEstimate();
              pose_exists = current_estimate.exists(pose_key) || initial_values_.exists(pose_key);
            } catch (...) {
              // If there's any error checking, assume pose doesn't exist
              pose_exists = false;
            }
            
            if (!pose_exists) {
              RCLCPP_DEBUG(rclcpp::get_logger("cone_mapping"), 
                          "Skipping observation from frame %d - pose no longer available", frame_id);
              continue;
            }
            
            // Find the observation from this frame
            for (const auto& obs : tentative->get_observations()) {
              if (obs.frame_id == frame_id) {
                // Create observation factor
                auto obs_noise = gtsam::noiseModel::Diagonal::Sigmas(
                    gtsam::Vector2(config_.cone_observation_noise, config_.cone_observation_noise));
                new_factors_.emplace_shared<ConeObservationFactor>(
                    pose_key, landmark_key, gtsam::Point2(obs.sensor_position.x(), obs.sensor_position.y()), obs_noise);
                valid_observations++;
                break;
              }
            }
          }
        }
        
        // If no valid observations could be added, skip this landmark
        if (valid_observations == 0) {
          RCLCPP_WARN(rclcpp::get_logger("cone_mapping"), 
                      "Skipping landmark promotion - no valid observations in current graph");
          // Clean up only if actually added
          if (initial_values_.exists(landmark_key)) {
            initial_values_.erase(landmark_key);
          }
          if (landmarks_.find(landmark_id) != landmarks_.end()) {
            landmarks_.erase(landmark_id);
          }
          next_landmark_id_--;
          continue;
        }
        */ // END OF COMMENTED OUT SECTION
        
        promoted_ids.push_back(id);
        
        RCLCPP_INFO(rclcpp::get_logger("cone_mapping"), 
                    "Promoted tentative landmark %d to confirmed landmark %d (color: %d, observations: %zu)", 
                    id, landmark_id, static_cast<int>(color), tentative->get_observation_count());
      }
    }
    
    // Remove promoted tentative landmarks
    for (int id : promoted_ids) {
      tentative_landmarks_.erase(id);
    }
  }
};

} // namespace cone_stellation