#pragma once

#include <memory>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "cone_stellation/viewer/viewer_base.hpp"
#include "cone_stellation/common/cone.hpp"
#include "cone_stellation/mapping/cone_mapping.hpp"

namespace cone_stellation {
namespace viewer {

/**
 * @brief SLAM-specific visualization for cone mapping results
 * 
 * Handles visualization of:
 * - Mapped cone landmarks with proper colors
 * - Factor graph edges (pose-pose, pose-landmark, inter-landmark)
 * - Keyframe poses
 * - Optimized trajectory
 */
class SLAMVisualizer : public ViewerBase {
public:
  using Ptr = std::shared_ptr<SLAMVisualizer>;
  
  SLAMVisualizer(rclcpp::Node* node) : node_(node) {
    setName("SLAMVisualizer");
  }
  
  bool initialize() override {
    // Create publishers
    landmark_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/landmarks", 10);
    factor_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/factor_graph", 10);
    keyframe_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/keyframes", 10);
    path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("/slam/path", 10);
    
    initialized_ = true;
    return true;
  }
  
  void shutdown() override {
    clear();
    initialized_ = false;
  }
  
  bool isInitialized() const override { return initialized_; }
  
  void update() override {
    // Update is triggered by specific visualization calls
  }
  
  void clear() override {
    publishDeleteAll("/slam/landmarks");
    publishDeleteAll("/slam/factor_graph");
    publishDeleteAll("/slam/keyframes");
  }
  
  /**
   * @brief Visualize cone landmarks from mapping
   */
  void visualizeLandmarks(const std::unordered_map<int, ConeLandmark::Ptr>& landmarks) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Delete all marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    delete_marker.ns = "cone_landmarks";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // Add cone markers
    for (const auto& [id, landmark] : landmarks) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = node_->now();
      marker.ns = "cone_landmarks";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Position
      marker.pose.position.x = landmark->position().x();
      marker.pose.position.y = landmark->position().y();
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      // Scale
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.5;
      
      // Color based on cone type
      setMarkerColor(marker, landmark->color());
      marker.color.a = 0.8;
      
      markers.markers.push_back(marker);
      
      // Add text label
      visualization_msgs::msg::Marker text_marker = marker;
      text_marker.ns = "cone_ids";
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.pose.position.z = 0.7;
      text_marker.scale.x = 0.0;
      text_marker.scale.y = 0.0;
      text_marker.scale.z = 0.3;
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      text_marker.text = std::to_string(id);
      
      markers.markers.push_back(text_marker);
    }
    
    landmark_pub_->publish(markers);
  }
  
  /**
   * @brief Visualize factor graph structure
   */
  void visualizeFactorGraph(const gtsam::NonlinearFactorGraph& graph, 
                           const gtsam::Values& values) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Delete all marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    delete_marker.ns = "factors";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    int marker_id = 0;
    
    // Iterate through factors
    for (const auto& factor : graph) {
      if (!factor) continue;
      
      visualization_msgs::msg::Marker line_marker;
      line_marker.header.frame_id = "map";
      line_marker.header.stamp = node_->now();
      line_marker.ns = "factors";
      line_marker.id = marker_id++;
      line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // Get factor keys to determine type
      const auto& keys = factor->keys();
      if (keys.size() < 2) continue;
      
      // Determine factor type and set color/width accordingly
      char type1 = gtsam::Symbol(keys[0]).chr();
      char type2 = gtsam::Symbol(keys[1]).chr();
      
      if (type1 == 'x' && type2 == 'x') {
        // Odometry factor (pose-to-pose) - thick green line
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        line_marker.scale.x = 0.05;
        line_marker.ns = "odometry_factors";
      } else if ((type1 == 'x' && type2 == 'l') || (type1 == 'l' && type2 == 'x')) {
        // Observation factor (pose-to-landmark) - thin blue line
        line_marker.color.r = 0.0;
        line_marker.color.g = 0.5;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.6;
        line_marker.scale.x = 0.02;
        line_marker.ns = "observation_factors";
      } else if (type1 == 'l' && type2 == 'l') {
        // Inter-landmark factor - red dashed line
        line_marker.color.r = 1.0;
        line_marker.color.g = 0.0;
        line_marker.color.b = 0.0;
        line_marker.color.a = 0.8;
        line_marker.scale.x = 0.03;
        line_marker.ns = "inter_landmark_factors";
      } else {
        // Other factors - gray
        line_marker.color.r = 0.5;
        line_marker.color.g = 0.5;
        line_marker.color.b = 0.5;
        line_marker.color.a = 0.5;
        line_marker.scale.x = 0.02;
      }
      
      // Create line between factor nodes
      if (keys.size() == 2) {
        try {
          geometry_msgs::msg::Point p1, p2;
          
          // Try to get positions
          if (values.exists(keys[0]) && values.exists(keys[1])) {
            // Extract positions from poses or landmarks
            extractPosition(values, keys[0], p1);
            extractPosition(values, keys[1], p2);
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            markers.markers.push_back(line_marker);
          }
        } catch (...) {
          // Skip if values not available
        }
      }
    }
    
    factor_pub_->publish(markers);
  }
  
  /**
   * @brief Visualize keyframe poses
   */
  void visualizeKeyframes(const std::unordered_map<int, gtsam::Pose2>& keyframe_poses) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Delete all marker
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    delete_marker.ns = "keyframes";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    // Add keyframe markers
    for (const auto& [id, pose] : keyframe_poses) {
      // Create arrow marker for pose
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "map";
      arrow.header.stamp = node_->now();
      arrow.ns = "keyframes";
      arrow.id = id;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;
      
      // Position
      arrow.pose.position.x = pose.x();
      arrow.pose.position.y = pose.y();
      arrow.pose.position.z = 0.1;  // Slightly above ground
      
      // Orientation from yaw angle
      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta());
      arrow.pose.orientation.x = q.x();
      arrow.pose.orientation.y = q.y();
      arrow.pose.orientation.z = q.z();
      arrow.pose.orientation.w = q.w();
      
      // Scale
      arrow.scale.x = 0.5;  // Arrow length
      arrow.scale.y = 0.1;  // Arrow width
      arrow.scale.z = 0.1;  // Arrow height
      
      // Color - cyan for keyframes
      arrow.color.r = 0.0;
      arrow.color.g = 1.0;
      arrow.color.b = 1.0;
      arrow.color.a = 0.8;
      
      markers.markers.push_back(arrow);
      
      // Add text label for keyframe ID
      visualization_msgs::msg::Marker text;
      text.header = arrow.header;
      text.ns = "keyframe_ids";
      text.id = id;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      
      text.pose.position.x = pose.x();
      text.pose.position.y = pose.y();
      text.pose.position.z = 0.8;  // Above arrow
      text.pose.orientation.w = 1.0;
      
      text.scale.z = 0.3;  // Text height
      
      text.color.r = 1.0;
      text.color.g = 1.0;
      text.color.b = 1.0;
      text.color.a = 1.0;
      
      text.text = "KF" + std::to_string(id);
      
      markers.markers.push_back(text);
    }
    
    keyframe_pub_->publish(markers);
  }
  
  /**
   * @brief Update and publish SLAM path
   */
  void updatePath(const nav_msgs::msg::Path& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    path_pub_->publish(path);
  }

private:
  void publishDeleteAll(const std::string& topic) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);
    
    if (topic == "/slam/landmarks") {
      landmark_pub_->publish(markers);
    } else if (topic == "/slam/factor_graph") {
      factor_pub_->publish(markers);
    } else if (topic == "/slam/keyframes") {
      keyframe_pub_->publish(markers);
    }
  }
  
  void setMarkerColor(visualization_msgs::msg::Marker& marker, ConeColor color) {
    switch (color) {
      case ConeColor::YELLOW:
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case ConeColor::BLUE:
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        break;
      case ConeColor::RED:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        break;
      case ConeColor::ORANGE:
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        break;
      default:
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
  }
  
  void extractPosition(const gtsam::Values& values, gtsam::Key key, 
                      geometry_msgs::msg::Point& point) {
    if (gtsam::Symbol(key).chr() == 'x') {
      // Pose
      auto pose = values.at<gtsam::Pose2>(key);
      point.x = pose.x();
      point.y = pose.y();
      point.z = 0.0;
    } else {
      // Landmark
      auto landmark = values.at<gtsam::Point2>(key);
      point.x = landmark.x();
      point.y = landmark.y();
      point.z = 0.0;
    }
  }
  
  rclcpp::Node* node_;
  bool initialized_ = false;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr factor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr keyframe_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

} // namespace viewer
} // namespace cone_stellation