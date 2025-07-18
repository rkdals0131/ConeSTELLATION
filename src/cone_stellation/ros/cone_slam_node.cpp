#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cone_stellation/preprocessing/cone_preprocessor.hpp"
#include "cone_stellation/mapping/cone_mapping.hpp"
#include "cone_stellation/common/tentative_landmark.hpp"
#include "cone_stellation/util/ros_utils.hpp"
#include "custom_interface/msg/tracked_cone_array.hpp"

namespace cone_stellation {

class ConeSLAMNode : public rclcpp::Node {
public:
  ConeSLAMNode() 
    : Node("cone_slam"), 
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(this) {
    
    // Load configuration
    load_config();
    
    // Initialize components
    preprocessor_ = std::make_shared<ConePreprocessor>(preprocess_config_);
    mapping_ = std::make_shared<ConeMapping>(mapping_config_);
    
    // Subscribers
    cone_sub_ = this->create_subscription<custom_interface::msg::TrackedConeArray>(
        "/fused_sorted_cones_ukf_sim", 10,
        std::bind(&ConeSLAMNode::cone_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 100,
        std::bind(&ConeSLAMNode::odom_callback, this, std::placeholders::_1));
    
    // Publishers
    map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/landmarks", 10);
    factor_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/slam/inter_landmark_factors", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/slam/trajectory", 10);
    
    // Timers
    visualization_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ConeSLAMNode::visualization_callback, this));
    
    // Initialize map->odom transform as identity
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(map_to_odom);
    
    RCLCPP_INFO(this->get_logger(), "ConeSLAM node initialized");
    RCLCPP_INFO(this->get_logger(), "Inter-landmark factors: %s", 
                mapping_config_.enable_inter_landmark_factors ? "ENABLED" : "DISABLED");
  }

private:
  void load_config() {
    // Preprocessing parameters
    preprocess_config_.max_cone_distance = 
        this->declare_parameter("preprocessing.max_cone_distance", 20.0);
    preprocess_config_.min_cone_confidence = 
        this->declare_parameter("preprocessing.min_cone_confidence", 0.5);
    preprocess_config_.enable_pattern_detection = 
        this->declare_parameter("preprocessing.enable_pattern_detection", true);
    preprocess_config_.line_fitting_threshold = 
        this->declare_parameter("preprocessing.line_fitting_threshold", 0.2);
    preprocess_config_.min_cones_for_line = 
        this->declare_parameter("preprocessing.min_cones_for_line", 3);
    preprocess_config_.association_threshold = 
        this->declare_parameter("preprocessing.association_threshold", 1.0);
    preprocess_config_.max_tracking_frames = 
        this->declare_parameter("preprocessing.max_tracking_frames", 10);
    
    // Mapping parameters
    mapping_config_.enable_inter_landmark_factors = 
        this->declare_parameter("mapping.enable_inter_landmark_factors", true);
    mapping_config_.inter_landmark_distance_noise = 
        this->declare_parameter("mapping.inter_landmark_distance_noise", 0.1);
    mapping_config_.optimize_every_n_frames = 
        this->declare_parameter("mapping.optimize_every_n_frames", 10);
    mapping_config_.min_covisibility_count = 
        this->declare_parameter("mapping.min_covisibility_count", 2);
    mapping_config_.max_landmark_distance = 
        this->declare_parameter("mapping.max_landmark_distance", 10.0);
    mapping_config_.max_association_distance = 
        this->declare_parameter("association.max_association_distance", 2.0);
    
    // Tentative landmark parameters
    TentativeLandmark::min_observations_ = 
        this->declare_parameter("tentative_landmark.min_observations", 3);
    TentativeLandmark::min_time_span_ = 
        this->declare_parameter("tentative_landmark.min_time_span", 0.5);
    TentativeLandmark::max_position_variance_ = 
        this->declare_parameter("tentative_landmark.max_position_variance", 0.5);
    TentativeLandmark::min_color_confidence_ = 
        this->declare_parameter("tentative_landmark.min_color_confidence", 0.6);
    TentativeLandmark::max_observations_ = 
        this->declare_parameter("tentative_landmark.max_observations", 20);
    
    // Keyframe parameters
    keyframe_translation_threshold_ = 
        this->declare_parameter("keyframe.translation_threshold", 1.0);
    keyframe_rotation_threshold_ = 
        this->declare_parameter("keyframe.rotation_threshold", 0.2);
  }
  
  void cone_callback(const custom_interface::msg::TrackedConeArray::SharedPtr msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Received cone detection with %zu cones", msg->cones.size());
    
    // Get current robot pose
    geometry_msgs::msg::TransformStamped transform;
    try {
      // First try to get odom->base_link transform (from dummy publisher)
      transform = tf_buffer_.lookupTransform("odom", "base_link", 
                                           tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }
    
    // Convert to Eigen
    Eigen::Isometry3d sensor_pose = Eigen::Isometry3d::Identity();
    sensor_pose.translation() = Eigen::Vector3d(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
    sensor_pose.rotate(Eigen::Quaterniond(
        transform.transform.rotation.w,
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z));
    
    // Convert ROS message to internal format
    auto observations = from_ros_msg(*msg);
    
    // Preprocess observations
    auto processed = preprocessor_->process(observations, sensor_pose, 
                                          rclcpp::Time(msg->header.stamp).seconds());
    
    // Check if this should be a keyframe
    if (should_create_keyframe(sensor_pose)) {
      // Create estimation frame
      auto frame = std::make_shared<EstimationFrame>();
      frame->timestamp = rclcpp::Time(msg->header.stamp).seconds();
      frame->T_world_sensor = sensor_pose;
      frame->cone_observations = processed;
      frame->is_keyframe = true;
      
      // Add to mapping
      mapping_->add_keyframe(frame);
      
      last_keyframe_pose_ = sensor_pose;
      
      RCLCPP_INFO(this->get_logger(), "Added keyframe with %zu cone observations", 
                  processed->cones.size());
    }
  }
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Store odometry for motion model
    last_odom_ = *msg;
  }
  
  bool should_create_keyframe(const Eigen::Isometry3d& current_pose) {
    if (!last_keyframe_pose_) {
      return true; // First frame
    }
    
    // Check translation
    double trans_dist = (current_pose.translation() - 
                        last_keyframe_pose_->translation()).norm();
    if (trans_dist > keyframe_translation_threshold_) {
      return true;
    }
    
    // Check rotation
    Eigen::AngleAxisd angle_diff(current_pose.rotation() * 
                                 last_keyframe_pose_->rotation().transpose());
    if (std::abs(angle_diff.angle()) > keyframe_rotation_threshold_) {
      return true;
    }
    
    return false;
  }
  
  void visualization_callback() {
    // Publish map->odom transform (identity for now)
    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = this->now();
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.translation.x = 0.0;
    map_to_odom.transform.translation.y = 0.0;
    map_to_odom.transform.translation.z = 0.0;
    map_to_odom.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(map_to_odom);
    
    // Get current estimates
    auto landmarks = mapping_->get_landmarks();
    
    // Only publish if we have landmarks
    if (!landmarks.empty()) {
      // Publish cone map
      auto map_markers = create_cone_markers(landmarks, "map", this->now());
      map_pub_->publish(map_markers);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Publishing %zu landmarks", landmarks.size());
    }
    
    // Publish factor graph visualization
    try {
      auto factor_graph = mapping_->get_factor_graph();
      auto values = mapping_->get_current_estimate();
      
      if (factor_graph.size() > 0) {
        auto factor_markers = create_factor_markers(factor_graph, values, "map", this->now());
        factor_pub_->publish(factor_markers);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Publishing %zu factors", factor_graph.size());
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Failed to publish factors: %s", e.what());
    }
    
    // Publish current pose
    try {
      auto values = mapping_->get_current_estimate();
      if (!values.empty()) {
        // Get latest pose - need to find the highest pose index
        int latest_pose_id = -1;
        for (int i = 0; i < 1000; i++) { // reasonable upper bound
          gtsam::Symbol pose_key('x', i);
          if (values.exists(pose_key)) {
            latest_pose_id = i;
          } else {
            break;
          }
        }
        
        if (latest_pose_id >= 0) {
          gtsam::Symbol latest_pose_key('x', latest_pose_id);
          auto pose2d = values.at<gtsam::Pose2>(latest_pose_key);
          
          geometry_msgs::msg::PoseStamped pose_msg;
          pose_msg.header.stamp = this->now();
          pose_msg.header.frame_id = "map";
          pose_msg.pose.position.x = pose2d.x();
          pose_msg.pose.position.y = pose2d.y();
          pose_msg.pose.position.z = 0.0;
          
          tf2::Quaternion q;
          q.setRPY(0, 0, pose2d.theta());
          pose_msg.pose.orientation = tf2::toMsg(q);
          
          pose_pub_->publish(pose_msg);
          
          // Publish TF
          geometry_msgs::msg::TransformStamped tf_msg;
          tf_msg.header = pose_msg.header;
          tf_msg.child_frame_id = "base_link_slam";
          tf_msg.transform.translation.x = pose2d.x();
          tf_msg.transform.translation.y = pose2d.y();
          tf_msg.transform.translation.z = 0.0;
          tf_msg.transform.rotation = pose_msg.pose.orientation;
          
          tf_broadcaster_.sendTransform(tf_msg);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Current pose: x=%.2f, y=%.2f, theta=%.2f", 
                              pose2d.x(), pose2d.y(), pose2d.theta());
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Visualization error: %s", e.what());
    }
  }
  
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  
  // Subscribers
  rclcpp::Subscription<custom_interface::msg::TrackedConeArray>::SharedPtr cone_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr factor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  
  // SLAM components
  ConePreprocessor::Ptr preprocessor_;
  ConeMapping::Ptr mapping_;
  
  // Configuration
  ConePreprocessor::Config preprocess_config_;
  ConeMapping::Config mapping_config_;
  
  // State
  std::optional<Eigen::Isometry3d> last_keyframe_pose_;
  nav_msgs::msg::Odometry last_odom_;
  
  // Parameters
  double keyframe_translation_threshold_;
  double keyframe_rotation_threshold_;
};

} // namespace cone_stellation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<cone_stellation::ConeSLAMNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}