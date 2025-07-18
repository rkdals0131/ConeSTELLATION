#!/usr/bin/env python3
"""
Visualization Utilities for ConeSTELLATION

Enhanced visualization functions for cone-based SLAM with factor graph support.
Optimized for performance with selective publishing and edge batching.
"""

import numpy as np
from typing import List, Tuple, Dict, Optional, Set
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import rclpy.duration


class VisualizationConfig:
    """Configuration for visualization features"""
    def __init__(self):
        self.publish_cones = True
        self.publish_trajectory = True
        self.publish_factor_edges = True
        self.publish_text_labels = False  # Performance: off by default
        self.factor_edge_max_distance = 30.0  # Don't show edges beyond this distance
        self.factor_edge_decimation = 1  # Show every N-th edge for performance
        self.cone_marker_type = Marker.CYLINDER
        self.edge_line_width = 0.02
        self.trajectory_line_width = 0.05


class FactorGraphVisualizer:
    """Specialized visualizer for factor graph edges"""
    
    # Edge color presets by factor type
    EDGE_COLORS = {
        'odometry': (0.0, 0.8, 0.0, 0.5),      # Green - pose-to-pose
        'observation': (0.0, 0.0, 0.8, 0.3),   # Blue - pose-to-landmark  
        'loop_closure': (0.8, 0.0, 0.8, 0.6),  # Purple - loop closures
        'gps': (0.8, 0.8, 0.0, 0.4),          # Yellow - GPS constraints
        'prior': (0.8, 0.0, 0.0, 0.6),        # Red - prior constraints
    }
    
    def __init__(self, config: VisualizationConfig = None):
        self.config = config or VisualizationConfig()
        self.edge_counter = 0
        
    def create_factor_edges_marker(self, 
                                  edges: List[Tuple[np.ndarray, np.ndarray, str]],
                                  namespace: str = "factor_edges",
                                  frame_id: str = "map",
                                  timestamp: Optional[object] = None) -> MarkerArray:
        """
        Create markers for factor graph edges with performance optimization
        
        Args:
            edges: List of (start_pos, end_pos, edge_type) tuples
            namespace: Marker namespace
            frame_id: TF frame
            timestamp: ROS timestamp
            
        Returns:
            MarkerArray with edge visualizations
        """
        marker_array = MarkerArray()
        
        # Group edges by type for efficient rendering
        edge_groups = {}
        for start_pos, end_pos, edge_type in edges:
            # Skip edges beyond max distance (performance)
            if np.linalg.norm(end_pos - start_pos) > self.config.factor_edge_max_distance:
                continue
                
            # Apply decimation
            self.edge_counter += 1
            if self.edge_counter % self.config.factor_edge_decimation != 0:
                continue
                
            if edge_type not in edge_groups:
                edge_groups[edge_type] = []
            edge_groups[edge_type].append((start_pos, end_pos))
        
        # Create one LINE_LIST marker per edge type
        marker_id = 0
        for edge_type, edge_list in edge_groups.items():
            if not edge_list:
                continue
                
            marker = Marker()
            marker.header.frame_id = frame_id
            if timestamp:
                marker.header.stamp = timestamp.to_msg()
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            
            # Set color based on edge type
            color_tuple = self.EDGE_COLORS.get(edge_type, (0.5, 0.5, 0.5, 0.5))
            marker.color.r = color_tuple[0]
            marker.color.g = color_tuple[1]
            marker.color.b = color_tuple[2]
            marker.color.a = color_tuple[3]
            
            marker.scale.x = self.config.edge_line_width
            marker.pose.orientation.w = 1.0
            
            # Add all edges of this type
            for start_pos, end_pos in edge_list:
                # Start point
                p1 = Point()
                p1.x = float(start_pos[0])
                p1.y = float(start_pos[1])
                p1.z = float(start_pos[2]) if len(start_pos) > 2 else 0.0
                marker.points.append(p1)
                
                # End point
                p2 = Point()
                p2.x = float(end_pos[0])
                p2.y = float(end_pos[1])
                p2.z = float(end_pos[2]) if len(end_pos) > 2 else 0.0
                marker.points.append(p2)
            
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
            marker_id += 1
            
        return marker_array
    
    def create_optimized_cone_markers(self,
                                     cones: Dict[int, Dict],
                                     visible_region: Optional[Tuple[float, float, float]] = None,
                                     namespace: str = "cones",
                                     frame_id: str = "map",
                                     timestamp: Optional[object] = None) -> MarkerArray:
        """
        Create cone markers with visibility culling for performance
        
        Args:
            cones: Dictionary of cone_id -> {'pos': [x,y], 'type': str}
            visible_region: Optional (x, y, radius) for visibility culling
            namespace: Marker namespace
            frame_id: TF frame
            timestamp: ROS timestamp
        """
        marker_array = MarkerArray()
        
        # Clear previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        if timestamp:
            delete_marker.header.stamp = timestamp.to_msg()
        delete_marker.ns = namespace
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Visibility culling
        visible_cones = cones
        if visible_region:
            cx, cy, radius = visible_region
            visible_cones = {}
            for cone_id, cone_data in cones.items():
                pos = cone_data['pos']
                if np.linalg.norm([pos[0] - cx, pos[1] - cy]) <= radius:
                    visible_cones[cone_id] = cone_data
        
        # Group cones by color for batch rendering
        color_groups = {}
        for cone_id, cone_data in visible_cones.items():
            color = cone_data['type']
            if color not in color_groups:
                color_groups[color] = []
            color_groups[color].append((cone_id, cone_data['pos']))
        
        # Create markers by color group
        marker_id = 0
        for color, cone_list in color_groups.items():
            for cone_id, pos in cone_list:
                marker = Marker()
                marker.header.frame_id = frame_id
                if timestamp:
                    marker.header.stamp = timestamp.to_msg()
                marker.ns = namespace
                marker.id = marker_id
                marker.type = self.config.cone_marker_type
                marker.action = Marker.ADD
                
                # Position
                marker.pose.position.x = float(pos[0])
                marker.pose.position.y = float(pos[1])
                marker.pose.position.z = -0.15  # Cone center height
                marker.pose.orientation.w = 1.0
                
                # Scale
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                
                # Color
                color_map = {
                    'yellow': (1.0, 1.0, 0.0, 0.8),
                    'blue': (0.0, 0.0, 1.0, 0.8),
                    'red': (1.0, 0.0, 0.0, 0.8),
                    'unknown': (0.5, 0.5, 0.5, 0.8)
                }
                color_tuple = color_map.get(color.lower(), (0.5, 0.5, 0.5, 0.8))
                marker.color.r = color_tuple[0]
                marker.color.g = color_tuple[1]
                marker.color.b = color_tuple[2]
                marker.color.a = color_tuple[3]
                
                marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
                marker_array.markers.append(marker)
                marker_id += 1
        
        return marker_array


def create_trajectory_marker(poses: List[Tuple[float, float]], 
                           color: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 0.8),
                           line_width: float = 0.05,
                           namespace: str = "trajectory",
                           frame_id: str = "map",
                           timestamp: Optional[object] = None) -> Marker:
    """Create an efficient trajectory marker using LINE_STRIP"""
    marker = Marker()
    marker.header.frame_id = frame_id
    if timestamp:
        marker.header.stamp = timestamp.to_msg()
    marker.ns = namespace
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    
    # Color and scale
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    marker.scale.x = line_width
    marker.pose.orientation.w = 1.0
    
    # Add poses
    for x, y in poses:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        marker.points.append(p)
    
    marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
    return marker


class SLAMVisualizationManager:
    """High-level manager for all SLAM visualizations"""
    
    def __init__(self, node, config: VisualizationConfig = None):
        self.node = node
        self.config = config or VisualizationConfig()
        self.factor_viz = FactorGraphVisualizer(self.config)
        
        # Publishers
        self.cone_pub = node.create_publisher(MarkerArray, '/slam/cones', 10)
        self.trajectory_pub = node.create_publisher(Marker, '/slam/trajectory', 10) 
        self.factor_edge_pub = node.create_publisher(MarkerArray, '/slam/factor_edges', 10)
        
        # State tracking for incremental updates
        self.last_trajectory_length = 0
        self.published_cone_ids: Set[int] = set()
        
    def update_visualization(self,
                           cones: Optional[Dict[int, Dict]] = None,
                           trajectory: Optional[List[Tuple[float, float]]] = None,
                           factor_edges: Optional[List[Tuple[np.ndarray, np.ndarray, str]]] = None,
                           robot_pose: Optional[Tuple[float, float, float]] = None):
        """
        Update all visualizations with new data
        
        Args:
            cones: Cone positions and types
            trajectory: List of robot poses
            factor_edges: Factor graph edges
            robot_pose: Current robot pose for visibility culling
        """
        timestamp = self.node.get_clock().now()
        
        # Update cones with visibility culling
        if cones is not None and self.config.publish_cones:
            visible_region = None
            if robot_pose:
                # Only show cones within 50m of robot
                visible_region = (robot_pose[0], robot_pose[1], 50.0)
            
            markers = self.factor_viz.create_optimized_cone_markers(
                cones, visible_region, timestamp=timestamp
            )
            self.cone_pub.publish(markers)
        
        # Update trajectory (incremental if possible)
        if trajectory is not None and self.config.publish_trajectory:
            if len(trajectory) > self.last_trajectory_length:
                # Only send new portion
                new_portion = trajectory[self.last_trajectory_length:]
                marker = create_trajectory_marker(new_portion, timestamp=timestamp)
                self.trajectory_pub.publish(marker)
                self.last_trajectory_length = len(trajectory)
        
        # Update factor edges
        if factor_edges is not None and self.config.publish_factor_edges:
            markers = self.factor_viz.create_factor_edges_marker(
                factor_edges, timestamp=timestamp
            )
            self.factor_edge_pub.publish(markers)