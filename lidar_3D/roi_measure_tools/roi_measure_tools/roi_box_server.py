#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from interactive_markers import InteractiveMarkerServer
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)


class RoiBoxServer(Node):
    def __init__(self):
        super().__init__('roi_box_server')

        self.declare_parameter('frame_id', 'livox_frame')
        self.declare_parameter('server_namespace', 'roi_box')
        self.declare_parameter('state_topic', '/roi_box_state')

        self.declare_parameter('box_size_x', 0.2)
        self.declare_parameter('box_size_y', 0.3)
        self.declare_parameter('box_size_z', 0.2)

        self.declare_parameter('init_x', 2.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_z', 0.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.server_namespace = self.get_parameter('server_namespace').value
        self.state_topic = self.get_parameter('state_topic').value

        self.box_size_x = float(self.get_parameter('box_size_x').value)
        self.box_size_y = float(self.get_parameter('box_size_y').value)
        self.box_size_z = float(self.get_parameter('box_size_z').value)

        self.init_x = float(self.get_parameter('init_x').value)
        self.init_y = float(self.get_parameter('init_y').value)
        self.init_z = float(self.get_parameter('init_z').value)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.state_pub = self.create_publisher(Marker, self.state_topic, qos)

        self.create_subscription(Float32, '/roi_mean_distance', self.mean_callback, 10)
        self.text_pub = self.create_publisher(Marker, '/roi_box_text', 10)
        self.latest_mean = None

        self.server = InteractiveMarkerServer(self, self.server_namespace)

        self.current_pose = Pose()
        self.current_pose.position.x = self.init_x
        self.current_pose.position.y = self.init_y
        self.current_pose.position.z = self.init_z
        self.current_pose.orientation.w = 1.0

        self.make_roi_box()
        self.publish_box_state()

        self.create_timer(0.5, self.publish_box_state)

        self.get_logger().info(
            f'ROI box server started. frame={self.frame_id}, server_ns={self.server_namespace}, '
            f'state_topic={self.state_topic}'
        )
        self.get_logger().info('In RViz, use the "Interact" tool to drag/rotate the ROI box.')

    def make_visual_box_marker(self) -> Marker:
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = self.box_size_x
        marker.scale.y = self.box_size_y
        marker.scale.z = self.box_size_z

        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 0.25
        return marker

    def add_axis_control(self, int_marker, name, interaction_mode, ox, oy, oz, ow=1.0):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = ow
        control.orientation.x = ox
        control.orientation.y = oy
        control.orientation.z = oz
        control.interaction_mode = interaction_mode
        int_marker.controls.append(control)

    def make_roi_box(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame_id
        int_marker.name = 'roi_box'
        int_marker.description = 'ROI Box'
        int_marker.scale = max(self.box_size_x, self.box_size_y, self.box_size_z) * 1.8
        int_marker.pose = self.current_pose

        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(self.make_visual_box_marker())
        int_marker.controls.append(box_control)

        # Move controls
        self.add_axis_control(
            int_marker, 'move_x', InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0
        )
        self.add_axis_control(
            int_marker, 'move_y', InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0
        )
        self.add_axis_control(
            int_marker, 'move_z', InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0
        )

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.server.applyChanges()

    def mean_callback(self, msg: Float32):
        self.latest_mean = msg.data

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_DOWN,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            self.current_pose = copy.deepcopy(feedback.pose)
            self.publish_box_state()

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            p = feedback.pose.position
            self.get_logger().info(
                f'ROI pose: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
            )

    def publish_box_state(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'roi_box_state'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(self.current_pose)

        marker.scale.x = self.box_size_x
        marker.scale.y = self.box_size_y
        marker.scale.z = self.box_size_z

        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 0.25

        self.state_pub.publish(marker)

        if self.latest_mean is not None:
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'roi_box_text'
            text_marker.id = 0
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = self.current_pose.position.x
            text_marker.pose.position.y = self.current_pose.position.y
            text_marker.pose.position.z = self.current_pose.position.z + self.box_size_z / 2.0 + 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'mean: {self.latest_mean:.3f} m'
            self.text_pub.publish(text_marker)


def main(args=None):
    rclpy.init(args=args)
    node = RoiBoxServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()