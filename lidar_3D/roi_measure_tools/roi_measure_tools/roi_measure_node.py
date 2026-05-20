#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker


def quat_to_rot_matrix(qx, qy, qz, qw):
    n = qx * qx + qy * qy + qz * qz + qw * qw
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)

    s = 2.0 / n
    xx = qx * qx * s
    yy = qy * qy * s
    zz = qz * qz * s
    xy = qx * qy * s
    xz = qx * qz * s
    yz = qy * qz * s
    wx = qw * qx * s
    wy = qw * qy * s
    wz = qw * qz * s

    # Rotation matrix: local -> world
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ], dtype=np.float32)


class RoiMeasureNode(Node):
    def __init__(self):
        super().__init__('roi_measure_node')

        self.declare_parameter('point_topic', '/livox/lidar')
        self.declare_parameter('box_topic', '/roi_box_state')
        self.declare_parameter('roi_points_topic', '/roi_points')
        self.declare_parameter('text_topic', '/roi_text')
        self.declare_parameter('min_points', 20)

        self.point_topic = self.get_parameter('point_topic').value
        self.box_topic = self.get_parameter('box_topic').value
        self.roi_points_topic = self.get_parameter('roi_points_topic').value
        self.text_topic = self.get_parameter('text_topic').value
        self.min_points = int(self.get_parameter('min_points').value)

        self.latest_box = None

        self.create_subscription(PointCloud2, self.point_topic, self.cloud_callback, 10)
        self.create_subscription(Marker, self.box_topic, self.box_callback, 10)

        self.roi_pub = self.create_publisher(PointCloud2, self.roi_points_topic, 10)
        self.text_pub = self.create_publisher(Marker, self.text_topic, 10)
        self.mean_pub = self.create_publisher(Float32, '/roi_mean_distance', 10)

        self.last_warn_ns = 0

        # 用于3秒内积累点云及定时发布
        self.accumulated_roi_points = []
        self.latest_header = None
        self.timer = self.create_timer(3.0, self.timer_callback)

        self.get_logger().info(
            f'ROI measure node started. point_topic={self.point_topic}, box_topic={self.box_topic}'
        )

    def box_callback(self, msg: Marker):
        self.latest_box = msg

    def cloud_callback(self, cloud_msg: PointCloud2):
        if self.latest_box is None:
            return

        if cloud_msg.header.frame_id != self.latest_box.header.frame_id:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_warn_ns > 2_000_000_000:
                self.get_logger().warn(
                    f'Frame mismatch: cloud={cloud_msg.header.frame_id}, '
                    f'box={self.latest_box.header.frame_id}. '
                    f'Please keep them in the same frame first.'
                )
                self.last_warn_ns = now_ns
            return

        generator = point_cloud2.read_points(
            cloud_msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )
        pts = np.array([list(p) for p in generator], dtype=np.float32)

        if pts.size == 0:
            return

        center = np.array([
            self.latest_box.pose.position.x,
            self.latest_box.pose.position.y,
            self.latest_box.pose.position.z
        ], dtype=np.float32)

        q = self.latest_box.pose.orientation
        rot = quat_to_rot_matrix(q.x, q.y, q.z, q.w)

        # world -> local for row vectors
        local_pts = (pts - center) @ rot

        half = np.array([
            self.latest_box.scale.x / 2.0,
            self.latest_box.scale.y / 2.0,
            self.latest_box.scale.z / 2.0
        ], dtype=np.float32)

        mask = np.all(np.abs(local_pts) <= half, axis=1)
        roi = pts[mask]

        if roi.shape[0] > 0:
            self.accumulated_roi_points.append(roi)
        self.latest_header = cloud_msg.header

    def timer_callback(self):
        if not self.latest_header or not self.latest_box:
            return

        center = np.array([
            self.latest_box.pose.position.x,
            self.latest_box.pose.position.y,
            self.latest_box.pose.position.z
        ], dtype=np.float32)

        if len(self.accumulated_roi_points) == 0:
            self.publish_text(
                frame_id=self.latest_header.frame_id,
                pos=center,
                text='ROI (3s): 0 points'
            )
            return

        # 合并3秒内积累的所有点
        accumulated_roi = np.vstack(self.accumulated_roi_points)
        self.accumulated_roi_points = []  # 清空缓存，进入下一个3秒周期

        # 发布累积的框内点云
        roi_cloud = point_cloud2.create_cloud_xyz32(self.latest_header, accumulated_roi.tolist())
        self.roi_pub.publish(roi_cloud)

        dists = np.linalg.norm(accumulated_roi, axis=1)
        mean_dist = float(np.mean(dists))
        min_dist = float(np.min(dists))
        centroid = np.mean(accumulated_roi, axis=0)
        centroid_dist = float(np.linalg.norm(centroid))
        
        mean_msg = Float32()
        mean_msg.data = mean_dist
        self.mean_pub.publish(mean_msg)

        if accumulated_roi.shape[0] < self.min_points:
            text = (
                f'points(3s): {accumulated_roi.shape[0]}\n'
                f'too few points (< {self.min_points})\n'
                f'min: {min_dist:.3f} m\n'
                f'centroid: {centroid_dist:.3f} m'
            )
            self.publish_text(
                frame_id=self.latest_header.frame_id,
                pos=centroid,
                text=text
            )
            self.get_logger().info(
                f'[TOO FEW] points={accumulated_roi.shape[0]}, '
                f'min={min_dist:.3f} m, '
                f'centroid={centroid_dist:.3f} m'
            )
            return

        text = (
            f'points(3s): {accumulated_roi.shape[0]}\n'
            f'min_dist: {min_dist:.3f} m\n'
            f'mean_dist: {mean_dist:.3f} m\n'
            f'centroid_dist: {centroid_dist:.3f} m'
        )

        self.publish_text(
            frame_id=self.latest_header.frame_id,
            pos=centroid,
            text=text
        )

        self.get_logger().info(
            f'3s accumulated: points={accumulated_roi.shape[0]}, '
            f'min={min_dist:.3f} m, '
            f'mean={mean_dist:.3f} m, '
            f'centroid={centroid_dist:.3f} m'
        )

    def publish_text(self, frame_id, pos, text):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'roi_text'
        marker.id = 1
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = float(pos[0])
        marker.pose.position.y = float(pos[1])
        marker.pose.position.z = float(pos[2] + 0.25)
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = text
        self.text_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RoiMeasureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()