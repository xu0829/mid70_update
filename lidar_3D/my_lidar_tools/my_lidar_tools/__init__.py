#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
import math

class PointCloudDistanceCalc(Node):
    def __init__(self):
        super().__init__('pc_distance_calc')
        
        # 1. 订阅 RViz2 中的 Publish Point 点击话题
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10)
            
        # 2. 订阅 Livox 的雷达点云话题
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.pc_callback,
            10)
            
        # 存储暂时的点击状态
        self.clicked_points = []
        self.ready_to_calc = False

        self.get_logger().info("点云测距节点已启动。请在 RViz2 的顶栏使用 'Publish Point' 工具依次点击【两个点】来框选一个区域！")

    def click_callback(self, msg):
        # RViz中点击后，接收到 3D 坐标
        pt = (msg.point.x, msg.point.y, msg.point.z)
        self.clicked_points.append(pt)
        
        if len(self.clicked_points) == 1:
            self.get_logger().info(f"收到第 1 个点: x={pt[0]:.2f}, y={pt[1]:.2f}, z={pt[2]:.2f}。请点击第 2 个点完成框选...")
        elif len(self.clicked_points) == 2:
            self.get_logger().info(f"收到第 2 个点: x={pt[0]:.2f}, y={pt[1]:.2f}, z={pt[2]:.2f}")
            self.ready_to_calc = True # 标记已点击两个点，让点云回调函数开始计算
            self.get_logger().info("区域框选完成，正在计算该区域内点云的平均距离...")

    def pc_callback(self, msg):
        # 只有集齐两个点才进行计算，算完一次就清空，准备下一次框选
        if not self.ready_to_calc:
            return

        self.ready_to_calc = False 
        
        # 获取框选区域的边界 (最小和最大 X, Y, Z)
        x_coords = [p[0] for p in self.clicked_points]
        y_coords = [p[1] for p in self.clicked_points]
        z_coords = [p[2] for p in self.clicked_points]
        
        # 清空点击点，为下一次框选做准备
        self.clicked_points.clear()
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        min_z, max_z = min(z_coords), max(z_coords)
        
        # 由于点击时可能 Z 轴高度有误差，或者点在 2D 平面上点击，这里可以放宽 Z 轴的容差（比如把Z方向上下扩展1米）
        # 如果你想严格按照点击的三维进行框选，可以去掉以下两行注释。
        min_z -= 1.0
        max_z += 1.0

        # 将二进制的 PointCloud2 解析为包含 (x,y,z) 的列表
        points = pc2.read_points_list(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        valid_points_count = 0
        total_distance = 0.0
        
        for p in points:
            px, py, pz = p[0], p[1], p[2]
            
            # 判断点是否在框选的长方体范围内
            if (min_x <= px <= max_x) and \
               (min_y <= py <= max_y) and \
               (min_z <= pz <= max_z):
                # 累加它到雷达原点 (0,0,0) 的真实物理欧氏距离
                dist_to_origin = math.sqrt(px**2 + py**2 + pz**2)
                total_distance += dist_to_origin
                valid_points_count += 1
        
        # 输出结果
        if valid_points_count > 0:
            avg_distance = total_distance / valid_points_count
            self.get_logger().info(f"== 框选计算成功 ==")
            self.get_logger().info(f"框选范围内共提取到 {valid_points_count} 个有效点")
            self.get_logger().info(f"该区域内点云到雷达原点的【平均距离】为: {avg_distance:.3f} 米\n")
            self.get_logger().info("你可以继续点击新的两点进行下一次框选...\n")
        else:
            self.get_logger().warn(f"提取失败：框选范围内没有打到任何有效点云！请重新点击两点！\n")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceCalc()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()