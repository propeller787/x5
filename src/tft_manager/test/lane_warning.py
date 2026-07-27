#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import numpy as np

class LaneWarningVisualizer(Node):
    def __init__(self):
        super().__init__('lane_warning_visualizer')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/sensor/laneWarningPoint',
            self.listener_callback,
            10)
        
        self.points = []
        self.lock = threading.Lock()
        
        self.get_logger().info('LaneWarning Visualizer started')

    def listener_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 24:
                self.points = []
                for i in range(12):
                    x = msg.data[2*i]
                    y = msg.data[2*i + 1]
                    self.points.append((x, y))
                # 显示左右侧起点（L1和R1）
                self.get_logger().info(f'Received 12 points: L1={self.points[0]}, R1={self.points[6]}')

def update_plot(frame, visualizer, ax):
    with visualizer.lock:
        if visualizer.points:
            ax.clear()
            
            left_points = visualizer.points[:6]
            right_points = visualizer.points[6:]
            
            if left_points:
                left_x = [p[0] for p in left_points]
                left_y = [p[1] for p in left_points]
                ax.scatter(left_x, left_y, c='red', label='Left Points', s=50, zorder=5)
                ax.plot(left_x, left_y, 'r--', alpha=0.5, label='Left Curve')
                
                for i, (x, y) in enumerate(left_points):
                    ax.text(x + 5, y + 5, f'L{i+1}', fontsize=8, color='red')
            
            if right_points:
                right_x = [p[0] for p in right_points]
                right_y = [p[1] for p in right_points]
                ax.scatter(right_x, right_y, c='blue', label='Right Points', s=50, zorder=5)
                ax.plot(right_x, right_y, 'b--', alpha=0.5, label='Right Curve')
                
                for i, (x, y) in enumerate(right_points):
                    ax.text(x + 5, y + 5, f'R{i+1}', fontsize=8, color='blue')
            
            # 计算中心点
            center_x = (left_points[0][0] + right_points[0][0]) / 2
            center_y = (left_points[0][1] + right_points[0][1]) / 2
            ax.scatter(center_x, center_y, c='green', marker='o', s=100, label='Center', zorder=10)
            
            # 使用像素坐标范围
            all_x = [p[0] for p in visualizer.points]
            all_y = [p[1] for p in visualizer.points]
            x_min, x_max = min(all_x) - 50, max(all_x) + 50
            y_min, y_max = min(all_y) - 50, max(all_y) + 50
            
            ax.set_xlabel('X (pixels)')
            ax.set_ylabel('Y (pixels)')
            ax.set_title('Lane Warning Points (Rear View Camera)')
            ax.legend()
            ax.grid(True)
            ax.axis('equal')
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_max, y_min)  # Y轴翻转，模拟图像坐标系
    
    return ax,

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = LaneWarningVisualizer()
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.grid(True)
    ax.set_xlabel('X (pixels)')
    ax.set_ylabel('Y (pixels)')
    ax.set_title('Lane Warning Points (Waiting for data...)')
    ax.set_xlim(0, 800)
    ax.set_ylim(600, 0)  # Y轴翻转
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    ros_thread.start()
    
    ani = animation.FuncAnimation(fig, update_plot, fargs=(visualizer, ax), 
                                  interval=100, blit=False)
    
    plt.show()
    
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
