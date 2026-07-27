#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import threading

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
        
        self.get_logger().info('LaneWarning OpenCV Visualizer started')

    def listener_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 24:
                self.points = []
                for i in range(12):
                    x = msg.data[2*i]
                    y = msg.data[2*i + 1]
                    self.points.append((x, y))

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = LaneWarningVisualizer()
    
    canvas_width = 800
    canvas_height = 600
    offset_x = canvas_width // 2
    offset_y = canvas_height // 2
    scale = 100
    
    ros_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    ros_thread.start()
    
    while True:
        canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255
        
        cv2.line(canvas, (0, offset_y), (canvas_width, offset_y), (200, 200, 200), 1)
        cv2.line(canvas, (offset_x, 0), (offset_x, canvas_height), (200, 200, 200), 1)
        
        with visualizer.lock:
            if visualizer.points:
                all_x = [p[0] for p in visualizer.points]
                all_y = [p[1] for p in visualizer.points]
                max_val = max(max(all_x, key=abs), max(all_y, key=abs), 2.0)
                current_scale = min(scale, (canvas_width // 2 - 50) / max_val)
                
                left_points = visualizer.points[:6]
                right_points = visualizer.points[6:]
                
                left_x_px = []
                left_y_px = []
                for i, (x, y) in enumerate(left_points):
                    px = int(offset_x + x * current_scale)
                    py = int(offset_y - y * current_scale)
                    left_x_px.append(px)
                    left_y_px.append(py)
                    
                    cv2.circle(canvas, (px, py), 8, (0, 0, 255), -1)
                    cv2.putText(canvas, f'L{i+1}', (px + 10, py + 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                if len(left_x_px) >= 2:
                    cv2.polylines(canvas, [np.array(list(zip(left_x_px, left_y_px)), np.int32)], 
                                  isClosed=False, color=(0, 0, 255), thickness=2, lineType=cv2.LINE_AA)
                
                right_x_px = []
                right_y_px = []
                for i, (x, y) in enumerate(right_points):
                    px = int(offset_x + x * current_scale)
                    py = int(offset_y - y * current_scale)
                    right_x_px.append(px)
                    right_y_px.append(py)
                    
                    cv2.circle(canvas, (px, py), 8, (255, 0, 0), -1)
                    cv2.putText(canvas, f'R{i+1}', (px + 10, py + 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                
                if len(right_x_px) >= 2:
                    cv2.polylines(canvas, [np.array(list(zip(right_x_px, right_y_px)), np.int32)], 
                                  isClosed=False, color=(255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
                
                cv2.circle(canvas, (offset_x, offset_y), 12, (0, 255, 0), -1)
                cv2.putText(canvas, 'Center', (offset_x + 15, offset_y + 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        cv2.putText(canvas, 'Lane Warning Points', (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        cv2.putText(canvas, 'Red: Left Points | Blue: Right Points', (20, canvas_height - 20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        cv2.imshow('Lane Warning Visualizer', canvas)
        
        key = cv2.waitKey(30)
        if key == 27:
            break
    
    cv2.destroyAllWindows()
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
