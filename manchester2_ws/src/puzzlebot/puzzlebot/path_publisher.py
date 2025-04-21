#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Publisher para publicar el path
        self.path_publisher = self.create_publisher(PoseArray, '/path', 10)
        
        # Path en metros
        points = [[-2.0, 0.0], 
                  [5.0, 2.0], 
                  [0.0, 2.0], 
                  [3.0, 0.0]]
        
        # Hz
        publish_rate = 0.5
        
        # Crear el path como PoseArray
        self.path = PoseArray()
        self.path.header.frame_id = "map"  # Coordenadas global
        
        for point in points:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = 0.0
            self.path.poses.append(pose)
        
        # Timer para publicar el path periódicamente
        self.timer = self.create_timer(1.0/publish_rate, self.publish_path)
        
        self.get_logger().info(f'Path publisher iniciado con {len(points)} puntos') #número de puntos
    
    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    path_publisher = PathPublisher()
    
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        pass
    finally:        
        path_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()