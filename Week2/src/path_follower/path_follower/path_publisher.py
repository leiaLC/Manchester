
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from my_custom_msgs.msg import PathPose
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        
        # Publisher para publicar cada punto como PathPose
        self.path_publisher = self.create_publisher(PathPose, '/path', 10)
        
        # Lista de puntos con [x, y, yaw (rad), v, w, t]
        self.points = [
            [0.5, -0.5, 0.0, 0.2, 0.5, 10.0],
            [1.0, 0.5, np.pi/4, 0.2, 0.5, 8.0],
            [1.5, -0.5, np.pi/2, 0.2, 0.5, 6.0],
            [2.0, 0.5, np.pi, 0.2, 0.5, 7.0],
            [2.5, 0.0, np.pi, 0.2, 0.5, 7.0],
            [0.0, 0.0, np.pi, 0.2, 0.5, 7.0]
        ]

        # Hz de publicación
        self.publish_rate = 0.5
        self.index = 0
        
        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_pose)

        self.get_logger().info(f'PathPose publisher iniciado con {len(self.points)} puntos')
    
    def publish_next_pose(self):
        if self.index >= len(self.points):
            self.get_logger().info("Todos los puntos fueron publicados.")
            return
        
        x, y, yaw, v, w, t = self.points[self.index]

        # Crear mensaje tipo PathPose
        msg = PathPose()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        # Convertir yaw a quaternion (sólo z y w se usan en 2D)
        msg.pose.orientation.z = float(np.sin(yaw / 2.0))
        msg.pose.orientation.w = float(np.cos(yaw / 2.0))

        msg.linear_velocity = v
        msg.angular_velocity = w
        msg.time = t

        # Publicar
        self.path_publisher.publish(msg)
        self.get_logger().info(f'Publicado punto {self.index + 1}: ({x}, {y}, {np.degrees(yaw)}°)')

        self.index += 1

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
