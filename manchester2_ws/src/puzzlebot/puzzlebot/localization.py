import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

class Localization(Node):

    def __init__(self):
        super().__init__('localization')

        self.X = 0.0
        self.Y = 0.0
        self.ang = 0.0

        self.r = 0.05  # Radio de la rueda (en metros)
        self.l = 0.2   # Distancia entre ruedas (en metros)
        self.sample_time = 0.01

        self.start_time = 0.0
        self.current_time = 0.0
        self.last_time = 0.0
        self.first = True

        self.V = 0.0
        self.Omega = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.sub_encR = self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, 10)
        self.sub_encL = self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_msg = Odometry()

        # Timer a 100 Hz
        self.timer = self.create_timer(self.sample_time, self.run)

        self.get_logger().info("Localization Node Started.")

    def encR_callback(self, msg):
        self.wr = msg.data

    def encL_callback(self, msg):
        self.wl = msg.data

    def run(self):
        # Tiempo
        if self.first:
            self.start_time = self.get_clock().now()
            self.last_time = self.start_time
            self.first = False
            return

        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds * 1e-9

        if dt >= self.sample_time:
            # Cinemática diferencial
            self.V = (self.r / 2.0) * (self.wr + self.wl)
            self.Omega = (self.r / self.l) * (self.wr - self.wl)

            # Actualización de estado
            self.X += self.V * np.cos(self.ang) * dt
            self.Y += self.V * np.sin(self.ang) * dt
            self.ang += self.Omega * dt

            self.last_time = self.current_time

            self.publish_odometry()

    def angle_to_quaternion(self, yaw):
        # Convertir un ángulo en Z a cuaternión (solo rotación en Z)
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def publish_odometry(self):
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.frame_id = 'odom'

        self.odom_msg.pose.pose.position.x = self.X
        self.odom_msg.pose.pose.position.y = self.Y
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation = self.angle_to_quaternion(self.ang)

        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.twist.twist.linear.x = self.V
        self.odom_msg.twist.twist.angular.z = self.Omega

        self.odom_pub.publish(self.odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
