import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_custom_msgs.msg import PathPose
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.subscription = self.create_subscription(PathPose, '/path', self.path_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.queue = []
        self.current_point = None

        # Estado actual del robot
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.theta_actual = 0.0

        # Límites de velocidades
        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.5

        # Tolerancia angular
        self.angle_tolerance = 0.05

        # Timer para control
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Variables de control
        self.start_time = None
        self.turn_duration = None
        self.move_duration = None
        self.state = "IDLE"

        self.get_logger().info('Nodo PathFollower iniciado.')

    def path_callback(self, msg: PathPose):
        self.queue.append(msg)
        self.get_logger().info(f'Se añadió un nuevo punto a la cola: x={msg.pose.position.x}, y={msg.pose.position.y}')

    def desired_angle(self, x, y):
        dx = x - self.x_actual
        dy = y - self.y_actual
        return np.arctan2(dy, dx)

    def normalize_angle(self, angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def control_loop(self):
        if self.current_point is None:
            if self.queue:
                self.current_point = self.queue.pop(0)
                self.state = "TURNING"
                self.start_time = self.get_clock().now().nanoseconds / 1e9

                target_x = self.current_point.pose.position.x
                target_y = self.current_point.pose.position.y

                angle = self.desired_angle(target_x, target_y) - self.theta_actual
                angle = self.normalize_angle(angle)
                self.turn_duration = abs(angle) / np.clip(self.current_point.angular_velocity, 0.01, self.max_angular_velocity)

                distance = np.sqrt((target_x - self.x_actual)**2 + (target_y - self.y_actual)**2)
                self.move_duration = distance / np.clip(self.current_point.linear_velocity, 0.01, self.max_linear_velocity)

                self.turn_direction = np.sign(angle)
                self.linear_velocity = np.clip(self.current_point.linear_velocity, -self.max_linear_velocity, self.max_linear_velocity)
                self.angular_velocity = np.clip(self.current_point.angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)

                self.get_logger().info(f'Nuevo objetivo: x={target_x}, y={target_y} → girar {angle:.2f} rad en {self.turn_duration:.2f}s, avanzar {distance:.2f}m en {self.move_duration:.2f}s')
            else:
                self.publish_stop()
                return

        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time

        cmd_vel = Twist()

        if self.state == "TURNING":
            if elapsed_time < self.turn_duration:
                cmd_vel.angular.z = self.turn_direction * self.angular_velocity
            else:
                self.state = "MOVING"
                self.start_time = current_time
                self.get_logger().info("Giro completado. Comenzando avance.")

        elif self.state == "MOVING":
            if elapsed_time < self.move_duration:
                cmd_vel.linear.x = self.linear_velocity
            else:
                self.get_logger().info(f'Punto alcanzado: x={self.current_point.pose.position.x}, y={self.current_point.pose.position.y}')
                # Actualiza pose simulada como si se hubiera llegado exactamente al punto
                self.x_actual = self.current_point.pose.position.x
                self.y_actual = self.current_point.pose.position.y
                self.theta_actual += self.turn_direction * self.angular_velocity * self.turn_duration
                self.theta_actual = self.normalize_angle(self.theta_actual)

                self.current_point = None
                self.state = "IDLE"
                self.publish_stop()
                return

        self.cmd_vel_publisher.publish(cmd_vel)

    def publish_stop(self):
        cmd_vel = Twist()
        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()