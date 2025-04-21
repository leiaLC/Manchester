import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from my_custom_msgs.msg import PathPose

from nav_msgs.msg import Odometry

import numpy as np
import math

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.subscription = self.create_subscription(PathPose, '/path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.getPos_callback,10)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        

        self.queue = []
        self.current_point = None

        # Estado actual del robot
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.theta_actual = 0.0

        #Referencias
        self.ref_angle = 0.0
        self.ref_distance = 0.0

        self.target_x = 0.0
        self.target_y = 0.0

        #Ganancias
        self.kp_lineal = 0.1
        self.kp_angular = 0.25

        # Límites de velocidades
        self.max_linear_velocity = 0.2
        self.max_angular_velocity = 1.5

        # Tolerancia angular
        self.angle_tolerance = 0.01

        # Tolerancia linela
        self.distance_tolerance = 0.01

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

    def getPos_callback(self, msg: Odometry):
        self.x_actual = msg.pose.pose.position.x
        self.y_actual = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
    
        # Calcular el ángulo de orientación (yaw)
        self.theta_actual = math.atan2(2.0 * (qw * qz), 1.0 - 2.0 * (qz * qz))

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
                self.start_time = self.get_clock().now().nanoseconds / 1e9

                # self.state = "TURNING"
                

                self.target_x = self.current_point.pose.position.x
                self.target_y = self.current_point.pose.position.y

                self.ref_angle = self.desired_angle(self.target_x, self.target_y) - self.theta_actual
                self.ref_angle = self.normalize_angle(self.ref_angle)
                # self.turn_duration = abs(angle) / np.clip(self.current_point.angular_velocity, 0.01, self.max_angular_velocity)

                self.ref_distance = np.sqrt((self.target_x - self.x_actual)**2 + (self.target_y - self.y_actual)**2)
                # self.move_duration = distance / np.clip(self.current_point.linear_velocity, 0.01, self.max_linear_velocity)

                self.turn_direction = np.sign(self.ref_angle)
                # self.linear_velocity = np.clip(self.current_point.linear_velocity, -self.max_linear_velocity, self.max_linear_velocity)
                # self.angular_velocity = np.clip(self.current_point.angular_velocity, -self.max_angular_velocity, self.max_angular_velocity)

                self.get_logger().info(f'Nuevo objetivo: x={self.target_x}, y={self.target_y} → girar {self.ref_angle:.2f} rad, avanzar {self.ref_distance:.2f}m')
            else:
                self.publish_stop()
                return

        # current_time = self.get_clock().now().nanoseconds / 1e9
        # elapsed_time = current_time - self.start_time

        cmd_vel = Twist()

        #Checa si tiene que ser global
        error_x = self.target_x - self.x_actual
        error_y = self.target_y - self.y_actual
        error_lineal = np.sqrt(error_x ** 2 + error_y ** 2)

        error_ang = math.atan2(error_y,error_x) - self.theta_actual

        cmd_vel.linear.x = self.kp_lineal * error_lineal
        cmd_vel.angular.z = self.kp_angular * error_ang 

        if(abs(error_ang) < self.angle_tolerance  and abs(error_lineal < self.distance_tolerance)):
            self.current_point = None
            self.publish_stop()
            return

        self.cmd_vel_publisher.publish(cmd_vel)

    def publish_stop(self):
        cmd_vel = Twist()
        self.cmd_vel_publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
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