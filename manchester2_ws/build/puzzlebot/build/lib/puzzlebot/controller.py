import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.subscription = self.create_subscription(PoseArray, '/path', self.path_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.coordinates = []
        self.last_path_hash = None
        self.current_target_index = 0

        # Estado del robot
        self.x_actual = 0.0
        self.y_actual = 0.0
        self.theta_actual = 0.0

        # Control de movimiento
        self.angular_velocity = 1.0  # constante
        self.max_linear_velocity = 0.2  # límite superior
        self.min_linear_velocity = 0.05

        self.angle_tolerance = 0.05
        self.dist_tolerance = 0.05

        # Timer y tiempo
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.start_time = None
        self.move_duration = None
        self.turn_duration = None
        self.linear_velocity = 0.0  # se recalcula dinámicamente

        self.state = 2  # STANDBY

        self.MOVING = 0
        self.TURNING = 1
        self.STOPPED = 2

        self.get_logger().info('Nodo PathFollower iniciado.')

    def path_callback(self, msg):
        current_coords = [[p.position.x, p.position.y] for p in msg.poses]
        current_hash = hash(tuple(map(tuple, current_coords)))

        if current_hash != self.last_path_hash:
            self.coordinates = current_coords
            self.last_path_hash = current_hash
            self.current_target_index = 0
            self.state = self.TURNING
            self.get_logger().info(f'Nuevo path recibido con {len(self.coordinates)} puntos.')
        else:
            self.get_logger().debug('Path no ha cambiado.')

    def calculate_distance_to_target(self, x, y):
        return np.sqrt((x - self.x_actual) ** 2 + (y - self.y_actual) ** 2)

    def desired_angle(self, x, y):
        return np.arctan2(y - self.y_actual, x - self.x_actual)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def control_loop(self):
        if not self.coordinates or self.current_target_index >= len(self.coordinates):
            self.publish_stop()
            return

        target_x, target_y = self.coordinates[self.current_target_index]
        cmd_vel = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if self.state == self.STOPPED:
            self.current_target_index += 1
            if self.current_target_index < len(self.coordinates):
                self.state = self.TURNING
            else:
                self.get_logger().info("Path completado.")
            return

        elif self.state == self.TURNING:
            target_angle = self.desired_angle(target_x, target_y)
            angle_error = self.normalize_angle(target_angle - self.theta_actual)

            if abs(angle_error) < self.angle_tolerance:
                self.start_time = current_time
                distance = self.calculate_distance_to_target(target_x, target_y)

                # Definir un tiempo deseado (por ejemplo 5s para cualquier punto a máxima distancia)
                tiempo_deseado = max(1.0, min(5.0, distance / self.min_linear_velocity))
                self.move_duration = tiempo_deseado
                self.linear_velocity = distance / tiempo_deseado

                # Limitar velocidad lineal
                self.linear_velocity = min(self.linear_velocity, self.max_linear_velocity)

                self.state = self.MOVING
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.angular.z = self.angular_velocity if angle_error > 0 else -self.angular_velocity

        elif self.state == self.MOVING:
            distance = self.calculate_distance_to_target(target_x, target_y)

            if distance < self.dist_tolerance:
                self.state = self.STOPPED
                cmd_vel.linear.x = 0.0
                self.get_logger().info(f"Punto {self.current_target_index} alcanzado.")
            else:
                elapsed_time = current_time - self.start_time
                if elapsed_time < self.move_duration:
                    cmd_vel.linear.x = self.linear_velocity
                else:
                    # Tiempo agotado, frenar y asumir que llegamos al punto
                    cmd_vel.linear.x = 0.0
                    self.state = self.STOPPED
                    self.get_logger().info(f"Punto {self.current_target_index} alcanzado por tiempo límite.")

                # Corrección de ángulo durante movimiento
                target_angle = self.desired_angle(target_x, target_y)
                angle_error = self.normalize_angle(target_angle - self.theta_actual)
                cmd_vel.angular.z = 0.5 * angle_error  # proporcional

        # Publicar y actualizar pose simulada
        self.cmd_vel_publisher.publish(cmd_vel)
        dt = self.timer_period
        self.x_actual += cmd_vel.linear.x * np.cos(self.theta_actual) * dt
        self.y_actual += cmd_vel.linear.x * np.sin(self.theta_actual) * dt
        self.theta_actual += cmd_vel.angular.z * dt
        self.theta_actual = self.normalize_angle(self.theta_actual)

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
