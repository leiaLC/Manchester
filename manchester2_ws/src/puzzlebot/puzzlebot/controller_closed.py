import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry
import numpy as np

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.subscription = self.create_subscription(PoseArray, '/path', self.path_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.coordinates = []
        self.last_path_hash = None
        self.current_target_index = 0

        # Estado del robot (ahora se actualiza con odometría)
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

        self.get_logger().info('Nodo PathFollower iniciado.')

    def odom_callback(self, msg):
        # Actualizar la posición y orientación del robot con datos de odometría
        self.x_actual = msg.pose.pose.position.x
        self.y_actual = msg.pose.pose.position.y
        
        # Convertir cuaternión a ángulo de Euler (solo yaw)
        q = msg.pose.pose.orientation
        self.theta_actual = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 
                                     1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def path_callback(self, msg):
        current_coords = [[p.position.x, p.position.y] for p in msg.poses]
        current_hash = hash(tuple(map(tuple, current_coords)))

        if current_hash != self.last_path_hash:
            self.coordinates = current_coords
            self.last_path_hash = current_hash
            self.current_target_index = 0
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

        # Calcular errores
        distance = self.calculate_distance_to_target(target_x, target_y)
        target_angle = self.desired_angle(target_x, target_y)
        angle_error = self.normalize_angle(target_angle - self.theta_actual)

        # Verificar si se alcanzó el punto
        if distance < self.dist_tolerance:
            self.get_logger().info(f"Punto {self.current_target_index} alcanzado.")
            self.current_target_index += 1
            if self.current_target_index >= len(self.coordinates):
                self.publish_stop()
            return

        # Control proporcional
        linear_k = 0.05 # Buscar que se puedan ingresar por el usuario
        angular_k = 0.5

        cmd_vel = Twist()
        cmd_vel.linear.x = min(self.max_linear_velocity, linear_k * distance) # Falta ponerlo dentro de los límites minimos y máximos de nuestro robot
        cmd_vel.angular.z = angular_k * angle_error

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