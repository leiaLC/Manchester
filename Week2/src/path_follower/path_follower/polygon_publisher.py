import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from rclpy.parameter import Parameter

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')

        # Parámetros del polígono
        self.declare_parameter('sides', 4)

        # Parámetros de velocidad
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 1.5)
        self.declare_parameter('side_length', 2.0)
        self.declare_parameter('figure_time', 43.14)  # aproximado para un cuadrado de 4 lados de 2m

        self.sides = self.get_parameter('sides').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.distance = self.get_parameter('side_length').value

        self.timer_period = 0.1

        # Ángulo de rotación en radianes
        self.angle = 2 * np.pi / self.sides

        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Estados
        self.MOVING = 0
        self.TURNING = 1
        self.STOPPED = 2

        # Estado actual
        self.state = self.MOVING

        # Contadores y temporizadores
        self.side_count = 0
        self.start_time = self.get_clock().now().nanoseconds / 1e9

        # Tiempo estimado para recorrer un lado (en segundos)
        self.side_time = self.distance / self.linear_velocity

        # Tiempo estimado para girar el ángulo calculado (en segundos)
        self.turn_time = self.angle / self.angular_velocity

        # Tiempo total
        self.figure_time = self.turn_time * (self.sides) + self.side_time * self.sides
        self.set_parameters([Parameter('figure_time', Parameter.Type.DOUBLE, self.figure_time)])

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(f'Iniciando polígono de {self.sides} lados')
        self.get_logger().info(f'Ángulo de giro: {self.angle * 180 / np.pi} grados')

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        if self.get_parameter('figure_time').value != self.figure_time:
            self.set_parameters([Parameter('figure_time', Parameter.Type.DOUBLE, self.figure_time)])

        if self.get_parameter('linear_velocity').value != self.linear_velocity:
            self.set_parameters([Parameter('linear_velocity', Parameter.Type.DOUBLE, self.linear_velocity)])

        self.cmd_vel = Twist()

        if self.state == self.MOVING:
            self.cmd_vel.linear.x = self.linear_velocity
            self.cmd_vel.angular.z = 0.0

            if elapsed >= self.side_time:
                self.start_time = current_time
                self.state = self.TURNING
                self.get_logger().info(f'Completado lado {self.side_count + 1}')

        elif self.state == self.TURNING:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.angular_velocity

            if elapsed >= self.turn_time:
                self.start_time = current_time
                self.side_count += 1

                if self.side_count >= self.sides:
                    self.state = self.STOPPED
                    self.side_count = 0
                    self.get_logger().info('Polígono completado')
                else:
                    self.state = self.MOVING
                    self.get_logger().info(f'Girando para lado {self.side_count + 1}')

        elif self.state == self.STOPPED:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0

        self.movement_publisher.publish(self.cmd_vel)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'sides':
                if param.value < 3:
                    self.get_logger().warn("Figure can't be less than 3")
                    return SetParametersResult(successful=False, reason="Figure can't be less than 3")
                else:
                    self.sides = param.value
                    self.angle = 2 * np.pi / self.sides
                    self.turn_time = self.angle / self.angular_velocity
                    self.figure_time = self.turn_time * (self.sides) + self.side_time * self.sides
                    self.get_logger().info(f'Cambiado a polígono de {self.sides} lados')
                    self.get_logger().info(f'Nuevo ángulo de giro: {self.angle * 180 / np.pi} grados')
                    self.state = self.MOVING
                    self.side_count = 0
                    self.start_time = self.get_clock().now().nanoseconds / 1e9

            elif param.name == 'linear_velocity':
                if abs(param.value) > 0.2:
                    self.get_logger().warn("Speed can't be more than 0.2 m/s")
                    return SetParametersResult(successful=False, reason="Speed can't be more than 0.2 m/s")
                else:
                    self.linear_velocity = param.value
                    self.side_time = self.distance / self.linear_velocity
                    self.figure_time = self.turn_time * (self.sides) + self.side_time * self.sides
                    self.get_logger().info(f'Velocidad lineal actualizada: {self.linear_velocity}')

            elif param.name == 'angular_velocity':
                if abs(param.value) > 1.5:
                    self.get_logger().warn("Speed can't be more than 1.5 rad/s")
                    return SetParametersResult(successful=False, reason="Speed can't be more than 1.5 rad/s")
                else:
                    self.angular_velocity = param.value
                    self.turn_time = self.angle / self.angular_velocity
                    self.figure_time = self.turn_time * (self.sides) + self.side_time * self.sides
                    self.set_parameters([Parameter('figure_time', Parameter.Type.DOUBLE, self.figure_time)])
                    self.get_logger().info(f'Velocidad angular actualizada: {self.angular_velocity}')

            elif param.name == 'side_length':
                if param.value <= 0:
                    self.get_logger().warn("Length must be more than 0")
                    return SetParametersResult(successful=False, reason="Length must be more than 0")
                else:
                    self.distance = param.value
                    self.side_time = self.distance / self.linear_velocity
                    self.figure_time = self.turn_time * (self.sides - 1) + self.side_time * self.sides
                    self.get_logger().info(f'Medida de lado: {self.distance}')

            elif param.name == 'figure_time':
                min_figure_time = (self.angle / 1.5) * (self.sides) + (self.distance / 0.2) * self.sides
                if param.value < min_figure_time:
                    self.get_logger().warn("Time is too fast")
                    return SetParametersResult(successful=False, reason="Time is too fast")
                else:
                    self.figure_time = param.value
                    total_turn_time = self.turn_time * (self.sides)
                    total_side_time = self.figure_time - total_turn_time
                    self.side_time = total_side_time / self.sides
                    self.linear_velocity = self.distance / self.side_time
                    self.get_logger().info(f'Tiempo de figura total: {self.figure_time}')

        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    polygon_publisher = PolygonPublisher()

    try:
        rclpy.spin(polygon_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        polygon_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
