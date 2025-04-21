import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        
        # Parámetros del polígono
        self.declare_parameter('sides', 4)
        
        # Parámetros de velocidad
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 1.5)
        self.declare_parameter('timer_period', 0.1)
        
        self.sides = self.get_parameter('sides').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.timer_period = self.get_parameter('timer_period').value
        
        # Ángulo de rotación en radianes
        self.angle = 2 * np.pi / self.sides
        
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estados
        self.MOVING = 0
        self.TURNING = 1
        self.STOPPED = 2
        
        # Estado actual
        self.state = self.MOVING
        
        # Contadores
        self.side_count = 0
        self.time_count = 0
        
        # Tiempo estimado para recorrer un lado (en segundos)
        self.side_time = 3.0 
        
        # Tiempo estimado para girar el ángulo calculado (en segundos)
        self.turn_time = self.angle / self.angular_velocity
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'Iniciando polígono de {self.sides} lados')
        self.get_logger().info(f'Ángulo de giro: {self.angle * 180 / np.pi} grados')
    
    def timer_callback(self):
        self.cmd_vel = Twist()

        if self.state == self.MOVING:
            self.cmd_vel.linear.x = self.linear_velocity
            self.cmd_vel.angular.z = 0.0
            
            self.time_count += self.timer_period
            
            # Verificar si ha completado un lado
            if self.time_count >= self.side_time:
                self.time_count = 0
                self.state = self.TURNING
                self.get_logger().info(f'Completado lado {self.side_count + 1}')
        
        elif self.state == self.TURNING:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.angular_velocity
            
            self.time_count += self.timer_period
            
            # Verificar si ha completado el giro
            if self.time_count >= self.turn_time:
                self.time_count = 0
                self.side_count += 1
                
                # Verificar si ha completado todos los lados
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
            
            self.time_count += self.timer_period
        
        # Publish
        self.movement_publisher.publish(self.cmd_vel)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'sides':
                if (param.value < 3):
                    self.get_logger().warn("Figure can't be less than 3")
                    return SetParametersResult(successful=False, reason="Figure can't be less than 3")
                else:
                    self.sides = param.value
                    self.angle = 2 * np.pi / self.sides
                    self.turn_time = self.angle / self.angular_velocity
                    self.get_logger().info(f'Cambiado a polígono de {self.sides} lados')
                    self.get_logger().info(f'Nuevo ángulo de giro: {self.angle * 180 / np.pi} grados')
                    
                    # Reiniciar el estado
                    self.state = self.MOVING
                    self.side_count = 0
                    self.time_count = 0
            
            elif param.name == 'linear_velocity':
                self.linear_velocity = param.value
                self.get_logger().info(f'Velocidad lineal actualizada: {self.linear_velocity}')
                
            elif param.name == 'angular_velocity':
                self.angular_velocity = param.value
                self.turn_time = self.angle / self.angular_velocity
                self.get_logger().info(f'Velocidad angular actualizada: {self.angular_velocity}')
                
            elif param.name == 'timer_period':
                self.timer_period = param.value
                self.timer.timer_period_ns = int(self.timer_period * 1e9)
                self.get_logger().info(f'Periodo del timer actualizado: {self.timer_period}')
        
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