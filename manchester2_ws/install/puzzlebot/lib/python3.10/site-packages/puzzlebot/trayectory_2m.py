import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class PolygonPublisher2m(Node):
    def __init__(self):
        super().__init__('trayectory_2m')
        
        # Parámetros del polígono
        self.declare_parameter('sides', 4)
        
        # Parámetros de velocidad
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 1.5)
        
        self.sides = self.get_parameter('sides').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        # Distancia fija a 2m
        self.side_distance = 2.0
        
        # Ángulo de rotación en radianes
        self.angle = 2 * np.pi / self.sides

        self.timer_period = 0.1 # Es importante cmabiar de acuerdo a los limites de velocidades que metimos / FALTA HACER PRUEBAS PARA DEFINIR BIEN LIMITES
        
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Estados
        self.MOVING = 0
        self.TURNING = 1
        self.STOPPED = 2
        
        # Estado actual
        self.state = self.MOVING
        
        # Contadores
        self.side_count = 0
        self.distance_moved = 0.0
        self.angle_turned = 0.0
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'Iniciando polígono de {self.sides} lados')
        self.get_logger().info(f'Ángulo de giro: {self.angle * 180 / np.pi} grados')
        self.get_logger().info(f'Cada lado tendrá {self.side_distance} metros')
    
    def timer_callback(self):
        self.cmd_vel = Twist()

        if self.state == self.MOVING:
            self.cmd_vel.linear.x = self.linear_velocity
            self.cmd_vel.angular.z = 0.0
            
            # d = v * t
            distance_increment = self.linear_velocity * self.timer_period
            self.distance_moved += distance_increment
            
            # Publish
            self.movement_publisher.publish(self.cmd_vel)
            
            # Revisa distancia recorrida
            if self.distance_moved >= self.side_distance:
                self.distance_moved = 0.0
                self.state = self.TURNING
                self.get_logger().info(f'Completado lado {self.side_count + 1}')

                self.cmd_vel.linear.x = 0.0
                self.movement_publisher.publish(self.cmd_vel)
        
        elif self.state == self.TURNING:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.angular_velocity
            
            # Publish
            self.movement_publisher.publish(self.cmd_vel)
            
            # Ángulo recorrido
            angle_increment = self.angular_velocity * self.timer_period
            self.angle_turned += angle_increment
            
            # Revisa ángulo recorrido
            if self.angle_turned >= self.angle:
                self.angle_turned = 0.0
                self.side_count += 1
                
                # Verificar si ha completado todos los lados
                if self.side_count >= self.sides:
                    self.state = self.STOPPED
                    self.side_count = 0
                    self.get_logger().info('Polígono completado')
                
                    self.cmd_vel.angular.z = 0.0
                    self.movement_publisher.publish(self.cmd_vel)
                else:
                    self.state = self.MOVING
                    self.get_logger().info(f'Girando para lado {self.side_count + 1}')

                    self.cmd_vel.angular.z = 0.0
                    self.movement_publisher.publish(self.cmd_vel)
        
        elif self.state == self.STOPPED:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            self.movement_publisher.publish(self.cmd_vel)
    
    def parameters_callback(self, params):
        for param in params:
            # Para lados
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
                    self.distance_moved = 0.0
                    self.angle_turned = 0.0
            
            # Para vel_lineal
            elif param.name == 'linear_velocity':
                if (param.value < 0.1): # MODIFICAR AA VEL MIN
                    self.get_logger().warn("Linear velocity can't be less than ...")
                    return SetParametersResult(successful=False, reason="Linear velocity can't be less than ...")
                elif (param.value > 0.3): # MODIFICAR AA VEL MAX
                    self.get_logger().warn("Linear velocity can't be more than ...")
                    return SetParametersResult(successful=False, reason="Linear velocity can't be more than ...")
                else:
                    self.linear_velocity = param.value
                    self.get_logger().info(f'Velocidad lineal actualizada: {self.linear_velocity}')
                
            # Para vel_angular
            elif param.name == 'angular_velocity':
                if (param.value < 0.5): # MODIFICAR AA VEL MIN
                    self.get_logger().warn("Angular velocity can't be less than ...")
                    return SetParametersResult(successful=False, reason="Angular velocity can't be less than ...")
                elif (param.value > 2.5): # MODIFICAR AA VEL MAX
                    self.get_logger().warn("Angular velocity can't be more than ...")
                    return SetParametersResult(successful=False, reason="Angular velocity can't be more than ...")
                else:
                    self.angular_velocity = param.value
                    self.turn_time = self.angle / self.angular_velocity
                    self.get_logger().info(f'Velocidad angular actualizada: {self.angular_velocity}')
        
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    polygon_publisher_2m = PolygonPublisher2m()
    
    try:
        rclpy.spin(polygon_publisher_2m)
    except KeyboardInterrupt:
        pass
    finally:        
        polygon_publisher_2m.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()