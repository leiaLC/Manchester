# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult



#Class Definition
class TeleopController(Node):
    def __init__(self):
        super().__init__('setpoint_node')

        #Paramaters
        self.declare_parameter('sides', 3)
        self.sides = self.get_parameter('sides').value

        self.timer_period = 0.1
        self.linear_velocity = 0.2
        self.angular_velocity = 1.5

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
        
        self.distance = 1.0
        
        self.side_time = self.distance / self.linear_velocity #tiempo estimado para que cada lado sea de 1m 

        #Angulos
        self.real_angle = 0
        self.desired_angle = 0

        #Create publisher for the IMU

        self.imu_angle = self.create_subscription(Float32, '/yaw', self.angle_output_cb,10)

        #Create a timer for publishing the setpoint data 
        self.timer = self.create_timer(self.timer_period, self.timer_cb) 
        

        self.get_logger().info("Polygon_IMU_Started \U0001F680")

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.cmd_vel = Twist()

    def angle_output_cb(self, msg):
        self.real_angle = msg.data
        if (int(self.real_angle) >= self.desired_angle):
            self.cmd_vel.angular.z = 0.0
            if(self.state == self.TURNING):
                self.side_count +=1
            # Verificar si ha completado todos los lados
            if (self.state != self.STOPPED):
                if self.side_count >= self.sides:
                    self.state = self.STOPPED
                    self.side_count = 0
                    self.get_logger().info('Polígono completado')
                else:
                    self.state = self.MOVING
                    self.get_logger().info(f'Girando para lado {self.side_count + 1}')
        else:
            self.cmd_vel.angular.z = self.angular_velocity
        self.movement_publisher.publish(self.cmd_vel)


    #Timer Callback
    def timer_cb(self):
        
        if self.state == self.MOVING:
            self.cmd_vel.linear.x = self.linear_velocity
            self.cmd_vel.angular.z = 0.0
            
            self.time_count += self.timer_period
            
            # Verificar si ha completado un lado
            if self.time_count >= self.side_time:
                self.time_count = 0
                self.state = self.TURNING
                self.get_logger().info(f'Completado lado {self.side_count + 1}')
                self.desired_angle = self.desired_angle + (360 / self.sides)
        
        elif self.state == self.TURNING:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = self.angular_velocity
                        
        
        elif self.state == self.STOPPED:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.0
            
            self.time_count += self.timer_period
        
        # Publish
        self.movement_publisher.publish(self.cmd_vel)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'sides':
                if param.value > 10 or param.value < 3:
                    self.get_logger().warn(f"Invalid mode '{param.value}'. Use a value between 3 and 9.")
                    return SetParametersResult(successful=False, reason="invalid")
                else:
                    self.sides = param.value
                    self.angle = 2 * np.pi / self.sides

                    self.get_logger().info(f'Cambiado a polígono de {self.sides} lados')
                    self.get_logger().info(f'Nuevo ángulo de giro: {self.angle * 180 / np.pi} grados')
                    
                    # Reiniciar el estado
                    self.state = self.MOVING
                    self.side_count = 0
                    self.time_count = 0

        return SetParametersResult(successful=True)


#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = TeleopController()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()