# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rcl_interfaces.msg import SetParametersResult

# Class Definition
class Trayectory(Node):
    def __init__(self):
        super().__init__('trayectory_node')
        self.declare_parameter('Lados', 3)
        self.n_lados = self.get_parameter('Lados').value
        self.sample_time = 0.1
        self.lineal = 0.2 # m/s
        self.ang = 2.0
        self.lado = True
        self.desired_ang = 360 / self.n_lados
        self.marg_error = 5.0 
        
        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        #self.IMU_sub = self.create_subscription(Float32, '/imu', self.imu_callback, 10)
        
        # Create a timer for publishing the setpoint data
        self.timer = self.create_timer(self.sample_time, self.timer_cb)
        
        # Create a messages and variables to be used
        self.cmd_vel = Twist()
        
        # Params callback
        self.add_on_set_parameters_callback(self.parameters_callback)
    
    # Timer Callback
    def timer_cb(self):
        if ((self.lado == True) and (self.n_lados > 0)):
            self.x = self.lineal
            self.z = 0.0
            self.lado = False
            self.n_lados -= 1
        elif ((self.lado == False) and (self.n_lados > 0)):
            self.x = 0.0
            self.z = self.ang
            self.lado = True
        else:
            self.x = 0.0
            self.z = 0.0
            
        self.cmd_vel.linear.x = self.x
        self.cmd_vel.angular.z = self.z
        self.signal_publisher.publish(self.cmd_vel)
    

    def parameters_callback(self, params):
        for param in params:
            # system gain parameter check
            if param.name == "Lados":
                # check if it is negative
                if (param.value < 3):
                    self.get_logger().warn("Figure can't be less than 3")
                    return SetParametersResult(successful=False, reason="Figure can't be less than 3")
                else:
                    self.n_lados = param.value 
                    self.desired_ang = 360 / self.n_lados 
                    self.get_logger().info(f"Sides updated to {self.n_lados}")
        return SetParametersResult(successful=True)
    
    def imu_callback(self, IMU_read):
        self.actual_deg = IMU_read.data
        if abs(self.desired_ang - self.actual_deg) <= self.marg_error:
            self.ang = 0.0
        else:
            self.ang = 2.0

# Main
def main(args=None):
    rclpy.init(args=args)
    set_point = Trayectory()
    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()