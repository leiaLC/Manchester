# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from rcl_interfaces.msg import SetParametersResult

#Class Definition
class SetPoint(Node):
    def __init__(self):
        super().__init__('setpoint_node')

    
        self.sample_time = 0.1
        self.x = 0.0
        self.z = 0.0

        #Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.joy_sub = self.create_subscription(Joy, '/joy',self.joy_callback,10)

        #Create a timer for publishing the setpoint data 
        self.timer = self.create_timer(self.sample_time, self.timer_cb) 

        #Create a messages and variables to be used
        self.cmd_vel = Twist()

        self.get_logger().info("Teleop Controller Started \U0001F680")


    def joy_callback(self,msg):
        self.z = msg.axes[3]
        self.x = msg.axes[4]

    #Timer Callback
    def timer_cb(self):

        self.cmd_vel.linear.x = self.x/5
        self.cmd_vel.linear.z = self.z/2
        self.signal_publisher.publish(self.cmd_vel)


#Main
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPoint()

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