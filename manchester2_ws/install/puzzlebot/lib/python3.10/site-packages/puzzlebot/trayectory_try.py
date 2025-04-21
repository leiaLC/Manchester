import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import numpy as np

class PolygonPublisher(Node):
    def __init__(self):
        super().__init__('polygon_publisher')
        
        # Polygon parameters
        self.declare_parameter('sides', 4)
        
        # Velocity parameters with limits
        self.declare_parameter('linear_velocity', 0.2)  # m/s
        self.declare_parameter('angular_velocity', 1.5)  # rad/s
        
        # Fixed timer period (0.1s)
        self.timer_period = 0.1
        
        self.sides = self.get_parameter('sides').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        # Fixed distance per side (2m)
        self.side_distance = 2.0
        
        # Rotation angle in radians
        self.angle = 2 * np.pi / self.sides
        
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # States
        self.MOVING = 0
        self.TURNING = 1
        self.STOPPED = 2
        
        # Current state
        self.state = self.MOVING
        
        # Counters
        self.side_count = 0
        self.distance_moved = 0.0
        self.angle_turned = 0.0
        
        # Time needed to complete each movement
        self.side_time = self.side_distance / self.linear_velocity
        self.turn_time = self.angle / self.angular_velocity
        
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info(f'Starting {self.sides}-sided polygon')
        self.get_logger().info(f'Rotation angle: {self.angle * 180 / np.pi} degrees')
        self.get_logger().info(f'Each side: {self.side_distance} meters')
        self.get_logger().info(f'Side time: {self.side_time:.2f}s, Turn time: {self.turn_time:.2f}s')
    
    def timer_callback(self):
        msg = Twist()

        if self.state == self.MOVING:
            msg.linear.x = self.linear_velocity
            msg.angular.z = 0.0
            
            # Calculate distance moved in this period
            self.distance_moved += self.linear_velocity * self.timer_period
            
            # Publish movement command
            self.movement_publisher.publish(msg)
            
            # Check if we've moved the required distance
            if self.distance_moved >= self.side_distance:
                self.distance_moved = 0.0
                self.state = self.TURNING
                self.get_logger().info(f'Completed side {self.side_count + 1}')
                
                # Stop before turning
                msg.linear.x = 0.0
                self.movement_publisher.publish(msg)
        
        elif self.state == self.TURNING:
            msg.linear.x = 0.0
            msg.angular.z = self.angular_velocity
            
            # Calculate angle turned in this period
            self.angle_turned += self.angular_velocity * self.timer_period
            
            # Publish turning command
            self.movement_publisher.publish(msg)
            
            # Check if we've turned the required angle
            if self.angle_turned >= self.angle:
                self.angle_turned = 0.0
                self.side_count += 1
                
                # Check if polygon is complete
                if self.side_count >= self.sides:
                    self.state = self.STOPPED
                    self.side_count = 0
                    self.get_logger().info('Polygon completed')
                    
                    # Stop completely
                    msg.angular.z = 0.0
                    self.movement_publisher.publish(msg)
                else:
                    self.state = self.MOVING
                    self.get_logger().info(f'Turning for side {self.side_count + 1}')
                    
                    # Stop turning before moving
                    msg.angular.z = 0.0
                    self.movement_publisher.publish(msg)
        
        elif self.state == self.STOPPED:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.movement_publisher.publish(msg)
    
    def parameters_callback(self, params):
        for param in params:
            if param.name == 'sides':
                if param.value < 3:
                    self.get_logger().warn("Number of sides can't be less than 3")
                    return SetParametersResult(successful=False, 
                                            reason="Number of sides can't be less than 3")
                else:
                    self.sides = param.value
                    self.angle = 2 * np.pi / self.sides
                    self.turn_time = self.angle / self.angular_velocity
                    self.get_logger().info(f'Changed to {self.sides}-sided polygon')
                    self.get_logger().info(f'New rotation angle: {self.angle * 180 / np.pi} degrees')
                    
                    # Reset state
                    self.state = self.MOVING
                    self.side_count = 0
                    self.distance_moved = 0.0
                    self.angle_turned = 0.0
            
            elif param.name == 'linear_velocity':
                min_linear = 0.1  # m/s
                max_linear = 0.3  # m/s
                
                if param.value < min_linear:
                    self.get_logger().warn(f"Linear velocity can't be less than {min_linear} m/s")
                    return SetParametersResult(successful=False,
                                             reason=f"Linear velocity can't be less than {min_linear} m/s")
                elif param.value > max_linear:
                    self.get_logger().warn(f"Linear velocity can't be more than {max_linear} m/s")
                    return SetParametersResult(successful=False,
                                             reason=f"Linear velocity can't be more than {max_linear} m/s")
                else:
                    self.linear_velocity = param.value
                    self.side_time = self.side_distance / self.linear_velocity
                    self.get_logger().info(f'Linear velocity updated: {self.linear_velocity} m/s')
                    self.get_logger().info(f'New side time: {self.side_time:.2f}s')
                
            elif param.name == 'angular_velocity':
                min_angular = 0.5  # rad/s
                max_angular = 2.5  # rad/s
                
                if param.value < min_angular:
                    self.get_logger().warn(f"Angular velocity can't be less than {min_angular} rad/s")
                    return SetParametersResult(successful=False,
                                             reason=f"Angular velocity can't be less than {min_angular} rad/s")
                elif param.value > max_angular:
                    self.get_logger().warn(f"Angular velocity can't be more than {max_angular} rad/s")
                    return SetParametersResult(successful=False,
                                             reason=f"Angular velocity can't be more than {max_angular} rad/s")
                else:
                    self.angular_velocity = param.value
                    self.turn_time = self.angle / self.angular_velocity
                    self.get_logger().info(f'Angular velocity updated: {self.angular_velocity} rad/s')
                    self.get_logger().info(f'New turn time: {self.turn_time:.2f}s')
        
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