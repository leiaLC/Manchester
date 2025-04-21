import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus
import time

# MPU6050 settings
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
GYRO_ZOUT_H = 0x47

bus = smbus.SMBus(1)

def mpu_init():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    return value if value < 32768 else value - 65536

def get_gyro_z():
    gz = read_raw_data(GYRO_ZOUT_H) / 131.0
    return gz

class YawPublisher(Node):
    def __init__(self):
        super().__init__('mpu6050_yaw_publisher')
        self.publisher_ = self.create_publisher(Float32, '/yaw', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)  # 100 Hz

        mpu_init()
        self.get_logger().info("Calibrando bias del giroscopio...")

        self.bias_z = 0.0
        samples = 100
        for _ in range(samples):
            self.bias_z += get_gyro_z()
            time.sleep(0.01)
        self.bias_z /= samples

        self.get_logger().info(f"Bias Z calibrado: {self.bias_z:.2f}")
        self.yaw = 0.0
        self.last_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gz = get_gyro_z()
        self.yaw += (gz - self.bias_z) * dt

        msg = Float32()
        msg.data = self.yaw
        self.publisher_.publish(msg)

        self.get_logger().info_throttle(1.0, f"Yaw angle: {self.yaw:.2f}°")

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
