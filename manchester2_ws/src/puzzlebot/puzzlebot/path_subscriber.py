import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')
        
        # Subscriber para recibir el path
        self.subscription = self.create_subscription(PoseArray, '/path', self.path_callback, 10)
        
        # Publisher para publicar los puntos recibidos
        self.received_publisher = self.create_publisher(String, '/recieved', 10)
        
        # Arreglo para almacenar las coordenadas
        self.coordinates = []
        self.last_path_hash = None
        
        self.get_logger().info('Path subscriber iniciado, esperando mensajes...')

    def path_callback(self, msg):
        current_coords = []
        for pose in msg.poses:
            current_coords.append([pose.position.x, pose.position.y])
        
        current_hash = hash(tuple(map(tuple, current_coords)))
        
        if current_hash != self.last_path_hash:
            self.coordinates = current_coords
            self.last_path_hash = current_hash
            
            # Mostrar coordenadas en el log
            coord_str = "\n".join([f"[{x:.2f}, {y:.2f}]" for x, y in self.coordinates])
            self.get_logger().info(f'Nuevo path recibido con {len(self.coordinates)} puntos:\n{coord_str}')
            
            # Publicar las coordenadas
            msg_str = String()
            msg_str.data = str(self.coordinates)
            self.received_publisher.publish(msg_str)
        else:
            self.get_logger().debug('Path recibido pero no ha cambiado, ignorando...')

def main(args=None):
    rclpy.init(args=args)
    path_subscriber = PathSubscriber()
    
    try:
        rclpy.spin(path_subscriber)
    except KeyboardInterrupt:
        pass
    finally:        
        path_subscriber.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()