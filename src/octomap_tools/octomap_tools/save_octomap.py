import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
import os

class OctomapSaver(Node):
    def __init__(self):
        super().__init__('octomap_saver')
        self.subscription = self.create_subscription(
            Octomap,
            '/rtabmap/octomap_full',
            self.listener_callback,
            10)
        self.get_logger().info('Esperando mensagem em /rtabmap/octomap_full...')

    def listener_callback(self, msg):
        file_path = os.path.expanduser('~/mapas_rtabmap/mapa_casa_lisboa.ot.bt')
        
        with open(file_path, 'wb') as f:
            # Escrever header
            header = f"# Octomap OcTree file\n# size: {len(msg.data)}\n"
            f.write(header.encode('utf-8'))

            # Escrever data binária
            f.write(bytearray(msg.data))
        
        self.get_logger().info(f'Octomap guardado em: {file_path}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OctomapSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

