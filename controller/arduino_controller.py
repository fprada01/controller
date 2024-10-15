import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller_node')

        self.publisher_connection = self.create_publisher(String, 'connection', 10) # 10 é o limite de mensagens na fila
        self.timer = self.create_timer(1, self.ConnectionMessage) # 1 é o tempo em segundos do timer

    def ConnectionMessage(self):
        msg = String()
        msg.data = 'ArduinoController Online'
        self.publisher_connection.publish(msg)
        
        self.get_logger().info('Publishing: "%s"' % msg.data)



def main(args=None):
    rclpy.init(args=args)  # Inicializa a comunicação ROS 2
    node = ArduinoController()  # Instancia o nó

    try:
        rclpy.spin(node)  
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # Destroi o nó quando encerrar
        rclpy.shutdown()

if __name__ == '__main__':
    main()