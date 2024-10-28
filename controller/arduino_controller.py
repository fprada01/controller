import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
from interfaces.srv import ModulesService
from interfaces.msg import StringList
import json

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller_node')

        self.modules_list = [['Motor para Lente', 'Motor DC', 'Ligado', 'Controle: Manual', 'Velocidade: 60'], 
                             ['Motor para Espelho ','Motor de Passo', 'Desligado','Controle: Automático', 'velocidade']]
                             
        self.publisher_connection = self.create_publisher(String, 'connection', 10)
        self.timer = self.create_timer(3, self.ConnectionMessage) 

        self.srv_modules_list = self.create_service(ModulesService, 'modules_list_service', self.response_modules_list)
        self.srv_module_edited = self.create_service(ModulesService, 'module_edited', self.response_module_edited_srv)


    def response_modules_list(self, request, response):
        self.get_logger().info(f"Pedido recebido: {request.input_string}")
        response.output_string = json.dumps(self.modules_list)
        
        return response

    def response_module_edited_srv(self, request, response):
        self.get_logger().info(f"Modulo editado recebido: {request.input_string}")
        self.modules_list = json.loads(request.input_string)
        self.get_logger().info(f'{self.modules_list}')
        response.output_string = 'Modulo editado recebido'

        return response

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