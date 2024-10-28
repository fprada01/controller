import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
from interfaces.srv import ModulesService
from interfaces.msg import StringList
import json

class ArduinoController(Node):
    def __init__(self):
        super().__init__('arduino_controller_node')
        self.get_logger().info("Arduino Controller Online")

        self.modules_list = [['Motor para Lente', 'Motor DC', 'Ligado', 'Controle: Manual', 'Velocidade: 60'], 
                             ['Motor para Espelho ','Motor de Passo', 'Desligado','Controle: Automatico', 'Velocidade: 20', 'Posicao: 15']]
                             
        self.srv_modules_list = self.create_service(ModulesService, 'modules_list_service', self.response_modules_list)
        self.srv_module_edited = self.create_service(ModulesService, 'module_edited', self.response_modules_list_edited_srv)

        self.sub_module_parameters = self.create_subscription(String, 'module_parameters', self.sub_changed_parameters, 10)
        self.sub_module_parameters

    def sub_changed_parameters(self, msg):
        self.get_logger().info(f'Recebido: {msg}')

    def response_modules_list(self, request, response):
        self.get_logger().info(f"Pedido recebido: {request.input_string}")
        response.output_string = json.dumps(self.modules_list)
        
        return response

    def response_modules_list_edited_srv(self, request, response):
        self.modules_list = json.loads(request.input_string)
        response.output_string = 'Modulo editado recebido'

        self.get_logger().info(f'{self.modules_list}')
        return response


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