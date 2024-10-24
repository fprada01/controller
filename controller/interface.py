import rclpy # type: ignore
from rclpy.node import Node # type: ignore
from std_msgs.msg import String # type: ignore
import tkinter as tk
from .frames import (MenuFrame, ModulesFrame, AddModule, EditModule, ErroModulesFrame)
import yaml
import os
from ament_index_python.packages import get_package_share_directory # type: ignore
import time
from interfaces.srv import ModulesService
from interfaces.msg import StringList
import json

class Interface(Node):
    def __init__(self):
        # Inicializa o nó ROS 2
        Node.__init__(self, 'interface_node')

        self.start_time = time.time()
        self.get_logger().info(f'{self.start_time}')

        self.load_yaml_files()
        
        self.set_window_and_frames()

        self.subscription_connection = self.create_subscription(String, 'connection', self.connection_received, 10)
        self.subscription_connection

        self.show_frame(MenuFrame)

        self.client_modules_list = self.create_client(ModulesService, 'modules_list_service')
        self.root.after(1000, self.connect_service)

        self.root.after(100, self.ros_spin)

    def load_yaml_files(self):
        try:
            config_file_path = os.path.join(get_package_share_directory('controller'), 'config', 'databank.yaml')

            with open(config_file_path, 'r') as file:
                self.databank = yaml.safe_load(file)
            
            self.old_connections = self.databank.get('old_connections', {})
            for other_node in self.old_connections:
                self.old_connections[other_node] = 'Offline'

            self.modules_list = self.databank.get('modules_list', [])
            for module in self.modules_list:
                module[1] = 'Desligado'

            self.get_logger().info(f'{self.old_connections}')
            self.get_logger().info(f'{self.modules_list}')
            
        except Exception as e:
            self.get_logger().error(f"Failed to load yaml file: {e}")
            self.old_connections = {}

    def connect_service(self):
        if self.client_modules_list.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service is now available!')
        else:
            self.get_logger().info('Waiting for service to become available...')
            self.root.after(1000, self.connect_service)

    def set_window_and_frames(self):
        # Criação a janela principal
        self.root = tk.Tk()  
        self.root.title("Controle Arduino / Voris")
        self.root.geometry("600x400")

        # Contêiner para trocar os frames
        container = tk.Frame(self.root)
        container.pack(fill="both", expand=True)

        # Inicializar os frames
        self.frames = {}
        for F in (MenuFrame, EditModule, ModulesFrame, AddModule, ErroModulesFrame):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.frames[MenuFrame].update_box_text(self.old_connections)
    
    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.ros_spin)  # Reagendar o método para rodar continuamente

    def show_frame(self, frame_class):
        if frame_class == ModulesFrame:
            if self.client_modules_list.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Ready to change to Modules Frame')
                self.send_request_modules_service()
                self.frames[ModulesFrame].show_and_update_modules_divs()
                self.get_logger().info(f'{self.modules_list}')
            else:
                self.get_logger().info('Arduino Controller inst available')
                frame_class = ErroModulesFrame
        
        frame = self.frames[frame_class]
        frame.tkraise()


    def turn_off_interface(self):
        self.save_yaml_files()

        self.root.quit()
        
    def connection_received(self, msg):
        already_registered = False
        for other_node in self.old_connections:
            if (msg.data == (f'{other_node} Online')):
                self.get_logger().info('I heard: "%s"' % msg.data)
                self.old_connections[other_node] = 'Online'

                already_registered = True
            
        if already_registered == False:
            self.old_connections[(msg.data[:-7])] = 'Online'

        self.frames[MenuFrame].update_box_text(self.old_connections)
    
    def save_yaml_files(self):
        try:
            package_share_directory = get_package_share_directory('controller')
            config_file_path = os.path.join(package_share_directory, 'config', 'databank.yaml')
            
            data = {
                'old_connections': self.old_connections,
                'modules_list': self.modules_list
            }

            with open(config_file_path, 'w') as file:
                yaml.dump(data, file, default_flow_style=False)

            self.get_logger().info(f"Banco de dados salvo em {config_file_path}")

        except Exception as e:
            self.get_logger().error(f"Falha em salvar banco de dados: {e}")

    def send_request_modules_service(self):
        # Cria a solicitação com uma string
        request = ModulesService.Request()
        request.input_string = "Interface precisa da lista de módulos"
        
        # Envia a solicitação e espera a resposta
        future = self.client_modules_list.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.modules_list = json.loads(response.output_string)
            self.get_logger().info(f'{self.modules_list}')
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")




def main(args=None):
    rclpy.init(args=args)  # Inicializa a comunicação ROS 2
    node = Interface()  # Instancia o nó

    try:
        node.root.mainloop()  # Inicia o loop Tkinter
    except KeyboardInterrupt:
        node.root.quit()
        node.destroy_node() 
        rclpy.shutdown()
    finally:
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
