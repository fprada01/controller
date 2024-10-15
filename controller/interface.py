import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import messagebox
from .frames import (MenuFrame, ModulesFrame)
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import time

class Interface(Node):
    def __init__(self):
        # Inicializa o nó ROS 2
        Node.__init__(self, 'interface_node')

        self.start_time = time.time()
        self.get_logger().info(f'{self.start_time}')
        
        self.set_window_and_frames()

        self.start_connections()
        self.subscription_connection = self.create_subscription(String, 'connection', self.connection_received, 10)
        self.subscription_connection

        self.show_frame(MenuFrame)

        self.root.after(100, self.ros_spin)
    
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
        for F in (MenuFrame, ModulesFrame):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")
    
    def ros_spin(self):
        '''Executa o ROS 2 spin'''
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.ros_spin)  # Reagendar o método para rodar continuamente

    def show_frame(self, frame_class):
        '''Exibe o frame passado como argumento'''
        frame = self.frames[frame_class]
        frame.tkraise()

    def turn_off_interface(self):
        self.save_connections()

        self.root.quit()

    def start_connections(self):
        try:
            config_file_path = os.path.join(get_package_share_directory('controller'), 'config', 'old_connections.yaml')

            with open(config_file_path, 'r') as file:
                self.old_connections = yaml.safe_load(file)
            for other_node in self.old_connections:
                self.old_connections[other_node] = 'Offline'

        except Exception as e:
            self.get_logger().error(f"Failed to load connections file: {e}")
            self.old_connections = {}
        
        self.frames[MenuFrame].update_box_text(self.old_connections)
        
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
    
    def save_connections(self):
        try:
            package_share_directory = get_package_share_directory('controller')
            config_file_path = os.path.join(package_share_directory, 'config', 'old_connections.yaml')
            
            with open(config_file_path, 'w') as file:
                yaml.dump(self.old_connections, file, default_flow_style=False)
            self.get_logger().info(f"Connections saved to {config_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save connections file: {e}")



def main(args=None):
    rclpy.init(args=args)  # Inicializa a comunicação ROS 2
    node = Interface()  # Instancia o nó

    try:
        node.root.mainloop()  # Inicia o loop Tkinter
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()
