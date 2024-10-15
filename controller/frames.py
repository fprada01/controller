import tkinter as tk
from tkinter import messagebox

class MenuFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        
        label = tk.Label(self, text="Menu Inicial", font=("Arial", 20))
        label.pack(pady=20)

        # Botão Start
        button_arduino = tk.Button(self, text="Controle de Módulos Arduino", command=lambda: controller.show_frame(ModulesFrame))
        button_arduino.pack(pady=10)

        # Botão Desligar
        button_quit = tk.Button(self, text="Desligar", command=self.controller.turn_off_interface)
        button_quit.pack(pady=10)

        self.text_box = tk.Text(self, height=10, width=40)
        self.text_box.pack(pady=20)
        #self.box_connection = controller.old_connections["ArduinoController"]
        
    def update_box_text(self, dict):
        self.text_box.delete("1.0", tk.END)

        for key, text in dict.items():
            self.text_box.insert(tk.END, f'{key}: {text}\n')

        #self.after(2000, self.update_dynamic_text)


class ModulesFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        label = tk.Label(self, text="Você está na Janela 2", font=("Arial", 20))
        label.pack(pady=20)

        # Botão para voltar ao Menu
        button_back = tk.Button(self, text="Voltar para Janela 1", command=lambda: controller.show_frame(MenuFrame))
        button_back.pack(pady=10)




