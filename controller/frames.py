import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

class MenuFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        
        create_label(self, "Menu Inicial")

        create_button(self, "Controle de Módulos Arduino", lambda: controller.show_frame(ModulesFrame))

        create_button(self, "Desligar", self.controller.turn_off_interface)

        self.text_box =  tk.Text(self, height=10, width=40)
        self.text_box.pack(pady=20)
        
    def update_box_text(self, dict):
        self.text_box.delete("1.0", tk.END)

        for key, text in dict.items():
            self.text_box.insert(tk.END, f'{key}: {text}\n')

    
class ModulesFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        create_label(self, "Controle de Módulos Arduino")

        modules = ['Motor DC', "motor de passo"]
        for module in modules:
            create_modules_div(self)

        create_button(self, "Menu", lambda: controller.show_frame(MenuFrame))

class SetModule(tk.Frame):
    def _init_(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller


def create_button(parent, text_button, function=None):
        button = tk.Button(parent, text=text_button, command=function)
        button.pack(pady=10)

def create_label(parent, text_label):
    label = tk.Label(parent, text=text_label, font=("Arial", 20))
    label.pack(pady=20)

def create_text(parent):
    message = tk.Message(parent, text="Este é um exemplo de texto que pode ser maior", width=80)
    message.pack()

def create_modules_div(parent):
    div = tk.Frame(parent, width=100, height=200, borderwidth=5, relief="flat", bg="lightblue")
    div.pack_propagate(False)  # Impede que o Frame se ajuste ao conteúdo
    div.pack(side='left', padx=10, pady=10)

    create_text(div)
    create_button(div, 'Config')
