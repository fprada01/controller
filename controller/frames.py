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

        self.divs_created = []

        create_label(self, "Controle de Módulos Arduino")
        create_button(self, "Menu", lambda: self.controller.show_frame(MenuFrame))

        self.show_and_update_modules_divs()

    def show_and_update_modules_divs(self):
        for div in self.divs_created:
            div.destroy()
        self.divs_created.clear()

        for module in self.controller.modules_list:
            self.divs_created.append(self.create_modules_div(module))


    def create_modules_div(self, module):   
        div = tk.Frame(self, width=200, height=300, borderwidth=5, relief="flat", bg="lightblue")
        div.pack_propagate(False)  # Impede que o Frame (div) se ajuste ao conteúdo
        div.pack(side='left', padx=10, pady=10)

        label = tk.Label(div, text=module[0])
        label.pack()

        message = tk.Message(div, text=f'Tipo: {module[1]}', width=80)
        message.pack()

        message = tk.Message(div, text=f'Status: {module[2]}', width=80)
        message.pack()

        text = '\n'.join(module[3:])
        message = tk.Message(div, text=text, width=80)
        message.pack()

        create_button(div, 'Config', lambda: self.controller.show_frame(EditModule))

        return div

class ErroModulesFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        create_label(self, "Erro ao Carregar Página")
        create_label(self, "Nó Arduino Controller não está disponível")

        create_button(self, "Menu", lambda: self.controller.show_frame(MenuFrame))

class AddModule(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller


class EditModule(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller




    def show_module(module):
        print('dfd')     




def create_button(parent, text_button, function=None):
        button = tk.Button(parent, text=text_button, command=function)
        button.pack(pady=10)

def create_label(parent, text_label):
    label = tk.Label(parent, text=text_label, font=("Arial", 20))
    label.pack(pady=20)


