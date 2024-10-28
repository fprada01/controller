import tkinter as tk
from tkinter import messagebox
from tkinter import ttk

class MenuFrame(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        
        create_label(self, "Menu Inicial")

        create_button(self, "Configuração de Módulos Arduino", lambda: controller.show_frame(ModulesFrame))

        create_button(self, "Controle Manual de Módulos", lambda: controller.show_frame(ManualControl))

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

        create_label(self, "Configuração de Módulos Arduino")
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

        create_button(div, 'Config', lambda: self.controller.show_frame(EditModule, module))

        return div

class ErroModulesList(tk.Frame):
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
        create_button(self, "Voltar", lambda: self.controller.show_frame(ModulesFrame))
        create_button(self, "Salvar", self.salve_edit_module)

        self.message_name = tk.Message(self, text='Nome: ', width=80)
        self.message_name.pack()
        self.entry_name = tk.Entry(self)
        self.entry_name.pack()

        self.message_type = tk.Message(self, text=None, width=120)
        self.message_type.pack()

        self.button_turn_on_module()

        self.button_control = tk.Button(self, text=None, command=self.click_button_control)
        self.button_control.pack_forget()


    def update_frame_editmodule(self, module):
        self.old_module_name = module[0]
        self.change_module = module.copy()

        self.entry_name.delete(0, tk.END)
        self.entry_name.insert(0, f'{self.change_module[0]}')

        self.message_type.config(text=f'Tipo: {self.change_module[1]}')

        self.button_turn_on.config(text=self.change_module[2])
        self.change_button_control(False)
        
        if self.change_module[1] == 'Motor de Passo':
            self.change_button_control(True)
        elif self.change_module[1] == 'Motor DC':
            self.change_button_control(True)


    def button_turn_on_module(self):
        def turn_on():
            if self.change_module[2] == 'Ligado':
                self.change_module[2] = 'Desligado'
                self.button_turn_on.config(text='Desligado')
            else:
                self.change_module[2] = 'Ligado'
                self.button_turn_on.config(text='Ligado')

        self.button_turn_on = tk.Button(self, text=None, command=turn_on)
        self.button_turn_on.pack(pady=10)

    def change_button_control(self, change=False):
        if change == True:
            self.button_control.pack(pady=10)
            self.button_control.config(text=f'{self.change_module[3]}')
        else:
            self.button_control.pack_forget()

    def click_button_control(self):
        if self.change_module[3] == 'Controle: Automatico':
            self.change_module[3] = 'Controle: Manual'
            self.button_control.config(text='Controle: Manual')

        elif self.change_module[3] == 'Controle: Manual':
            self.change_module[3] = 'Controle: Automatico'
            self.button_control.config(text='Controle: Automatico')

    def salve_edit_module(self):
        self.change_module[0] = self.entry_name.get()
        for i, module in enumerate(self.controller.modules_list):
            if self.controller.modules_list[i][0] == self.old_module_name:
                self.controller.modules_list[i] = self.change_module

                self.update_frame_editmodule(self.controller.modules_list[i])
        self.controller.send_edited_modules_list()
        
class ManualControl(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        create_label(self, "Controle Manual de Módulos")
        create_button(self, "Menu", lambda: self.controller.show_frame(MenuFrame))

        self.divs_created = []

    def show_and_update_control_divs(self):
        for div in self.divs_created:
            div.destroy()
        self.divs_created.clear()

        for module in self.controller.modules_list:
            if module[3] == 'Controle: Manual':
                self.divs_created.append(self.create_control_div(module))

    def create_control_div(self, module):
        div = tk.Frame(self, width=300, height=300, borderwidth=5, relief="flat", bg="lightblue")
        div.pack_propagate(False)  # Impede que o Frame (div) se ajuste ao conteúdo
        div.pack(side='left', padx=10, pady=10)

        div.grid_columnconfigure(0, minsize=70)
        div.grid_columnconfigure(1, minsize=30)
        div.grid_columnconfigure(2, minsize=70)
        div.grid_columnconfigure(3, minsize=30)
        div.grid_columnconfigure(4, minsize=30)

        div.grid_rowconfigure(0, minsize=50)
        div.grid_rowconfigure(1, minsize=50)
        div.grid_rowconfigure(2, minsize=50)
        div.grid_rowconfigure(3, minsize=50)
        div.grid_rowconfigure(4, minsize=50)


        label_name = tk.Label(div, text=module[0])
        label_name.grid(row=0, column=2)

        message_type = tk.Message(div, text=f'Tipo: {module[1]}', width=100)
        message_type.grid(row = 1, column=0, columnspan=2)

        message_status = tk.Message(div, text=f'Status: {module[2]}', width=100)
        message_status.grid(row = 1, column=3, columnspan=2)

        if module[1] == 'Motor DC':
            create_button_grid(div, row=3, column=0, text_button='-5', 
                               function= lambda:click_button_add(self.controller, module, 4, -5, message_velocity))
            create_button_grid(div, row=3, column=1, text_button='-1', 
                               function= lambda:click_button_add(self.controller, module, 4, -1, message_velocity))
            message_velocity = tk.Message(div, text=f'{module[4]}', width=100)
            message_velocity.grid(row = 3, column=2)
            create_button_grid(div, row=3, column=3, text_button='+1', 
                               function= lambda:click_button_add(self.controller, module, 4, 1, message_velocity))
            create_button_grid(div, row=3, column=4, text_button='+5', 
                               function= lambda:click_button_add(self.controller, module, 4, 5, message_velocity))

        elif module[1] == 'Motor de Passo':
            create_button_grid(div, row=3, column=0, text_button='-5',
                               function=lambda: click_button_add(self.controller, module, 4, -5, message_velocity))
            create_button_grid(div, row=3, column=1, text_button='-1',
                               function=lambda: click_button_add(self.controller, module, 4, -1, message_velocity))
            message_velocity = tk.Message(div, text=f'{module[4]}', width=100)
            message_velocity.grid(row = 3, column=2)
            create_button_grid(div, row=3, column=3, text_button='+1',
                               function=lambda: click_button_add(self.controller, module, 4, 1, message_velocity))
            create_button_grid(div, row=3, column=4, text_button='+5',
                               function=lambda: click_button_add(self.controller, module, 4, 5, message_velocity))

            create_button_grid(div, row=4, column=0, text_button='-5',
                               function=lambda: click_button_add(self.controller, module, 5, -5, message_position))
            create_button_grid(div, row=4, column=1, text_button='-1',
                               function=lambda: click_button_add(self.controller, module, 5, -1, message_position))
            message_position = tk.Message(div, text=f'{module[5]}', width=100)
            message_position.grid(row = 4, column=2)
            create_button_grid(div, row=4, column=3, text_button='+1',
                               function=lambda: click_button_add(self.controller, module, 5, 1, message_position))
            create_button_grid(div, row=4, column=4, text_button='+5',
                               function=lambda: click_button_add(self.controller, module, 5, 5, message_position))

        return div
    
def click_button_add(controller, module, attribute_index, add_number, widget):
    parts = module[attribute_index].split(': ')
    parts[1] = str(int(parts[1]) + add_number)
    module[attribute_index] = ": ".join(parts)

    widget.config(text=f'{module[attribute_index]}')

    controller.pub_changed_parameters(f'{module}')
    controller.send_edited_modules_list()

def create_button_grid(parent, row, column, text_button, function=None):
    button = tk.Button(parent, text=text_button, command=function)
    button.grid(row=row, column=column)

def create_button(parent, text_button, function=None):
    button = tk.Button(parent, text=text_button, command=function)
    button.pack(pady=10)

def create_label(parent, text_label):
    label = tk.Label(parent, text=text_label, font=("Arial", 20))
    label.pack(pady=20)


