import tkinter as tk
from tkinter import ttk


# Função para o botão
def on_button_click():
    print("Botão clicado!")

# Função principal
def main():
    # Criar a janela principal
    root = tk.Tk()
    root.title("Exemplo com Botão, Texto e Imagem")
    root.geometry("400x300")  # Define o tamanho da janela

    # Criar um Frame (equivalente a uma div)
    frame = ttk.Frame(root, padding=10)
    frame.pack(fill='both', expand=True)

    # Criar um Label (Texto)
    label = ttk.Label(frame, text="Esse é um exemplo de interface no Tkinter!")
    label.pack(pady=10)

    # Carregar uma imagem




    # Criar um Botão
    button = ttk.Button(frame, text="Clique aqui", command=on_button_click)
    button.pack(pady=10)

    # Iniciar o loop do Tkinter
    root.mainloop()

main()