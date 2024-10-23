# Pacote Arduino Controler

Esse pacote ROS 2 fornece dois nós em python
- O nó arduino_controller é responsável pela comunicação principalmente entre arduino e interface, além de se comunicar com outros possíveis nós
- O nó interface é uma interface para visualização de dados e controle do arduino através do arduino_controller

## Comandos e possíveis soluções para erros

# Comandos básicos
1. Chamar ROS 2 instalado no computador em windows
call C:\<LOCAL_DO_ARQUIVO>\local_setup.bat

Exemplo:
call C:\Users\Voris\Desktop\ROS2\ros2-humble-20240523-windows-release-amd64\ros2-windows\local_setup.bat

2. Configurar Domínio (*n* - número do domínio)
set ROS_DOMAIN_ID=*n*

3. Configurar para operar somente no localhost
set ROS_LOCALHOST_ONLY=1

4. Navegue até o workspace
C:\Users\felipe.prada\Desktop\Felipe_Prada\SM3 - Stereo Ativo\workspace

4. Chamar o ROS 2 do workspace do pacote
call install/setup.bat

5. Contruir esse pacote em específico
colcon build --packages-select controller

6. Rodar esses nós
ros2 run controller interface
ros2 run controller arduino_controller

# Durante o desenvolvimento com ROS 2 podem acontecer inúmeros problemas, mas se encontra soluções e algumas são por comandos

1. Se precisar chamar a IDE do Visual Studio Community

Exemplo:
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64



# Erros de Compilação do ROS2
Alguns erros podem acontecer durante a instalação do ROS2

1. 1 package failed: mimick_vendor

New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Control\FileSystem" `
-Name "LongPathsEnabled" -Value 1 -PropertyType DWORD -Force

Explicação: https://github.com/ros2/ros2/issues/1368#issuecomment-1731286686

2. 1 package failed: rosidl_generator_c 
AttributeError: module 'em' has no attribute 'BUFFERED_OPT'

pip install --upgrade empy
pip install empy==3.3.4

