---
title: "Instrução de Execução"
sidebar_position: 3
---

# Instrução de Execução da aplicação
## Introdução
Esta seção da documentação detalha os passos necessários para executar a aplicação, conforme o estágio atual do desenvolvimento do projeto. Serão abordadas as etapas necessárias para executar tanto o backend quanto o frontend, considerando o desenvolvimento de uma aplicação completa de comunicação com o robô.

## Setup Inicial do Robô
Para iniciar a utilização do projeto, é essencial garantir que todo o setup do robô esteja funcionando. Para isso, utiliza-se o protocolo SSH, que permite acessar o terminal do Raspberry Pi acoplado ao Turtlebot.
Uma vez com o terminal acessado via SSH, é necessário executar os seguintes comandos:
- `ros2 launch turtlebot3_bringup robot.launch.py `-> Que é responsável por iniciar o serviço de comunicação do robô
- `cat .bashrc` -> Que irá mostrar às variáveis de ambiente do sistema. Lá é necessário observar o valor de "ROS_DOMAIN_ID"
Fora do terminal SSH, na máquina local, é necessário validar que a variável de ambiente "ROS_DOMAIN_ID" no computador seja a mesma utilizada pelo robô, para garantir que a comunicação entre ambos ocorra. Para isso, pode-se rodar o mesmo comando `cat .bashrc` e verificar o valor.

## Passo a passo para a Execução do Backend
### Comunicação da aplicação com a câmera
Para exibir a imagem da câmera acoplada no TurtleBot3 na página web, é primeiro necessário estabelecer comunicação com o WebSocket por meio do ROSbridge. Para isso, é necessário instalar o ROSbridge com o comando:
`sudo apt install ros-humble-rosbridge-suite`

Em seguida, deve-se garantir que na máquina onde o frontend está rodando, o WebSocket também esteja ativo, utilizando o comando:
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

Adicionalmente, o arquivo `sender.py`, descrito na seção *Metodologia - Comunicação Câmera/Cálculo da Latência*, também deve estar em execução. Para isso, no caminho relativo deste arquivo `src/controllers/comunicacao_camera`, utiliza-se o comando `python3 sender.py`. Este comando pode variar dependendo do sistema operacional. Importante salientar que o `sender.py` deve estar sendo executado na Raspberry Pi, visto que a câmera acoplada está conectada a ela.

## Passo a passo para a Execução do Frontend
### Aplicação Web
Para que o frontend seja exibido em uma página web pelo localhost, é necessário inicialmente navegar até o caminho relativo `src/frontend`, onde estão localizadas as dependências do frontend. Após isso, deve-se executar o comando `npm i` para instalar todas as dependências necessárias para rodar o projeto. Em seguida, executa-se o comando `npm start` para iniciar a aplicação.

## Conclusão
Com estes comandos, será possível ter uma aplicação rodando com um frontend conectado às suas dependências necessárias pelo backend, tornando-o funcional.

