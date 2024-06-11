---
sidebar_position: 5
title: "Como Executar"
---

# Instrução de Execução da aplicação
## Introdução
Nesta seção da documentação, explicaremos os passos necessários para rodar a aplicação, conforme o estágio atual do desenvolvimento do projeto. Abordaremos as etapas necessárias para executar tanto o backend quanto o frontend, considerando que estamos desenvolvendo uma aplicação completa de comunicação com o robô.
## Setup Inicial do Robô

Para começar a utilização do projeto é necessário primeiro garantir que todo o setup do robô está funcionando, para isso será utilizado o protocolo SSH que permite utilizar o terminal do raspberry pi acoplado ao turtlebot.

### Rede Wi-Fi e Conexão SSH
Antes de tudo, é necessário estar configurado na rede Wi-Fi. 

SSID: Inteli.Iot. Senha: @Intelix10T#

Depois, precisamos executar alguns comandos no robô remotamente. Para isso, devemos executar os comandos: 

```bash
ssh grupo2@grupo2.local
```

E preencha com a senha Repipe2

### Bringup do Robô 

Uma vez como terminal acessado via ssh, é preciso rodar os seguintes comandos: 
- `ros2 launch turtlebot3_bringup robot.launch.py `-> Que é responsável por iniciar o serviço de comunicação do robô
- `cat .bashrc` -> Que irá mostrar às variáveis de ambiente do sistema. Lá é necessário observar o valor de "ROS_DOMAIN_ID"
Fora do terminal ssh, na sua própria máquina, é necessário validar que a variável de ambiente "ROS_DOMAIN_ID" no seu computador é a mesma utilizada pelo seu robô, para garantir que a comunicação dos 2 acontecerá. Para isso pode rodar o mesmo comando `cat .bashrc` e verificar o valor.

## Passo a passo para a Execução do Backend
### Comunicação da aplicação com a câmera
Para que consiga ser exibido a imagem da câmera acoplada no TurtleBot3 na página web, precisa-se primeiramente fazer a comunicação com o websocket por meio do ROSbridge. Para isso será necessário instalar o ROSbridge pelo comando:<br/>
`sudo apt install ros-humble-rosbridge-suite`<br/>

Em seguida, é necessário que na máquina em que o frontend esteja rodando o WebSocket também esteja, para tanto o seguinte comando deve ser acionado:<br/>
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`<br/>

Além disso, o arquivo `sender.py`, explicado na seção *Metodologia - Comunicação Câmera/Cálculo da Latência*, também deve estar rodando, para isso é necessário estar no caminho relativo deste arquivo `src/controllers/comunicacao_camera` coloque o sequinte comando: `python3 sender.py`, este comando pode mudar dependo do seu sistema operacional. No entanto, é importante destacar que o `sender.py`deve estar sendo rodado na Raspberry pi, visto que a câmera acoplada está conectada nela.

## Passo a passo para a Execução do Frontend
### Aplicação Web
Para que a o frontend seja exibido numa página web pelo localhost é necessário primeiramente migrar para o seguinte caminho relativo: `src/frontend`, onde estão localizadas as dependências do frontend. Após isso deve-se executar o comando `npm i` para instalar todas as dependências necessárias para rodar o projeto. Após isso, execute o seguinte comando para iniciar a aplicação: `npm run start`. 

## Conclusão
A partir destes comandos, será possível ter uma aplicação rodando com um forntend conectado às suas dependências necessárias pelo backend, tornando-o funcional. 

