# Instrução de Execução da aplicação
## Introdução
Esta seção da documentação detalha os passos necessários para executar a aplicação, conforme o estágio atual do desenvolvimento do projeto. Serão abordadas as etapas para operar tanto o backend quanto o frontend, considerando o desenvolvimento de uma aplicação completa de comunicação com o robô.

## Setup Inicial do Robô
Para iniciar a utilização do projeto, é crucial garantir que todo o setup do robô esteja operacional, utilizando o protocolo SSH para acessar o terminal do Raspberry Pi acoplado ao Turtlebot.

### Rede Wi-Fi e Conexão SSH
Antes de tudo, é necessário configurar na rede Wi-Fi. 

SSID: Inteli.Iot. Senha: @Intelix10T#

Após a conexão, os comandos no robô são executados remotamente:

```bash
ssh grupo2@grupo2.local
```

E preencha com a senha Repipe2

### Bringup do Robô 
Uma vez acessado o terminal via SSH, os seguintes comandos devem ser executados:
- `ros2 launch turtlebot3_bringup robot.launch.py `-> Responsável por iniciar o serviço de comunicação do robô.
- `cat .bashrc` -> Exibe as variáveis de ambiente do sistema. É essencial verificar o valor de "ROS_DOMAIN_ID".
Fora do terminal SSH, na máquina do usuário, deve-se validar que a variável de ambiente "ROS_DOMAIN_ID" seja a mesma utilizada pelo robô, para garantir a comunicação entre ambos. Isso é feito rodando o comando `cat .bashrc` na máquina do usuário para verificar o valor.

## Passo a passo para a Execução do Backend
### Comunicação da aplicação com a câmera
Para exibir a imagem da câmera acoplada ao TurtleBot3 na página web, é necessário primeiro estabelecer comunicação com o WebSocket através do ROSbridge. Isso é realizado com o comando:
`sudo apt install ros-humble-rosbridge-suite`

Em seguida, garante-se que o WebSocket esteja operando na máquina que executa o frontend, utilizando:
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`

Além disso, o arquivo `sender.py`, descrito na seção *Metodologia - Comunicação Câmera/Cálculo da Latência*, também deve estar em funcionamento. Para isso, no diretório `src/controllers/comunicacao_camera`, executa-se: `python3 sender.py`
Este comando pode variar dependendo do sistema operacional. É crucial que o `sender.py` seja executado na Raspberry Pi, pois a câmera acoplada está conectada a ela.

## Passo a passo para a Execução do Frontend
### Aplicação Web
Para exibir o frontend numa página web local, inicialmente navega-se até `src/frontend`, onde estão as dependências do frontend. Em seguida, instala-se as dependências necessárias com `npm i`, e inicia-se a aplicação com `npm run start`.

## Conclusão
Com estes comandos, é possível operar uma aplicação com um frontend conectado às dependências requeridas pelo backend, assegurando sua funcionalidade.