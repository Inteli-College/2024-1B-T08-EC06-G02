# Metodologia Sprint 3

Este documento detalha as atualizações da Sprint 3 do grupo Repipe. O foco principal nas últimas duas semanas foi também a integração e utilização do sensor de distância a laser 360 Laser Distance Sensor (LDS-02) para que, quando uma pessoa estivesse controlando o robô, o sensor auxiliaria a evitar colisões. No decorrer do documento, discutiremos a instalação e configuração do LDS-01, a coleta e análise dos dados de varredura do ambiente ao redor do robô e a implementação da interface de comunicação necessária para o funcionamento do sensor.

## Setup do LDS-01 (LIDAR)

Como o Lidar é um scanner a laser 2D capaz de realizar varreduras de 360 graus, coletando um conjunto de dados ao redor do robô para ser utilizado em SLAM (Simultaneous Localization and Mapping) e Navegação e que tinhamos temos ele instalado no TurtleBot Burguer, optamos por utiliza-lo e fazer a comunicação para que ele enviasse as mensagens de distância dos objetos que estavam ao seu redor.

E para conseguirmos configurar o lidar, tivemos que seguir alguns passos que podem ser vistos aqui abaixo:
```cmd
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
```
Depois, clonamos o repositório:
```cmd
git clone -b ros2-devel https://github.com/ROBOTIS-GIT/ld08_driver.git
```
Voltamos para a pasta do workspace com:
```cmd
cd ~/turtlebot3_ws/
```
E por fim, copilamos o pacote com:
```cmd
colcon build --symlink-install
```
Após compilar o pacote, configuramos o nosso bashrc para dar source no arquivo de instalação do pacote com:
```cmd
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
```
Também precisamos especificar o modelo do LIDAR em uma variável de ambiente. Rode:
```cmd
echo 'export LDS_MODEL=LDS-02' >> ~/.bashrc
```
Lembre-se que para poder utilizar essas mudanças sem reiniciar o terminal ainda precisa rodar:
```cmd
source ~/.bashrc
```

## Código do sensor LIDAR: