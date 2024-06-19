---
title: "Execução"
sidebar_position: 3
---

Este documento guiará por todas as etapas e processos necessários para que o projeto funcione, desde a compatibilidade de sistemas até a configuração do robô e do computador que o controlará.

:::note[Atenção]
A instalação pode levar algum tempo, devido à quantidade de arquivos necessários e seus tamanhos.
:::

## Compatibilidade de Sistemas

Este projeto é compatível apenas com o sistema operacional Ubuntu 22.04. Se o seu computador não estiver executando este sistema, acesse este link para instalar o Ubuntu: [Instalando Ubuntu](https://rmnicola.github.io/m6-ec-encontros/O01/ubuntu).

## Configurando o Turtlebot3

O primeiro passo para executar o projeto é configurar o Turtlebot3. Para isso, é necessário acessá-lo remotamente, instalar suas dependências e colocá-lo em modo de operação.

### Acessando o Turtlebot3

Para isso, será necessário utilizar uma conexão SSH com o robô.

Antes de tudo, atualize os pacotes do computador e instale o pacote SSH. Abra um terminal pressionando `Ctrl` + `Alt` + `T` e copie o seguinte comando:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install ssh
```

Agora, para acessar o robô, utilize o seguinte comando no mesmo terminal:

```bash
ssh grupo2@grupo2.local
```

### Instalando o ROS Humble 

Atualize os pacotes do Raspberry Pi:

```bash
sudo apt update && sudo apt upgrade -y
```

Se você receber uma mensagem informando que o pacote dpkg não foi atualizado com sucesso, execute o seguinte comando:

```bash
sudo apt-get --with-new-pkgs upgrade dpkg
```

### Instalando os pacotes do Turtlebot3

Para instalar os pacotes do Turtlebot3, basta executar o seguinte comando no terminal:

```bash
sudo apt install ros-humble-turtlebot3*
```

## Rodando diretamente pelo computador

### Instalando o ROS 2

Para instalar o ROS 2, siga estes passos:

1. Navegue até a pasta de Scripts do projeto:

```bash
cd src/Scripts
```

2. Em seguida, execute o arquivo `start_project.sh`. Este arquivo é responsável por configurar todo o ambiente de execução do projeto, o que pode levar alguns minutos para ser finalizado:

```bash
source start_project.sh
```

### Configurando o Turtlebot3

#### Acessando o robô

É importante ressaltar que, para configurar o Turtlebot, primeiro é preciso acessá-lo remotamente, por meio de uma chave SSH.

Para acessar o robô, execute o seguinte comando:

```bash
ssh grupo2@grupo2.local
```

#### Instalando o ROS Humble

1. Atualize os pacotes do computador:

```bash
sudo apt update && sudo apt upgrade -y
```

2. Adicione o repositório apt do Ubuntu. Para isso, execute o seguinte comando no terminal:

```bash
sudo apt-add-repository universe
```

3. Baixe a chave GPG, executando o seguinte comando:

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

4. Agora, é preciso adicionar a chave GPG do ROS à sua lista de repositórios. Basta executar o seguinte comando:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

5. Atualize os pacotes do sistema, executando o seguinte comando:

```bash
sudo apt update
```

6. Instale o pacote do ROS Humble Desktop:

```bash
sudo apt install ros-humble-desktop -y
```

7. Atualize os pacotes do sistema, para evitar conflitos:

```bash
sudo apt update && sudo apt upgrade -y
```

8. Adicione o caminho do inicializador do ROS Humble ao terminal do Turtlebot3, assim, toda vez que o terminal for aberto, os comandos do ROS Humble serão reconhecidos:

```bash
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
```

#### Instalando pacotes Turtlebot3

```bash
sudo apt install ros-humble-turtlebot3* -y
```

### Finalizando as Configurações

Para finalizar, é preciso definir o modelo do robô, o seu `ROS_DOMAIN_ID` e colocá-lo em modo de operação.

1. Para definir o modelo e o `ROS_DOMAIN_ID`, execute este comando:

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc && echo "export ROS_DOMAIN_ID=2" >> ~/.bashrc
```

2. Agora, basta colocar o robô em modo de operação:

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

## Executando o projeto

Para executar o projeto, abra um novo terminal pressionando `Ctrl` + `Alt` + `T` e siga estas etapas.

### Baixando o projeto

Antes de executar o que foi criado durante esta 2ª Sprint, é preciso baixar o repositório do grupo. Execute:

```bash
# Baixando o git
sudo apt install git -y

# Clonando o repositório
git clone https://github.com/Inteli-College/2024-1B-T08-EC06-G02/
```

### Rodando script de execução

Para executar o projeto, é necessário apenas executar o arquivo `start_project.sh` na pasta `Scripts`.

```bash
# Para navegar até a pasta `Scripts`
cd 2024-1B-T08-EC06-G02/src/Scripts

# Para executar o script de execução do projeto
source start_project.sh
```

## Referências 

[1] ROS - Robot Operating System. Diponível em [ROS Home](https://www.ros.org/). Acesso em 11 de maio de 2024.

[2] O que é SSH? | Protocolo Secure Shell (SSH). Disponível em [Cloudflare Learning](https://www.cloudflare.com/pt-br/learning/access-management/what-is-ssh/#:~:text=O%20protocolo%20Secure%20Shell%20(SSH)%20%C3%A9%20um%20m%C3%A9todo%20para%20enviar,e%20criptografar%20conex%C3%B5es%20entre%20dispositivos.). Acesso em 11 de maio de 2024.

[3] Instalação do Ubuntu. Disponível em [rmnicola](https://rmnicola.github.io/m6-ec-encontros/O01/ubuntu). Acesso em 4 de maio de 2024.

[4] Setup do Turtlebot3. Disponível em [rmnicola](https://rmnicola.github.io/m6-ec-encontros/setupturtle). Acesso em 4 de maio de 2024.
