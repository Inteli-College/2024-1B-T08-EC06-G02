# Metodologia

A metodologia para essa etapa do projeto primeiramente foi de estar em posse de um robô TurtleBot3 que estava montado e continha um micro cartão SD. Nesse sentido, foi extraído o micro cartão SD do robô e posto em um notebook. Após isso, para configuração inicial do sistema foi vista a documentação do Rodrigo Mangoni Nicola, disposta em: `https://rmnicola.github.io/m6-ec-encontros/setupturtle`.

Com isso em mente, foi baixado a imagem do raspberry pi imager, disposta em: [https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/) para ser inserido no micro cartão SD. A configuração do sistema na opção **Operating System** foi escolhido o Ubuntu Desktop 22.04.4 LTS (64 bit) e para o **Storage** foi selecionado o micro cartão SD. Essa etapa mais detalhada pode ser vista da Imagem 1 a Imagem 8.

<h2 align="center"> Imagem 1 - Tela incial do Raspberry Pi </h2>

![alt text](../../../static/img/sprint_2/tela_incial_raspberry.png)

<h6 align="center"> Fonte: Raspberry Pi Imager </h6>

Na Imagem 1 se tem as primeiras impressões da interface do Raspberry Pi Imager para começar a instalação do sistema.

<h2 align="center"> Imagem 2 - Seleção do Operating System </h2>

![alt text](../../../static/img/sprint_2/operating_system.png)
<h6 align="center"> Fonte: Material de Rodrigo Mangoni Nicola. Disponível em: (https://rmnicola.github.io/m6-ec-encontros/setupturtle) </h6>

Na imagem 2 se tem a opção para selecionar o sistema operacional desejado para trabalhar.

<h2 align="center"> Imagem 3 - Seleção de outro sistema </h2>

![alt text](../../../static/img/sprint_2/escolha_outra_opcao.png)
<h6 align="center"> Fonte: Material de Rodrigo Mangoni Nicola. Disponível em: (https://rmnicola.github.io/m6-ec-encontros/setupturtle) </h6>

<h2 align="center"> Imagem 4 - Seleção do Ubuntu </h2>

Na Imagem 3 se tem a opção para selecionar um sistema operacional diferente que não está diposto nessa primeira tela. Clique nele para ir em outras opçẽos.

![alt text](../../../static/img/sprint_2/selecao_ubuntu.png)
<h6 align="center"> Fonte: Material de Rodrigo Mangoni Nicola. Disponível em: (https://rmnicola.github.io/m6-ec-encontros/setupturtle) </h6>

<h2 align="center"> Imagem 5 - Seleção da versão do Ubuntu </h2>

![alt text](../../../static/img/sprint_2/versao_ubuntu.png)
<h6 align="center"> Fonte: Raspberry Pi Imager </h6>

Na Imagem 5 foi selecionada a opção do Ubuntu que foi utilizada na aplicação.

Após essas etapas é necessário seleconar o **Storage** que aparecerá a primeira opção que a entrada do micro cartão SD está lendo. 


<h2 align="center"> Imagem 6 - Seleção do Storage </h2>

![alt text](../../../static/img/sprint_2/storage.png)
<h6 align="center"> Fonte: Raspberry Pi Imager </h6>


<h2 align="center"> Imagem 7 - Seleção do micro cartão SD </h2>

![alt text](../../../static/img/sprint_2/escolha_cartao.png)
<h6 align="center"> Fonte: Material de Rodrigo Mangoni Nicola. Disponível em: (https://rmnicola.github.io/m6-ec-encontros/setupturtle) </h6>

Na Imagem 7 está sendo selecionado a unidade que se deseja colocar a imagem do sistema operacional escolhido.

<h2 align="center"> Imagem 8 - Seleção do micro cartão SD </h2>

![alt text](../../../static/img/sprint_2/write.png)
<h6 align="center"> Fonte: Material de Rodrigo Mangoni Nicola. Disponível em: (https://rmnicola.github.io/m6-ec-encontros/setupturtle) </h6>

Na Imagem 8 foi finalizada a tarefa de instalação do sistema operacional no micro cartão SD. Quando houver o término da instalção retire o micro cartão SD do notebook/computador e coloque novamente no TurtleBot3.

Após esse processo foi ligado o TurtleBot3 com o micro cartão SD inserido e a partir disso foi mostrado as telas para configuração do Ubuntu, foi configurado conforme o grupo visou necessário, ou seja, colocando nome de usuário, senha, conexão rede e etc.

Após isso, foi instalado o ROS baseado no material de  Rodrigo Mangoni Nicola disponível em [https://rmnicola.github.io/m6-ec-encontros/E01/ros](https://rmnicola.github.io/m6-ec-encontros/E01/ros). Assim, foi usado os seguintes comandos para instalação no terminal:

**1-** `sudo apt-add-repository universe` 

**2-** `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg` 

**3-** `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null` 

**4-** `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null` 

**5-** `sudo apt update` 

**6-** `sudo apt install ros-humble-desktop` 

**7-** `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc` 
