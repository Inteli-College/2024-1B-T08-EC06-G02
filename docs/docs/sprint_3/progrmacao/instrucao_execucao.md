# Instrução de Execução da aplicação
## Introdução
Nesta seção da documentação, explicaremos os passos necessários para rodar a aplicação, conforme o estágio atual do desenvolvimento do projeto. Abordaremos as etapas necessárias para executar tanto o backend quanto o frontend, considerando que estamos desenvolvendo uma aplicação completa de comunicação com o robô.

## Passo a paaso para a Execução do Backend
### Comunicação da aplicação com a câmera
Para que consiga ser exibido a imagem da câmera acoplada no TurtleBot3 na página web, precisa-se primeiramente fazer a comunicação com o websocket por meio do ROSbridge. Para isso será necessário instalar o ROSbridge pelo comando:<br/>
`sudo apt install ros-humble-rosbridge-suite`<br/>

Em seguida, é necessário que na máquina em que o frontend esteja rodando o WebSocket também esteja, para tanto o seguinte comando deve ser acionado:<br/>
`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`<br/>

Além disso, o arquivo `sender.py`, explicado na seção *Metodologia - Comunicação Câmera/Cálculo da Latência*, também deve estar rodando, para isso é necessário estar no caminho relativo deste arquivo `src/controllers/comunicacao_camera` coloque o sequinte comando: `python3 sender.py`, este comando pode mudar dependo do seu sistema operacional. No entanto, é importante destacar que o `sender.py`deve estar sendo rodado na Raspberry pi, visto que a câmera acoplada está conectada nela.

## Passo a passo para a Execução do Frontend
### Aplicação Web
Para que a o frontend seja exibido numa página web pelo localhost é necessário primeiramente migrar para o seguinte caminho relativo: `src/frontend`, onde estão localizadas as dependências do frontend. Dessa forma, execute o seguinte comando para iniciar a aplicação: `npm start`. 

## Conclusão
A partir destes comandos, será possível ter uma aplicação rodando com um forntend conectado às suas dependências necessárias pelo backend, tornando-o funcional. 

