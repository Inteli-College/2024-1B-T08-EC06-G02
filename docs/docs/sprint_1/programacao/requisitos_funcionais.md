# Requisitos funcionais 
&emsp; Um requisito funcional é uma declaração que descreve o comportamento esperado de um sistema, ou seja, especifica o que o sistema deve fazer para atender às necessidades ou expectativas do usuário. Os requisitos funcionais representam as características ou funcionalidades que o usuário percebe ao interagir com o sistema. Ao contrário dos requisitos não funcionais, que detalham aspectos internos do sistema, como desempenho e segurança, os requisitos funcionais se concentram no que o sistema deve realizar. [1]

&emsp; Esses requisitos normalmente consistem em duas partes: função e comportamento. A função descreve a ação que o sistema deve executar (por exemplo, "calcular o imposto sobre vendas"). O comportamento, por sua vez, especifica como essa função deve ser realizada (por exemplo, "o sistema deve calcular o imposto sobre vendas multiplicando o preço de compra pela alíquota do imposto"). [1]

&emsp; Com essa descrição do que seriam esses tipos de requisitos para presente solução pensada foram elencados os seguintes requisitos funcionais:

**RF01:**  O robô deve verificar se há tubos sujos após a primeira limpeza. Nesse requisito, o usuário controlando o robô irá acessar a câmera para visualizar o que está sendo detectado e o sistema irá informar através dessa interface se está sujo ou não a tubulação.

**RF02:** O sistema deve indicar quais tubos estão sujos. Nesse requisito, o usuário irá ver a priori verificará todos os tubos através da câmera e após esse processo, ele verá na interface do sistema quais tubos estão sujos.

**RF03:** O sistema emite um sinal de alerta no visor em caso de uma possível colisão a 1 metro de distância. Nesse requisito, o usuário estará pilotando o robô via controle remoto, mas no caso do robô chegar próximo a um objeto ou algum ser vivo a 1 metro de distância acionará na tela um alerta para o usuário.

**RF04:** O sistema deve medir a temperatura do reboiler. Nesse requisito, o usuário irá até o local dos reboilers pelo robô, com isso o mesmo vai verificar a temperatura do local e examinar se o robô pode prosseguir para verificação dos reboilers ou não, pois dependendo da temperatura traria malefícios a estrutura e componentes do robô. 

**RF05:**O sistema deve ser multiplataforma. Nesse requisito, o usuário pode interagir com aplicação no caso para poder acessar a API e pegar os dados para ser posto na própria plataforma da Ativos.



# Bibliografia:
[1] Jain, Anushtha. 2022. ‘What Are Functional Requirements: Examples, Definition, Complete Guide’, Visure Solutions (Visure Solutions) <https://visuresolutions.com/pt/blog/functional-requirements/> [accessed 20 April 2024]

