---
title: UX
sidebar_position: 4
---
# Documentação Final da Programação

Esta seção é dedicada à descrição detalhada do que foi desenvolvido na parte de backend e na comunicação com o hardware durante o desenvolvimento do projeto.

Inicialmente, desenvolveu-se uma arquitetura inicial que definiu como e quais tecnologias seriam utilizadas no desenvolvimento do projeto. Esse planejamento foi crucial para estabelecer uma base sólida e assegurar que todos os componentes se integrassem de maneira eficiente. Para mais detalhes sobre o desenvolvimento da arquitetura, acesse [essa página](./docs/sprint_1/programacao/proposta_inicial_arquitetura.md).

Posteriormente, avançou-se para o desenvolvimento com o hardware, onde foi utilizado o robô TurtleBot3, escolhido devido às necessidades específicas do projeto para a Atvos. Nesse contexto, configurou-se o mini-computador Raspberry Pi 4, e realizaram-se os primeiros testes de comunicação com os comandos do robô, incluindo teleoperação utilizando o ROS e a biblioteca do TurtleBot. Esses testes foram essenciais para garantir que o hardware pudesse executar as tarefas planejadas de maneira eficiente. Para mais detalhes sobre essa fase, acesse [a página referente](./docs/sprint_2/como_executar.md).

Para atender às necessidades do projeto de identificação de quais reboilers estão sujos, foi implementada uma câmera acoplada ao robô para coletar informações visuais. Além de desenvolver uma plataforma para receber as imagens, foi necessário criar um código para calcular a latência, ou seja, o tempo de demora entre a captura da imagem pela câmera e seu recebimento pela plataforma. Esse processo é crucial para garantir a precisão e a eficiência do sistema de monitoramento. O desenvolvimento dessa parte pode ser encontrado [nessa seção](./docs/sprint_3/programacao/camera_latencia.md).

Considerando que o robô será teleoperado, ou seja, controlado à distância, desenvolveu-se um recurso de segurança utilizando um sensor de distância a laser Lidar. Este sensor é essencial para evitar colisões e obter informações de distância ao redor do robô, aprimorando significativamente a navegação e a segurança operacional. A integração e utilização do Lidar durante o desenvolvimento do projeto são descritas em detalhe [nessa seção](./docs/sprint_3/programacao/lidar.md).

Após a instalação da câmera, adotou-se um modelo de visão computacional para analisar as imagens capturadas. Esse modelo é capaz de identificar a presença ou ausência de sujeira nos tubos, enviando um feedback à tela de comunicação para informar se o cano está obstruído ou não. Essa implementação melhora significativamente a capacidade do robô de realizar inspeções automatizadas. Mais informações sobre essa parte do projeto podem ser encontradas [nessa seção](./docs/sprint_4/visao_computacional.md).

O backend...

Foram realizados testes na solução final com base nos [requisitos](./docs/entrega_final/analise_de_requisitos/requisitos.md) previamente apurados. Esses testes são essenciais para que a Atvos entenda as limitações da solução desenvolvida e identifique áreas de melhoria. Esses testes e suas análises estão detalhados [aqui](./docs/entrega_final/analise_de_requisitos/requisitos.md).

Alem disso, foram desenvolvidas funcionalidades para suportar a integração de diferentes componentes e a comunicação entre eles. Além disso, sugeriu-se a implementação de gráficos para visualizar os dados coletados pela solução. Três tipos de gráficos foram propostos: mapa de calor a partir das imagens da câmera, monitoramento da bateria ao longo do tempo e medição da latência. Essa visualização de dados é crucial para monitorar e analisar o desempenho do sistema. Mais detalhes sobre essa sugestão podem ser encontrados [nessa seção](./docs/sprint_3/graficos_api.md).

Por fim, a versão final de execução do projeto, incluindo todas as integrações e testes realizados, pode ser encontrada [aqui](./docs/sprint_4/como_executar.md). Essa documentação final é fundamental para garantir que todas as partes interessadas possam entender e replicar o trabalho desenvolvido.