---
sidebar_position: 1
---

# Explicação da Latência no Projeto

## Introdução

Este documento tem como objetivo explicar a função de latência implementada no projeto de publicação de frames de vídeo usando ROS 2 (Robot Operating System). No contexto deste projeto, a latência refere-se ao tempo de atraso entre a captura de um frame pela câmera e a publicação deste frame como uma mensagem no tópico ROS. Entender e monitorar essa latência é crucial para aplicações em que o tempo real é um fator importante, como robótica, vigilância e sistemas de controle.

## Funcionamento da latência no projeto

A latência é um fator crítico em sistemas de visão computacional e robótica, onde decisões baseadas em vídeo precisam ser tomadas rapidamente. Monitorar a latência ajuda a:

**Avaliar o Desempenho:** Identificar gargalos no sistema que podem atrasar o processamento de vídeo.

**Melhorar a Qualidade do Serviço:** Ajustar parâmetros do sistema para reduzir a latência e melhorar a resposta em tempo real.

**Diagnosticar Problemas:** Detectar problemas com a câmera ou o sistema de captura que possam estar introduzindo atrasos inesperados.

# Dados e possiveis gráficos para a visualização da Latência:





# Conclusão
A função latencia implementada neste projeto é essencial para monitorar e otimizar a latência na captura e publicação de frames de vídeo. A medição contínua e a exibição da latência permitem identificar e corrigir problemas rapidamente, garantindo que o sistema funcione de maneira eficiente e responsiva.


# Explicação sobre a Bateria no projeto

## Introdução
Este documento tem como objetivo explicar a função da bateria e como as pessoas que estão operando o robô podem interfirir na vida útil da mesma. Lembrando sempre que estamos usando uma bateria de lítio então o mal uso pode ocorrer uma explosão (em pequena escala, claro, por conta do robô e a bateria serem pequenos). A seguir vamos falar mais sobre a bateria que estamos usando e o que devemos fazer para prolongar o período de vida útil dela tambem colocaremos um gráfico informativo sobre a vida útil dela, tomando e não tomando os devidos cuidados.

## Expecificações da bateria
- Modelo: LB-12
- Li-po 11.1V
- 1800 mAh
- 3 células com 3,7V cada
- Peso: 106g
- Tamano: 88mm x 35mm x 26mm

# Bateria LB-12 (cuidando adequadamente)

# Bateria LB-12 (Não cuidando adequadamente)

# Conclusão
