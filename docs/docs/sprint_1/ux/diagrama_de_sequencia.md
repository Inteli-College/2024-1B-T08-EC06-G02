---
title: "Diagrama de Sequência"
sidebar_position: 4
---

# O que é um Diagrama de Sequência?

Um diagrama de sequência é uma representação gráfica que demonstra como objetos interagem em uma sequência de eventos ao longo do tempo. Ele descreve a troca de mensagens entre esses objetos em um cenário específico, mostrando a ordem em que as mensagens são enviadas e recebidas. Os diagramas de sequência são amplamente utilizados na modelagem de sistemas orientados a objetos, fornecendo uma visão dinâmica do comportamento do sistema.

## Diagrama 1: Rubens Ferreira

Este diagrama retrata as interações de Rubens Ferreira, o operador do robô. Como parte de suas atividades, Rubens precisa de uma visualização em tempo real da câmera do robô. Isso é representado por um loop, que mostra a contínua visualização da visão do robô.

Simultaneamente à teleoperação do robô, o mesmo está capturando imagens dos canos, que são processadas pelo backend da aplicação. Esse processamento é feito para reduzir a carga de trabalho do robô, tornando-o escalável para computadores mais potentes.

Se o backend identificar um cano sujo, ele envia um alerta visual e sonoro para a tela do operador, além de armazenar o status do cano no banco de dados.

<div align="center">
<h2 align="center"> Diagrama de Sequência - Rubens Ferreira </h2>

![Diagrama de Sequência Rubens Ferreira](/img/sprint_1/diagrama_de_sequencia_rubens_ferreira.drawio.png)

<h6 align="center"> Fonte: Elaboração Grupo Repipe </h6>
</div>

## Diagrama 2: Marcos Batista

O segundo diagrama representa as interações de Marcos Batista, o técnico de limpeza responsável pela limpeza dos canos. Neste cenário, Marcos precisa identificar quais canos não foram limpos corretamente. Para isso, ele utiliza a tela de visualização do mapeamento dos canos escaneados pelo robô.

Ao acessar a página, o sistema faz uma requisição ao backend para buscar os status de todos os canos, de forma ordenada, armazenando esses dados na memória RAM em um loop.

O backend, então, ordena esses dados do banco de dados para criar um mapa dos canos, facilitando a visualização por parte de Marcos.

Finalmente, esse mapa é retornado ao frontend, permitindo que Marcos identifique rapidamente quais canos estão limpos ou sujos, otimizando seu tempo de trabalho.

<div align="center">
<h2 align="center"> Diagrama de Sequência - Marcos Batista </h2>

![Diagrama de Sequência Marcos Batista](/img/sprint_1/diagrama_de_sequencia_marcos_batista.drawio.png)

<h6 align="center"> Fonte: Elaboração Grupo Repipe </h6>
</div>

## Referências 

[1] Diagrama de sequência. Disponível em : [IBM](https://www.ibm.com/docs/pt-br/rsm/7.5.0?topic=uml-sequence-diagrams#:~:text=Um%20diagrama%20de%20seq%C3%BC%C3%AAncia%20consiste,estruturas%20de%20controle%20entre%20objetos.)