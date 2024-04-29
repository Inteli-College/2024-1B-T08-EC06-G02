---
title: "Diagrama de sequência"
sidebar_position : 3
---

# O que é o diagrama de sequência?


Um diagrama de sequência é uma representação gráfica de como objetos interagem em uma determinada sequência de eventos ao longo do tempo. Ele descreve a troca de mensagens entre esses objetos dentro de um cenário específico, mostrando a ordem em que as mensagens são enviadas e recebidas. Os diagramas de sequência são amplamente utilizados na modelagem de sistemas orientados a objetos e são especialmente úteis para visualizar o comportamento dinâmico de um sistema.

## Diagrama 1

Esse diagrama é referernte ao Rubens Ferreira, o operador do robô. No caso dele, como será preciso que hajá uma visão em tempo real da câmera do robô, há um loop, que representa a visualização da visão robô.

Ao mesmo tempo que há a teleoperação do robô, o mesmo fará as capturas de imagens dos canos, que serão processadas pelo backend da aplicação, a fim de reduzir o uso de processamento do robô, tornando o processamento do dasdos escaláveis com computadores mais potentes.

Se o backend intentificar que um cano está sujo, ele envia um alerta visual e sonoro para a tela do operador, alertando o status do cano, ao mesmo tempo que esse status é armazenado no database.

<div align="center">
**Imagem 1** - Diagrama de sequência Rubens Ferreira 

![Diagrama de Sequência Rubens Ferreira](/img/diagrama_de_sequencia_rubens_ferreira.drawio.png)

Fonte: Elaboração grupo Repipe
</div>


## Diagrama 2

O segundo diagrama é referrente ao Marcos Batista, o técnico de limpeza, responsável por fazer a limpeza dos canos. Nesse cenário, ele busca saber quais são os canos que não foram limpos propriadamente. Então, a tela de visualização que ele usará é o mapeamento dos canos escaneados pelo robô.

Primeiramente, ao entrar na página, o sistema faŕa um requisição ao back, para buscar os status de todos os canos, de forma ordenada. Isso ocorre em um loop, que armazenas os dados na memória ram.

Em seguida, o backend ordena os dados lidos do database, para formar um mapa dos canos, a fim de tornar a visualização intuitiva.

Por fim, esse map dos dados são retornados ao frontend, permitindo que Marcos Batista visualize os canos que estão limpos ou sujos, permitindo-o economizar tempo na relimpeza.

<div align="center">
**Imagem 2** - Diagrama de sequência Rubens Ferreira

![Diagrama de Sequência Marcos Batista](/img/diagrama_de_sequencia_marcos_batista.drawio.png)

Fonte: Elaboração grupo Repipe
</div>