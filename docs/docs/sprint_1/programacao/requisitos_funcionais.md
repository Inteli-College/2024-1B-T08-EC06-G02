---
sidebar_position : 1
title : "Requisitos Funcionais"
---

# Requisitos Funcionais 

Um requisito funcional é uma declaração que descreve o comportamento esperado de um sistema, ou seja, especifica o que o sistema deve fazer para atender às necessidades ou expectativas do usuário. Os requisitos funcionais representam as características ou funcionalidades que o usuário percebe ao interagir com o sistema. Ao contrário dos requisitos não funcionais, que detalham aspectos internos do sistema, como desempenho e segurança, os requisitos funcionais se concentram no que o sistema deve realizar. [1]

Esses requisitos normalmente consistem em duas partes: função e comportamento. A função descreve a ação que o sistema deve executar (por exemplo, "calcular o imposto sobre vendas"). O comportamento, por sua vez, especifica como essa função deve ser realizada (por exemplo, "o sistema deve calcular o imposto sobre vendas multiplicando o preço de compra pela alíquota do imposto"). [1]

Com essa descrição do que seriam esses tipos de requisitos para presente solução pensada foram elencados os seguintes requisitos funcionais (RF):

**RF01:** O sistema deve ser operado remotamente. Neste requisito, o Operador Industrial irá controlar o robô à distância para realizar o percurso ou comando especificado, eliminando a necessidade de estar fisicamente presente nos reboilers para a verificação dos tubos.

**RF02:** O robô deve verificar se há tubos sujos após a primeira limpeza. Neste requisito, o Operador Industrial, controlando o robô, irá acessar a câmera para visualizar o que está sendo detectado. O sistema, por meio dessa interface, informará se a tubulação está suja ou não.

**RF03:** O robô deve ter um sistema de colisão. Neste requisito, o Operador Industrial estará pilotando o robô via controle remoto. No entanto, caso o robô se aproxime a uma determinada distância de um objeto ou ser vivo, o alerta será acionado na tela para notificar o usuário.

**RF04:** O sistema deve ser escalável. Neste requisito, o analista de dados da Atvos poderá interagir com os dados captados por meio de uma API que se comunica com o dashboard proprietário deles.

**RF05:** O sistema deve ter uma interface para o controle do robô. Neste requisito, o Operador Industrial utilizará essa interface para monitorar o trajeto do robô e observar os locais para os quais ele foi direcionado.

**RF06:** O robô deve possuir um código em seu sistema que possibilite sua movimentação. Neste requisito, o Operador Industrial utilizará um controle para guiar o robô ao longo do caminho, utilizando o sistema projetado para receber esses comandos e direcionar o robô adequadamente.


## Referências
[1] Jain, Anushtha. 2022. ‘What Are Functional Requirements: Examples, Definition, Complete Guide’, Visure Solutions (Visure Solutions) [https://visuresolutions.com/pt/blog/functional-requirements/](https://visuresolutions.com/pt/blog/functional-requirements/) [accessed 20 April 2024]