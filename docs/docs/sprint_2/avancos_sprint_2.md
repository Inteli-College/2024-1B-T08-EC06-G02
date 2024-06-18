---
sidebar_position: 2
---

# Avanços da Sprint

Durante a segunda Sprint de desenvolvimento do projeto, o grupo Repipe avançou tanto na programação quanto na documentação, construindo sobre o progresso realizado na primeira Sprint. Foram elaborados:

## Programação

Na 2ª Sprint, o conteúdo de programação desenvolvido foi focado nos primeiros processos de comunicação com o robô, sendo a comunicação do robô com um computador por uma rede local [(LAN)](#1-o-que-é-uma-lan-rede-local-disponível-em--cloudflare-acesso-em-30-de-abril-de-2024), onde os comandos são feitos por uma interface de linha de comando [(CLI)](#2-o-que-é-uma-cli-e-como-ela-impacta-seu-trabalho-de-programação-disponível-em-hostinger). Para garantir que o sistema funcione adequadamente, mesmo em um cenário inesperado, foi criado um mecanismo de emergência para parar o robô sem prejudicar sua integridade ou a das pessoas.

### Comunicação LAN (Local Area Network)

Uma rede LAN, ou Rede de Área Local, trata-se de uma rede de computadores feita em uma infraestrutura de curta distância, sendo comumente usada em pequenas empresas, por exemplo.

Para o projeto, foi optado por esse padrão de rede, pois o robô não precisa ser operado por uma pessoa que esteja fora da planta de refino da cana-de-açúcar. Sendo assim, uma rede local (LAN) atende perfeitamente ao projeto.

### Interface por Linha de Comando (CLI)

Como o nome sugere, uma CLI é uma interface na qual um usuário pode interagir com o programa por meio de linhas de comando. Ou seja, toda a interação que o usuário faz com o programa ocorre com comandos digitados em um terminal.

### Sistema de Segurança

Com o objetivo de prevenir que o robô sofra danos em caso de falha do sistema, será implementado um protocolo de segurança acionado por um dispositivo externo, que encerrará os processos do robô.

## Documentação

Na parte de documentação, o grupo refinou a prova de conceito do projeto [(PoC)](#3-diferencial-da-prova-de-conceito-poc-para-projetos-em-ti-disponível-em-positivo-acesso-em-1-de-maio-de-2024), além de documentar detalhadamente a metodologia de criação e execução dos testes.

Para instruir a utilização do projeto, foi preparado um documento explicando cada etapa de como executá-lo e operá-lo, bem como utilizar a CLI do projeto e como ocorre a comunicação entre a CLI e o robô.

### Prova de Conceito (PoC)

Uma PoC, do inglês *Proof of Concept*, é a evidência documentada de que um projeto é viável.

Nela, são realizadas pesquisas de mercado, análises de viabilidade de tecnologias e custos de implementação. Isso permite identificar possíveis erros antes mesmo do desenvolvimento, reduzindo custos e maximizando o alcance da plataforma.

### Como utilizar o projeto

Esta documentação é fundamental para que o operador possa utilizar a solução criada pelo grupo Repipe. Nela estão descritos os processos iniciais, quais ferramentas são utilizadas no projeto e como instalá-las.

Após a instalação das ferramentas necessárias, é explicado como executar o código-fonte do projeto.

Por fim, é detalhado como operar o robô pela Interface por Linha de Comando (CLI).

## Referências

[1] O que é uma LAN (rede local)?. Disponível em: [cloudflare](https://www.cloudflare.com/pt-br/learning/network-layer/what-is-a-lan/). Acesso em 30 de abril de 2024.

[2] O que é uma CLI e Como ela Impacta Seu Trabalho de Programação. Disponível em: [hostinger](https://www.hostinger.com.br/tutoriais/o-que-e-cli). Acesso em 30 de abril de 2024.

[3] Diferencial da Prova de Conceito (PoC) para projetos em TI. Disponível em: [positivo](https://www.meupositivo.com.br/panoramapositivo/prova-de-conceito/). Acesso em 1 de maio de 2024.
