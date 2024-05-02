---
sidebar_position : 1
---


# Avanços da Sprint

Durante a segunda Sprint de desenvolvimento do projeto, o grupo Repipe avançou tanto na programação quanto na documentação, construindo sobre o progresso realizado na primeira Sprint. Foram elaborados:

## Programação:

Nessa 2° Sprint, o conteúdo de programação desenvolvido foi focado nos primeiros processos de comunicação com o robô, sendo a comunicação do robô com um computador por uma rede local [(LAN)](#1-o-que-é-uma-lan-rede-local-disponível-em--cloudflare-acesso-em-30-de-abril-de-2024), onde os comando são feitos por uma interface de comando de linha [(CLI)](#2-o-que-é-uma-cli-e-como-ela-impacta-seu-trabalho-de-programação-disponível-em-hostinger). Para garantir que o sistema funcione nos conformes, mesmo em um cenário inesperado, foi criado um mecanismo de emergência, para parar o robô, sem prejudicar sua intefridade ou das pessoas.

### Comunicação LAN (Local Area Network)

Uma rede LAN, ou Rede de Area Local, do inglês, trata-se um uma rede de computadores feitas em uma infraestrutura de curta área de atuação, sendo comumente usada em pequenas empresas, por exemplo.

Para o projeto foi optado esse padrão de rede, pois o robô não precisa ser operado por uma pessoa que esteja fora da planta de refino da cana de açúcar. Sendo assim, uma rede local (LAN), atende perfeitamente o projeto.

### Interface por Linha de Comando (CLI)

Como o nome propõe, uma CLI é uma interface, que um usuário pode interagir com o programa, que funciona por meio de linhas de comando. Ou seja, toda a interação que o usuário faz com o programa, ocorre com comandos digitados, em um terminal.

### Sistema de Segurança

A fim de previnir que o robô sofra danos em caso de pane do sistema, haverá um protocolo de segurança, acionado por um dispositivo externo, onde fará com que os processos do robô sejam encerrados.


## Documentação:

Na parte de documentação, o grup refinou a prova de conceito do projeto [(PoC)](#3-diferencial-da-prova-de-conceito-poc-para-projetos-em-ti-disponível-em-positivo-acesso-em-1-de-maio-de-2024), além da documentação detalhada sobre a metodologia de criação e excecução dos teste.

Para instruir a utilização do projeto, foi preparado um documento, explicando cada etapa de como executá-lo e operá-lo, bem como usar a CLI do projeto e como a comunicação da CLI com o robô funciona.

### Prova de Conceito (PoC)

Uma PoC, do inglês *Proof of Concept*, trata-se da evidência documentada de que umn projeto é viável. 

Nela, são feitas as pesquisas de mercado, viabilidade de tecnologias e custos de criação. Isso possibilita que um projeto tenha possíveis erros indentificados, antes memos do seu desenvolvimento, reduzindo custos e maximizando o alcance da plataforma.


### Como utilizar o projeto

Essa documentação é a chave principal para que o(a) operador(a) consiga utilizar a solução criada pelo grupo Repipe. Nela estarão os processos iniciais de quais ferramentas são utilizados no projeto, e como baixá-las.

Após as intalações das ferramentas necessárias, será explicado como executar o código fonte do projeto. 

Por fim, como operar o robô pela Interface por Linha de Comando (CLI).


## Referências 

###### [1] O que é uma LAN (rede local)?. Disponível em : [cloudflare](https://www.cloudflare.com/pt-br/learning/network-layer/what-is-a-lan/). Acesso em 30 de abril de 2024.

###### [2] O que é uma CLI e Como ela Impacta Seu Trabalho de Programação. Disponível em [hostinger](https://www.hostinger.com.br/tutoriais/o-que-e-cli)

###### [3] Diferencial da Prova de Conceito (PoC) para projetos em TI. Disponível em [positivo](https://www.meupositivo.com.br/panoramapositivo/prova-de-conceito/). Acesso em 1 de maio de 2024.