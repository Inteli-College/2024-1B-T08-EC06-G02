---
sidebar_position: 3
title: "Teste de Requisitos"
---

Este documento é responsável por detalhar testes de funcionalidade para requisitos funcionais e testes de validação para requisitos não funcionais. 

Os requisitos funcionais estabelecidos anteriormente podem ser visualizados [aqui](https://inteli-college.github.io/2024-1B-T08-EC06-G02/sprint_1/programacao/requisitos_funcionais), já os requisitos não funcionais neste [link](https://inteli-college.github.io/2024-1B-T08-EC06-G02/sprint_1/programacao/requisitos_nao_funcionais). 

# Testes de funcionalidade: requisito funcional
## Contexto
Dos requisitos funcionais estabelecidos anteriormente, escolhemos o:

**RF03**: O robô deve ter um sistema de colisão. Neste requisito, o Operador Industrial estará pilotando o robô via controle remoto. No entanto, caso o robô se aproxime a uma determinada distância de um objeto ou ser vivo, o alerta será acionado na tela para notificar o usuário.

O RF escolhido se dá pela sua importância na teleoperação do robô e por mesclar fatores como controle pela interface gráfica, sistema de colisão e alerta acionado. 

## Roteiro de teste
Todo teste é seguido por uma **tarefa**. E toda tarefa é realizada por um **perfil/persona** que obtém um **resultado geral** e um resultado **por etapas**. 

Sendo assim, para garantir que o RF esteja de acordo com o estabelecido, criamos critérios para avaliar se o teste foi realizado com sucesso ou não. 

No geral, avaliamos 4 fatores: 

1. Tarefa a ser realizada
2. Perfil/persona que está realizando o teste;
3. Resultado geral: sucesso, conseguiu com dificuldade e não conseguiu;
4. Resultado por etapa: descrição detalhando o ocorrido.  

E a condução dos testes deve acontecer da seguinte forma:
1. Explicação do projeto para o testador, destacando o objetivo (tarefa) a ser realizada e a posição como persona.
2. Realizar o setup previamente do robô: rodar a interface, o BringUp do turtlebot, a conexão WebSocket e o envio dos dados da câmera.
3. Fornecer o notebook para teleoperar o robô e anotar as informações coletadas. 

## Relatório de execução
![Relatório de execução](../../static/img/sprint_4/testes_rf_registro.png)

Os testes foram executados por 4 pessoas, além do teste realizado pelo grupo. Os resultados podem ser visualizados [neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) do Google Sheets.

No geral, este foi o resultado obtido: 

| Tarefa | Perfil/Persona | Resultado Geral |
|--------|----------------|----------------|
|   1    | Operador Industrial | Sucesso | 
|   1    | Operador Industrial | Conseguiu com dificuldade |
|   1    | Operador Industrial | Não conseguiu |
|   1    | Operador Industrial | Sucesso |
|   1    | Operador Industrial | Sucesso |

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

![Relatório de execução](../../static/img/sprint_4/testes_rf_ocorrencias.png)


## Conclusão
Completar

# Testes de validação: requisito não funcional
## Contexto

Dos requisitos não funcionais estabeledcidos anteriormente, escolhemos os:

1. **RNF01 - Autonomia do robô:** Diante da operação contínua do robô, durante o processo de limpeza, o mesmo deve ter autonomia necessária para verificar ao menos um reboiler, por vez. Sendo assim, é preciso ter uma autonomia de no mínimo 4 horas.

Esse RNF escolhido para compor os testes é muito importante pois precisamos entender a autonomia da bateria e como ela se comporta com todos os componentes funcionanado ao mesmo tempo (respbery, câmera, motores). Esse é um bom teste para podermos enteder e analisar se faz sentido colocar uma bateria mais potênte para alimetar o robô e poder ter mais autonomia no dia a dia na inspeção dos reboilers.

2. **RNF03 - Latência de comunicação** Visto que haverá uma teleoperação em tempo real do robô, o sitema deve ter uma latência de comunicação estável e baixa. Nesse caso, uma latência de no máximo 150ms, com picos de 30ms para mais ou menos.

O RNF escolhido se dá pela importância na teleoperação do robô e é de super importância esse dado ja que, a latência varia de forma com a distância e o ambiente onde esteja para se comunicar e controlar o robô.

3. **RNF05 - Interfácie interativa:** Com base no nivel de letramento digital dos possíveis usuários, interfácie do usuário precisa ser fácilmente compreendida ao primeiro contato e possuir os princípios das heurísticas de Nilsen. 

Esse RNF é de suma importância, pois permite compreender como o usuário interage com a página disponibilizada, facilitando o uso dos controles para comandar o robô. Através desses testes, poderemos ver se os comandos e teclas dispostos na tela estão fáceis para o usuário poder usar o robô com facilidade.
Partindo desse teste, poderemos entender e aplicar modificações em nossa página com o auxílio dos feedbacks fornecidos.

## Roteiro de teste
Todo teste é seguido por uma **tarefa**. E toda tarefa é realizada por um **perfil/persona** que obtém um **resultado geral** e um resultado **por etapas**. 

Sendo assim, para garantir que o RNF esteja de acordo com o estabelecido, criamos critérios para avaliar se o teste foi realizado com sucesso ou não. 

No geral, avaliamos 4 fatores: 

1. Tarefa a ser realizada
2. Perfil/persona que está realizando o teste;
3. Resultado geral: sucesso, conseguiu com dificuldade e não conseguiu;
4. Resultado por etapa: descrição detalhando o ocorrido.

E a condução dos testes deve acontecer da seguinte forma:
1. Explicação do projeto para o testador, destacando o objetivo (tarefa) a ser realizada e a posição como persona.
2. Realizar o setup previamente do robô: rodar a interface, o BringUp do turtlebot, a conexão WebSocket e o envio dos dados da câmera.
3. Fornecer o notebook para teleoperar o robô e anotar as informações coletadas. 

## Tarefa para o teste de autonomia do robô
O "Teste de autonomia" em questão, será realizado com o grupo de desenvolvedores para poder visualizar como a bateria se comprta com todos os componentes em funcionamento e o quanto de tempo ela poderá suportar. Esse teste é importante pois precisamos dessa teste da autonomia da bateria justamente para entender como o robô se comporta a medida que a bateria vai descarregando e quanto tempo de duração é o que ela aguenta.

***Depois, colocar as etapas das tarefas de 1 a 4 ou 3 para que ele (persona/tester) possa colocar o que deu certo e o que nao deu certo na atividade proposta***

## Relatório de execução
...img...

Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) do Google Sheets.

No geral, este foi o resultado obtido: 

| Tarefa | Perfil/Persona | Resultado Geral |
|--------|----------------|----------------|
|   1    | aluno | Sucesso | 

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

## Conclusão
...

## Tarefa para o teste de latência
O "Teste de latência" em questão, será realizado com o grupo de desenvolvedores para poder visualizar como a latência se comporta dependendo de onde a pessoa está com o computador eberto e fazendo o teste para poder controlar o robô. E claro, se a persona/tester conseguiu enviar os comando para o robô e ele conseguiu se movimentar.
Será avaliado nesse teste também o tempo de reinderização da imagem para que a pessoa que esteja controlando consiga ver o que a câmera está transmitindo com uma qualidade razoável tambem, para poder ver se há sujeiras ou algum tipo de defeito nos canos do reboiler. 

***Depois, colocar as etapas das tarefas de 1 a 4 ou 3 para que ele (persona/tester) possa colocar o que deu certo e o que nao deu certo na atividade proposta***

## Relatório de execução
...img...

Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) do Google Sheets.

No geral, este foi o resultado obtido: 

| Tarefa | Perfil/Persona | Resultado Geral |
|--------|----------------|----------------|
|   1    | aluno | Sucesso | 

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

## Conclusão
...

## Tarefa para o teste de interface interativa
O "Teste de Interface Interativa" será realizado pelo grupo de desenvolvedores para avaliar a usabilidade da interface do usuário. Este teste é fundamental para garantir que a interface seja facilmente compreendida ao primeiro contato, seguindo os princípios das heurísticas de Nielsen. Durante o teste, os desenvolvedores observarão como os usuários interagem com a interface, identificando possíveis dificuldades e coletando feedback para melhorias. A realização deste teste é essencial para assegurar que os controles do robô sejam intuitivos e eficientes para todos os usuários, independentemente do seu nível de letramento digital.

***Depois, colocar as etapas das tarefas de 1 a 4 ou 3 para que ele (persona/tester) possa colocar o que deu certo e o que nao deu certo na atividade proposta***

## Relatório de execução
...img...

Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) do Google Sheets.

No geral, este foi o resultado obtido: 

| Tarefa | Perfil/Persona | Resultado Geral |
|--------|----------------|----------------|
|   1    | aluno | Sucesso | 

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

## Conclusão
...