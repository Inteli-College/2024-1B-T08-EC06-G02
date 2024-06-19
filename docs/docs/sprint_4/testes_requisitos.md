---
sidebar_position: 3
title: "Teste de Requisitos"
---
# Teste de Requisitos 

Este documento é responsável por detalhar testes de funcionalidade para requisitos funcionais e testes de validação para requisitos não funcionais. 

Os requisitos funcionais estabelecidos anteriormente podem ser visualizados [aqui](https://inteli-college.github.io/2024-1B-T08-EC06-G02/sprint_1/programacao/requisitos_funcionais), já os requisitos não funcionais neste [link](https://inteli-college.github.io/2024-1B-T08-EC06-G02/sprint_1/programacao/requisitos_nao_funcionais). 

## Testes de funcionalidade: requisito funcional
### Contexto
Dos requisitos funcionais estabelecidos anteriormente, escolhemos o:

**RF03**: O robô deve ter um sistema de colisão. Neste requisito, o Operador Industrial estará pilotando o robô via controle remoto. No entanto, caso o robô se aproxime a uma determinada distância de um objeto ou ser vivo, o alerta será acionado na tela para notificar o usuário.

O RF escolhido se dá pela sua importância na teleoperação do robô e por mesclar fatores como controle pela interface gráfica, sistema de colisão e alerta acionado. 

### Roteiro de teste
Todo teste é seguido por uma **tarefa**. E toda tarefa é realizada por um **perfil/persona** que obtém um **resultado geral** e um resultado **por etapas**. 

Sendo assim, para garantir que o RF esteja de acordo com o estabelecido, criamos critérios para avaliar se o teste foi realizado com sucesso ou não. O RF escolhido, neste caso, é a tarefa. 

No geral, avaliamos 4 fatores: 

1. Tarefa a ser realizada
2. Perfil/persona que está realizando o teste;
3. Resultado geral: sucesso, conseguiu com dificuldade e não conseguiu;
4. Resultado por etapa: descrição detalhando o ocorrido.  

E a condução dos testes deve acontecer da seguinte forma:
1. Explicação do projeto para o testador, destacando o objetivo (tarefa) a ser realizada e a posição como persona.
2. Realizar o setup previamente do robô: rodar a interface, o BringUp do turtlebot, a conexão WebSocket e o envio dos dados da câmera.
3. Fornecer o notebook para teleoperar o robô e anotar as informações coletadas. 

### Relatório de execução
![Relatório de execução](../../static/img/sprint_4/testes_rf_registro.png)

Os testes foram executados por 4 pessoas, além do teste realizado pelo grupo. Os resultados podem ser visualizados [neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) do Google Sheets.

No geral, este foi o resultado obtido: 

| Tarefa | Usuários | Perfil/Persona | Resultado Geral |
|--------|---------|----------------|----------------|
|   1    | Grupo   | Operador Industrial | Sucesso |
|   1    | Antônio Ribeiro (3º Eng Comp)   | Operador Industrial | Sucesso | 
|   1    | Kaiane Souza (2º Eng Soft)   | Operador Industrial | Sucesso |
|   1    | André Leal (Téc Laboratório)   | Operador Industrial | Sucesso |
|   1    | Pablo Ruan (3º Eng Comp)   | Operador Industrial | Conseguiu com dificuldade |

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing) Google Sheets.

Nele também é possível observar se o problema relatado é um problema de funcionalidade, usabilidade ou compreensão. 

![Relatório de execução](../../static/img/sprint_4/testes_rf_ocorrencias.png)


### Conclusão
Como conclusão do teste de funcionalidade realizado, tivemos os seguintes feedbacks: 

1.	Controle fácil do robô.
2.	Movimentação intuitiva via teclado.
3.	Resposta rápida para movimentação.
4.	Sistema de alerta via pop-up eficaz.
5.	Resposta rápida do vídeo e imagem.

E como pontos de melhoria, tivemos: 

1.	**Posicionamento do Pop-up**: Mover o pop-up para o centro da tela para destacar mais eficazmente o sistema de colisão.
2.	**Informações adicionais no Pop-up**: Inserir no pop-up informações sobre a direção da colisão (frente, trás, direita, esquerda) para ajudar na orientação do usuário.
3.	**Indicação dos botões**: Fornecer indicações claras de quais botões devem ser apertados, possivelmente destacando-os visualmente ou incluindo legendas explicativas.
4.	**Feedback constante na interface**: Implementar um sistema que ofereça feedback constante sobre a direção em que o robô está se movendo, usando, por exemplo, indicações de cor nos botões ou setas.
5.	**Melhoria na disposição dos elementos da interface**: Aperfeiçoar a responsividade e a disposição geral dos elementos na interface para melhorar a usabilidade e a estética geral.


## Testes de validação: requisito não funcional
### Contexto

Dos requisitos não funcionais estabeledcidos anteriormente, escolhemos os:

1. **RNF01 - Autonomia do robô:** Diante da operação contínua do robô, durante o processo de limpeza, o mesmo deve ter autonomia necessária para verificar ao menos um reboiler, por vez. Sendo assim, é preciso ter uma autonomia de no mínimo 4 horas.

Esse RNF escolhido para compor os testes é muito importante pois precisamos entender a autonomia da bateria e como ela se comporta com todos os componentes funcionanado ao mesmo tempo (respbery, câmera, motores). Esse é um bom teste para podermos enteder e analisar se faz sentido colocar uma bateria mais potênte para alimetar o robô e poder ter mais autonomia no dia a dia na inspeção dos reboilers.

2. **RNF03 - Latência de comunicação** Visto que haverá uma teleoperação em tempo real do robô, o sitema deve ter uma latência de comunicação estável e baixa. Nesse caso, uma latência de no máximo 150ms, com picos de 30ms para mais ou menos.

O RNF escolhido se dá pela importância na teleoperação do robô e é de super importância esse dado ja que, a latência varia de forma com a distância e o ambiente onde esteja para se comunicar e controlar o robô.

3. **RNF05 - Interfácie interativa:** Com base no nivel de letramento digital dos possíveis usuários, interfácie do usuário precisa ser fácilmente compreendida ao primeiro contato e possuir os princípios das heurísticas de Nilsen. 

Esse RNF é de suma importância, pois permite compreender como o usuário interage com a página disponibilizada, facilitando o uso dos controles para comandar o robô. Através desses testes, poderemos ver se os comandos e teclas dispostos na tela estão fáceis para o usuário poder usar o robô com facilidade.
Partindo desse teste, poderemos entender e aplicar modificações em nossa página com o auxílio dos feedbacks fornecidos.

### Roteiro de teste
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
O "Teste de autonomia" em questão, será realizado com o grupo de desenvolvedores para poder visualizar como a bateria se comporta com todos os componentes em funcionamento e o quanto de tempo ela poderá suportar. Esse teste é importante pois precisamos dessa teste da autonomia da bateria justamente para entender como o robô se comporta a medida que a bateria vai descarregando e quanto tempo de duração é o que ela aguenta.

**1. Autonomia do Robô (RNF01):**
- Passo 1: Monte um ambiente controlado com obstáculos representativos e uma área de movimento para o robô.
- Passo 2: Crie um script que simule as atividades típicas do robô, como movimento, coleta de dados e comunicação.
- Passo 3: Pegue a bateria totalmente carregada 12.6V e acople no robô
- Passo 4: Lique um cronômetro e execute o script por um período e verifique se o consumo simulado de energia está dentro da capacidade da bateria do robô.
***OBS: Vale lembrar que o turtlebot por padrão começa a apitar quando chega as 11.0V como medida de segurança e quando começar a apitar nós paramos o cronômetro para ver quanto tempo deu***
- Passo 5: Depois que apitou, parâmos o cronômetro e vemos quanto tempo deu de funcionamento do robô.


### Relatório de execução
Segue imagem do relatório de execução de autonomia do robô com a bateria em 100% (12.6V):

![RNF_de_autonomia_do_robô](../../static/img/sprint_4/RNF_autonomia_robo.png)

Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit#gid=1538901939) do Google Sheets.

No geral, este foi o resultado obtido: 

| #       | Nome          | Perfil/Persona                   | Teste                                                                                   | Resultado Geral |
|---------|---------------|----------------------------------|-----------------------------------------------------------------------------------------|-----------------|
| PASSO 1 | GRUPO         | Aluno (Cecília 2º Eng Comp)      | Monte um ambiente controlado com obstáculos representativos e uma área de movimento para o robô. | Sucesso         |
| PASSO 2 | GRUPO         | Aluno (Cecília 2º Eng Comp)      | Crie um script que simule as atividades típicas do robô, como movimento, coleta de dados e comunicação. | Sucesso         |
| PASSO 3 | GRUPO         | Aluno (Cecília 2º Eng Comp)      | Pegue a bateria totalmente carregada 12.6V e acople no robô | Sucesso         |
| PASSO 4 | GRUPO         | Aluno (Cecília 2º Eng Comp)      | Lique um cronômetro e execute o script por um período e verifique se o consumo simulado de energia está dentro da capacidade da bateria do robô.| Sucesso  |
| PASSO 5 | GRUPO         | Aluno (Cecília 2º Eng Comp)      | Depois que apitou, parâmos o cronômetro e vemos quanto tempo deu de funcionamento do robô. | Sucesso         |

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

### Conclusão
Os testes mostraram que o robô não alcançou o objetivo inicial de quatro horas, mas continuou funcionando com eficiência por três horas e vinte minutos. A diferença de quarenta minutos indica a possibilidade de melhorar a gestão de energia ou otimizar os componentes do robô para aumentar a eficiência da bateria. Para continuar, é fundamental estudar como várias condições operacionais, como variações de carga e condições ambientais, afetam a duração da bateria. A análise cuidadosa das ocorrências e reações registradas ajudará a identificar pontos de melhoria técnica específicos, garantindo melhorias contínuas na performance e na confiabilidade do robô em futuras implementações.

## Tarefa para o teste de latência
O "Teste de latência" em questão, será realizado com o grupo de desenvolvedores para poder visualizar como a latência se comporta dependendo de onde a pessoa está com o computador eberto e fazendo o teste para poder controlar o robô. E claro, se a persona/tester conseguiu enviar os comando para o robô e ele conseguiu se movimentar.
Será avaliado nesse teste também o tempo de reinderização da imagem para que a pessoa que esteja controlando consiga ver o que a câmera está transmitindo com uma qualidade razoável tambem, para poder ver se há sujeiras ou algum tipo de defeito nos canos do reboiler. 

**2. Latência de Comunicação (RNF03):**
- Passo 1: Configuração e execução do código para começar a avaliação da latência e FPS.
- Passo 2: Estabeleça uma conexão de teleoperação entre o controlador e o robô.
- Passo 3: Meça a latência de comunicação durante a operação do robô com o auxílio da propriedado do front que mostra o FPS e a Latência.
- Passo 4: Deixe o robô no laboratório e rode o programa dentro do ateliê para visualizar a latência e o FPS. Depois, rode o programa da biblioteca para visualizar a latência e o FPS e por último, rode do refeitório para fazer o mesmo teste.

### Relatório de execução
Segue imagem do relatório de execução da latência e FPS do robô:

![RNF_de_latência_e_FPS_do_robô](../../static/img/sprint_4/RNF_latencia_robo.png)

Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit#gid=1538901939) do Google Sheets.

No geral, este foi o resultado obtido: 

| #       | Nome          | Perfil/Persona                   | Teste                                                                                   | Resultado Geral |
|---------|---------------|----------------------------------|-----------------------------------------------------------------------------------------|-----------------|
| PASSO 1 | GRUPO         | Aluno (José Alencar 2º Eng Comp)      | Configuração e execução do código para começar a avaliação da latência e FPS. | Sucesso         |
| PASSO 2 | GRUPO         | Aluno (José Alencar 2º Eng Comp)      | Estabeleça uma conexão de teleoperação entre o controlador e o robô. | Conseguiu com dificuldade         |
| PASSO 3 | GRUPO         | Aluno (José Alencar 2º Eng Comp)      | Meça a latência de comunicação durante a operação do robô com o auxílio da propriedado do front que mostra o FPS e a Latência. | Sucesso         |
| PASSO 4 | GRUPO         | Aluno (José Alencar 2º Eng Comp)      | Deixe o robô no laboratório e rode o programa dentro do ateliê para visualizar a latência e o FPS. Depois, rode o programa da biblioteca para visualizar a latência e o FPS e por último, rode do refeitório para fazer o mesmo teste.| Sucesso  |

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit#gid=1538901939)  Google Sheets.

### Conclusão
Os testes de latência e FPS do robô mostraram que ele manteve a latência dentro de limites aceitáveis em vários ambientes, apesar de alguns problemas iniciais com a conexão de teleoperação. Isso mostra a força do sistema. Os resultados indicam que a otimização contínua é necessária, principalmente na conexão, para aumentar a eficiência operacional em ambientes difíceis. Além disso, é aconselhável realizar uma análise minuciosa das ocorrências com o objetivo de orientar melhorias e ajustes técnicos no futuro.

## Tarefa para o teste de interface interativa
O "Teste de Interface Interativa" será realizado pelo grupo de desenvolvedores para avaliar a usabilidade da interface do usuário. Este teste é fundamental para garantir que a interface seja facilmente compreendida ao primeiro contato, seguindo os princípios das heurísticas de Nielsen. Durante o teste, os desenvolvedores observarão como os usuários interagem com a interface, identificando possíveis dificuldades e coletando feedback para melhorias. A realização deste teste é essencial para assegurar que os controles do robô sejam intuitivos e eficientes para todos os usuários, independentemente do seu nível de letramento digital.

### Heurísticas de Nielsen

As heurísticas de Nielsen são um conjunto de dez princípios gerais para o design de interface de usuário, propostos por Jakob Nielsen. Elas servem como diretrizes para avaliar a usabilidade de um sistema. Aqui estão resumidamente:

1. ***Visibilidade do status:*** Informar sempre o usuário sobre o que está acontecendo.
2. ***Compatibilidade com o mundo real:*** Usar linguagem e conceitos familiares ao usuário.
3. ***Controle e liberdade:*** Fornecer formas fáceis de desfazer e refazer ações.
4. ***Consistência e padrões:*** Manter uniformidade no design e seguir convenções.
5. ***Prevenção de erros:*** Evitar que erros aconteçam com um design cuidadoso.
6. ***Reconhecimento em vez de lembrança:*** Tornar opções visíveis para reduzir a carga de memória.
7. ***Flexibilidade e eficiência:*** Permitir atalhos e personalização para usuários experientes.
8. ***Design estético e minimalista:*** Evitar informações irrelevantes ou desnecessárias.
9. ***Ajuda no reconhecimento de erros:*** Mensagens de erro claras e úteis.
10. ***Ajuda e documentação:*** Fornecer documentação fácil de buscar e usar.

Essas heurísticas são amplamente utilizadas no design de interfaces para melhorar a usabilidade e a experiência do usuário e essas heurísticas vão nos ajudar com as métricas e avaliações a serem feitas internamente.

**3. Interface Interativa (RNF05):**
- Passo 1: Desenvolva protótipos interativos da interface do usuário com base nos requisitos de design.
- Passo 2: Apresente a interface do usuário simulada aos usuários e observe sua interação e feedback.
- Passo 3: Avalie se a interface do usuário é facilmente compreensível e atende aos princípios das heurísticas de Nielsen.

### Relatório de execução
Segue imagem do relatório de visualização da interface interativa para controlar o robô e visualizar a imagem da câmera:

![RNF_de_interface_interativa_do_robô](../../static/img/sprint_4/RNF_interface_robo.png)


Os testes foram executados pelo grupo de desenvolvedores. Os resultados podem ser visualizados[neste arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit#gid=1538901939) do Google Sheets.

No geral, este foi o resultado obtido: 

| #       | Nome          | Perfil/Persona                   | Teste                                                                                   | Resultado Geral |
|---------|---------------|----------------------------------|-----------------------------------------------------------------------------------------|-----------------|
| PASSO 1 | GRUPO         | Aluno (Murilo 2º Eng Comp)      | Desenvolva protótipos interativos da interface do usuário com base nos requisitos de design. | Sucesso         |
| PASSO 2 | GRUPO         | Aluno (Murilo 2º Eng Comp)      | Apresente a interface do usuário simulada aos usuários e observe sua interação e feedback. | Conseguiu com dificuldade         |
| PASSO 3 | GRUPO         | Aluno (Murilo 2º Eng Comp)      | Avalie se a interface do usuário é facilmente compreensível e atende aos princípios das heurísticas de Nielsen. | Não deu certo         |

Além disso, destacamos os principais problemas observados, classificando por nível de severidade e potenciais melhorias. Isto pode ser observado na segunda aba "ocorrências" do [mesmo arquivo](https://docs.google.com/spreadsheets/d/1FnYlsAU4UXWCYReaynA9F_aY9sgXys9OXnpAjph5xss/edit?usp=sharing)  Google Sheets.

### Conclusão
Os testes de interface interativa revelaram que, apesar do sucesso inicial no desenvolvimento dos protótipos, a interface enfrentou dificuldades em termos de usabilidade e não atendeu plenamente às heurísticas de Nielsen. Isso sublinha a necessidade de melhorias focadas em tornar a interface responsiva. Para futuros desenvolvimentos, recomenda-se revisar os elementos problemáticos com base no feedback dos usuários e realizar testes adicionais para garantir que a interface seja eficaz e satisfatória para o controle operacional do robô.

## Proposta de melhoria
### Melhorar Bateria
Durante os testes, constatou-se que a bateria atual de 1800mAh não sustenta a operação do robô pelo período desejado de quatro horas. Através de análises comparativas e testes de desempenho, identificou-se que uma bateria de 2200mAh é capaz de atender a necessidade de maior autonomia. A substituição proposta visa não apenas alcançar as quatro horas de operação contínua mas também aumentar a confiabilidade do robô durante operações prolongadas, otimizando a eficiência operacional.

### Melhoria na Interface do Usuário
A avaliação da interface atual revelou falhas no cumprimento das heurísticas de Nielsen, principalmente relacionadas à visibilidade do sistema e feedback ao usuário. Para melhorar a interação, propõe-se centralizar os pop-ups de alerta, garantindo que eles captem eficientemente a atenção do usuário, além de implementar um design responsivo para adaptar a interface a diferentes dispositivos. Estas mudanças são esperadas para facilitar o entendimento e controle do robô pelos usuários, reduzindo o tempo de aprendizado e aumentando a satisfação e eficiência nas operações.