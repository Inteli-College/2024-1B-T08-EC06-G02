---
sidebar_position: 2
title: "Requisitos não funcionais"
---

# Requisitos não funcionais (RNF)

## Entendendo os requisitos não funcionais

Primeiramente, vamos entender o que são requisitos não funcionais. Um requisito funcional se refere a uma especificação do sistema que descreve uma funcionalidade específica que o sistema deve cumprir. Por outro lado, um requisito não funcional se refere a uma especificação do sistema que descreve critérios de qualidade, restrições ou limitações que o sistema deve atender. Em outras palavras, enquanto um requisito funcional descreve o que o sistema deve fazer, um requisito não funcional descreve como o sistema deve fazer.


## RNF do sistema

Aqui está a tabela dos requisitos não funcionais:

| ID RNF | Nome do Requisito | Categoria | Descrição | Métricas | Justificativa |
|--------|-------------------|-----------|-----------|----------|---------------|
| RNF01  | Duração de Bateria e Uso Contínuo do Robô   |  | A duração do robô deve ter um tempo de operação mínima de 4 horas, sendo capaz de verificar todos os canos do reboiler. Ou seja, sua bateria deve ao menos conseguir suportar analisar 2000 canos em uma única jornada de trabalho. O robô será colocado em operação contínua e marcado o início da operação e o tempo em que o robô ficar sem bateria. | Tempo de operação contínua | O foco é garantir que o robô tenha autonomia suficiente para realizar uma jornada de trabalho completa, verificando os canos do reboiler sem interrupções.|
||
| RNF02  | Precisão na Detecção |  | O sistema deve assegurar uma precisão mínima na detecção de obstáculos e seres vivos, com erro tolerável menor ou igual a 5%. | Comparação com referência conhecida | O foco é na precisão do sistema na detecção de obstáculos e seres vivos. Esse requisito visa assegurar uma precisão específica na determinação da distância entre o robô e objetos ou seres vivos, garantindo a integridade do ambiente e a segurança dos mesmos, evitando colisões. |
||
| RNF03  | Latência de Comunicação |   | A latência de comunicação do sistema não deve exceder 100 milissegundos (ms) em qualquer momento durante a operação do robô. | Limite: 100 ms | Garantir uma comunicação ágil e eficiente para operação em tempo real do robô.|
||
| RNF04  | Tempo de Escaneamento dos Canos |   | O escaneamento dos canos deve ser feito em um tempo máximo de 5s, a fim de otimizar o tempo de verificação dos canos.| Tempo de escaneamento em segundos . Limite de  5 segundos| É preciso que o escaneamento seja rápido para que o reboiler seja verificado em um curto período de tempo, sem impactar a rotina de manutenção da empresa|
||
| RNF06  | Usabilidade da Interface de Controle do Robô | Usuabilidade | A interface de controle do robô deve ser projetada de forma intuitiva e amigável, permitindo que o Operador Industrial monitore o trajeto do robô e observe os locais para os quais ele foi direcionado com facilidade e eficiência. | A usabilidade da interface será avaliada utilizando métricas de facilidade de aprendizado, eficiência de uso, taxa de erro do usuário e satisfação do usuário. |
| RNF07  | Eficiência do Código de Movimentação | Eficiência | O código de movimentação precisa ser rápido às respostas do operador | Tempo de resposta do robô quando uma ação é executada. | O robô precisa realizar as movimentações com um tempo curto de resposta, para se evitarem possíveis erros de operação. |
||
| RNF09  | Precisão do sistema de alerta de colisão | Segurança | A precisão do sistemad e colisão garante a integridade do robô | Distância de deteção de obstáculos em metros, além da taxa de detecção na distância de 1 metro pelas ditância menores que 1 metro | Um sistema de alerta de colisão efetivo e com tempo de resposta rápido é essencial para garantir a segurança do robô, do Operador Industrial e de outras pessoas presentes no ambiente de operação. Notificações rápidas e precisas sobre possíveis colisões permitem que o Operador Industrial tome medidas imediatas para evitar acidentes.


<!-- A ser discutida com o grupo  | RNF05 | Partabilidade do sistema | Portabildiade | O sistema deve ser portátil cara garantir sua operação em diferentes plataformas, incluindo computadores -->

<!-- A ser discutida com o grupo | RNF08 |  -->


## Testes

| ID RNF | Objetivo do Teste | Métodos de Medição | Passos do Teste | Execução de Teste | Análise de Resultados | Interpretação dos dados |
|--------|-------------------|--------------------|-----------------|-------------------|-----------------------|-------------------------|
| RNF01  | Verificar se o robô opera continuamente por pelo menos 4h | Registro do tempo de operação | Colocar o robô em operação contínua. | Registrar o tempo em que o robô foi ligado até o momento em que a bateria se esgota.| Comparar o tempo registrado com o requisito de 4 horas. | Apenas será aceito se o tempo de operação contínua for igual ou superior a 4 horas.|
||
| RNF02  | Validar a precisão na detecção de obstáculos e seres vivos | Comparação com referência conhecida | Posicionar objetos de teste conhecidos a distâncias pré-determinadas. | Operar o robô para que detecte os objetos e registrar as distâncias medidas. | Comparar as medições do robô com as distâncias reais dos objetos. | A detecção deve ter um erro tolerável menor ou igual a 5%. |
||
| RNF03  | Verificar se a latência de comunicação não excede 100ms | Medição contínua da latência em tempo real | Iniciar o sistema de controle do robô e a infraestrutura de comunicação. | Monitorar a latência durante operações de teleoperação com o robô. | Analisar os dados para garantir que a latência não ultrapasse 100ms em nenhum momento. | A latência de comunicação do sistema não deve exceder 100ms durante a operação do robô.|
||
| RNF04  | Verificar o tempo gasto para escanear o cano | Escanear ocano e cronometrar o tempo da operação | Posicionar um cano entupimendo. | Inicinar a indentificação do estado do cano, cronometrando o tempo de execução da tarefa | 
||
| RNF06  | Avaliar a usabilidade da interface de controle do robô. | Testes de usabilidade com usuários reais. | Solicitar à um grupo de alinos, funcionários e professores do inteli para testarem a interface da solução. | Os testadores  realizarão as tarefas designadas usando a interface de controle do robô, enquanto os observadores registram métricas de desempenho e comportamento. | Analisar os dados coletados durante os testes de usabilidade para identificar áreas de melhoria na interface de controle do robô. | A interface de controle do robô será considerada satisfatória se os Operadores Industriais conseguirem completar as tarefas atribuídas de forma eficiente, com baixa taxa de erro e alta satisfação geral. Quaisquer problemas identificados durante os testes serão abordados e corrigidos para melhorar a usabilidade da interface.|
||
| RNF07  | Metrificar o tempo de resposta dos comandos ao robô | Teste de eficiência e stress. | Submeter uma séria de comandos ao robô em diferentes ambientes de operação. | Utilizar o controle para enviar comandos de movimentação ao robô e observar seu comportamento em tempo real. Testar o robô em cenários que representem desafios comuns encontrados durante sua operação, como obstáculos no caminho, mudanças repentinas no terreno e interferências externas. | Analisar o tempo de resposta do sistema aos comandos de movimentação, bem como sua capacidade de lidar com situações adversas sem comprometer a segurança do robô ou das pessoas ao seu redor. | O código de movimentação do robô será considerado satisfatório se demonstrar eficiência no processamento dos comandos de movimentação e robustez na operação em diferentes condições. Quaisquer problemas identificados durante os testes serão corrigidos para melhorar o desempenho e a confiabilidade do código.
||
| RNF09  | Avaliar a efetividade e o tempo de resposta do sistema de alerta de colisão. | Testes de simulação e testes de tempo. | Simular situações de possível colisão a 1 metro de distância e registrar a detecção e a resposta do sistema de alerta. | Posicionar o robô em direção a objetos ou seres vivos a 1 metro de distância e observar se o sistema de alerta é acionado corretamente. Cronometrar o tempo entre a detecção da ameaça e a exibição do alerta no visor. | Verificar se o sistema de alerta de colisão é acionado quando necessário e se o tempo de resposta está dentro dos limites especificados. | O sistema de alerta de colisão será considerado satisfatório se demonstrar efetividade na detecção de possíveis colisões a 1 metro de distância e um tempo de resposta rápido para notificar o Operador Industrial. Quaisquer problemas identificados durante os testes serão corrigidos para melhorar a efetividade e o tempo de resposta do sistema de alerta de colisão.