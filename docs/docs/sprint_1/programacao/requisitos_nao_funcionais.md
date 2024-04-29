---
sidebar_position: 2
title: "Requisitos não funcionais"
---

# Requisitos não funcionais (RNF)

## Entendendo os requisitos não funcionais

Primeiramente, vamos entender o que são requisitos não funcionais. Um requisito funcional se refere a uma especificação do sistema que descreve uma funcionalidade específica que o sistema deve cumprir. Por outro lado, um requisito não funcional se refere a uma especificação do sistema que descreve critérios de qualidade, restrições ou limitações que o sistema deve atender. Em outras palavras, enquanto um requisito funcional descreve o que o sistema deve fazer, um requisito não funcional descreve como o sistema deve fazer.

## RNF do sistema

<!-- | ID RNF | Nome do Requisito | Categoria | Descrição | Métricas | Justificativa |
|--------|-------------------|-----------|-----------|----------|---------------|
| RNF01  | Duração de Bateria e Uso Contínuo do Robô   |  | A duração do robô deve ter um tempo de operação mínima de 4 horas, sendo capaz de verificar todos os canos do reboiler. Ou seja, sua bateria deve ao menos conseguir suportar analisar 2000 canos em uma única jornada de trabalho. O robô será colocado em operação contínua e marcado o início da operação e o tempo em que o robô ficar sem bateria. | Tempo de operação contínua | O foco é garantir que o robô tenha autonomia suficiente para realizar uma jornada de trabalho completa, verificando os canos do reboiler sem interrupções.|
||
| RNF02  | Precisão na Detecção |  | O sistema deve assegurar uma precisão mínima na detecção de obstáculos e seres vivos, com erro tolerável menor ou igual a 5%. | Comparação com referência conhecida | O foco é na precisão do sistema na detecção de obstáculos e seres vivos. Esse requisito visa assegurar uma precisão específica na determinação da distância entre o robô e objetos ou seres vivos, garantindo a integridade do ambiente e a segurança dos mesmos, evitando colisões. |
||
| RNF03  | Latência de Comunicação |   | A latência de comunicação do sistema não deve exceder 100 milissegundos (ms) em qualquer momento durante a operação do robô. | Limite: 100 ms | Garantir uma comunicação ágil e eficiente para operação em tempo real do robô.|
||
| RNF04  | Tempo de Escaneamento dos Canos |   | O escaneamento dos canos deve ser feito em um tempo máximo de 5s, a fim de otimizar o tempo de verificação dos canos.| Tempo de escaneamento em segundos . Limite de  5 segundos| É preciso que o escaneamento seja rápido para que o reboiler seja verificado em um curto período de tempo, sem impactar a rotina de manutenção da empresa|
||
| RNF05  | Usabilidade da Interface de Controle do Robô | Usuabilidade | A interface de controle do robô deve ser projetada de forma intuitiva e amigável, permitindo que o Operador Industrial monitore o trajeto do robô e observe os locais para os quais ele foi direcionado com facilidade e eficiência. | A usabilidade da interface será avaliada utilizando métricas de facilidade de aprendizado, eficiência de uso, taxa de erro do usuário e satisfação do usuário. |
||
| RNF06  | Eficiência do Código de Movimentação | Eficiência | O código de movimentação precisa ser rápido às respostas do operador | Tempo de resposta do robô quando uma ação é executada. | O robô precisa realizar as movimentações com um tempo curto de resposta, para se evitarem possíveis erros de operação. |
||
| RNF07  | Precisão do sistema de alerta de colisão | Segurança | A precisão do sistemad e colisão garante a integridade do robô | Distância de deteção de obstáculos em metros, além da taxa de detecção na distância de 1 metro pelas ditância menores que 1 metro | Um sistema de alerta de colisão efetivo e com tempo de resposta rápido é essencial para garantir a segurança do robô, do Operador Industrial e de outras pessoas presentes no ambiente de operação. Notificações rápidas e precisas sobre possíveis colisões permitem que o Operador Industrial tome medidas imediatas para evitar acidentes.
||
| RNF08  | Escalabilidade do sistema através de uma API para integração com o dashboard da Atvos | Integração | O sistema deve ser escalável para permitir a interação com os dados captados por meio de uma API que se comunica com o dashboard proprietário da Atvos. | Capacidade de integração, flexibilidade para aumentar o volume de dados processados. | A escalabilidade do sistema é crucial para garantir que ele possa lidar com volumes crescentes de dados e integração com sistemas externos, como o dashboard da Atvos, sem comprometer o desempenho ou a eficiência operacional. | -->

**RNF01 - Autonomia do robô:** Diante da operação contínua do robô, durante o processo de limpeza, o mesmo deve ter autonomia necessária para verificar ao menos um reboiler, por vez. Sendo assim, é preciso ter uma autonomia de no mínimo 4 horas.

**RNF02 - Precisão de reconhecimento:** É preciso que o sistema tenha um precisão de 85% no escaneamento dos canos.

**RNF03 - Latência de comunicação:** Visto que haverá uma teleoperação em tempo real do robô, o sitema deve ter uma latência de comunicação estável e baixa. Nesse caso, uma latência de no máximo 150ms, com picos de 30ms para mais ou menos.

**RNF04 - Tempo de escaneamento:** Tendo em mente a quantidade de canos de um reboiler, o robô deve ter um tempo máximo de checagem dos canos de 7 segundos.

**RNF05 - Interfácie interativa:** Com base no nivel de letramento digital dos possíveis usuários, interfácie do usuário precisa ser fácilmente compreendida ao primeiro contato e possuir os princípios das heurísticas de Nilsen. 

**RNF06 - Robustês de monimentação:** O projeto precisa ter uma construção que permita o robô operar em pisos com desníveis de solo, poças de água e outro obstáculos.

**RNF07 - Distância de colisão:** O sistema de colisão o robô deve ter uma detecção de obstáculos a uma distância de 1 metro.

**RNF08- Atualização de dados:** Os dados do sistema deverão ser captados pela API, uma vez ao dia.
 
## Testes

<!-- | ID RNF | Objetivo do Teste | Métodos de Medição | Passos do Teste | Execução de Teste | Análise de Resultados | Interpretação dos dados |
|--------|-------------------|--------------------|-----------------|-------------------|-----------------------|-------------------------|
| RNF01  | Verificar se o robô opera continuamente por pelo menos 4h | Registro do tempo de operação | Colocar o robô em operação contínua. | Registrar o tempo em que o robô foi ligado até o momento em que a bateria se esgota.| Comparar o tempo registrado com o requisito de 4 horas. | Apenas será aceito se o tempo de operação contínua for igual ou superior a 4 horas.|
||
| RNF02  | Validar a precisão na detecção de obstáculos e seres vivos | Comparação com referência conhecida | Posicionar objetos de teste conhecidos a distâncias pré-determinadas. | Operar o robô para que detecte os objetos e registrar as distâncias medidas. | Comparar as medições do robô com as distâncias reais dos objetos. | A detecção deve ter um erro tolerável menor ou igual a 5%. |
||
| RNF03  | Verificar se a latência de comunicação não excede 100ms | Medição contínua da latência em tempo real | Iniciar o sistema de controle do robô e a infraestrutura de comunicação. | Monitorar a latência durante operações de teleoperação com o robô. | Analisar os dados para garantir que a latência não ultrapasse 100ms em nenhum momento. | A latência de comunicação do sistema não deve exceder 100ms durante a operação do robô.|
||
| RNF04  | Verificar o tempo gasto para escanear o cano | Escanear ocano e cronometrar o tempo da operação | Posicionar um cano entupimendo. | Inicinar a indentificação do estado do cano, cronometrando o tempo de execução da tarefa | 
||
| RNF05  | Avaliar a usabilidade da interface de controle do robô. | Testes de usabilidade com usuários reais. | Solicitar à um grupo de alinos, funcionários e professores do inteli para testarem a interface da solução. | Os testadores  realizarão as tarefas designadas usando a interface de controle do robô, enquanto os observadores registram métricas de desempenho e comportamento. | Analisar os dados coletados durante os testes de usabilidade para identificar áreas de melhoria na interface de controle do robô. | A interface de controle do robô será considerada satisfatória se os Operadores Industriais conseguirem completar as tarefas atribuídas de forma eficiente, com baixa taxa de erro e alta satisfação geral. Quaisquer problemas identificados durante os testes serão abordados e corrigidos para melhorar a usabilidade da interface.|
||
| RNF06  | Metrificar o tempo de resposta dos comandos ao robô | Teste de eficiência e stress. | Submeter uma séria de comandos ao robô em diferentes ambientes de operação. | Utilizar o controle para enviar comandos de movimentação ao robô e observar seu comportamento em tempo real. Testar o robô em cenários que representem desafios comuns encontrados durante sua operação, como obstáculos no caminho, mudanças repentinas no terreno e interferências externas. | Analisar o tempo de resposta do sistema aos comandos de movimentação, bem como sua capacidade de lidar com situações adversas sem comprometer a segurança do robô ou das pessoas ao seu redor. | O código de movimentação do robô será considerado satisfatório se demonstrar eficiência no processamento dos comandos de movimentação e robustez na operação em diferentes condições. Quaisquer problemas identificados durante os testes serão corrigidos para melhorar o desempenho e a confiabilidade do código.
||
| RNF07 | Avaliar a efetividade e o tempo de resposta do sistema de alerta de colisão. | Testes de simulação e testes de tempo. | Simular situações de possível colisão a 1 metro de distância e registrar a detecção e a resposta do sistema de alerta. | Posicionar o robô em direção a objetos ou seres vivos a 1 metro de distância e observar se o sistema de alerta é acionado corretamente. Cronometrar o tempo entre a detecção da ameaça e a exibição do alerta no visor. | Verificar se o sistema de alerta de colisão é acionado quando necessário e se o tempo de resposta está dentro dos limites especificados. | O sistema de alerta de colisão será considerado satisfatório se demonstrar efetividade na detecção de possíveis colisões a 1 metro de distância e um tempo de resposta rápido para notificar o Operador Industrial. Quaisquer problemas identificados durante os testes serão corrigidos para melhorar a efetividade e o tempo de resposta do sistema de alerta de colisão.
||
| RNF08 | Verificar a integração do sistema com o dashboard da Atvos | Testes de integração | Integrar o sistema com o dashboard da Atvos e verificar a comunicação através da API. | Enviar dados do sistema para o dashboard e confirmar a correta visualização e processamento desses dados. | Verificar se os dados são transmitidos corretamente entre o sistema e o dashboard, sem perda de informações ou erros de comunicação. | A integração será considerada satisfatória se os dados forem transmitidos de forma eficiente e precisa entre o sistema e o dashboard da Atvos, garantindo uma interação fluida e sem problemas entre os dois sistemas.| -->


**1. Autonomia do Robô (RNF01):**
- Cenário de Teste: Simule a operação contínua do robô em um ambiente controlado e monitore sua autonomia.
- Método de Simulação: Execute uma sequência de comandos que simulem a operação do robô e acompanhe o consumo de energia.
- Critério de Aceitação: O consumo simulado de energia durante um período equivalente a 4 horas de operação não deve exceder a capacidade da bateria do robô.

**2. Precisão de Reconhecimento (RNF02):** 
- Cenário de Teste: Utilize um conjunto de dados simulados de imagens de canos com diferentes níveis de limpeza e avalie a precisão do sistema.
- Método de Simulação: Gere imagens simuladas de canos com diferentes níveis de sujeira e utilize-as como entrada para o sistema de reconhecimento.
- Critério de Aceitação: O sistema deve alcançar uma precisão de pelo menos 85% na classificação correta da limpeza dos canos em relação às imagens simuladas.

**3. Latência de Comunicação (RNF03):**
- Cenário de Teste: Simule operações de teleoperação em tempo real do robô e meça a latência de comunicação.
- Método de Simulação: Utilize ferramentas de simulação de rede para introduzir a latência desejada durante a comunicação.
- Critério de Aceitação: A latência de comunicação simulada não deve exceder 150ms, com picos de 30ms para mais ou menos.

**4. Tempo de Escaneamento (RNF04):**
- Cenário de Teste: Simule o escaneamento de uma série de canos e registre o tempo necessário para a conclusão.
- Método de Simulação: Crie um conjunto de dados simulados representando os canos de um reboiler e cronometre o tempo que o sistema leva para escaneá-los.
- Critério de Aceitação: O tempo de escaneamento simulado para todos os canos não deve exceder 7 segundos.

**5. Interfácie Interativa (RNF05):**
- Cenário de Teste: Apresente a interface do usuário simulada a diferentes usuários e avalie sua compreensão e usabilidade.
- Método de Simulação: Utilize protótipos interativos da interface do usuário e conduza sessões de teste com usuários representativos.
- Critério de Aceitação: A interface do usuário simulada deve ser facilmente compreensível ao primeiro contato e deve seguir os princípios das heurísticas de Nielsen.

**6. Robustez de Monumentação (RNF06):**
- Cenário de Teste: Simule a operação do robô em diferentes condições de terreno e registre seu desempenho.
- Método de Simulação: Utilize um ambiente simulado que reproduza condições realistas de terreno e obstáculos.
- Critério de Aceitação: O robô deve ser capaz de operar sem problemas em condições simuladas de desníveis de solo, poças de água e outros obstáculos.

**7. Distância de Colisão (RNF07):**
- Cenário de Teste: Simule a detecção de obstáculos em diferentes distâncias e registre os resultados.
- Método de Simulação: Utilize objetos simulados para representar obstáculos e meça a capacidade do sistema de detectá-los.
- Critério de Aceitação: O sistema de detecção de obstáculos simulado deve funcionar corretamente a uma distância mínima de 1 metro.

**8. Atualização de Dados (RNF08):**
- Cenário de Teste: Simule a atualização dos dados do sistema pela API e verifique sua precisão e integridade.
- Método de Simulação: Desenvolva um ambiente simulado que emule a interação entre o sistema e a API de dados.
- Critério de Aceitação: Os dados simulados do sistema devem ser captados pela API e atualizados corretamente pelo menos uma vez ao dia, sem falhas ou erros perceptíveis.