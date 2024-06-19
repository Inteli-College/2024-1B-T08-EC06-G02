---
sidebar_position: 2
title: "Requisitos Não-Funcionais"
---

# Requisitos não funcionais (RNF)

## Entendendo os requisitos não funcionais

Primeiramente, deve-se entender o que são requisitos não funcionais. Um requisito funcional se refere a uma especificação do sistema que descreve uma funcionalidade específica que o sistema deve cumprir. Por outro lado, um requisito não-funcional se refere a uma especificação do sistema que descreve critérios de qualidade, restrições ou limitações que o sistema deve atender. Em outras palavras, enquanto um requisito funcional descreve o que o sistema deve fazer, um requisito não-funcional descreve como o sistema deve fazer.

## RNF do sistema

**RNF01 - Autonomia do robô:** Diante da operação contínua do robô, durante o processo de limpeza, o mesmo deve ter autonomia necessária para verificar ao menos um reboiler, por vez. Sendo assim, é preciso ter uma autonomia de no mínimo 4 horas.

**RNF02 - Precisão de reconhecimento:** É preciso que o sistema tenha um precisão de 85% no escaneamento dos canos.

**RNF03 - Latência de comunicação:** Visto que haverá uma teleoperação em tempo real do robô, o sitema deve ter uma latência de comunicação estável e baixa. Nesse caso, uma latência de no máximo 150ms, com picos de 30ms para mais ou menos.

**RNF04 - Tempo de escaneamento:** Tendo em mente a quantidade de canos de um reboiler, o robô deve ter um tempo máximo de checagem dos canos de 7 segundos.

**RNF05 - Interface interativa:** Com base no nivel de letramento digital dos possíveis usuários, interface do usuário precisa ser fácilmente compreendida ao primeiro contato e possuir os princípios das heurísticas de Nilsen. 

**RNF06 - Robustês de monimentação:** O projeto precisa ter uma construção que permita o robô operar em pisos com desníveis de solo, poças de água e outro obstáculos.

**RNF07 - Distância de colisão:** O sistema de colisão o robô deve ter uma detecção de obstáculos a uma distância de 1 metro.

**RNF08- Atualização de dados:** Os dados do sistema deverão ser captados pela API, uma vez ao dia.
 
## Testes

**1. Autonomia do Robô (RNF01):**
- Passo 1: Monte um ambiente controlado com obstáculos representativos e uma área de movimento para o robô.
- Passo 2: Crie um script que simule as atividades típicas do robô, como movimento, coleta de dados e comunicação.
- Passo 3: Implemente um sistema de monitoramento de energia que registre o consumo de energia do robô durante a simulação.
- Passo 4: Execute o script por um período equivalente a 4 horas de operação e verifique se o consumo simulado de energia está dentro da capacidade da bateria do robô.

**2. Precisão de Reconhecimento (RNF02):**
- Passo 1: Monte um conjunto de dados simulados representando diferentes condições de limpeza dos canos.
- Passo 2: Desenvolva um algoritmo de reconhecimento de imagens que classifique a limpeza dos canos.
- Passo 3: Alimente as imagens simuladas ao sistema de reconhecimento e registre as classificações feitas.
- Passo 4: Avalie a precisão do sistema comparando as classificações com os dados de referência e verifique se o sistema atinge uma precisão de pelo menos 85%.

**3. Latência de Comunicação (RNF03):**
- Passo 1: Configure um ambiente de simulação de rede com as características desejadas de latência.
- Passo 2: Estabeleça uma conexão de teleoperação entre o controlador e o robô através da rede simulada.
- asso 3: Meça a latência de comunicação durante a operação do robô e registre os resultados.
- Passo 4: Verifique se a latência de comunicação simulada está dentro dos limites especificados, com picos de 30ms para mais ou menos.

**4. Tempo de Escaneamento (RNF04):**
- Passo 1: Gere um conjunto de dados simulados representando os canos a serem escaneados.
- Passo 2: Implemente o algoritmo de escaneamento e cronometre o tempo necessário para escanear todos os canos.
- Passo 3: Repita o processo várias vezes para garantir consistência nos resultados.
- Passo 4: Verifique se o tempo de escaneamento simulado para todos os canos não ultrapassa 7 segundos.

**5. Interface Interativa (RNF05):**
- Passo 1: Desenvolva protótipos interativos da interface do usuário com base nos requisitos de design.
- Passo 2: Recrute usuários representativos para participar das sessões de teste.
- Passo 3: Apresente a interface do usuário simulada aos usuários e observe sua interação e feedback.
- Passo 4: Avalie se a interface do usuário é facilmente compreensível e atende aos princípios das heurísticas de Nielsen.

**6. Robustez de Movimentação (RNF06):**
- Passo 1: Configure um ambiente de simulação que reproduza condições realistas de terreno e obstáculos.
- Passo 2: Execute o robô em diferentes condições de terreno e registre seu desempenho.
- Passo 3: Introduza desafios, como desníveis de solo e poças de água, para testar a robustez do robô.
- Passo 4: Verifique se o robô opera sem problemas nas condições simuladas, lidando adequadamente com os obstáculos.

**7. Distância de Colisão (RNF07):**
- Passo 1: Posicione objetos simulados em diferentes distâncias em relação ao robô.
- Passo 2: Ative o sistema de detecção de obstáculos e registre sua capacidade de detectar os objetos em diferentes distâncias.
- Passo 3: Varie a distância dos objetos e repita o processo para diferentes cenários.
- Passo 4: Verifique se o sistema de detecção de obstáculos funciona corretamente a uma distância mínima de 1 metro.

**8. Atualização de Dados (RNF08):**
- Passo 1: Desenvolva um ambiente simulado que simule a interação entre o sistema e a API de dados.
- Passo 2: Configure a simulação para realizar atualizações de dados programadas.
- Passo 3: Verifique se os dados simulados são captados pela API e atualizados corretamente.
- Passo 4: Avalie a precisão e integridade dos dados atualizados e verifique se as atualizações ocorrem sem falhas ou erros perceptíveis.

## Referências 

[1] Requisitos não funcionais: o guia completo!. Disponível em : [betrybe](https://blog.betrybe.com/tecnologia/requisitos-nao-funcionais/)
