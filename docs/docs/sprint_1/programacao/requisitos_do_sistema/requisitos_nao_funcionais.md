---
sidebar_position: 2
title: "Requisitos não funcionais"
---

# Requisitos não funcionais (RNF)

## Entendendo os requisitos não funcionais
Primeiramente, vamos entender o que são requisitos não funcionais. Um requisito funcional se refere a uma especificação do sistema que descreve uma funcionalidade específica que o sistema deve cumprir. Por outro lado, um requisito não funcional se refere a uma especificação do sistema que descreve critérios de qualidade, restrições ou limitações que o sistema deve atender. Em outras palavras, enquanto um requisito funcional descreve o que o sistema deve fazer, um requisito não funcional descreve como o sistema deve fazer.

Por exemplo, se um requisito funcional diz que é preciso ter uma comunicação em tempo real com o robô, um possível requisito não funcional seria uma taxa de latência máxima de 250ms.

## RNF do sistema

Aqui está a tabela dos requisitos não funcionais:

| ID RNF | Nome do Requisito | Descrição | Métricas | Justificativa |
|--------|-------------------|-----------|----------|---------------|
| RNF01  | Duração de Bateria e Uso Contínuo do Robô   | A duração do robô deve ter um tempo de operação mínima de 4 horas, sendo capaz de verificar todos os canos do reboiler. Ou seja, sua bateria deve ao menos conseguir suportar analisar 2000 canos em uma única jornada de trabalho. O robô será colocado em operação contínua e marcado o início da operação e o tempo em que o robô ficar sem bateria. | Tempo de operação contínua       | O foco é garantir que o robô tenha autonomia suficiente para realizar uma jornada de trabalho completa, verificando os canos do reboiler sem interrupções.                                                                                                                                                                                                                                         |
| RNF02  | Precisão na Detecção      | O sistema deve assegurar uma precisão mínima na detecção de obstáculos e seres vivos, com erro tolerável menor ou igual a 0,5%.                                                                 | Comparação com referência conhecida | O foco é na precisão do sistema na detecção de obstáculos e seres vivos. Esse requisito visa assegurar uma precisão específica na determinação da distância entre o robô e objetos ou seres vivos, garantindo a integridade do ambiente e a segurança dos mesmos, evitando colisões. |
| RNF03  | Latência de Comunicação   | A latência de comunicação do sistema não deve exceder 100 milissegundos (ms) em qualquer momento durante a operação do robô.                                                                  | Limite: 100 ms                     | Garantir uma comunicação ágil e eficiente para operação em tempo real do robô.                                                                                                                                                                                                                                                                                                                            |

## Testes

| ID RNF | Objetivo do Teste | Métodos de Medição | Passos do Teste | Execução de Teste | Análise de Resultados | Interpretação dos dados |
|--------|-------------------|--------------------|-----------------|-------------------|-----------------------|-------------------------|
| RNF01  | Verificar se o robô opera continuamente por pelo menos 4h | Registro do tempo de operação | Colocar o robô em operação contínua. | Registrar o tempo em que o robô foi ligado até o momento em que a bateria se esgota.| Comparar o tempo registrado com o requisito de 4 horas. | Apenas será aceito se o tempo de operação contínua for igual ou superior a 4 horas.|
| RNF02  | Validar a precisão na detecção de obstáculos e seres vivos | Comparação com referência conhecida | Posicionar objetos de teste conhecidos a distâncias pré-determinadas. | Operar o robô para que detecte os objetos e registrar as distâncias medidas. | Comparar as medições do robô com as distâncias reais dos objetos. | A detecção deve ter um erro tolerável menor ou igual a 0,5%. |
| RNF03  | Verificar se a latência de comunicação não excede 100ms | Medição contínua da latência em tempo real | Iniciar o sistema de controle do robô e a infraestrutura de comunicação. | Monitorar a latência durante operações de teleoperação com o robô. | Analisar os dados para garantir que a latência não ultrapasse 100ms em nenhum momento. | A latência de comunicação do sistema não deve exceder 100ms durante a operação do robô.|
