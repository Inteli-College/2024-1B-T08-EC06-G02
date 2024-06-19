---
title : Matriz de Riscos
sidebar_position : 1
---

# Matriz de Riscos

A matriz de riscos é uma ferramenta essencial na gestão de projetos, permitindo a identificação, classificação e gestão dos potenciais riscos ao longo de todo o ciclo de vida do projeto. Ela está sujeita a alterações de acordo com os eventos que ocorrem durante o projeto. Estruturada para visualizar a probabilidade de ocorrência de cada risco em comparação com o impacto que ele pode ter no projeto, a matriz facilita a priorização das ações de mitigação. Além disso, ela também inclui possibilidades que podem aumentar o valor do projeto, as quais são consideradas na seção de oportunidades. [1]

<h2 align="center"> Matriz de Riscos </h2>

![Matriz de Riscos](/img/sprint_1/matriz-de-risco.jpg)

<h6 align="center"> Fonte: Elaboração Grupo Repipe </h6>
 

## Planos de Prevenção e Ataque

Além da elaboração da Matriz de Risco, é elaborado um plano de prevenção para evitar que o risco aconteça e um plano de ataque para o preparo caso haja a possibilidade dos riscos se tornarem realidade.

### 1. Atvos não fornecer dados reais do que seria um reboiler sujo para o treinamento do robô
- **Prevenção**: Nas duas primeiras Sprints (15/04/2024 - 10/05/2024) combinar com Atvos a hipótese de recebimento dos dados das condiçẽos que caracterizm se um tubo esta sujo ou não para o treinamento.
- **Ataque**: Na eventualidade de não ser possível utilizar os dados fornecidos pela Atvos, será necessário recorrer a dados sintéticos/mockados para o treinamento do modelo.

### 2.Mau entendimento do TAP
- **Prevenção**: Definir claramente o escopo e entendimento do projeto no primeiro encontro com a Atvos (26/04/2024).
- **Ataque**: Adaptar o planejamento do projeto com base nas discussões realizadas e realocar prioridades para garantir o progresso contínuo do projeto.

### 3. Robô não conseguir distinguir se os tubos precisam de uma segunda limpeza
- **Prevenção**: Desenvolver um algoritmo de visão computacional robusto que busque confirmar ao máximo a necessidade de uma segunda limpeza.

- **Ataque**: Conversar com o parceiro (Atvos) sobre o 'número mínimo de sujeira que poderia ser detectado' para calibrar adequadamente o treinamento do robô e garantir a detecção precisa de sua condição de limpeza.


### 4. Robô conseguir categorizar se os tubos estão limpos ou sujos em ambiente controlado e não conseguir em casos reais
- **Prevenção**: Empregar técnicas de validação cruzada e regularização durante o desenvolvimento do modelo.
- **Ataque**: Reajustar o modelo com novos dados e revisar a abordagem de treinamento, visando sempre eliminar qualquer viés presente no conjunto de dados de treinamento que possa interferir no funcionamento da solução.

### 5. Robô não estabelecer conexão a rede para enviar as informações coletados para o banco de dados da Atvos
- **Prevenção**: Implementar mecanismos de reconexão automática e sistemas de backup para a transferência de dados, garantindo que nenhuma informação seja perdida durante o processo.
- **Ataque**: Armazenar dados localmente no robô até que a conexão seja restabelecida ou até que a transferência seja realizada manualmente.

### 6.Robô fornecer output errado do status do tubo, isso é, se está sujo ou não
- **Prevenção**: Refinar algoritmos de avaliação e realizar testes extensivos em diversas condições.
- **Ataque**: Recalibrar o sistema de visão computacional do robô e realizar inspeções de controle de qualidade, garantindo assim a reavaliação contínua dos algoritmos de leitura a cada modificação realizada.

### 7. Robô não conseguir se movimentar no terreno que os reboilers estão alocados
- **Prevenção**: Projetar o robô para lidar com diferentes tipos de terrenos e realizar testes de mobilidade para garantir sua capacidade de navegar eficientemente em ambientes variados.
- **Ataque**: Fazer ajustes técnicos no robô ou alterar a estratégia de inserção do mesmo no terreno.

# Referências 
[1] REDACAO PAPOCA. O que é matriz de risco? Aprenda como montar + exemplo. Esfera Energia. Disponível em: [https://blog.esferaenergia.com.br/gestao-empresarial/matriz-de-risco](https://blog.esferaenergia.com.br/gestao-empresarial/matriz-de-risco). Acesso em: 29 abr. 2024.

‌
