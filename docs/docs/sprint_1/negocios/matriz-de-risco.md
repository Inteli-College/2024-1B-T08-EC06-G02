---
sidebar_position : 2
---
![Matriz de Riscos](../../static/img/matriz-de-risco.png)
# Matriz de Riscos

A matriz de riscos é uma ferramenta essencial na gestão de projetos que permite identificar, classificar e gerenciar os potenciais riscos durante todo o ciclo de vida de um projeto, o que significa que ela está sempre sujeita a alterações de acordo com os acontecimentos ao longo do projeto. Ela é estruturada para visualizar a probabilidade de ocorrência de cada risco contra o impacto que ele pode ter no projeto, facilitando assim a priorização das ações de mitigação. Além disso, também são inclúidas possibilidades que podem aumentar ainda mais o valor do projeto, sendo essas possibilidades incluídas na parte de oportunidades. 

## Planos de Prevenção e Ataque

Além da elaboração da matriz, é elaborado tanto um plano de prevenção (para evitar que o risco aconteça) tanto um plano de ataque (para se preparar com a possibilidade do risco se tornar realidade) para cada risco.

### 1. Atvos não fornecerem dados para treino
- **Prevenção**: Tentar combinar com eles o mais cedo possível e validar a hipótese de recebermos dados deles para o treinamento.
- **Ataque**: Utilizar dados sintéticos / mockados para o treinamento do modelo.

### 2. Mudança no escopo do projeto
- **Prevenção**: Definir claramente o escopo nos encontros com a atvos.
- **Ataque**: Reavaliar e adaptar o planejamento do projeto de acordo com os encontros e a TAPI e realocar prioridades conforme necessário.

### 3. Robô não conseguir entregar respostas binárias (Está limpo / não está limpo)
- **Prevenção**: Desenvolver um algoritmo robusto de visão computacional buscando confirmar ao máximo a necessidade de uma segunda lavagem.
- **Ataque**: Trabalhar com faixas de avaliação e definir o que necessita uma segunda lavagem e o que não necessita.

### 4. Modelo sofrer de overfitting e não ser corrigido
- **Prevenção**: Utilizar técnicas de validação cruzada e regularização durante o desenvolvimento do modelo.
- **Ataque**: Reajustar o modelo com novos dados e revisar a abordagem de treinamento.

### 5. Robô não estabelecer conexão para enviar os dados coletados
- **Prevenção**: Implementar mecanismos de reconexão automática e sistemas de backup para transferência de dados.
- **Ataque**: Armazenar dados localmente no robô até que a conexão seja restabelecida ou transferir manualmente.

### 6. Robô não avaliar corretamente se o cano foi limpo ou não
- **Prevenção**: Refinar algoritmos de avaliação e realizar testes extensivos em diversas condições.
- **Ataque**: Recalibrar o sistema de visão do robô e realizar inspeções de controle de qualidade.

### 7. Robô não conseguir se movimentar no terreno do robolier
- **Prevenção**: Desenhar o robô para lidar com diferentes terrenos e realizar testes de mobilidade.
- **Ataque**: Fazer ajustes técnicos no robô ou mudar a estratégia de inserção no ambiente.

### 8. Falhas na integração
- **Prevenção**: Constantemente validar o funcionamento de uma feature em um contexto total e ir integrando conforme o desenvolvimento.
- **Ataque**: Realizar um diagnóstico para identificar e corrigir as falhas de integração específicas.
