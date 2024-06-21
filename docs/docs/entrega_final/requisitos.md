---
title: Análise de Requisitos
sidebar_position: 3
---
# Introdução 
Nesta seção, serão apresentados os requisitos funcionais e não funcionais do sistema. Os requisitos funcionais descrevem as ações específicas que o sistema deve realizar para atender às necessidades dos usuários. Eles detalham as funcionalidades e comportamentos esperados, definindo o que o sistema deve fazer. Já os requisitos não funcionais especificam as características de qualidade, restrições e limitações do sistema, abordando como o sistema deve operar para ser eficiente, seguro e utilizável. Com uma compreensão clara desses requisitos, é possível garantir que o sistema desenvolvido atenderá às expectativas de desempenho, segurança e usabilidade, além de cumprir suas funções essenciais.

# Requisitos Funcionais
Um requisito funcional é uma declaração que descreve o comportamento esperado de um sistema, ou seja, especifica o que o sistema deve fazer para atender às necessidades ou expectativas do usuário. Os requisitos funcionais representam as características ou funcionalidades que o usuário percebe ao interagir com o sistema. Ao contrário dos requisitos não funcionais, que detalham aspectos internos do sistema, como desempenho e segurança, os requisitos funcionais se concentram no que o sistema deve realizar. [1]

Esses requisitos normalmente consistem em duas partes: função e comportamento. A função descreve a ação que o sistema deve executar (por exemplo, "calcular o imposto sobre vendas"). O comportamento, por sua vez, especifica como essa função deve ser realizada (por exemplo, "o sistema deve calcular o imposto sobre vendas multiplicando o preço de compra pela alíquota do imposto"). [1]

Com essa descrição do que seriam esses tipos de requisitos para presente solução pensada foram elencados os seguintes requisitos funcionais (RF):

**RF01:** O sistema deve ser operado remotamente. Neste requisito, o Operador Industrial irá controlar o robô à distância para realizar o percurso ou comando especificado, eliminando a necessidade de estar fisicamente presente nos reboilers para a verificação dos tubos.

**RF02:** O robô deve verificar se há tubos sujos após a primeira limpeza. Neste requisito, o Operador Industrial, controlando o robô, irá acessar a câmera para visualizar o que está sendo detectado. O sistema, por meio dessa interface, informará se a tubulação está suja ou não.

**RF03:** O robô deve ter um sistema de colisão. Neste requisito, o Operador Industrial estará pilotando o robô via controle remoto. No entanto, caso o robô se aproxime a uma determinada distância de um objeto ou ser vivo, o alerta será acionado na tela para notificar o usuário.

**RF04:** O sistema deve ser escalável. Neste requisito, o analista de dados da Atvos poderá interagir com os dados captados por meio de uma API que se comunica com o dashboard proprietário deles.

**RF05:** O sistema deve ter uma interface para o controle do robô. Neste requisito, o Operador Industrial utilizará essa interface para monitorar o trajeto do robô e observar os locais para os quais ele foi direcionado.

**RF06:** O robô deve possuir um código em seu sistema que possibilite sua movimentação. Neste requisito, o Operador Industrial utilizará um controle para guiar o robô ao longo do caminho, utilizando o sistema projetado para receber esses comandos e direcionar o robô adequadamente.


## Referências
[1] Jain, Anushtha. 2022. ‘What Are Functional Requirements: Examples, Definition, Complete Guide’, Visure Solutions (Visure Solutions) [https://visuresolutions.com/pt/blog/functional-requirements/](https://visuresolutions.com/pt/blog/functional-requirements/) [accessed 20 April 2024]

# Requisitos Não Funcionais
Os requisitos não funcionais definem os critérios de qualidade e restrições que o sistema deve atender, abordando aspectos como desempenho, segurança, usabilidade e confiabilidade. Esses requisitos são fundamentais para garantir que o sistema opere de maneira eficaz e segura, proporcionando uma boa experiência ao usuário. A seguir, apresentamos os requisitos não funcionais estabelecidos para este projeto, detalhando as características que o sistema deve possuir para ser considerado adequado e eficiente.

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

Ao concluir a apresentação dos requisitos funcionais e não funcionais, é essencial reconhecer a importância de ambos na construção de um sistema robusto e eficiente. Os requisitos funcionais garantem que o sistema cumpra suas funções essenciais e atenda às expectativas dos usuários, enquanto os requisitos não funcionais asseguram que o sistema opere com qualidade, segurança e desempenho adequados. Juntos, esses requisitos fornecem uma base sólida para o desenvolvimento, implementação e avaliação do sistema, contribuindo para o sucesso do projeto e a satisfação dos usuários finais.

# Conclusão
Ao concluir a análise dos requisitos, é possível ter uma visão mais clara do que a solução deve abranger. Assim, inicia-se o desenvolvimento do projeto, abrangendo tanto a plataforma web quanto o backend de comunicação com o robô.