---
title : "Análise Financeira"
sidebar_position: 2
---

## Análise de Prova de Conceito (PoC)

### Bill of Materials (BOM)

No contexto da Prova de Conceito, a seção de Bill of Materials (BOM) descreve detalhadamente os custos associados aos componentes de hardware utilizados no projeto. Esses custos são fundamentais para entender o investimento inicial necessário e para planejar o financiamento do projeto.

- **Raspberry Pi 4**: R$ 590 (comprado no Brasil)
- **Turtlebot3 Burger**: R$ 10.000 (R$ 5.000 para o robô + R$ 5.000 de imposto)
- **Webcam C270**: R$ 170 (comprado no Brasil)

### Análise Financeira da PoC

A Prova de Conceito (PoC) é essencial para validar a viabilidade técnica e funcional da solução antes de investimentos maiores. Ela testa se os componentes de hardware e software podem integrar-se efetivamente para atender aos requisitos do projeto, identificando desafios potenciais e ajustando a estratégia de desenvolvimento com base em resultados tangíveis. Nessa Análise financeira da Poc foram considerados 2 cenários, primeiro uma implementação do conceito média, que utiliza valores médios do mercado, e uma implementação mínima que procura ao máximo reduzir os custos e ainda fornecer uma boa validação da idéia. 

- **Desenvolvedores Backend**: R$ 40/h
  - 3 horas/dia x 5 dias/semana x 4 semanas = R$ 2,400 total
- **Desenvolvedores Frontend**: R$ 40/h
  - 3 horas/dia x 5 dias/semana x 4 semanas = R$ 2,400 total
- **Product Owner (PO)**: - **Product Owner (PO)**: R$ 9.000/mês

**Custo Base em serviços da PoC**: R$ 13,800

**Custo Total em serviços com impostos e lucro**: R$ 18,512 = 1,1 * (13,800/0.82) (incluindo impostos de 18% e lucro de 10%)

**Custo Total + Hardware**: R$29,272

### Otimização de Custos

Para uma implementação mínima da PoC, foi estipulada uma segunda sessão com o objetivo de reduzir ao máximo os custos, a fim de disponibilizar uma prova de conceito pelo menor preço possível. 
- **Desenvolvedores Backend**: R$ 18/h
  - 2 horas/dia x 5 dias/semana x 10 semanas = R$ 1,800 total
- **Desenvolvedores Frontend**: R$ 18/h
  - 2 horas/dia x 5 dias/semana x 10 semanas = R$ 1,800 total
- **Product Owner (PO)**: R$ 6,000/mês

**Custo Base em serviços da implementação mínima=**: R$ 9,600

**Custo Total do Produto Mínimo com impostos e lucro**: R$ 12,878 = 1,1 * (9,600/0.82) (incluindo impostos de 18% e lucro de 10%)

**Custo Total + Hardware**: R$23,638

### Considerações Gerais

Para ambas as versões da prova de conceito, não serão considerados custos com hospedagem ou serviços de cloud. A escolha entre o produto mínimo e o mais avançado deve levar em consideração o orçamento disponível e o nível de entrega desejado.

## Análise financeira, projeto final

A análise financeira do projeto final é uma pesquisa mais aprofundada visando um produto mais definitivo para a Atvos, bem como os valores de criação e implementação dessa solução.

### Conceito final do projeto

Pensando em uma solução definitiva para a Atvos, o robô não só deve ser uma ferramenta de verificação de imagens, para a coleta de dados, como também um mecanismo de limpeza dos canos dos reboilers.

Com base no cenário desejado de atuação do robô, que é o reboiler pré-esfriamento, o robô deve ser capaz de aguentar altas temperaturas, bem como ser capaz de se locomover dentro da estrutura e ter autonomia para a limpeza e inspeção de todos os canos de um reboiler. 

Além disso, com a finalidade de integrar todas as plantas, o sistema deve ter uma integração com a nuvem, a fim de guardar os dados de forma segura e acessível ao sistema de BI deles.

:::note[Importante]
Deve-se notar que o tempo de desenvolvimento e implementação do projeto é estimado em um ano.
:::

### Precificando

Considerando as especificações eletro-mecânicas do robô, que deve ser compacto, para caber nos canos do reboiler, resistente à altas temperaturas e móvel suficiente para se locomover entre os canos dos reboilers; será preciso um equipe de dois engenheiros mecatrônicos para fazerem todo o desenvolvimento do robô. Esse processo tem estimativa de durar 9 meses, incluindo desenhos iniciais, prototipação, testagem e verificações finais com integração.

- **Engenheiro mecatrônico**: R$7,160/mês
  - 2 engenheiros x 9 meses = R$129,880.00
- **Custo de prototipação**: R$230,000/mês

Visando a otimização dos dados coletados e a estruturação de um armazenamento na nuvem, será preciso um engenheiro de dados. Não só, um analista de dados será responsável por fazer os ajustes necessários com o BI da Atvos. Tendo em mente que essa parte do projeto será feita mais ao seu final, a duração será de 3 meses.

- **Engenheiro de dados**: R$9,250/mês
  - 1 engenheiro de dados x 3 meses = R$27,750.00
- **Analista de dados**: $3,929.00
  - 1 analista de dados x 3 meses = R$7,858.00

Para que o projeto tenha seus sistemas integrados, bem como as interfaces de operação, comunicação e visualização de dados, serão necessários 3 desenvolvedores plenos. É importante considerar que os desenvolvedores estarão durante todo o desenvolvimento do projeto, totalizando 12 meses.

- **Desenvolvedor**: R$5,294/mês
  - 3 desenvolvedores x 12 meses = R$190,584.00

Com o intuito de manter o andamento do projeto de forma ordenada e contínua, um PO será necessário. Esse, também estará durante todo o desenvolvimento do projeto.

- **Product Owner (PO)**: R$9,198.00/mês
  - 1 PO x 12 meses = R$110,376.00

Considerando uma média de 1 robô para cada 4 reboiler e sabendo que são aproximadamente 32 reboiler por refinaria, o projeto teria seu valor cotado em **R$3,214,677.20**, por refinaria, sem impostos.

Considerando imposto de 18% e lucro de 10% (da mesma forma que foi utilizado na PoC), temos como valor por refinaria: **R$4,312,371.58**

### Fontes

[1] **Webcam C270**: [Amazon](https://www.amazon.com.br/Chamadas-Grava%C3%A7%C3%B5es-Widescreen-Logitech-Equipamentos/dp/B003PAOAWG/ref=asc_df_B003PAOAWG/?tag=googleshopp00-20&linkCode=df0&hvadid=379712974695&hvpos=&hvnetw=g&hvrand=10570527231774725958&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1001773&hvtargid=pla-521463501059&psc=1&mcid=5791f36b8833353392495198dc82fc2a)

[2] **Turtlebot3 Burger**: [Aliexpress](https://pt.aliexpress.com/item/1005004405764315.html?src=google)

[3] **Raspberry Pi 4**: [Robocore](https://www.robocore.net/placa-raspberry-pi/raspberry-pi-4-4gb?gad_source=1&gclid=Cj0KCQjw6auyBhDzARIsALIo6v-AFlENSy9z7KWwSZ_ZM2WIk3mL0nxqUJWdMTAzWaftDLnGBUoZboEaAl0EEALw_wcB)

[4] **Salário Desenvolvedor Júnior**: [Talent.com](https://br.talent.com/salary?job=desenvolvedor+j%C3%BAnior)

[5] **Salário Product Owner**. Disponível em [glassdoor](https://www.glassdoor.com.br/Sal%C3%A1rios/product-owner-po-sal%C3%A1rio-SRCH_KO0,16.htm). Acesso em 15 de maio de 2024.

[6] **Engenheiro Mecatrônico**. Disponível em [vagas](https://www.vagas.com.br/cargo/engenheiro-mecatronico). Acesso em 14 de maio de 2024.

[7] **Salário Engenheiro de Dados**. Disponível em [glassdoor](https://www.glassdoor.com.br/Sal%C3%A1rios/engenheiro-de-dados-sal%C3%A1rio-SRCH_KO0,19.htm). Acesso em 15 de maio de 2024.

[8] **Salário Analista de Dados**. Disponível em [glassdoor](https://www.glassdoor.com.br/Sal%C3%A1rios/analista-de-dados-junior-sal%C3%A1rio-SRCH_KO0,24.htm). Acesso em 15 de maio de 2024.

[9] **Salário Desenvolvedor**. Disponível em [glassdoor](https://www.glassdoor.com.br/Sal%C3%A1rios/desenvolvedor-sal%C3%A1rio-SRCH_KO0,13.htm). Acesso em 15 de maio de 2024.
