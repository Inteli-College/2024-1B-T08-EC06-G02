---
sidebar_position: 5
title: "Database"
---

**Database**, ou Banco de Dados, é uma estrutura que permite o armazenamento persistente de informações, garantindo que os dados sejam salvos de forma segura e não se percam facilmente.

Para este projeto, adotamos um banco de dados relacional utilizando SQLite3, devido às estruturas de dados bem definidas e às relações diretas entre as informações.

## Estrutura de Dados

O *database* foi dividido em duas partes principais: uma para os dados dos usuários e outra para os dados dos reboilers.

### Dados de Usuários

Dado que a interação de múltiplos usuários não é o foco principal desta aplicação, a estrutura do banco de dados é bastante simples, consistindo em apenas duas tabelas:

1. **Users**: Armazena dados dos usuários, como nome, e-mail, senha e cargo.
2. **Roll**: Define os níveis de autorização dos usuários com base em seus cargos.

Essa abordagem foi escolhida visando a escalabilidade. Se o projeto continuar a se desenvolver e sua utilização aumentar, um banco de dados que suporte múltiplos usuários com diferentes cargos facilitará a gestão do sistema.

<div align="center">

#### Imagem 1 - Tabelas de Usuários
- user_id
- user_email
- user_password
- user_role
- user_name
##### Fonte: Elaboração grupo Repipe
</div>

- **Roll**: Cargos que um usuário pode ter na plataforma.
- **Users**: Usuários da plataforma, com seus logins, nomes e senhas.

### Dados dos Reboilers

Esta parte do banco de dados trata das informações sobre o estado dos reboilers. Para facilitar a visualização dos dados, o grupo Repipe optou por dividir os reboilers em quadrantes e sub-quadrantes, permitindo que os sub-quadrantes com canos sujos sejam destacados em um mapa de calor.

Além disso, foi considerada a escalabilidade até nessa parte do banco de dados. É possível incluir as refinarias que a Atvos possui, relacionando os reboilers às suas respectivas refinarias, permitindo um controle e análise mais detalhados dos dados fornecidos pela plataforma.

<div align="center">

#### Imagem 2 - Tabelas de Reboilers
- reboiler_id
- reboiler_num_pipes
- refinary_id
- refinary_name
#### Fonte: Elaboração grupo Repipe
</div>

## Referências

- [1] Introdução: SQL e NoSQL — trabalhando com bancos relacionais e não relacionais. Disponível em [Alura](https://www.alura.com.br/artigos/sql-nosql-bancos-relacionais-nao-relacionais?utm_term=&utm_campaign=%5BSearch%5D+%5BPerformance%5D+-+Dynamic+Search+Ads+-+Artigos+e+Conte%C3%BAdos&utm_source=adwords&utm_medium=ppc&hsa_acc=7964138385&hsa_cam=11384329873&hsa_grp=111087461203&hsa_ad=687448474447&hsa_src=g&hsa_tgt=dsa-425656816943&hsa_kw=&hsa_mt=&hsa_net=adwords&hsa_ver=3&gad_source=1&gclid=CjwKCAjwmYCzBhA6EiwAxFwfgEN0vKfEhIl31-eAlOPxrhZ8Um4oEOQs9YIdmi9MTRyybBXayq5uYRoCRwAQAvD_BwE). Acesso em 5 de junho de 2024.

## APIs e BACKEND
É possível consultar a documentação das APIs de duas maneiras diferentes. A primeira é rodando o projeto e consultando o endereço /docs (Localmente seria a consulta do url [http://127.0.0.1:8000/docs](http://127.0.0.1:8000/docs), neste link é possível testar interativamente cada um dos endpoints.
A outra maneira é acessando a documentação estática escrita via postman disponível clicando [aqui](https://documenter.getpostman.com/view/30920057/2sA3XLEPYY#b8d4e7ba-7d73-4251-8934-04f564c64650)


*Toda a documentação referente ao backend e as APIs está disponível no seguinte URL: https://documenter.getpostman.com/view/30920057/2sA3XLEPYY#b8d4e7ba-7d73-4251-8934-04f564c64650*

