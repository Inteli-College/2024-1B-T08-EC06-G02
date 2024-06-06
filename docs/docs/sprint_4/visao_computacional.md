---
sidebar_position: 1
title: "Sistema de Visão Computacional"
---

### Este documento deve detalhar o sistema de visão computacional da sprint 4

# Visão Computacional
Visão computacional é um campo da inteligência artificial que se concentra em capacitar os computadores a interpretar e compreender o mundo visual de maneira semelhante aos seres humanos. Essa disciplina envolve o desenvolvimento de algoritmos e modelos que permitem a um sistema extrair informações úteis de imagens e vídeos. As aplicações de visão computacional são amplas e variadas, incluindo o reconhecimento de objetos, detecção de rostos, análise de movimento, e segmentação de imagens.

Em termos técnicos, a visão computacional utiliza técnicas de processamento de imagem e aprendizado de máquina para analisar os dados visuais. Por exemplo, um sistema de visão computacional pode ser treinado para reconhecer diferentes tipos de frutas em uma imagem ao ser alimentado com um grande número de exemplos rotulados. O treinamento envolve a construção de modelos matemáticos que podem identificar padrões e características distintivas das frutas.

Nesse sentido, esta etapa do projeto envolveu a construção de um algoritmo de visão computacional para identificação de canos sujos. Além disso, para o treinamento, foi feita a construção de um banco de dados de imagens próprio. Assim, a primeira etapa foi a construção dessa base.

## Base de Dados
A base de dados deste projeto foi criada utilizando o Chat GPT-4 para gerar imagens que seriam utilizadas no início do treinamento. Esta medida se mostrou necessária devido à falta de imagens representativas da solução trabalhada em plataformas de bancos de imagens gratuitas, como Roboflow e Google. Assim, com o uso do Chat GPT-4, o conjunto de imagens geradas pode ser visto abaixo:

<h2 align="center">Imagem 1 - Base de dados para treinamento </h2>

![Imagem 1 do Wireframe - tela de login](/docs/static/img/sprint_4/canos_sujos.png)
<h6 align="center"> Fonte: Elaboração grupo Repipe </h6>

Tendo em mente essa base de dados, nota-se que há vários tipos de imagens em diferentes contextos, para que o modelo possa aprender com as diversas circunstâncias encontradas na solução.

Nessa perspectiva, com as imagens geradas, foi necessário transformá-las no modelo do YOLOv8. Em uma primeira instância, foi utilizado o **labelme** (que pode ser instalado pelo comando `pip install labelme`) para fazer a demarcação dos pontos que seriam analisados para o treino do modelo. Com isso, para cada uma das vinte imagens, foi gerado um arquivo no formato JSON com os pontos selecionados.

Após esse processo de demarcação dos pontos pelo software, foi feita a instalação do **Labelme2YOLO** através da clonagem do repositório `https://github.com/rooneysh/Labelme2YOLO.git`. Em seguida, foi executado o arquivo `labelme2yolo.py` para converter os arquivos .json para .txt. Além disso, esse script já faz a separação das imagens em treino e validação.

Por fim, com a base de imagens finalizada, deu-se início à construção do algoritmo para o treinamento e validação da visão computacional.


