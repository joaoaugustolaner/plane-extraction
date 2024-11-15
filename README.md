# Ferramenta para extração de planos

### Aluno: 
- João Augusto Tonial laner

### Orientador:
- Leandro Tonietto

Esse projeto cria um panorama a partir de fotos tiradas de um afloramento e processa uma nuvem de 
pontos associada a essas fotos, correlacionando cada pixel da imagem com um respectivo ponto. 
Após a correlação realizada, o usuário pode selecionar os pontos intrínsecos a nuvem de pontos 
e ao apertar "enter", um plano de ajuste é calculado para os pontos selecionados. 

---
## Estrura do Projeto:

```
.
├── CMakeLists.txt
├── LICENSE
├── README.md
├── include
│   ├── ClickHandler.h
│   ├── CloudPointProcessor.h
│   └── Stitcher.h
├── resources
│   ├── images
│   │   ├── IMG_5375.JPG
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   └── IMG_5417.JPG
│   ├── panorama.jpg
│   └── point-cloud.txt
└── src
    ├── ClickHandler.cpp
    ├── CloudPointProcessor.cpp
    ├── Main.cpp
    └── Stitcher.cpp
``` 
---
## Pré requisitos:

- **CMake 3.10+**
- **C++ 17** 
- **OpenCV 4.6**

---
## Instalação:

1. Clone o repositório para seu diretório local:
    - ```git clone https://gitlab.com/joaoaugustolaner/plane-extraction.git && cd plane-extraction/```

2. Crie um diretório `build` na raiz do projeto clonado:
    - ```mkdir build/ && cd build/```

3. Execute o **CMakeLists.txt** para gerar o Makefile e então execute o comando make:
    - ```cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..```
    4. Crie um link simbólico na **raiz do projeto** para que a IDE possa achar com mais facilidade, caso necesário.
    - ```ln -s build/compile_commands.json compile_commands.json``` 
---


## Rodando o projeto:
1. Baixe as [imagens e nuvem de pontos](https://drive.proton.me/urls/6GXD9KN6ZW#i5eDGZEwj1FK);
2. Crie o diretório `/resources` na **raiz do projeto**;
    - ``` mkdir /resources && cd resources/ ```

3. Extraia as imagens em `/resources`, assim como a nuvem de pontos.
    > Caso esteja na em `.../plane-extraction/resources`:

    - ```unzip path/to/images ./ && mv path/to/point-cloud.txt ./```   

4. Confira se **todas as imagens** (IMG_5375.JPG ... IMG_5417.JPG) estão presentes no diretório `/resources/images`.
5. Confira se o **arquivo com a nuvem de pontos** (point-cloud.txt) está em `/resources`

6. Vá ao diretório **build** criado anteriormente e execute o programa;
    - `cd ../build`
    - `make`
    - `./app `

