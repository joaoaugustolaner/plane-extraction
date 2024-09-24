# Trabalho de Conclusão de Curso

### Aluno: 
- João Augusto Tonial laner

### Orientador:
- Leandro Tonietto

Esse projeto cria um panorama a partir de fotos tiradas do local e processa uma nuvem de pontos
correlacionando cada pixel da imagem com um respectivo ponto. O método de escolha foi o ponto mais próximo.

---
## Estrura do Projeto:

```
├── CMakeLists.txt
├── README.md
├── include
│   ├── CloudPointProcessor.hpp
│   └── Stitcher.hpp
├── resources
│   ├── images
│   │   ├── IMG_5375.JPG
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   ├── .
│   │   └── IMG_5417.JPG
│   ├── pano
│   │   └── panorama.jpg
│   └── point-cloud.txt
└── src
    ├── CloudPointProcessor.cpp
    ├── Main.cpp
    └── Stitcher.cpp
``` 
---
## Pré requisitos:

- **CMake 3.10+**
- **C++ 17** 
- **OpenCV 4.10**

---
## Instalação:

1. Clone o repositório para seu diretório local:
    `git clone https://gitlab.com/joaoaugustolaner/tc.git && cd tc/`

2. Crie um diretório `build` na raiz do projeto clonado:
    `mkdir build/ && cd build/`

3. Execute o **CMakeLists.txt** para gerar o make file e então execute o comando make:
    `cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..`
    `make`
4. Crie um link simbólico na **raiz do projeto** para que a IDE possa achar com mais facilidade, caso necesário.
    `ln -s build/compile_commands.json compile_commands.json` 
---

## Rodando o projeto:
- Baixe as [imagens e nuvem de pontos](https://drive.proton.me/urls/A8XBF53KSM#DcBf37BM8kmAa).
- Extraia as imagens em `/resources`, assim como a nuvem de pontos.

- Confira se **todas as imagens** (IMG_5375.JPG ... IMG_5417.JPG) estão presentes no diretório `/resources/images`.
- Confira se o **arquivo com a nuvem de pontos**  está em `/resources`

- Vá ao diretório **build** `cd build/` e digite no seu terminal: `./TC`
