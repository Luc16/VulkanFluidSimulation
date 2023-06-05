# Simulação de fluido com grades
[![pt-br](https://img.shields.io/badge/LEIAME-PT--BR-A3BE8C.svg?style=for-the-badge)](README-ptbr.md)

Essa é uma simulação de fluidos utilizando grades. A ideia principal é utilizar uma
grade para simular os atributos do fluido como densidade e velocidade. Os algoritmos para 
atualizar o fluido são baseados nas equações de Navier-Stokes.

Existem duas simulações nesse diretório uma sendo 2D e outra sendo 3D. A versão 2D está
mais polida e para rodá-la é recomendado utilizar o modo `release` 


## Controles

Para simulação 2D pode-se clickar o mouse para criar fluido e move-lo para mudar as velocidades.
Em `wallmode` pode-se utilizar o mouse para criar as paredes.

Para a simulação 3D:

| Tecla   | Ação                |
|---------|---------------------|
| `w`     | move para frente    |
| `s`     | move para trás      |
| `a`     | move para esquerda  |
| `d`     | move para direita   |
| `q`     | move para cima      |
| `e`     | move para baixo     |
| `Setas` | rotacionam a câmera |


## Referencias

As ideias e conceitos explorados nessa simulação são baseadas nos artigos (em inglês):

- **Stable Fluids** por Jos Stam
  <br>https://pages.cs.wisc.edu/~chaol/data/cs777/stam-stable_fluids.pdf
- **Real-Time Fluid Dynamics for Games** por Jos Stam
  <br> http://graphics.cs.cmu.edu/nsp/course/15-464/Fall09/papers/StamFluidforGames.pdf

## Rodando o projeto

Para rodar essa simulação leia o [README do projeto](../../README.md)


## Licença
Esse projeto pode ser distribuido utilizando a [licença MIT](LICENSE.md).