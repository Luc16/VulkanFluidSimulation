# Simulações de fluido utilizando Vulkan

Esse repositório possui diversos algoritmos de simulação de fluidos implementados utilizando
a API Vulkan. As simulações mais relevantes tem seus próprios READMEs explicando suas particularidades
A maior parte do códio base de Vulkan pode ser encontrado no diretório [lib](src/lib).


## Simulações Relevantes

### [Simulações com Grade](src/grid)

Essas são simulações de fluidos utilizando grades para representar os atributos do fluido, 
como densidade e pressão.

<p align="center">
  <img width="50%" height="50%" src="https://github.com/Luc16/VulkanFluidSimulation/assets/33912482/d2a46d67-23d1-4e15-a922-cae68596c805">
</p>

### [Simulações com SPH](src/SPH)

Essas são simulações baseadas na técnica da Hidrodinâmica de Partículas Suavizadas, a qual
utiliza partículas para aproximar o comportamento de fluidos.

## Dependências

Para rodar o projeto, é necessário ter instalado algumas dependências, sendo elas: 

- Vulkan
- glfw3
- CMake >= 3.24.0

## Rodando o projeto

Os executáveis podem ser gerados utilizando CMake. Em Linux pode-se utilizar esses comandos
a partir da pasta do projeto.

```
cmake -Bbuild
cd build
make
```

As vezes é útil gerar os executáveis em modo `release`. Isso faz com que o código seja mais 
rápido e acaba sendo melhor para a maioria das simulações. No entanto demora mais tempo para 
compilar. Isso pode ser feito com:

```
cmake -Bbuild-release -DCMAKE_BUILD_TYPE=Release
cd build-release
make
```

Todos os executáveis disponíveis pode ser vistos utilizando o comando `ls`. Para rodar
um executável basta utilizar o comando:

```
./executable
```

## Licença
Esse projeto pode ser distribuido utilizando a [licença MIT](LICENSE.md).