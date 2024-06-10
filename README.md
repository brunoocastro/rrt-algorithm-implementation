# Description

[RRT (Rapidly exploring random tree)](https://en.wikipedia.org/wiki/Rapidly_exploring_random_tree#:~:text=A%20rapidly%20exploring%20random%20tree,building%20a%20space%2Dfilling%20tree.) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree.

On this repository we will implement it in a different ways, using ```Python and C++

# Pseudo-code algorithm

For a general configuration space C, the algorithm in pseudocode is as follows:

```pseudocode
Algorithm BuildRRT
    Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
    Output: RRT graph G

    G.init(qinit)
    for k = 1 to K do
        qrand ← RAND_CONF()
        qnear ← NEAREST_VERTEX(qrand, G)
        qnew ← NEW_CONF(qnear, qrand, Δq)
        G.add_vertex(qnew)
        G.add_edge(qnear, qnew)
    return G
```

"←" denotes assignment. For instance, "largest ← item" means that the value of the largest changes to the value of the item.

"return" terminates the algorithm and outputs the following value.

# Implementations

We already implemented pure RRT in `Python, as you can [see here](/`python/README.md)

We also [got a C++ implementation](https://github.com/nikhilchandak/Rapidly-Exploring-Random-Trees) and changed it to solve our problem. You can [see it here](/c++/README.md)

## About

In this project, we did a comparison between the C++ and ```Python implementations of a Path Planning algorithm called Rapidly-exploring Random Trees (RRTs). Our approach aims to validate the best implementation of the same algorithm in a competitive environment, where the competition objectives depend on the algorithm's performance.

We gonna use these algorithms to manage the navigation of the robots from a Small Size Soccer (SSL), and the first part of the navigation is the path planning. If this step takes a lot of time to execute, the match goal must be lost. So, we aim to find the best one with a competitive performance.

# Environments

The environment is an amount of team and enemy robot positions. The Field has 4,5m x 3m, translated by the vision system, it extracts the robot's positions in millimeters, being a grid of 4500x3000 position matrix.

We created three strategic environments to compare the algorithm performance, based on common match states that will be solved in the real matches. The robot position is described by 4 params:

- ID
- Position X
- Position Y
- Orientation (compared by the field center line)

With this, we gonna receive from the SSL-Vision system as the above ones, and we gonna use this to compare the algorithm performance. The start point is the position of the teammate robot with ID = 0. Used goal points will be available above team configurations. The positions are described from -150 to 150 in the Y axis, being the (0,0) a reference to the central point of the field.

### Configurações de Partida de Futebol - RoboCup SSL

#### Configuração 1: 1-2 (Goleiro-Defesa-Ataque)

```python
# Configuração 1: 1-2 (Goleiro-Defesa-Ataque)
team = [
    Robot(0, -225, 0, 0.0),   # Goleiro
    Robot(1, -100, -75, 0.0), # Defesa Esquerda
    Robot(2, -100, 75, 0.0)   # Defesa Direita
]

enemies = [
    Robot(0, 100, -75, 0.0),  # Ataque Esquerda
    Robot(1, 100, 75, 0.0),   # Ataque Direita
    Robot(2, 225, 0, 0.0)     # Goleiro
]
```

####

Configuração 2: 2-1 (Defesa-Meio-Ataque)

```python
# Configuração 2: 2-1 (Defesa-Meio-Ataque)
team = [
    Robot(0, -225, -75, 0.0), # Defesa Esquerda
    Robot(1, -225, 75, 0.0),  # Defesa Direita
    Robot(2, -100, 0, 0.0)    # Meio
]

enemies = [
    Robot(0, 100, 0, 0.0),    # Meio
    Robot(1, 225, -75, 0.0),  # Ataque Esquerda
    Robot(2, 225, 75, 0.0)    # Ataque Direita
]
```

#### Configuração 3: 1-1-1 (Goleiro-Meio-Ataque)

```python
# Configuração 3: 1-1-1 (Goleiro-Meio-Ataque)
team = [
    Robot(0, -225, 0, 0.0),   # Goleiro
    Robot(1, -150, 0, 0.0),   # Meio
    Robot(2, -75, 0, 0.0)     # Ataque
]

enemies = [
    Robot(0, 75, 0, 0.0),     # Meio
    Robot(1, 150, 0, 0.0),    # Ataque
    Robot(2, 225, 0, 0.0)     # Goleiro
]
```

####

Configuração 4: 2-0-1 (Defesa-Ataque)

```python
# Configuração 4: 2-0-1 (Defesa-Ataque)
team = [
    Robot(0, -225, -75, 0.0), # Defesa Esquerda
    Robot(1, -225, 75, 0.0),  # Defesa Direita
    Robot(2, -100, 0, 0.0)    # Ataque
]

enemies = [
    Robot(0, 100, 0, 0.0),    # Meio
    Robot(1, 225, -75, 0.0),  # Ataque Esquerda
    Robot(2, 225, 75, 0.0)    # Ataque Direita
]
```

####

Configuração 5: 0-2-1 (Meio-Ataque)

```python
# Configuração 5: 0-2-1 (Meio-Ataque)
team = [
    Robot(0, -150, -75, 0.0), # Meio Esquerda
    Robot(1, -150, 75, 0.0),  # Meio Direita
    Robot(2, -75, 0, 0.0)     # Ataque
]

enemies = [
    Robot(0, 75, 0, 0.0),     # Meio
    Robot(1, 150, -75, 0.0),  # Meio Esquerda
    Robot(2, 150, 75, 0.0)    # Meio Direita
]
```
