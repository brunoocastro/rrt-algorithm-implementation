import math
import random
import time
from typing import List
import numpy as np
from matplotlib import pyplot as plt

# Cores
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)


class Map:
    def __init__(
        self, start_pos, goal_pos, table_size=4, screen_size=400, obs_positions=[]
    ) -> None:
        self.table_size = table_size
        self.state = np.zeros((self.table_size, self.table_size), dtype=int)
        if len(obs_positions) > 0:
            for obs in obs_positions:
                self.state[obs[0], obs[1]] = 1

        self.goal_pos = goal_pos
        self.start_pos = start_pos
        self.screen_size = screen_size

        self.tile_size = screen_size // table_size

        self.fig, self.ax = plt.subplots()

    def isFreePos(self, x, y):
        return self.state[x, y] == 0

    def hasCollision(self, from_pos, to_pos):
        """
        Verifica se há obstáculos entre duas posições no mapa.

        Args:
            from_pos (tuple): Posição de origem (x, y).
            to_pos (tuple): Posição de destino (x, y).

        Returns:
            bool: True se há uma colisão (obstáculo) entre as posições, False caso contrário.
        """
        from_x, from_y = from_pos
        to_x, to_y = to_pos
        dx = abs(to_x - from_x)
        dy = abs(to_y - from_y)
        step_x = -0.01 if from_x > to_x else 0.01
        step_y = -0.01 if from_y > to_y else 0.01
        error = dx - dy

        while True:
            if not self.isFreePos(from_x, from_y):
                return True  # Há um obstáculo entre as posições
            if from_x == to_x and from_y == to_y:
                break
            error2 = 2 * error
            if error2 > -dy:
                error -= dy
                from_x += step_x
            if error2 < dx:
                error += dx
                from_y += step_y

        return False  # Não há obstáculos entre as posições

    def render(self, tree_nodes):
        self.ax.clear()

        # Renderizando o mapa com bordas
        for i in range(self.table_size):
            for j in range(self.table_size):
                # Definindo cor do fundo
                color = "white" if self.state[i, j] == 0 else "black"
                # Adicionando retângulo com borda preta e centralizando o nó da árvore
                rect = plt.Rectangle(
                    (j, i),
                    self.tile_size,
                    self.tile_size,
                    color=color,
                    ec="black",
                    lw=2,
                )
                self.ax.add_patch(rect)
                if (i, j) in tree_nodes:
                    self.ax.plot(
                        j + self.tile_size / 2,
                        i + self.tile_size / 2,
                        "ro",
                        markersize=5,
                    )  # Nó da árvore

        # Renderizando o ponto de start
        self.ax.plot(self.start_pos[1], self.start_pos[0], "bo", markersize=10)

        # Renderizando o ponto de chegada
        self.ax.plot(self.goal_pos[1], self.goal_pos[0], "go", markersize=10)

        # Ligando os nós com linhas vermelhas
        for node in tree_nodes:
            if node.parent is not None:
                parent_pos = node.parent.getPos()
                print("parent pos", parent_pos)
                self.ax.plot(
                    [
                        node.y + (self.tile_size / 2),
                        parent_pos[1] + (self.tile_size / 2),
                    ],
                    [
                        node.x + (self.tile_size / 2),
                        parent_pos[0] + (self.tile_size / 2),
                    ],
                    "r-",
                )

        plt.xlim(0, self.table_size)
        plt.ylim(0, self.table_size)
        plt.gca().set_aspect("equal", adjustable="box")
        plt.pause(0.01)
        plt.draw()


class TreeNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        if self.parent is not None:
            self.parent.addChild(self)
        self.children = []

        print("New Node", self.x, self.y, self.parent)

    def getPos(self):
        return (self.x, self.y)

    def addChild(self, child):
        self.children.append(child)


class RRTAlgorithm:
    _nodeList: List[TreeNode] = []
    reachedGoal = False
    goalRange = 2
    reachDistance = 1

    def __init__(self, map: Map) -> None:
        self.map = map

    def addNode(self, x, y):
        isFree = self.map.isFreePos(x - 1, y - 1)

        if not isFree:
            raise Exception(f"Posição ({x}, {y}) não está livre!")

        nearestNode = self.findNearestNode(x, y)

        if nearestNode is not None:
            collided = self.map.hasCollision(nearestNode.getPos(), (x, y))

            if collided:
                raise Exception(f"Colisão entre {nearestNode.getPos()} e ({x}, {y})")

        newNode = TreeNode(x, y, nearestNode)

        self._nodeList.append(newNode)

        if (x, y) == map.goal_pos:
            self.reachedGoal = True

    def getNodes(self):
        return self._nodeList

    def getDistanceBetweenPoints(self, pos1, pos2):
        x1, y1 = pos1
        x2, y2 = pos2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # return (x2 - x1) ** 2 + (y2 - y1) ** 2  # Retorna a distância ao quadrado (menor distância)
        # return abs(x2 - x1) + abs(y2 - y1)  # Retorna a distância absoluta (maior distância)
        # return max(abs(x2 - x1), abs(y2 - y1))  # Retorna a distância máxima (maior distância)
        # return min(abs(x2 - x1), abs(y2 - y1))  # Retorna a distância mínima (menor distância)

    def findNearestNode(self, x, y):
        nearestNode = None
        nearestDistance = math.inf

        if len(self._nodeList) == 0:
            return nearestNode, nearestDistance

        for node in self._nodeList:
            distance = self.getDistanceBetweenPoints((x, y), node.getPos())
            if distance < nearestDistance:
                nearestNode = node
                nearestDistance = distance

        return nearestNode


# Configurações do mapa
obstacles = [
    (1, 1),
    (3, 3),
]
screen_size = 400
amount_of_tiles = 10
startPoint = (0, 0)
goalPoint = (5, 5)
map = Map(startPoint, goalPoint, 10, screen_size, obstacles)


def getRandomPos():
    randomX = random.randint(0, map.table_size - 1)
    randomY = random.randint(0, map.table_size - 1)

    return (randomX, randomY)


rrt = RRTAlgorithm(map)

# Loop principal
iterations = 0
blocked_iterations = 0
blocked_positions = []
running = True
while running:
    iterations += 1

    randomPos = getRandomPos()

    try:
        rrt.addNode(*randomPos)

        if rrt.reachedGoal:
            running = False

    except Exception as e:
        blocked_iterations += 1
        blocked_positions.append(randomPos)
        print(f"Iteração ({iterations}) bloqueada  : {e}")

    currentNodes = rrt.getNodes()

    map.render(currentNodes)
    time.sleep(0.5)

print("Chegou ao objetivo!")
print(f"Nós da árvore ({len(currentNodes)}) : {currentNodes}")
print(f"Iterações ({iterations}) : {currentNodes}")
print(f"Iterações bloqueadas ({blocked_iterations}) : {blocked_positions}")
