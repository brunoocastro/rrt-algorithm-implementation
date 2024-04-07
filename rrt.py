import math
import random
import time
from typing import List
import numpy as np
import matplotlib

matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

# Cores
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)


class TreeNode:
    def __init__(self, locationX, locationY, parent=None):
        self.x = locationX
        self.y = locationY
        self.parent = parent
        if self.parent is not None:
            self.parent.addChild(self)
        self.children = []

        print("New Node", self.x, self.y, self.parent)

    def getPos(self):
        return (self.x, self.y)

    def addChild(self, child):
        self.children.append(child)


class Map:

    drawnNodes = []

    def __init__(
        self, start_pos, goal_pos, table_size=100, goal_radius=2, obs_positions=[]
    ) -> None:
        self.table_size = table_size
        self.state = np.array(np.zeros((self.table_size, self.table_size), dtype=int))
        if len(obs_positions) > 0:
            for obs in obs_positions:
                self.state[obs[0], obs[1]] = 1

        self.goal = TreeNode(*goal_pos)
        self.start = TreeNode(*start_pos)
        self.goal_radius = goal_radius

        print(self.state)
        # Configurando mapa
        self.figure = plt.figure("Map")
        plt.imshow(self.state, cmap="binary")

        self.ax = self.figure.gca()
        plt.xlabel("Eixo X $(m)$")
        plt.ylabel("Eixo Y $(m)$")

        # Renderizando o ponto de start
        plt.plot(
            *self.start.getPos(),
            "ro",
            markersize=10,
        )

        # Renderizando o ponto de chegada
        plt.plot(
            *self.goal.getPos(),
            "go",
            markersize=10,
        )

        # Renderizando região do GOAL
        goalRegion = plt.Circle(
            self.goal.getPos(), self.goal_radius, color="g", fill=False
        )
        self.ax.add_patch(goalRegion)

    def isFreePos(self, x, y):
        print("is Free", x, y, self.state.shape)

        return self.state[int(x), int(y)] == 0

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

    def samplePos(self):
        x = random.randint(0, self.state.shape[0])
        y = random.randint(0, self.state.shape[1])
        return np.array([x, y])

    def render(self, tree_nodes: List[TreeNode]):
        print("Redesenhando mapa")

        not_drawn_nodes = [node for node in tree_nodes if node not in self.drawnNodes]

        # Desenhando nós da arvore
        for node in not_drawn_nodes:
            self.drawnNodes.append(node)
            plt.plot(*node.getPos(), "bo", markersize=5)

        # # Ligando os nós com linhas
        for node in not_drawn_nodes:
            if node.parent is not None:
                self.ax.plot(
                    [
                        node.x,
                        node.parent.x,
                    ],
                    [
                        node.y,
                        node.parent.y,
                    ],
                    "b-",
                )

        plt.draw()  # Redesenha o gráfico
        plt.pause(
            0.01
        )  # Pausa a execução por um curto período de tempo para atualizar a janela


class RRTAlgorithm:
    _nodeList: List[TreeNode] = []
    reachedGoal = False
    goalRange = 2
    reachDistance = 1

    def __init__(self, map: Map, stepSize: int, numIterations: int) -> None:
        self.map = map
        self.max_iterations = min(numIterations, map.table_size**2)
        self.stepSize = stepSize
        self.num_waypoints = 0
        self.Waypoints = []

    def addNode(self, x, y):
        isFree = self.map.isFreePos(x, y)

        if not isFree:
            raise Exception(f"Posição ({x}, {y}) não está livre!")

        nearestNode = self.findNearestNode(x, y)
        print("Nearest", nearestNode)
        # if nearestNode is not None:
        #     collided = self.map.hasCollision(nearestNode.getPos(), (x, y))

        #     if collided:
        #         raise Exception(f"Colisão entre {nearestNode.getPos()} e ({x}, {y})")

        newNode = TreeNode(x, y, nearestNode)

        self._nodeList.append(newNode)

        if (x, y) == map.goal:
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
        nearestNode = self.map.start
        nearestDistance = math.inf

        if len(self._nodeList) == 0:
            return nearestNode

        for node in self._nodeList:
            distance = self.getDistanceBetweenPoints((x, y), node.getPos())
            if distance < nearestDistance:
                nearestNode = node
                nearestDistance = distance

        print("Nearest Node", nearestNode, nearestDistance)

        return nearestNode

    def getUnitVector(self, start: tuple, end: tuple, drawGeneratedVector=False):
        vector = np.array([*end]) - np.array([*start])
        length = np.linalg.norm(vector)

        if drawGeneratedVector:
            plt.quiver(
                *start,
                *(vector / length),
                angles="xy",
                scale_units="xy",
                scale=1,
                color="r",
                width=0.005,
            )

        return vector / length

    def steerToPoint(self, refNode: TreeNode, newPosition: np.ndarray[(2, 1)]):
        # Gera um vetor unitário que parte do start para o end
        directionVector = self.getUnitVector(
            refNode.getPos(), (newPosition[0], newPosition[1]), True
        )
        # Gera um offset do tamanho do passo na direção do vetor unitário
        offsetVector = self.stepSize * directionVector

        point = np.array(
            [np.ceil(refNode.x + offsetVector[0]), np.ceil(refNode.y + offsetVector[1])]
        )

        # Se o passo dado estiver fora do mapa, usa o limite do mapa
        if point[0] >= self.map.state.shape[1]:
            point[0] = self.map.state.shape[1]
        if point[1] >= self.map.state.shape[0]:
            point[1] = self.map.state.shape[0]

        # Gerou um ponto dentro do mapa, a uma distancia "step_size" do
        # ponto inicial na direção do vetor fornecido
        return point

    def tracePathToGoal(self, goal):
        pass


# Configurações do mapa
obstacles = [
    (1, 1),
    (3, 3),
]
goalRadius = 3
amount_of_tiles = 10
startPoint = (0, 0)
goalPoint = (10, 10)
map = Map(startPoint, goalPoint, 20, goalRadius, obstacles)


rrt = RRTAlgorithm(map, 2, 500)

# Loop principal
iterations = 0
blocked_iterations = 0
blocked_positions = []
running = True
while running:
    iterations += 1

    randomPos = map.samplePos()

    try:
        print("POS", randomPos, map.state[randomPos[0], randomPos[1]])
        nearestNode = rrt.findNearestNode(randomPos[0], randomPos[1]) or map.start

        newNode = rrt.steerToPoint(nearestNode, randomPos)
        print("new node", newNode)
        rrt.addNode(newNode[0], newNode[1])

        if rrt.reachedGoal:
            running = False

    except Exception as e:
        blocked_iterations += 1
        blocked_positions.append(randomPos)
        print(f"Iteração ({iterations}) bloqueada  : {e}")

    currentNodes = rrt.getNodes()

    map.render(currentNodes)
    time.sleep(1)

plt.show()

print("Chegou ao objetivo!")
print(f"Nós da árvore ({len(currentNodes)}) : {currentNodes}")
print(f"Iterações ({iterations}) : {currentNodes}")
print(f"Iterações bloqueadas ({blocked_iterations}) : {blocked_positions}")
