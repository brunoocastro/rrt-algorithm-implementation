import random
from typing import List, Type
import matplotlib
import numpy as np
from matplotlib import pyplot as plt

matplotlib.use("TkAgg")


class Helper:
    @staticmethod
    def getUnitVector(start: np.ndarray, end: np.ndarray, drawGeneratedVector=False):
        vector = end - start
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

    # Definition of different measurement types:
    @staticmethod
    def euclideanDistance(start: np.ndarray, end: np.ndarray):
        return np.linalg.norm(start - end)

    @staticmethod
    def manhattanDistance(start: np.ndarray, end: np.ndarray):
        return np.sum(np.abs(start - end))


class TreeNode:
    def __init__(
        self, locationX: np.int64, locationY: np.int64, parent: Type["TreeNode"] = None
    ):
        self.x = locationX
        self.y = locationY
        self.parent = parent
        if self.parent is not None:
            self.parent.addChild(self)
        self.children = []

    def getPos(self):
        return np.array([self.x, self.y])

    def addChild(self, child):
        self.children.append(child)


class Map:

    drawnNodes = []

    def __init__(
        self,
        start_pos,
        goal_pos,
        table_size: int = 100,
        goal_radius: int = 2,
        obs_positions: List[np.ndarray] = [],
    ) -> None:
        self.table_size = table_size
        self.state = np.array(np.zeros((self.table_size, self.table_size), dtype=int))
        if len(obs_positions) > 0:
            for obs in obs_positions:
                print(obs)
                if obs[0] < 0 or obs[0] >= self.state.shape[0]:
                    continue
                if obs[1] < 0 or obs[1] >= self.state.shape[1]:
                    continue
                self.state[obs[0], obs[1]] = 1

        self.goal = TreeNode(*goal_pos)
        self.start = TreeNode(*start_pos)
        self.goal_radius = goal_radius

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
        roundedPos = int(np.round(x, decimals=1)), int(np.round(y, decimals=1))

        if roundedPos[0] < 0 or roundedPos[0] >= self.state.shape[0]:
            raise ValueError("X out of map bounds")
        if roundedPos[1] < 0 or roundedPos[1] >= self.state.shape[1]:
            raise ValueError("Y out of map bounds")

        return self.state[roundedPos] == 0

    def hasCollision(self, from_pos: np.ndarray, to_pos: np.ndarray, resolution=0.1):
        # Gera um vetor unitário que parte do start para o end
        directionVector = Helper.getUnitVector(from_pos, to_pos)

        # Calcula o tamanho do vetor real
        distance = np.ceil(self.getDistanceBetweenPoints(from_pos, to_pos))

        for i in range(int(distance * (1 / resolution))):
            # Calcula a posição atual do vetor
            currentPos = from_pos + (directionVector * i * resolution)
            plt.plot(*currentPos, "yo", markersize=1)
            isFree = self.isFreePos(*currentPos)

            # Verifica se a posição atual do vetor é válida (sem colisão)
            if not isFree:
                return True

        return False

    def samplePos(self):
        x = random.randint(0, self.state.shape[0])
        y = random.randint(0, self.state.shape[1])
        return np.array([x, y])

    def getDistanceBetweenPoints(
        self, pos1: np.ndarray, pos2: np.ndarray, type="euclidean"
    ):

        types = {
            "euclidean": Helper.euclideanDistance,
            "manhattan": Helper.manhattanDistance,
        }

        return types[type](pos1, pos2)

    def render(self, tree_nodes: List[TreeNode]):
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
