import random
import time
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
                width=0.008,
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
    figure = None
    figureAxes = None

    def __init__(
        self,
        start_pos,
        goal_pos,
        tableShape: tuple = (100, 100),
        goalRadius: int = 2,
        obstacles: List[np.ndarray] = [],
        showGUI=False,
    ) -> None:
        self.mustRender = showGUI
        self.tableShape = tableShape
        self.state = np.array(np.zeros(self.tableShape), dtype=int)
        if len(obstacles) > 0:
            for obs in obstacles:
                print(obs)
                if obs[0] < 0 or obs[0] >= self.state.shape[0]:
                    continue
                if obs[1] < 0 or obs[1] >= self.state.shape[1]:
                    continue
                self.state[obs[0], obs[1]] = 1

        self.goal = TreeNode(*goal_pos)
        self.start = TreeNode(*start_pos)
        self.goal_radius = goalRadius

        if self.mustRender:
            self.startMapGUI()

    def isFreePos(self, x, y):
        roundedPos = int(x), int(y)

        if roundedPos[0] < 0 or roundedPos[0] >= self.state.shape[0]:
            raise ValueError(f"X out of map bounds: ({roundedPos})")
        if roundedPos[1] < 0 or roundedPos[1] >= self.state.shape[1]:
            raise ValueError(f"Y out of map bounds: ({roundedPos})")

        return self.state[roundedPos] == 0

    def hasCollision(self, from_pos: np.ndarray, to_pos: np.ndarray):
        # Gera um vetor unitário que parte do start para o end
        directionVector = Helper.getUnitVector(from_pos, to_pos)

        # Calcula o tamanho do vetor real
        distance = np.ceil(self.getDistanceBetweenPoints(from_pos, to_pos))
        for i in range(int(distance)):
            # Calcula a posição atual do vetor
            currentPos = from_pos + (directionVector * i)
            isFree = self.isFreePos(*currentPos)
            if self.mustRender:
                plt.plot(*currentPos, "yo", markersize=1.5)

            # Verifica se a posição atual do vetor é válida (sem colisão)
            if not isFree:
                return True

        return False

    def samplePos(self):
        x = random.randint(0, self.state.shape[0] - 1)
        y = random.randint(0, self.state.shape[1] - 1)
        return np.array([x, y])

    def getDistanceBetweenPoints(
        self, pos1: np.ndarray, pos2: np.ndarray, type="euclidean"
    ):

        types = {
            "euclidean": Helper.euclideanDistance,
            "manhattan": Helper.manhattanDistance,
        }

        return types[type](pos1, pos2)

    def startMapGUI(self):
        # Configurando mapa
        self.figure = plt.figure("Map")
        plt.imshow(self.state, cmap="binary")

        self.figureAxes = self.figure.gca()
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
            self.goal.getPos(),
            self.goal_radius,
            color="g",
            fill=False,
        )
        self.figureAxes.add_patch(goalRegion)

    def render(self, tree_nodes: List[TreeNode]):
        if not self.mustRender:
            return

        not_drawn_nodes = [node for node in tree_nodes if node not in self.drawnNodes]

        # Desenhando nós da arvore
        for node in not_drawn_nodes:
            self.drawnNodes.append(node)
            plt.plot(*node.getPos(), "bo", markersize=5)

        # # Ligando os nós com linhas
        for node in not_drawn_nodes:
            if node.parent is not None:
                self.figureAxes.plot(
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

    def renderFinalPath(self, finalPath: List[TreeNode]):
        if not self.mustRender:
            return

        for node in finalPath:
            plt.plot(*node.getPos(), "o", markersize=10, color="green")
            plt.plot(
                [
                    node.x,
                    node.parent.x,
                ],
                [
                    node.y,
                    node.parent.y,
                ],
                "-",
                linewidth=1,
                alpha=1,
                color="green",
            )

    def openGUI(self):
        if not self.mustRender:
            return

        plt.show()

    def saveMapFigure(self, allNodes, finalPath):
        # Save current map state
        mustRenderLastState = self.mustRender
        figureLastState = self.figure
        axLastState = self.figureAxes
        drawnNodesLastState = self.drawnNodes

        # Reset States
        self.mustRender = True
        self.figure = None
        self.figureAxes = None
        self.drawnNodes = []

        # Redraw entire map
        self.startMapGUI()
        self.render(allNodes)
        self.renderFinalPath(finalPath)

        # Save figure
        currentDateWithTime = f"{time.time()}"
        print("Current date", currentDateWithTime)
        plt.savefig(f"images/map-{currentDateWithTime}.png")

        # Return previous state
        self.mustRender = mustRenderLastState
        self.figureAxes = axLastState
        self.figure = figureLastState
        self.drawnNodes = drawnNodesLastState
