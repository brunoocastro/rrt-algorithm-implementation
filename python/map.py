import random
import time
from typing import List, Type
import matplotlib
import numpy as np
from matplotlib import pyplot as plt
import pandas as pd
import numba

matplotlib.use("TkAgg")


class Helper:
    @staticmethod
    def getUnitVector(start: np.ndarray, end: np.ndarray, drawGeneratedVector=False):
        vector = end - start
        length = np.linalg.norm(vector)

        # Check if value is not Nan or invalid
        unitVector = vector / length
        if np.isnan(unitVector).any():
            print("Deu nan o vetor", unitVector)
            return np.array([0, 0])

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

    goal = None
    start = None

    def __init__(
        self,
        startPos: tuple = (0, 0),
        goalPos: tuple = (97, 97),
        tableShape: tuple = (100, 100),
        goalRadius: int = 2,
        obstacles: List[np.ndarray] = [],
        showGUI=False,
    ) -> None:
        self.mustRender = showGUI
        self.tableShape = tableShape
        self.state = np.array(np.zeros(self.tableShape), dtype=int)

        self.setObstacles(obstacles)

        if goalPos is not None:
            self.setGoal(goalPos)

        if startPos is not None:
            self.setStart(startPos)

        self.goalRadius = goalRadius

        if self.mustRender:
            self.redrawBaseMap()

    def setObstacles(self, obstacles: List[np.ndarray]):
        self.obstacles = obstacles

        if len(obstacles) > 0:
            for obs in obstacles:
                if obs[0] < 0 or obs[0] >= self.state.shape[0]:
                    continue
                if obs[1] < 0 or obs[1] >= self.state.shape[1]:
                    continue
                self.state[obs[0], obs[1]] = 1

        if self.mustRender:
            self.redrawBaseMap()

    def setGoal(self, goal_pos, goalRadius=2):
        self.goal = TreeNode(*goal_pos)
        self.goalRadius = goalRadius

        if self.mustRender:
            self.redrawBaseMap()

    def setStart(self, start_pos):
        self.start = TreeNode(*start_pos)

        if self.mustRender:
            self.redrawBaseMap()

    @numba.njit
    def isFreePos(self, x, y):
        roundedPos = int(x), int(y)

        if roundedPos[0] < 0 or roundedPos[0] >= self.state.shape[0]:
            raise ValueError(f"X out of map bounds: ({roundedPos})")
        if roundedPos[1] < 0 or roundedPos[1] >= self.state.shape[1]:
            raise ValueError(f"Y out of map bounds: ({roundedPos})")

        return self.state[roundedPos] == 0

    @numba.njit
    def hasCollision(self, from_pos: np.ndarray, to_pos: np.ndarray):
        print(f"From {from_pos} to {to_pos}")

        pos_between_points = [
            (x, y)
            for x in range(int(from_pos[0]), int(to_pos[0] + 1))
            for y in range(int(from_pos[1]), int(to_pos[0] + 1))
        ]

        for pos in pos_between_points:
            isFree = self.isFreePos(*pos)
            print(f"Checking collision in {pos} = {isFree}")

            if self.mustRender:
                if not isFree:
                    plt.plot(*pos, "ro", markersize=8)
                else:
                    plt.plot(*pos, "yo", markersize=3)

            if not isFree:
                return True

        return False

    def samplePos(self):
        x = random.randint(0, self.state.shape[0] - 1)
        y = random.randint(0, self.state.shape[1] - 1)
        return np.array([x, y])

    @numba.njit
    def getDistanceBetweenPoints(
        self, pos1: np.ndarray, pos2: np.ndarray, type="euclidean"
    ):

        types = {
            "euclidean": Helper.euclideanDistance,
            "manhattan": Helper.manhattanDistance,
        }

        return types[type](pos1, pos2)

    def redrawBaseMap(self):
        # Limpando mapa caso já exista
        plt.clf()

        mapAdjustGap = 0.5

        # Configurando mapa
        self.figure = plt.figure("Map")

        if self.goal is not None:
            goalPos = self.goal.getPos()
            goalRegionX = [
                x
                for x in range(
                    goalPos[0] - self.goalRadius, goalPos[0] + self.goalRadius + 1
                )
            ]
            goalRegionY = [
                y
                for y in range(
                    goalPos[1] - self.goalRadius, goalPos[1] + self.goalRadius + 1
                )
            ]
            goalRegionSet = [(x, y) for x in goalRegionX for y in goalRegionY]

        for i in range(self.tableShape[0]):
            for j in range(self.tableShape[1]):
                alpha = 1
                if self.state[i, j] == 1:
                    cor = "black"
                elif self.goal is not None and (i, j) in goalRegionSet:
                    cor = "green"
                    alpha = 0.5
                else:
                    cor = "white"

                y = j - mapAdjustGap
                x = i - mapAdjustGap
                plt.fill(
                    [y, y + 1, y + 1, y],
                    [x, x, x + 1, x + 1],
                    color=cor,
                    alpha=alpha,
                    edgecolor="black",
                )

        self.figureAxes = self.figure.gca()
        plt.xlabel("Eixo X $(m)$")
        plt.ylabel("Eixo Y $(m)$")

        # Configurando limites dos eixos para centrar os pontos de início e fim nos quadrados
        plt.xlim(0 - mapAdjustGap, self.tableShape[0] - mapAdjustGap)
        plt.ylim(0 - mapAdjustGap, self.tableShape[1] - mapAdjustGap)

        # Definindo os ticks dos eixos x e y para mostrar apenas valores inteiros
        plt.xticks(range(self.tableShape[0]))
        plt.yticks(range(self.tableShape[1]))

        if self.start is not None:
            # Renderizando o ponto de start
            plt.plot(
                *self.start.getPos(),
                "ro",
                markersize=10,
            )

        if self.goal is not None:

            # Renderizando o ponto de chegada
            plt.plot(
                *self.goal.getPos(),
                "go",
                markersize=10,
            )

            # Renderizando região do GOAL
            goalRegion = plt.Circle(
                self.goal.getPos(),
                self.goalRadius + mapAdjustGap,
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

    def saveScreenshot(self, figPathWithName=f"images/map-{time.time()}"):
        if not self.mustRender:
            raise Exception(
                "Screenshots requires a initialized GUI. Rerun with mustRender equals True"
            )

        plt.savefig(f"{figPathWithName}.png")

    def saveMapFigure(self, allNodes, finalPath, cleanFig=False):
        if not self.mustRender or cleanFig:

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
            self.redrawBaseMap()
            self.render(allNodes)
            self.renderFinalPath(finalPath)

        # Save figure
        currentDateWithTime = f"{time.time()}"
        print("Current date", currentDateWithTime)
        plt.savefig(f"images/map-{currentDateWithTime}.png")

        if not self.mustRender or cleanFig:
            # Return previous state
            self.mustRender = mustRenderLastState
            self.figureAxes = axLastState
            self.figure = figureLastState
            self.drawnNodes = drawnNodesLastState

    def saveMapAsCSV(self, path):
        df = pd.DataFrame(self.state)

        print(df.shape, df.sample())

        df.to_csv(path)
