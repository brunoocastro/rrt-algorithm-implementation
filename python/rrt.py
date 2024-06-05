import math
import random
import time
from typing import List
import numpy as np
import pandas as pd
import numba

from map import Helper, Map, TreeNode


class RRTAlgorithm:
    _nodeList: List[TreeNode] = []
    reachedGoal = False
    goalRange = 2
    reachDistance = 1
    finalPath: List[TreeNode] = []

    def __init__(self, map: Map, stepSize: int, maxIterations: int) -> None:
        self.map = map
        self._nodeList = [map.start]
        self.maxIterations = maxIterations
        self.maxStepSize = stepSize

    def execute(self):
        iterations = 0

        while iterations < self.maxIterations:
            blockedIterations = 0
            blockedPositions = []
            running = True
            startTime = time.time()

            while running:
                iterations += 1
                if iterations % 100 == 0:
                    print(f"Iteração {iterations}")

                randomPos = map.samplePos()

                if randomPos[1] > self.map.state.shape[0]:
                    print("Y maior que X:", randomPos)

                try:
                    # nearestNode = (
                    #     self.findNearestNode(randomPos[0], randomPos[1]) or map.start
                    # )

                    nearestNode = (
                        self.findNearestNodeFromMatrix(randomPos[0], randomPos[1])
                        or map.start
                    )

                    newNodePosition = self.steerToPoint(nearestNode, randomPos)

                    collided = map.hasCollision(nearestNode.getPos(), newNodePosition)
                    if collided:
                        raise Exception(
                            f"Colisão entre {nearestNode.getPos()} e ({newNodePosition})"
                        )

                    self.addNode(newNodePosition[0], newNodePosition[1], nearestNode)

                    if self.reachedGoal:
                        running = False

                except Exception as e:
                    blockedIterations += 1
                    blockedPositions.append(randomPos)
                    print(f"Iteração ({iterations}) bloqueada  : {e}")

                currentNodes = self.getNodes()

                map.render(currentNodes)

            endTime = time.time()

            self.tracePathToGoal(self.getNodes()[-1])

            print(
                f"Chegou ao objetivo! Tempo de execução: {endTime - startTime} segundos"
            )
            print(f"Tamanho da árvore: {len(currentNodes)}")
            print(
                f"Tamanho do caminho: {len(self.finalPath)} = {len(self.finalPath) * self.maxStepSize} meters"
            )
            print(
                f"Iterações ({iterations}) x Iterações bloqueadas ({blockedIterations})"
            )

            # self.map.openGUI()
            self.map.saveMapFigure(self.getNodes(), self.finalPath)
            break

        if not self.reachedGoal:
            print(
                f"Não foi possível encontrar o objetivo em {self.maxIterations} iterações!"
            )

    def checkFoundGoal(self, node: TreeNode):
        distanceToGoal = self.map.getDistanceBetweenPoints(
            node.getPos(), self.map.goal.getPos()
        )
        found = distanceToGoal <= self.map.goalRadius

        if found:
            print("Reached Goal!")
            self.reachedGoal = True

        return found

    def addNode(self, x, y, parent=None):
        isFree = self.map.isFreePos(x, y)

        if not isFree:
            raise Exception(f"Posição ({x}, {y}) não está livre!")

        newNode = TreeNode(x, y, parent)

        self.checkFoundGoal(newNode)

        self._nodeList.append(newNode)

        return newNode

    def getNodes(self):
        return self._nodeList

    @numba.njit
    def findNearestNode(self, x, y):
        nearestNode = self.map.start
        nearestDistance = math.inf

        if len(self._nodeList) == 0:
            return nearestNode

        for node in self._nodeList:
            distance = self.map.getDistanceBetweenPoints(
                np.array([x, y]), node.getPos()
            )
            if distance < nearestDistance:
                nearestNode = node
                nearestDistance = distance

        return nearestNode

    @numba.njit
    def findNearestNodeFromMatrix(self, x, y):

        nodesPosition = np.array([node.getPos() for node in self._nodeList])

        # A vector with same shape of nodesPosition but with all values filled with (x,y)
        points = np.full_like(nodesPosition, (x, y))

        # The difference results in a matrix of distances
        distances = np.linalg.norm(points - nodesPosition, axis=1)

        # The index of the minimum distance
        nearestNode = self._nodeList[np.argmin(distances)]

        return nearestNode

    @numba.njit
    def steerToPoint(self, refNode: TreeNode, newPosition: np.ndarray[(2, 1)]):
        # Gera um vetor unitário que parte do start para o end
        directionVector = Helper.getUnitVector(
            refNode.getPos(), (newPosition[0], newPosition[1]), True
        )
        # Gera um offset do tamanho do passo na direção do vetor unitário
        offsetVector = self.maxStepSize * directionVector

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

    @numba.njit
    def tracePathToGoal(self, goalNode):

        if goalNode.parent is None:
            print("Found Path")
            print("Number of waypoints: ", len(self.finalPath))
            print("Path size:", len(self.finalPath) * self.maxStepSize)

            # Plot final path
            self.map.renderFinalPath(self.finalPath)

            return

        self.finalPath.insert(0, goalNode)
        self.tracePathToGoal(goalNode.parent)


# Configurações do mapa

mapShape = (4500, 3000)
precisionRadius = 150
startPoint = (100, 100)
goalPoint = (mapShape[0] - precisionRadius, mapShape[1] - precisionRadius)

obstacles = []
# obstaclesCount =mapShape[0] / 5
obstaclesCount = 5
for i in range(int(obstaclesCount)):
    # Gera uma posição aleatória
    randomPos = (
        random.randint(precisionRadius, mapShape[0] - 1),
        random.randint(precisionRadius, mapShape[1] - 1),
    )

    # Append a rectangle
    # Generate points to make a square with 10x10
    square_side = int(precisionRadius / 2)

    start_x = max(0, randomPos[0] - square_side)
    final_x = min(mapShape[0] - 1, randomPos[0] + square_side)
    start_y = max(0, randomPos[1] - square_side)
    final_y = min(mapShape[1] - 1, randomPos[1] + square_side)

    # Preencher quadrado com 1s
    for x_pos in range(start_x, final_x + 1):
        for y_pos in range(start_y, final_y + 1):
            obstacles.append((x_pos, y_pos))

showGUI = True

map = Map(startPoint, goalPoint, mapShape, precisionRadius, [], showGUI)

# Carregar os dados do arquivo CSV
# file_path = 'grid/ssl-el-obstacles.csv'  # Substitua pelo caminho do seu arquivo
# data = pd.read_csv(file_path, header=None)

# Converter os dados para uma matriz numpy
# obstacles_data = data.to_numpy()

# map.setObstacles(obstacles_data)

maxIterations = 500
rrt = RRTAlgorithm(map, precisionRadius, maxIterations)

try:
    rrt.execute()
    # map.openGUI()

    # pathPos = np.zeros(map.state.shape, dtype=int)
    # for node in rrt.getNodes():
    #     [x, y] = node.getPos()
    #     print("pos", x, y, pathPos[x, y], map.state[x, y])
    #     pathPos[x, y] += 5

    # print("PathPos", pathPos)

except KeyboardInterrupt:
    print("Loop encerrado pelo usuário.")
