import math
import random
import time
from typing import List
from matplotlib import pyplot as plt
import numpy as np

from map import Helper, Map, TreeNode


class RRTAlgorithm:
    _nodeList: List[TreeNode] = []
    reachedGoal = False
    goalRange = 2
    reachDistance = 1
    finalPath: List[TreeNode] = []

    def __init__(self, map: Map, stepSize: int, numIterations: int) -> None:
        self.map = map
        self.max_iterations = min(numIterations, map.table_size**2)
        self.maxStepSize = stepSize
        self.num_waypoints = 0
        self.Waypoints = []

    def execute(self):
        iterations = 0

        while iterations < self.max_iterations:
            blocked_iterations = 0
            blocked_positions = []
            running = True
            startTime = time.time()

            while running:
                iterations += 1
                print(f"Iteração {iterations}")

                randomPos = map.samplePos()

                try:
                    nearestNode = (
                        self.findNearestNode(randomPos[0], randomPos[1]) or map.start
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
                    blocked_iterations += 1
                    blocked_positions.append(randomPos)
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
                f"Iterações ({iterations}) x Iterações bloqueadas ({blocked_iterations})"
            )

            plt.show()

        if not self.reachedGoal:
            print(
                f"Não foi possível encontrar o objetivo em {self.max_iterations} iterações!"
            )

    def checkFoundGoal(self, node: TreeNode):
        distanceToGoal = self.map.getDistanceBetweenPoints(
            node.getPos(), self.map.goal.getPos()
        )
        found = distanceToGoal <= self.map.goal_radius

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

    def tracePathToGoal(self, goalNode):

        if goalNode.parent is None:
            print("Found Path")
            print("Number of waypoints: ", len(self.finalPath))
            print("Path size:", len(self.finalPath) * self.maxStepSize)

            # Plot final path
            for node in self.finalPath:
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

            return

        self.finalPath.insert(0, goalNode)
        self.tracePathToGoal(goalNode.parent)


# Configurações do mapa

tilesCount = 600
precisionRadius = 20
startPoint = (0, 0)
goalPoint = (tilesCount - precisionRadius, tilesCount - precisionRadius)

obstacles = []
for i in range(int(tilesCount / 2)):
    randomPos = (random.randint(0, tilesCount), random.randint(0, tilesCount))

    # # If was at goal or nearest to start, skip
    # if (
    #     Helper.euclideanDistance(randomPos, goalPoint) < precisionRadius
    #     or Helper.euclideanDistance(randomPos, startPoint) < precisionRadius
    # ):
    #     continue

    obstacles.append(randomPos)

map = Map(startPoint, goalPoint, tilesCount, precisionRadius, obstacles)


maxIterations = 5000
rrt = RRTAlgorithm(map, precisionRadius, maxIterations)

try:
    rrt.execute()

except KeyboardInterrupt:
    print("Loop encerrado pelo usuário.")
