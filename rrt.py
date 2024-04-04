import math
import random
from typing import List
import pygame
import numpy as np

# Cores
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)


class Map:
    gameSprites = {"goal": None, "start": None, "nodes": [], "tiles": []}

    def __init__(self, size=4, screen_size=400) -> None:
        self.size = size
        self.state = np.zeros((self.size, self.size), dtype=int)
        self.screen_size = screen_size

        # Inicialização do Pygame
        # pygame.init()
        # self.screen = pygame.display.set_mode((screen_size, screen_size))
        # pygame.display.set_caption("Mapa do Jogo")

    def isFreePos(self, x, y):
        return self.state[x, y] == 0

    def transformTileToMapPos(self, tileX, tileY):
        cell_size = self.screen_size // self.size
        x = math.ceil(tileX / cell_size)
        y = math.ceil(tileY / cell_size)
        print("Transf", x, y)
        return (y - 1, x - 1)

    def render(self):
        # print(self.state)

        return self.state
        # Desenha o mapa
        cell_size = self.screen_size // self.size
        for row in range(self.size):
            for col in range(self.size):
                tileBorder = pygame.Rect(
                    col * cell_size, row * cell_size, cell_size, cell_size
                )
                list_of_nodes = [
                    node for node in rrt.getNodes() if node.getPos() == (row, col)
                ]

                self.gameSprites["tiles"].append(tileBorder)
                pygame.draw.rect(self.screen, BLACK, tileBorder, 1)  # Adiciona borda
                if (row, col) == startPoint:
                    self.gameSprites["start"] = tileBorder
                    pygame.draw.rect(self.screen, BLUE, tileBorder)
                elif (row, col) == goalPoint:
                    self.gameSprites["goal"] = tileBorder
                    pygame.draw.rect(self.screen, GREEN, tileBorder)
                elif not self.isFreePos(row, col):
                    pygame.draw.rect(self.screen, BLACK, tileBorder)
                elif len(list_of_nodes) > 0:
                    print("Entrou", list_of_nodes)
                    node_tile = pygame.draw.circle(
                        self.screen,
                        RED,
                        [row + cell_size / 2, col + cell_size / 2],
                        cell_size / 3,
                    )
                    self.gameSprites["nodes"].append(node_tile)
                else:
                    pygame.draw.rect(self.screen, WHITE, tileBorder)


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

    def __init__(self, map: Map, start, goal) -> None:
        self.map = map
        self.startPoint = start
        self.goalPoint = goal

    def addNode(self, x, y):
        isFree = self.map.isFreePos(x - 1, y - 1)

        if not isFree:
            print(f"Posição ({x},{y}) não está livre!")
            return

        parent = self._nodeList[-1] if len(self._nodeList) > 0 else None

        newNode = TreeNode(x, y, parent)
        self._nodeList.append(newNode)

    def getNodes(self):
        return self._nodeList


# Configurações do mapa
screen_size = 400
map = Map(6)
startPoint = (0, 0)
goalPoint = (3, 3)


def getRandomPos():
    randomX = random.randint(0, map.size - 1)
    randomY = random.randint(0, map.size - 1)

    return (randomX, randomY)


rrt = RRTAlgorithm(map, startPoint, goalPoint)

# Loop principal
running = True
while running:
    # for event in pygame.event.get():
    #     if event.type == pygame.QUIT:
    #         running = False

    # # Roda o algoritmo RRT
    # try:
    #     new_random_pos = getRandomPos()
    #     print("Nova posição aleatória", new_random_pos)
    #     rrt.addNode(*new_random_pos)
    # except Exception as error:
    #     print(error)

    mapState = map.render()
    randomPos = getRandomPos()
    rrt.addNode(*randomPos)
    currentNodes = rrt.getNodes()

    nodesPos = [node.getPos() for node in currentNodes]

    # Print pretty map with:
    # 0 - Free
    # 1 - Obstacle
    # 2 - Start
    # 3 - Goal
    # 4 - Nodes
    printMap = mapState
    printMap[startPoint] = 2
    printMap[goalPoint] = 3
    printMap[nodesPos] = 4
    print(printMap)

    # pygame.display.flip()
    # lastEvent = pygame.event.wait(0)
    # print(lastEvent)

    # if lastEvent.type == pygame.MOUSEBUTTONUP:
    #     pos = pygame.mouse.get_pos()
    #     clicked_sprites = [s for s in map.gameSprites["tiles"] if s.collidepoint(pos)]
    #     print("Clicked", clicked_sprites, map.transformTileToMapPos(*pos))
    #     rrt.addNode(*map.transformTileToMapPos(*pos))
    #     print("Nodes", rrt.getNodes())


pygame.quit()
