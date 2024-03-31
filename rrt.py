import random
import pygame
import numpy as np

# Inicialização do Pygame
pygame.init()
screen_size = 400
screen = pygame.display.set_mode((screen_size, screen_size))
pygame.display.set_caption("Mapa do Jogo")


class Map:
    def __init__(self, size=4) -> None:
        self.size = size
        self.state = np.zeros((self.size, self.size), dtype=int)

    def isFreePos(self, x, y):
        print(self.state[x, y], self.state[x, y] == 0)
        return self.state[x, y] == 0


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


def getRandomPos():
    randomX = random.randint(0, map.size)
    randomY = random.randint(0, map.size)

    return (randomX, randomY)


class RRTAlgorithm:
    def __init__(self, start, goal) -> None:
        pass


# Configurações do mapa
map = Map(6)
startPoint = (0, 0)
goalPoint = (3, 3)  # Note que o índice é 3, pois o tamanho do mapa é 4x4

# Cores
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

nodeList = [TreeNode(*startPoint)]

# Loop principal
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Desenha o mapa
    cell_size = screen_size // map.size
    for row in range(map.size):
        for col in range(map.size):
            rect = pygame.Rect(col * cell_size, row * cell_size, cell_size, cell_size)
            pygame.draw.rect(screen, BLACK, rect, 10)  # Adiciona borda
            if (row, col) == startPoint:
                pygame.draw.rect(screen, BLUE, rect)
            elif (row, col) == goalPoint:
                pygame.draw.rect(screen, GREEN, rect)
            elif not map.isFreePos(row, col):
                pygame.draw.rect(screen, BLACK, rect)
            elif len([node.getPos() == (row, col) for node in nodeList]) > 0:
                pygame.draw.circle(
                    screen,
                    RED,
                    [row + cell_size / 2, col + cell_size / 2],
                    cell_size / 3,
                )
            else:
                pygame.draw.rect(screen, WHITE, rect)

    try:
        new_random_pos = getRandomPos()
        print(new_random_pos)
        if map.isFreePos(*new_random_pos):
            newNode = TreeNode(*new_random_pos)
            nodeList.append(newNode)
        else:
            print("Posição ocupada!")
    except Exception as error:
        print(error)

    pygame.display.flip()


pygame.quit()
