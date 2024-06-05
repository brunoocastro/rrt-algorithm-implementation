# %%
import math

import pandas as pd
from map import Map


# %%

SSL_VISION_INFO = {
    "ball_position": {
        "id": -1,
        "x": -826.5055541992188,
        "y": -1.11676025390625,
        "z": 0.0,
        "orientation": 0.0
    },
    "team_position": [
        {
            "id": 1,
            "x": -1499.075439453125,
            "y": 3.5936850717455116e-11,
            "z": 0.0,
            "orientation": 0.0
        },
        {
            "id": 2,
            "x": -1499.075439453125,
            "y": -1120.0,
            "z": 0.0,
            "orientation": -0.0
        },
        {
            "id": 0,
            "x": -2150.0,
            "y": -1.0179537534713745,
            "z": 0.0,
            "orientation": -0.00043125651427544653
        }
    ],
    "enemy_position": [
        {
            "id": 0,
            "x": 1497.6259765625,
            "y": 1120.0,
            "z": 0.0,
            "orientation": 3.1415927410125732
        },
        {
            "id": 1,
            "x": 1497.6259765625,
            "y": 1.688455907207509e-12,
            "z": 0.0,
            "orientation": -3.1415927410125732
        },
        {
            "id": 2,
            "x": 1497.6259765625,
            "y": -1120.0,
            "z": 0.0,
            "orientation": 3.1415927410125732
        }
    ],
    "field_size": [4500.0, 3000.0],
    "goal_size": 800.0
}

ROBOT_ID = 1
ROBOT_SIZE = 180

# %%
def get_robot_position(team_positions, robot_id):
    for robot in team_positions:
        if robot["id"] == robot_id:
            return robot["x"], robot["y"]
    return None

START_POSITION = get_robot_position(SSL_VISION_INFO["team_position"], ROBOT_ID)
GOAL_POSITION = (SSL_VISION_INFO["ball_position"]['x'],SSL_VISION_INFO["ball_position"]['y'])

def create_circle(center_x, center_y, radius=180, num_points=36):
    """
    Cria um círculo de pontos ao redor de um centro dado.

    :param center_x: Coordenada x do centro do círculo.
    :param center_y: Coordenada y do centro do círculo.
    :param radius: Raio do círculo. Padrão é 180.
    :param num_points: Número de pontos a serem gerados ao redor do círculo. Padrão é 36.
    :return: Lista de tuplas (x, y) representando os pontos ao redor do círculo.
    """
    points = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        point_x = center_x + radius * math.cos(angle)
        point_y = center_y + radius * math.sin(angle)
        points.append((int(point_x),int( point_y)))
    return points

def create_square(center_x, center_y, size = 180):
    """
    Cria um quadrado de pontos a partir de um centro dado
    """

    points = []
    startX = center_x - (size / 2)
    startY = center_y - (size / 2) 

    print("Square", startX, startY, size)

    for pointX in range(size):
        for pointY in range(size):
            points.append((int(startX + pointX), int(startY + pointY)))

    return points

def getParsedPos(pos, tableSize):
    return pos + (tableSize / 2)

def generate_obstacles(SHAPE):
    obstacles = []
    for robot in SSL_VISION_INFO["team_position"]:
        if robot['id'] == ROBOT_ID:
            continue

        posX = getParsedPos(int(robot['x']), SHAPE[0])
        posY = getParsedPos(int(robot['y']), SHAPE[1])

        print("Pos", (posX, posY))

        circle_points = create_circle(posX, posY,ROBOT_SIZE)
        for point in circle_points:
            obstacles.append(point)

    for enemy in SSL_VISION_INFO["enemy_position"]:

        posX = getParsedPos(int(enemy['x']), SHAPE[0])
        posY = getParsedPos(int(enemy['y']), SHAPE[1])

        circle_points = create_circle(posX, posY, ROBOT_SIZE)
        for point in circle_points:
            obstacles.append(point)

    return obstacles

# %%
# Default Size

SSL_FULL_SHAPE = (12000, 9000)
SSL_EL_FULL_SHAPE = (4500,3000)

showGUI = False

# %%
# SSL_FULL = Map(startPos=None, goalPos=None, tableShape=SSL_FULL_SHAPE, showGUI=showGUI)
# SSL_SMALL = Map(startPos=None, goalPos=None, tableShape=SSL_SMALL_SHAPE, showGUI=showGUI)

print("Generating map")
# SSL_EL_SMALL = Map(startPos=None, goalPos=None, tableShape=SSL_EL_SMALL_SHAPE, showGUI=showGUI)
SSL_EL_FULL = Map(startPos=None, goalPos=None, tableShape=SSL_EL_FULL_SHAPE, showGUI=showGUI)

print("Generating obstacles")
Obstacles = generate_obstacles(SSL_EL_FULL_SHAPE)
print("Obstacles generated", Obstacles)

# %%
default_path = 'grid/'

SSL_EL_FULL.saveMapAsCSV(default_path + 'ssl-el-clean.csv')
SSL_EL_FULL.setObstacles(Obstacles)

SSL_EL_FULL.saveMapAsCSV(default_path + 'ssl-el.csv')

df = pd.DataFrame(Obstacles)
df.to_csv(default_path + 'ssl-el-obstacles.csv', index=False)



