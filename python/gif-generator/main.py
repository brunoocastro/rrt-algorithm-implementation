import imageio
import pygame
import sys
import math

# Estrutura de dados fornecida
configuracao1 = {
    "team_position": [
        {"id": 0, "x": -2250.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
        {"id": 1, "x": -1000.0, "y": -750.0, "z": 0.0, "orientation": 0.0},
        {"id": 2, "x": -1000.0, "y": 750.0, "z": 0.0, "orientation": 0.0},
    ],
    "enemy_position": [
        {"id": 0, "x": 1000.0, "y": -750.0, "z": 0.0, "orientation": 0.0},
        {"id": 1, "x": 1000.0, "y": 750.0, "z": 0.0, "orientation": 0.0},
        {"id": 2, "x": 2250.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
    ],
    "field_size": [4500.0, 3000.0],
    "goal_size": 800.0,
}


# Função para visualizar a configuração
def visualize(config):
    pygame.init()

    # Configurações da tela
    field_width, field_height = config["field_size"]
    padding = 50  # Padding nas bordas
    screen_width, screen_height = (
        900 + 2 * padding,
        600 + 2 * padding,
    )  # Tamanho da tela com padding
    scale_x = (screen_width - 2 * padding) / field_width
    scale_y = (screen_height - 2 * padding) / field_height

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Visualizador de Futebol de Robôs SSL")

    # Cores
    white = (255, 255, 255)
    red = (255, 0, 0)
    black = (0, 0, 0)
    green = (0, 255, 0)
    blue = (0, 0, 255)

    # Função para desenhar um robô
    def draw_robot(x, y, color):
        radius = 18
        pygame.draw.circle(screen, color, (int(x), int(y)), radius)

    # Função para desenhar uma trajetória
    def draw_trajectory(trajectory):
        for i in range(len(trajectory) - 1):
            # Desenhar pontos da trajetória em verde
            x1 = padding + (trajectory[i][0] + field_width / 2) * scale_x
            y1 = padding + (field_height / 2 - trajectory[i][1]) * scale_y
            pygame.draw.circle(screen, (0, 255, 0), (int(x1), int(y1)), 5)

            x2 = padding + (trajectory[i + 1][0] + field_width / 2) * scale_x
            y2 = padding + (field_height / 2 - trajectory[i + 1][1]) * scale_y

            # Desenhar a seta de um ponto ao próximo
            pygame.draw.line(screen, white, (x1, y1), (x2, y2), 2)

            # Desenhar a cabeça da seta
            angle = math.atan2(y2 - y1, x2 - x1)
            arrow_head_length = 10
            arrow_angle = math.pi / 6  # Ângulo da cabeça da seta
            arrow_x1 = x2 - arrow_head_length * math.cos(angle - arrow_angle)
            arrow_y1 = y2 - arrow_head_length * math.sin(angle - arrow_angle)
            arrow_x2 = x2 - arrow_head_length * math.cos(angle + arrow_angle)
            arrow_y2 = y2 - arrow_head_length * math.sin(angle + arrow_angle)
            pygame.draw.line(screen, white, (x2, y2), (arrow_x1, arrow_y1), 2)
            pygame.draw.line(screen, white, (x2, y2), (arrow_x2, arrow_y2), 2)

        # Desenhar o último ponto da trajetória em verde
        x_last = padding + (trajectory[-1][0] + field_width / 2) * scale_x
        y_last = padding + (field_height / 2 - trajectory[-1][1]) * scale_y
        pygame.draw.circle(screen, (0, 255, 0), (int(x_last), int(y_last)), 5)

    def move_robot(robot, trajectory, gif_path):
        index = 0
        frames = []
        duration = 15  # Duração do GIF em segundos
        fps = 30  # Frames por segundo
        total_frames = duration * fps

        while index < len(trajectory):
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            screen.fill(black)

            # Desenhar campo
            pygame.draw.rect(
                screen,
                white,
                (
                    padding,
                    padding,
                    screen_width - 2 * padding,
                    screen_height - 2 * padding,
                ),
                2,
            )

            # Desenhar gols
            goal_width = 8
            goal_height = config["goal_size"] * scale_y
            pygame.draw.rect(
                screen,
                white,
                (padding, (screen_height - goal_height) / 2, goal_width, goal_height),
            )
            pygame.draw.rect(
                screen,
                white,
                (
                    screen_width - padding - goal_width,
                    (screen_height - goal_height) / 2,
                    goal_width,
                    goal_height,
                ),
            )

            # Desenhar robôs do time
            for rob in config["team_position"]:
                color = red if rob["id"] == 0 else white
                x = padding + (rob["x"] + field_width / 2) * scale_x
                y = padding + (field_height / 2 - rob["y"]) * scale_y
                draw_robot(x, y, color)

            # Desenhar robôs do time adversário
            for rob in config["enemy_position"]:
                x = padding + (rob["x"] + field_width / 2) * scale_x
                y = padding + (field_height / 2 - rob["y"]) * scale_y
                draw_robot(x, y, white)

            # Desenhar trajetória
            draw_trajectory(trajectory)

            # Mover robô controlado
            target_x, target_y = trajectory[index]
            current_x = robot["x"]
            current_y = robot["y"]

            # Calcular a direção do movimento
            direction_x = target_x - current_x
            direction_y = target_y - current_y
            distance = math.sqrt(direction_x**2 + direction_y**2)
            if distance > 0:
                direction_x /= distance
                direction_y /= distance

            # Atualizar posição do robô
            speed = 15  # Velocidade do robô
            robot["x"] += direction_x * speed
            robot["y"] += direction_y * speed

            # Verificar se o robô chegou ao ponto alvo
            if (
                math.sqrt((robot["x"] - target_x) ** 2 + (robot["y"] - target_y) ** 2)
                < speed
            ):
                index += 1

            x_robot = padding + (robot["x"] + field_width / 2) * scale_x
            y_robot = padding + (field_height / 2 - robot["y"]) * scale_y
            draw_robot(x_robot, y_robot, red)

            pygame.display.flip()

            # Capturar quadro atual
            frame = pygame.surfarray.array3d(screen)
            frames.append(frame.transpose([1, 0, 2]))

            pygame.time.delay(int(1000 / fps))

            # Se atingir o número total de frames, parar
            if len(frames) >= total_frames:
                break

        # Salvar GIF
        imageio.mimsave(gif_path, frames, fps=fps)

    # Inicializar o robô controlado e a trajetória
    robot = config["team_position"][0]
    trajectory = [
        (-2150, 0),
        (-2000, 0),
        (-1500, 250),
        (-1000, 250),
        (-500, 500),
        (0, 500),
        (500, 250),
        (1000, 0),
        (1500, 0),
        (2000, -250),
        (2200, -250),
    ]

    # Executar movimentação do robô ao longo da trajetória
    move_robot(robot, trajectory, "trajectory.gif")

    pygame.quit()
    sys.exit()


# Executar visualização
visualize(configuracao1)
