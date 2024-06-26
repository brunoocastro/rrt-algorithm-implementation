import pygame
import sys

# Estrutura de dados fornecida
configuracao1 = {
    "team_position": [
        {"id": 0, "x": -2000.0, "y": 0.0, "z": 0.0, "orientation": 0.0},  # Defensor esquerdo
        {"id": 1, "x": -1500.0, "y": 0.0, "z": 0.0, "orientation": 0.0},  # Defensor direito
        {"id": 2, "x": 1200.0, "y": 0.0, "z": 0.0, "orientation": 0.0},  # Meio-campo central
    ],
    "enemy_position": [
        {"id": 0, "x": 1250.0, "y": -750.0, "z": 0.0, "orientation": 0.0},  # Atacante esquerdo
        {"id": 1, "x": 1250.0, "y": 750.0, "z": 0.0, "orientation": 0.0},  # Atacante direito
        {"id": 2, "x": 0100.0, "y": 0.0, "z": 0.0, "orientation": 0.0},  # Atacante central
    ],
    "field_size": [4500.0, 3000.0],
    "goal_size": 800.0,
    "final_path": []
}

width = 450
height = 300

# Função para visualizar a configuração
def visualize(config):
    pygame.init()
    
    # Configurações da tela
    field_width, field_height = config["field_size"]
    padding = 50  # Padding nas bordas
    screen_width, screen_height = width + 2 * padding, height + 2 * padding  # Tamanho da tela com padding
    scale_x = (screen_width - 2 * padding) / field_width
    scale_y = (screen_height - 2 * padding) / field_height
    
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Visualizador de Futebol de Robôs SSL")

    # Cores
    white = (255, 255, 255)
    red = (255, 0, 0)
    black = (0, 0, 0)

    # Função para desenhar um robô
    def draw_robot(x, y, color):
        radius = 10
        pygame.draw.circle(screen, color, (int(x), int(y)), radius)
    
    # Loop principal
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        screen.fill(black)

        # Desenhar campo
        pygame.draw.rect(screen, white, (padding, padding, screen_width - 2 * padding, screen_height - 2 * padding), 2)
        
        # Desenhar gols
        goal_width = 8
        goal_height = config["goal_size"] * scale_y
        pygame.draw.rect(screen, white, (padding, (screen_height - goal_height) / 2, goal_width, goal_height))
        pygame.draw.rect(screen, white, (screen_width - padding - goal_width, (screen_height - goal_height) / 2, goal_width, goal_height))
        
        # Desenhar robôs do time
        for robot in config["team_position"]:
            color = red if robot["id"] == 0 else white
            x = padding + (robot["x"] + field_width / 2) * scale_x
            y = padding + (field_height / 2 - robot["y"]) * scale_y
            draw_robot(x, y, color)
        
        # Desenhar robôs do time adversário
        for robot in config["enemy_position"]:
            x = padding + (robot["x"] + field_width / 2) * scale_x
            y = padding + (field_height / 2 - robot["y"]) * scale_y
            draw_robot(x, y, white)

        if config["final_path"] and (len(config["final_path"])> 0):
            # Desenhar caminho final
            path = [(padding + (pos[0] + field_width / 2) * scale_x, padding + (field_height / 2 - pos[1]) * scale_y) for pos in config["final_path"]]
            pygame.draw.lines(screen, red, False, path, 2)

        pygame.display.flip()
    
    pygame.quit()
    sys.exit()

# Executar visualização
visualize(configuracao1)
