import imageio
import pygame
import sys
import math

# Estrutura de dados fornecida
configuracao1 = {
    "team_position": [
        {"id": 0, "x": -2050.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
        {"id": 1, "x": -1000.0, "y": -750.0, "z": 0.0, "orientation": 0.0},
        {"id": 2, "x": -1000.0, "y": 750.0, "z": 0.0, "orientation": 0.0},
    ],
    "enemy_position": [
        {"id": 0, "x": 1000.0, "y": -750.0, "z": 0.0, "orientation": 0.0},
        {"id": 1, "x": 1000.0, "y": 750.0, "z": 0.0, "orientation": 0.0},
        {"id": 2, "x": 2050.0, "y": 0.0, "z": 0.0, "orientation": 0.0},
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
        450 + 2 * padding,
        300 + 2 * padding,
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
    def draw_robot(x, y, color, radius=10):
        pygame.draw.circle(screen, color, (int(x), int(y)), radius)

    # Função para desenhar uma trajetória
    def draw_trajectory(trajectory):
        for i in range(len(trajectory) - 1):
            # Desenhar pontos da trajetória em verde
            x1 = padding + (trajectory[i][0] + field_width / 2) * scale_x
            y1 = padding + (field_height / 2 - trajectory[i][1]) * scale_y
            pygame.draw.circle(screen, (0, 0, 255), (int(x1), int(y1)), 5)

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

        # Desenhar o último ponto da trajetória em vermelho
        x_last = padding + (trajectory[-1][0] + field_width / 2) * scale_x
        y_last = padding + (field_height / 2 - trajectory[-1][1]) * scale_y
        pygame.draw.circle(screen, (255, 0, 0), (int(x_last), int(y_last)), 5)

    def move_robot(robot, trajectory, target, gif_path):
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

            for point in target:
                target_x = padding + (point[0] + field_width / 2) * scale_x
                target_y = padding + (field_height / 2 - point[1]) * scale_y
                draw_robot(target_x, target_y, green, 5)

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
    data = [
        # {"x": -2050, "y": 0, "angle": -1.72749},
        # {"x": -1904.54, "y": 13.1763, "angle": 3.08738},
        # {"x": -1661.74, "y": 163.946, "angle": 2.31757},
        # {"x": -1522.19, "y": 298.825, "angle": 2.47087},
        # {"x": -1352.19, "y": 328.694, "angle": 2.97515},
        # {"x": -1174.4, "y": 396.115, "angle": 2.75301},
        # {"x": -1009.72, "y": 444.501, "angle": 2.67859},
        # {"x": -912.789, "y": 478.396, "angle": 3.01901},
        # {"x": -637.662, "y": 502.164, "angle": 3.0368},
        # {"x": -411.69, "y": 545.87, "angle": 2.80272},
        # {"x": -287.687, "y": 541.755, "angle": -3.1274},
        # {"x": 2.35145, "y": 592.799, "angle": 2.66558},
        # {"x": 101.358, "y": 595.155, "angle": 3.13324},
        # {"x": 383.37, "y": 643.122, "angle": 2.82563},
        # {"x": 530.097, "y": 691.199, "angle": 2.47614},
        # {"x": 591.352, "y": 443.15, "angle": -1.10609},
        # {"x": 467, "y": 427, "angle": -0.160117},
        # {"x": 367, "y": 249, "angle": -1.11554},
        # {"x": -2199.9001888009966, "y": -83.24963276512835, "angle": 0},
        # {"x": -2152.361889559952, "y": -76.04621166118386, "angle": 0},
        # {"x": -2095.9767387066254, "y": -53.9817331494869, "angle": 0},
        # {"x": -2068.935164260127, "y": -4.17734246700752, "angle": 0},
        # {"x": -1999.6427254301948, "y": -29.708115320875777, "angle": 0},
        # {"x": -1928.3293922537855, "y": -51.01628281437752, "angle": 0},
        # {"x": -1889.84576093145, "y": -63.57339030045182, "angle": 0},
        # {"x": -1834.8742195554134, "y": -41.744023191011365, "angle": 0},
        # {"x": -1784.895587773467, "y": -54.0060005109342, "angle": 0},
        # {"x": -1687.9847615870706, "y": -66.77677713676167, "angle": 0},
        # {"x": -1597.844266187577, "y": -39.320875398212365, "angle": 0},
        # {"x": -1550.611081217317, "y": -54.45316701639695, "angle": 0},
        # {"x": -1514.0112896647483, "y": -48.06125391300702, "angle": 0},
        # {"x": -1449.3540538495067, "y": -30.08653864651842, "angle": 0},
        # {"x": -1379.9407013205523, "y": -60.65769360523177, "angle": 0},
        # {"x": -1345.6729435303664, "y": -22.227307365665183, "angle": 0},
        # {"x": -1329.9010970881661, "y": 32.68392984288471, "angle": 0},
        # {"x": -1285.9324312266608, "y": 24.508554743146988, "angle": 0},
        # {"x": -1197.7238751674033, "y": 0.2194720749885164, "angle": 0},
        # {"x": -1155.2529059196793, "y": -3.906421771803025, "angle": 0},
        # {"x": -1047.8464613703818, "y": 6.231976985559868, "angle": 0},
        # {"x": -977.1473150544346, "y": 61.55226942353329, "angle": 0},
        # {"x": -934.6470545936231, "y": 169.45167999419027, "angle": 0},
        # {"x": -827.1858543472422, "y": 196.70314266454488, "angle": 0},
        # {"x": -762.8937049532963, "y": 165.63453426632873, "angle": 0},
        # {"x": -716.0640098933015, "y": 149.33133119994727, "angle": 0},
        # {"x": -686.920281776215, "y": 178.74681183037478, "angle": 0},
        # {"x": -583.2771636748664, "y": 179.11473246958212, "angle": 0},
        # {"x": -522.4142559532288, "y": 95.87265277679762, "angle": 0},
        # {"x": -458.34878628471415, "y": 103.3241952367598, "angle": 0},
        # {"x": -404.1036633315098, "y": 91.51741117761367, "angle": 0},
        # {"x": -337.7486267688578, "y": 137.62078822217995, "angle": 0},
        # {"x": -264.76093327058516, "y": 123.04171463150419, "angle": 0},
        # {"x": -215.3484262609477, "y": 141.44198204581608, "angle": 0},
        # {"x": -137.38231957531025, "y": 115.49078771516906, "angle": 0},
        # {"x": -55.69251875851796, "y": 61.02273869958867, "angle": 0},
        # {"x": -23.23949517732717, "y": 78.45250630555665, "angle": 0},
        # {"x": 75.8120917274091, "y": 77.0603964758975, "angle": 0},
        # {"x": 86.4761690221826, "y": 156.14774659973796, "angle": 0},
        # {"x": 93.86914321284848, "y": 196.33117468719524, "angle": 0},
        # {"x": 145.3553290741479, "y": 302.3503907912693, "angle": 0},
    ]

    # trajectory = [(entry["x"], entry["y"]) for entry in data]
    # trajectory = []
    trajectory = [
        (-1959.5429740884565,65.78692793566779),
(-1912.1349657232568,76.98714029091298),
(-1864.6776603319222,66.28088541633178),
(-1802.072034263374,42.27918050599874),
(-1762.2098269480803,-34.620762716788704),
(-1709.7068536184715,-39.55760647011516),
(-1628.1147634036438,-118.00715973790761),
(-1617.0557116363188,-179.53093540319992),
(-1526.8610343456803,-204.08847361664198),
(-1490.5855967418324,-201.59356336957921),
(-1444.3394842784771,-203.8561236161836),
(-1363.9142479330624,-224.25040854438043),
(-1291.3843991278602,-218.04136246437656),
(-1252.6675064111214,-220.34551828612985),
(-1200.377836509322,-172.7404521490157),
(-1106.0719908667916,-117.26691765834767),
(-1073.7461232333922,-99.57813291005937),
(-1042.2937018449181,-78.43026418303202),
(-956.4808049765286,-100.43274613644144),
(-866.3221673839239,-143.6490722312542),
(-773.1390645779707,-152.43708316127208),
(-727.9124463026849,-125.20625787116273),
(-659.5834958859111,-128.32928336298187),
(-601.4766230282835,-150.44807016584673),
(-521.2166500699441,-148.98243342923274),
(-442.2136043533312,-156.49600102631052),
(-371.45421924045536,-203.2209058504627),
(-324.93045159724556,-206.72276333967943),
(-265.16164815802017,-176.40844831274853),
(-219.6575805814607,-141.28902862518999),
(-172.57470559213334,-154.45489748915725),
(-96.86757558659065,-70.45301318096267),
(-49.32058645163352,-78.22016648256385),
(50.71157317111965,-94.56116163316824),
(67.84449014370648,-157.21406862384902),
(153.64696988867354,-223.24333534692505),
(179.78596052471494,-280.8504574774845),
(244.15172321336695,-331.7261661764428),
(334.625189699319,-402.4115614117786),
(448.4770296640677,-437.41221762168857),
(546.5111592409776,-430.65997782241675),
(616.0066328565968,-468.42993669704197),
(731.8164354336523,-455.66377221565176),
(743.8927851660355,-352.53482288007945),
(755.6059340898664,-302.6047695022237),
(855.9033518434785,-294.1354587687679),
(924.28516485201,-293.3257947691586),
(964.070145107969,-296.73034108588195),
(1065.466900827906,-305.4072324401841),
(1165.4996698436298,-313.9674024619628),
(1229.1937228503998,-319.41793560704104),
(1290.9850921351199,-324.70564914491774),
(1321.9762801514935,-386.3121781283701),
(1386.9434614060197,-380.0146097860395),
(1442.2251261516567,-448.75444531773314),
(1540.1434221341756,-403.4812053345356),
(1592.9348940706204,-379.0726823505204),
(1645.0,-355.0),
    ]

    # target = (2250, 0)
    target = [
        (0, 0),
        (1500, 0),
        (1600, -400),
        (600, 600),
    ]

    # Executar movimentação do robô ao longo da trajetória
    move_robot(robot, trajectory, target, "trajectory4.gif")

    pygame.quit()
    sys.exit()


# Executar visualização
visualize(configuracao1)
