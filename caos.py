import pygame
import math
import random

pygame.init()

# Configurações da tela e parâmetros globais
WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulação Avançada de Corpos")
clock = pygame.time.Clock()

# Parâmetros iniciais
G_default = 1.0
G = G_default
dt = 0.1
epsilon = 5.0  # Softening factor para evitar singularidades
mode_3D = False  # Modo 2D inicialmente
collision_mode = "merge"  # "merge" ou "elastic"
paused = False

# Para gráficos de energia
energy_history = []

# Variáveis para drag and drop
dragged_body = None

# Parâmetro para projeção 3D
camera_distance = 500

# --- CLASSES E FUNÇÕES --- #

# Classe que representa um corpo (2D ou 3D)
class Body:
    def __init__(self, pos, vel, mass, color):
        # pos e vel são listas; se 3D, terão 3 componentes
        self.pos = list(pos)
        self.vel = list(vel)
        self.mass = mass
        self.color = color
        self.trail = []  # Histórico de posições para o rastro

    def get_projection(self):
        # Se estiver em 3D, projeta usando perspectiva; caso contrário, apenas centraliza
        if mode_3D:
            x, y, z = self.pos
            factor = camera_distance / (camera_distance + z)
            return (x * factor + WIDTH/2, y * factor + HEIGHT/2)
        else:
            # Em 2D, as posições são relativas ao centro da tela
            return (self.pos[0] + WIDTH/2, self.pos[1] + HEIGHT/2)

    def update_trail(self):
        proj = self.get_projection()
        self.trail.append(proj)
        if len(self.trail) > 150:
            self.trail.pop(0)

    def draw(self, surface):
        # Desenha o rastro
        if len(self.trail) > 1:
            pygame.draw.lines(surface, self.color, False, [(int(x), int(y)) for (x,y) in self.trail], 2)
        # Desenha o corpo
        proj = self.get_projection()
        pygame.draw.circle(surface, self.color, (int(proj[0]), int(proj[1])), 5)

# Cria corpos iniciais; em 3D, gera coordenada z também
def create_bodies(n=3):
    bodies = []
    for i in range(n):
        if mode_3D:
            pos = (random.uniform(-50,50), random.uniform(-50,50), random.uniform(-50,50))
            vel = (random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
        else:
            pos = (random.uniform(-50,50), random.uniform(-50,50))
            vel = (random.uniform(-1,1), random.uniform(-1,1))
        mass = random.choice([10, 15])
        color = [(255,0,0), (0,0,255), (255,255,0)][i % 3]
        bodies.append(Body(pos, vel, mass, color))
    return bodies

bodies = create_bodies()

# Função para computar a força gravitacional entre dois corpos (2D ou 3D)
def compute_force(b1, b2):
    if mode_3D:
        dx = b2.pos[0] - b1.pos[0]
        dy = b2.pos[1] - b1.pos[1]
        dz = b2.pos[2] - b1.pos[2]
        r = math.sqrt(dx*dx + dy*dy + dz*dz + epsilon**2)
        F = G * b1.mass * b2.mass / (r**3)
        return (F*dx, F*dy, F*dz)
    else:
        dx = b2.pos[0] - b1.pos[0]
        dy = b2.pos[1] - b1.pos[1]
        r = math.sqrt(dx*dx + dy*dy + epsilon**2)
        F = G * b1.mass * b2.mass / (r**3)
        return (F*dx, F*dy)

# Função para tratar colisões; alterna entre fusão e colisão elástica
def handle_collisions(bodies):
    new_bodies = bodies[:]
    i = 0
    while i < len(new_bodies):
        j = i+1
        merged = False
        while j < len(new_bodies):
            # Calcula a distância entre corpos
            if mode_3D:
                dx = new_bodies[j].pos[0] - new_bodies[i].pos[0]
                dy = new_bodies[j].pos[1] - new_bodies[i].pos[1]
                dz = new_bodies[j].pos[2] - new_bodies[i].pos[2]
                dist = math.sqrt(dx*dx+dy*dy+dz*dz)
            else:
                dx = new_bodies[j].pos[0] - new_bodies[i].pos[0]
                dy = new_bodies[j].pos[1] - new_bodies[i].pos[1]
                dist = math.sqrt(dx*dx+dy*dy)
            if dist < 10:
                if collision_mode == "merge":
                    m1 = new_bodies[i].mass
                    m2 = new_bodies[j].mass
                    new_mass = m1 + m2
                    new_pos = [(new_bodies[i].pos[k]*m1 + new_bodies[j].pos[k]*m2)/new_mass for k in range(len(new_bodies[i].pos))]
                    new_vel = [(new_bodies[i].vel[k]*m1 + new_bodies[j].vel[k]*m2)/new_mass for k in range(len(new_bodies[i].vel))]
                    new_color = tuple(min(255, (new_bodies[i].color[k]+new_bodies[j].color[k])//2) for k in range(3))
                    new_body = Body(new_pos, new_vel, new_mass, new_color)
                    new_body.trail = new_bodies[i].trail + new_bodies[j].trail
                    new_bodies.pop(j)
                    new_bodies.pop(i)
                    new_bodies.append(new_body)
                    merged = True
                    break
                elif collision_mode == "elastic":
                    # Simula colisão elástica simples: troca de componentes na direção normal
                    if mode_3D:
                        r_vec = [new_bodies[j].pos[k]-new_bodies[i].pos[k] for k in range(3)]
                        r_mag = math.sqrt(sum(x*x for x in r_vec)) + 1e-10
                        n = [x/r_mag for x in r_vec]
                        v_rel = [new_bodies[i].vel[k]-new_bodies[j].vel[k] for k in range(3)]
                        v_dot_n = sum(v_rel[k]*n[k] for k in range(3))
                        j_impulse = (2*v_dot_n)/(1/new_bodies[i].mass + 1/new_bodies[j].mass)
                        for k in range(3):
                            new_bodies[i].vel[k] -= j_impulse * n[k] / new_bodies[i].mass
                            new_bodies[j].vel[k] += j_impulse * n[k] / new_bodies[j].mass
                    else:
                        r_vec = [new_bodies[j].pos[k]-new_bodies[i].pos[k] for k in range(2)]
                        r_mag = math.sqrt(sum(x*x for x in r_vec)) + 1e-10
                        n = [x/r_mag for x in r_vec]
                        v_rel = [new_bodies[i].vel[k]-new_bodies[j].vel[k] for k in range(2)]
                        v_dot_n = sum(v_rel[k]*n[k] for k in range(2))
                        j_impulse = (2*v_dot_n)/(1/new_bodies[i].mass + 1/new_bodies[j].mass)
                        for k in range(2):
                            new_bodies[i].vel[k] -= j_impulse * n[k] / new_bodies[i].mass
                            new_bodies[j].vel[k] += j_impulse * n[k] / new_bodies[j].mass
            j += 1
        if merged:
            return handle_collisions(new_bodies)
        else:
            i += 1
    return new_bodies

# Função para desenhar um slider para G
def draw_slider(surface, x, y, w, h, value, min_val, max_val):
    pygame.draw.rect(surface, (180,180,180), (x, y, w, h))
    knob_x = x + int((value - min_val) / (max_val - min_val) * w)
    pygame.draw.circle(surface, (255,255,255), (knob_x, y + h//2), h//2)
    font = pygame.font.Font(None, 24)
    text = font.render(f"G: {value:.2f}", True, (255,255,255))
    surface.blit(text, (x, y - 25))
    return pygame.Rect(x, y, w, h)

# Função para desenhar informações gerais na tela
def draw_info(surface):
    font = pygame.font.Font(None, 24)
    info_lines = [
        f"Modo: {'3D' if mode_3D else '2D'} | Colisão: {collision_mode}",
        "P: Pausar/Retomar | D: Toggle 3D | C: Toggle colisão",
        "N: Adicionar corpo | Drag & Drop para mover corpos"
    ]
    for i, line in enumerate(info_lines):
        text = font.render(line, True, (255,255,255))
        surface.blit(text, (10, HEIGHT - 80 + i*20))

# Calcula energia cinética total
def total_kinetic_energy(bodies):
    KE = 0
    for b in bodies:
        if mode_3D:
            v2 = sum(v*v for v in b.vel)
        else:
            v2 = b.vel[0]**2 + b.vel[1]**2
        KE += 0.5 * b.mass * v2
    return KE

# Calcula energia potencial total (para cada par i<j)
def total_potential_energy(bodies):
    PE = 0
    n = len(bodies)
    for i in range(n):
        for j in range(i+1, n):
            if mode_3D:
                dx = bodies[j].pos[0] - bodies[i].pos[0]
                dy = bodies[j].pos[1] - bodies[i].pos[1]
                dz = bodies[j].pos[2] - bodies[i].pos[2]
                r = math.sqrt(dx*dx+dy*dy+dz*dz) + 1e-10
            else:
                dx = bodies[j].pos[0] - bodies[i].pos[0]
                dy = bodies[j].pos[1] - bodies[i].pos[1]
                r = math.sqrt(dx*dx+dy*dy) + 1e-10
            PE += -G * bodies[i].mass * bodies[j].mass / r
    return PE

# Reinicia a simulação: novos corpos e reseta G
def restart_simulation():
    global G
    G = G_default
    return create_bodies(3)

# Adiciona um corpo na posição do mouse com parâmetros aleatórios
def add_body(mouse_pos):
    if mode_3D:
        pos = (mouse_pos[0]-WIDTH/2, mouse_pos[1]-HEIGHT/2, random.uniform(-50,50))
        vel = (random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1))
    else:
        pos = (mouse_pos[0]-WIDTH/2, mouse_pos[1]-HEIGHT/2)
        vel = (random.uniform(-1,1), random.uniform(-1,1))
    mass = random.choice([10,15])
    color = random.choice([(255,0,0), (0,0,255), (255,255,0), (0,255,0), (255,0,255)])
    return Body(pos, vel, mass, color)

# Função para desenhar o botão de reiniciar
def draw_restart_button(surface):
    restart_rect = pygame.Rect(650, 550, 130, 40)
    pygame.draw.rect(surface, (180, 180, 180), restart_rect)
    font = pygame.font.Font(None, 24)
    text = font.render("Reiniciar", True, (255, 255, 255))
    surface.blit(text, (restart_rect.x + 10, restart_rect.y + 10))
    return restart_rect

# --- LOOP PRINCIPAL --- #
slider_rect = draw_slider(screen, 10, 40, 200, 20, G, 0.1, 2.0)
dragging_slider = False

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_p:
                paused = not paused
            elif event.key == pygame.K_d:
                mode_3D = not mode_3D
                # Recria corpos para o novo modo
                bodies = create_bodies(len(bodies))
            elif event.key == pygame.K_c:
                collision_mode = "elastic" if collision_mode=="merge" else "merge"
            elif event.key == pygame.K_n:
                bodies.append(add_body(pygame.mouse.get_pos()))
            elif event.key == pygame.K_r:
                G = G_default
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # clique esquerdo
                if slider_rect.collidepoint(event.pos):
                    dragging_slider = True
                else:
                    # Verifica se clicou em um corpo para arrastar
                    for b in bodies:
                        proj = b.get_projection()
                        if math.hypot(proj[0]-event.pos[0], proj[1]-event.pos[1]) < 10:
                            dragged_body = b
                            break
                    # Verifica se clicou no botão reiniciar
                    restart_rect = pygame.Rect(650, 550, 130, 40)
                    if restart_rect.collidepoint(event.pos):
                        bodies = create_bodies(3)
                        G = G_default
        elif event.type == pygame.MOUSEBUTTONUP:
            dragging_slider = False
            dragged_body = None
        elif event.type == pygame.MOUSEMOTION:
            if dragging_slider:
                rel_x = event.pos[0] - slider_rect.x
                new_G = 0.1 + (rel_x/slider_rect.width)*(2.0 - 0.1)
                G = max(0.1, min(2.0, new_G))
            if dragged_body is not None:
                # Atualiza posição do corpo para o mouse (converte para coordenadas de simulação)
                if mode_3D:
                    dragged_body.pos[0] = event.pos[0] - WIDTH/2
                    dragged_body.pos[1] = event.pos[1] - HEIGHT/2
                else:
                    dragged_body.pos[0] = event.pos[0] - WIDTH/2
                    dragged_body.pos[1] = event.pos[1] - HEIGHT/2

    if not paused:
        # Integração simples (Euler) para atualizar os corpos
        for b in bodies:
            net_force = [0,0] if not mode_3D else [0,0,0]
            for other in bodies:
                if b is not other:
                    f = compute_force(b, other)
                    if not mode_3D:
                        net_force[0] += f[0]
                        net_force[1] += f[1]
                    else:
                        for i in range(3):
                            net_force[i] += f[i]
            if not mode_3D:
                a = [net_force[0]/b.mass, net_force[1]/b.mass]
                b.vel[0] += a[0]*dt
                b.vel[1] += a[1]*dt
                b.pos[0] += b.vel[0]*dt
                b.pos[1] += b.vel[1]*dt
            else:
                a = [net_force[i]/b.mass for i in range(3)]
                for i in range(3):
                    b.vel[i] += a[i]*dt
                    b.pos[i] += b.vel[i]*dt
            b.update_trail()
        bodies = handle_collisions(bodies)
    
    # Calcula energias para exibir gráfico
    KE = total_kinetic_energy(bodies)
    restart_rect = draw_restart_button(screen)
    PE = total_potential_energy(bodies)
    energy_total = KE + PE
    energy_history.append(energy_total)
    if len(energy_history) > 200:
        energy_history.pop(0)
    
    # Renderização
    screen.fill((0,0,0))
    slider_rect = draw_slider(screen, 10, 40, 200, 20, G, 0.1, 2.0)
    restart_rect = draw_restart_button(screen)
    draw_info(screen)
    # Desenha gráfico de energia
    if len(energy_history) > 1:
        pts = []
        for i, val in enumerate(energy_history):
            x = 10 + i
            y = 200 - int(val/50)  # ajuste de escala
            pts.append((x, y))
        if len(pts) > 1:
            pygame.draw.lines(screen, (0,255,0), False, pts, 2)
    for b in bodies:
        # Desenha seta do vetor de força (net force)
        if not mode_3D:
            net_force = [0,0]
            for other in bodies:
                if b is not other:
                    f = compute_force(b, other)
                    net_force[0] += f[0]
                    net_force[1] += f[1]
            start = b.get_projection()
            end = (start[0] + net_force[0]*10, start[1] + net_force[1]*10)
            pygame.draw.line(screen, (255,255,255), start, end, 2)
        b.draw(screen)
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
