import pygame
import math
import heapq
import random

pygame.init()

WIDTH, HEIGHT = 1280, 720
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* 알고리즘 통합 시뮬레이터")

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (200, 40, 40)
BLUE = (0, 100, 255)

# 자율주행 경로에 사용할 초록색 계열 정의
LIME_GREEN = (50, 255, 50)  # 경로 선 및 주요 포인트
DARK_LIME = (0, 150, 0)     # 경로 도트

GREEN = (0, 200, 0) # 밝은 초록색 (잔디/자율주행 모드 배경)
DARK_GREEN = (0, 150, 0) # 어두운 초록색 (잔디깎은 후 색)
GRAY = (140, 140, 140)
YELLOW = (240, 200, 40) # 잔디깎기 모드 사용자 지정 경로 색상
MENU_BG = (50, 50, 70)
BUTTON_COLOR = (80, 200, 80)
BUTTON_HOVER = (100, 255, 100)
START_BUTTON_COLOR = (255, 100, 100)
OBSTACLE_COLOR = (80, 80, 80) 

clock = pygame.time.Clock()
try:
    # 'malgungothic.ttf' 파일이 없을 경우 대비
    font = pygame.font.Font("malgungothic.ttf", 20)
    font_small = pygame.font.Font("malgungothic.ttf", 16)
except:
    font = pygame.font.SysFont("malgungothic", 20)
    font_small = pygame.font.SysFont("malgungothic", 16)

# ====================================================================
# Button Class
# ====================================================================

class Button:
    def __init__(self, rect, text, color=BUTTON_COLOR, hover_color=BUTTON_HOVER):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.base_color = color
        self.hover_color = hover_color

    def draw(self, surf):
        mx, my = pygame.mouse.get_pos()
        color = self.hover_color if self.rect.collidepoint(mx, my) else self.base_color
        pygame.draw.rect(surf, color, self.rect)
        pygame.draw.rect(surf, BLACK, self.rect, 2)
        txt_surf = font.render(self.text, True, BLACK)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surf.blit(txt_surf, txt_rect)

    def is_clicked(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            if self.rect.collidepoint(mx, my):
                return True
        return False

# ====================================================================
# Robot Class
# ====================================================================

class Robot:
    def __init__(self, x, y, sim):
        self.sim = sim
        self.pos = pygame.Vector2(x, y)
        self.angle = -90.0
        self.speed = 2.2
        self.turn_speed = 3.5
        self.radius = 10  # 로봇의 물리적 반지름 (자율주행 마진 기준)
        self.color = RED
        self.in_avoid = False
        self.avoid_dir = 0
        self.avoid_timer = 0
        self.avoid_turn_frames = 30
        self.recovery_steps = 0
        self.finished = False
        self.is_moving = False
        self.mowing_radius = 5 # 로봇이 깎는 면적 (반지름 5로 최소화)

    def check_collision_at(self, pos):
        r = pygame.Rect(pos.x - self.radius, pos.y - self.radius, self.radius*2, self.radius*2)
        for obs in self.sim.obstacles:
            if r.colliderect(obs):
                return True
        return False

    def cut_grass(self, target_pos):
        global sim_grass_active
        if not sim_grass_active:
            return
            
        sim = self.sim
        gs = sim.grid_size
        area = sim.mowing_area_rect
        
        # 깎는 범위를 self.mowing_radius로 사용
        cut_radius_cells = math.ceil(self.mowing_radius / gs) + 1 
        
        mx = target_pos.x - area.left
        my = target_pos.y - area.top
        
        center_gx = int(mx // gs)
        center_gy = int(my // gs)
        
        grass_cols = len(sim.grass[0])
        grass_rows = len(sim.grass)

        for dy in range(-cut_radius_cells, cut_radius_cells + 1):
            for dx in range(-cut_radius_cells, cut_radius_cells + 1):
                gx, gy = center_gx + dx, center_gy + dy
                
                cell_center_x = (gx * gs + gs / 2) + area.left
                cell_center_y = (gy * gs + gs / 2) + area.top
                
                # 로봇이 이 격자 중심으로부터 self.mowing_radius 안에 있으면 깎음
                if math.dist((cell_center_x, cell_center_y), target_pos) <= self.mowing_radius + gs/2 : 
                    if 0 <= gx < grass_cols and 0 <= gy < grass_rows:
                        if sim.grass[gy][gx]:
                            grass_rect = pygame.Rect(
                                gx * gs + area.left, gy * gs + area.top, gs, gs
                            )
                            is_obstacle_area = False
                            for obs in sim.obstacles:
                                if obs.colliderect(grass_rect):
                                    is_obstacle_area = True
                                    break
                            
                            if not is_obstacle_area:
                                sim.grass[gy][gx] = False
                                sim.add_particles(target_pos.x, target_pos.y)

    def update(self):
        sim = self.sim
        
        if not self.is_moving or self.finished or not sim.path_points:
            return

        sensors = sim.sensor_readings((self.pos.x, self.pos.y), self.angle)
        front, left, right = sensors["front"], sensors["left"], sensors["right"] 
        FRONT_THRESHOLD = 28
        
        # 1. 장애물 회피 로직
        if not self.in_avoid and front < FRONT_THRESHOLD:
            self.avoid_dir = -1 if left > right else 1 
            self.in_avoid = True
            self.avoid_timer = int(self.avoid_turn_frames * max(0.6, (FRONT_THRESHOLD - front)/FRONT_THRESHOLD + 0.6))
            self.recovery_steps = 10
            return
        
        if self.in_avoid:
            self.angle += self.turn_speed * self.avoid_dir
            self.avoid_timer -= 1
            move_vec = pygame.Vector2(math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle)))
            proposed = self.pos + move_vec * (self.speed * 0.6)
            if not self.check_collision_at(proposed):
                self.pos = proposed
            if self.avoid_timer <= 0:
                self.in_avoid = False
            return

        # 2. 경로 추적 로직
        if sim.current_target_idx < len(sim.path_points):
            target = pygame.Vector2(sim.path_points[sim.current_target_idx])
            direction = target - self.pos
            dist = direction.length()
            
            # 다음 목표점으로 이동
            if dist < 10:
                sim.current_target_idx += 1
                if sim.current_target_idx >= len(sim.path_points):
                    self.finished = True
                
                global sim_grass_active
                if sim_grass_active:
                    self.cut_grass(target)
                return
            
            # 목표 각도 계산 및 회전
            target_angle = math.degrees(math.atan2(direction.y, direction.x))
            angle_diff = (target_angle - self.angle + 180) % 360 - 180
            
            if abs(angle_diff) > 6:
                self.angle += self.turn_speed * (1 if angle_diff > 0 else -1)
            else:
                # 전진
                move_vec = pygame.Vector2(math.cos(math.radians(self.angle)), math.sin(math.radians(self.angle)))
                proposed = self.pos + move_vec * self.speed
                
                if not self.check_collision_at(proposed):
                    self.pos = proposed
                    # 이동 중 잔디 깎기
                    if sim_grass_active:
                        self.cut_grass(self.pos)
                else:
                    # 전진 중 충돌 감지 -> 회피 시작
                    self.avoid_dir = -1 if left > right else 1
                    self.in_avoid = True
                    self.avoid_timer = int(self.avoid_turn_frames * 0.9)
                    self.recovery_steps = 8
                    return

    def draw(self, surf):
        pygame.draw.circle(surf, self.color, (int(self.pos.x), int(self.pos.y)), self.radius)
        hx = self.pos.x + math.cos(math.radians(self.angle)) * (self.radius + 12)
        hy = self.pos.y + math.sin(math.radians(self.angle)) * (self.radius + 12)
        pygame.draw.line(surf, BLACK, (int(self.pos.x), int(self.pos.y)), (int(hx), int(hy)), 2)
        
        sensors = self.sim.sensor_readings((self.pos.x, self.pos.y), self.angle)
        offsets = [0, -45, 45]
        keys = ['front','left','right']
        
        for i, offset in enumerate(offsets):
            a = math.radians(self.angle + offset)
            key = keys[i]
            length = sensors.get(key, 0) 
            ex = self.pos.x + math.cos(a)*length
            ey = self.pos.y + math.sin(a)*length
            pygame.draw.line(surf, BLUE, (self.pos.x, self.pos.y), (ex, ey), 1)
            # 센서 거리 표시 (픽스호크 모드에서 유용)
            dist_txt = font_small.render(f"{int(length)}", True, BLACK) 
            surf.blit(dist_txt, (ex, ey))

# ====================================================================
# Simulation Class
# ====================================================================

class Simulation:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.path_points = []
        self.user_defined_path = [] 
        self.current_target_idx = 0
        self.goal_point = None
        self.particles = []
        
        self.grid_size = 40 
        
        # 잔디깎기 시뮬레이션 영역을 화면 전체로 확장
        M_W, M_H = self.width, self.height
        M_X = 0
        M_Y = 0
        self.mowing_area_rect = pygame.Rect(M_X, M_Y, M_W, M_H)
        
        grass_cols = M_W // self.grid_size
        grass_rows = M_H // self.grid_size
        self.grass = [[True for _ in range(grass_cols)] for _ in range(grass_rows)]
        
        self.grass_texture = None
        self.cut_grass_texture = None

        self.robot = Robot(self.width//2, self.height-80, self)
        self.success_msg = ""
        self.mousing_obstacle = None
        
        # 픽스호크 GPS 모사 좌표계 설정
        self.LAT_START, self.LAT_END = 37.5, 37.4
        self.LON_START, self.LON_END = 127.0, 127.1
        
    def to_gps(self, pixel_pos):
        # 픽셀 좌표 (x, y) -> GPS (lat, lon)
        x, y = pixel_pos
        lat = self.LAT_START + (y / self.height) * (self.LAT_END - self.LAT_START)
        lon = self.LON_START + (x / self.width) * (self.LON_END - self.LON_START)
        return (lat, lon) 

    def from_gps(self, gps_pos):
        # GPS 좌표 (lat, lon) -> 픽셀 (x, y)
        lat, lon = gps_pos
        
        x = ((lon - self.LON_START) / (self.LON_END - self.LON_START)) * self.width
        y = ((lat - self.LAT_START) / (self.LAT_END - self.LAT_START)) * self.height
        
        return (x, y) 
        
    def add_particles(self, x, y):
        for _ in range(20):
            angle = random.uniform(0, 2*math.pi)
            speed = random.uniform(2, 4)
            color = random.choice([(255, 200, 0), (255, 50, 50), (50, 200, 255)])
            self.particles.append({
                "pos": [x, y],
                "vel": [math.cos(angle)*speed, math.sin(angle)*speed],
                "color": color,
                "life": random.randint(20, 40)
            })

    def update_particles(self):
        for p in self.particles[:]:
            p["pos"][0] += p["vel"][0]
            p["pos"][1] += p["vel"][1]
            p["life"] -= 1
            if p["life"] <= 0:
                self.particles.remove(p)

    def draw_particles(self, surf):
        for p in self.particles:
            pygame.draw.circle(surf, p["color"], (int(p["pos"][0]), int(p["pos"][1])), 3)
            
    # 잔디밭 그리기 함수
    def draw_grass(self, surf):
        gs = self.grid_size
        area = self.mowing_area_rect
        
        pygame.draw.rect(surf, GREEN, area)
        
        for y, row in enumerate(self.grass):
            for x, alive in enumerate(row):
                rect_x = x * gs + area.left
                rect_y = y * gs + area.top
                
                grass_rect = pygame.Rect(rect_x, rect_y, gs, gs)
                
                is_obstacle_area = False
                for obs in self.obstacles:
                    if obs.colliderect(grass_rect):
                        is_obstacle_area = True
                        break
                
                if is_obstacle_area:
                    pygame.draw.rect(surf, OBSTACLE_COLOR, grass_rect)
                elif not alive and self.cut_grass_texture: 
                    surf.blit(self.cut_grass_texture, grass_rect)
                elif alive and self.grass_texture: 
                    surf.blit(self.grass_texture, grass_rect)
                elif not alive: 
                    pygame.draw.rect(surf, DARK_GREEN, grass_rect)


    def mark_obstacle_grass(self):
        gs = self.grid_size
        area = self.mowing_area_rect
        grass_cols = len(self.grass[0])
        grass_rows = len(self.grass)
        
        for gy in range(grass_rows):
            for gx in range(grass_cols):
                grass_rect = pygame.Rect(
                    gx * gs + area.left, gy * gs + area.top, gs, gs
                )
                
                is_overlap = False
                for obs in self.obstacles:
                    if obs.colliderect(grass_rect):
                        is_overlap = True
                        break
                
                if is_overlap:
                    self.grass[gy][gx] = False
                elif self.grass[gy][gx] == False:
                    pass
                else:
                    self.grass[gy][gx] = True

    # A* 경로 탐색 로직 (안전 마진 적용)
    def find_safe_path(self, start_pos, end_pos):
        grid_size = self.grid_size
        width, height = self.width, self.height
        obstacle_cells = set()
        
        global sim_grass_active
        if sim_grass_active:
            SAFETY_MARGIN = self.robot.mowing_radius + 5 
        else:
            SAFETY_MARGIN = self.robot.radius + 1 

        for obs in self.obstacles:
            expanded_obs = obs.inflate(SAFETY_MARGIN * 2, SAFETY_MARGIN * 2) 

            x0 = expanded_obs.left // grid_size
            x1 = max(0, (expanded_obs.right - 1) // grid_size)
            y0 = expanded_obs.top // grid_size
            y1 = max(0, (expanded_obs.bottom - 1) // grid_size)
            
            for gx in range(x0, x1 + 1):
                for gy in range(y0, y1 + 1):
                    if 0 <= gx < width // grid_size and 0 <= gy < height // grid_size:
                        obstacle_cells.add((gx, gy))

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def to_grid(pos):
            return (int(pos[0] // grid_size), int(pos[1] // grid_size))

        def to_world(cell):
            return (cell[0] * grid_size + grid_size // 2, cell[1] * grid_size + grid_size // 2)

        start = to_grid(start_pos)
        goal = to_grid(end_pos)
        max_x, max_y = width // grid_size, height // grid_size
        
        if start in obstacle_cells or goal in obstacle_cells:
             return [] 


        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        directions = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(to_world(current))
                    current = came_from[current]
                path.reverse()
                
                simplified = []
                if path:
                    simplified.append(path[0])
                    for p in path[1:]:
                        if math.dist(simplified[-1], p) > 30:
                            simplified.append(p)
                    if simplified[-1] != path[-1]:
                        simplified.append(path[-1])
                return simplified

            for dx, dy in directions:
                neighbor = (current[0]+dx, current[1]+dy)
                if not (0 <= neighbor[0] < max_x and 0 <= neighbor[1] < max_y):
                    continue
                if neighbor in obstacle_cells:
                    continue
                    
                tentative_g = g_score[current] + (1.4 if dx!=0 and dy!=0 else 1)
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return [] 


    # A* 실패 시 대안 경로 탐색 (자동 우회 경로 생성)
    def find_alternative_path(self, start_pos, end_pos, max_attempts=10):
        start_vec = pygame.Vector2(start_pos)
        end_vec = pygame.Vector2(end_pos)
        line_vec = end_vec - start_vec
        
        perp_vec_right = pygame.Vector2(-line_vec.y, line_vec.x).normalize() 
        perp_vec_left = -perp_vec_right
        
        mid_point = (start_vec + end_vec) / 2
        
        base_offset = 150 
        
        for attempt in range(max_attempts):
            offset = base_offset + random.uniform(-50, 50) 
            
            if attempt % 2 == 0:
                alt_vec = perp_vec_right * offset
            else:
                alt_vec = perp_vec_left * offset

            alt_point_raw = mid_point + alt_vec
            alt_point = (alt_point_raw.x, alt_point_raw.y)
            
            if not self.mowing_area_rect.collidepoint(alt_point):
                continue

            path_seg1 = self.find_safe_path(start_pos, alt_point)
            if not path_seg1:
                continue

            path_seg2 = self.find_safe_path(alt_point, end_pos)
            if not path_seg2:
                continue

            return path_seg1 + path_seg2[1:]

        return []


    def ray_distance(self, origin, angle_deg, max_dist=100, step=4):
        a = math.radians(angle_deg)
        ox, oy = origin
        for r in range(0, max_dist, step):
            px = int(ox + math.cos(a)*r)
            py = int(oy + math.sin(a)*r)
            if px < 0 or px >= self.width or py < 0 or py >= self.height:
                return r
            for obs in self.obstacles:
                if obs.collidepoint(px, py):
                    return r
        return max_dist

    # 센서 값 반환 (키: 'front', 'left', 'right')
    def sensor_readings(self, pos, heading_deg):
        front = self.ray_distance(pos, heading_deg)
        left = self.ray_distance(pos, heading_deg - 45)
        right = self.ray_distance(pos, heading_deg + 45)
        return {"front": float(front), "left": float(left), "right": float(right)}

    # 경로 생성 로직 (잔디깎기/자율주행 모두 사용)
    def generate_full_path(self, start_pos, target_points):
        self.path_points = []
        self.success_msg = ""
        current_start = start_pos

        first_segment = self.find_safe_path(current_start, target_points[0])
        
        if not first_segment:
            first_segment = self.find_alternative_path(current_start, target_points[0])
            if not first_segment:
                self.success_msg = "경로 자동 수정 실패: 시작점에서 첫 경로점까지 완전히 막혀있습니다. (R키 초기화)"
                return False
        
        self.path_points.extend(first_segment)
        current_start = self.path_points[-1] 
        
        for i, next_goal in enumerate(target_points[1:]):
            segment = self.find_safe_path(current_start, next_goal)
            
            if not segment:
                alternative_segment = self.find_alternative_path(current_start, next_goal)
                
                if not alternative_segment:
                    self.success_msg = f"경로 자동 수정 실패: {i+1}번과 {i+2}번 사이를 완전히 우회할 수 없습니다. (R키 초기화)"
                    self.path_points = []
                    return False
                
                self.success_msg = f"막힌 구간이 있어 자동 우회 경로를 생성했습니다."
                segment = alternative_segment
            
            self.path_points.extend(segment[1:])
            current_start = self.path_points[-1]
        
        if self.path_points:
            self.robot.is_moving = True
            self.current_target_idx = 0
            self.robot.finished = False
            self.goal_point = self.path_points[-1]
            if "실패" not in self.success_msg:
                 self.success_msg = "경로 생성 성공: 막힌 구간은 자동으로 우회합니다!"
            return True
        else:
            self.success_msg = "경로를 생성할 수 없습니다. (R키로 초기화하세요)"
            return False


    def handle_events(self, event, cut_grass=False):
        
        global sim_pixhawk_active
        mx, my = pygame.mouse.get_pos()
        self.mousing_obstacle = pygame.Rect(mx-25, my-25, 50, 50)
        
        # 잔디깎기/자율주행 공통: 마우스 클릭으로 경로점 추가
        if not self.robot.is_moving:
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.mowing_area_rect.collidepoint(mx, my):
                    self.user_defined_path.append((mx, my))
                    self.success_msg = f"경로점 {len(self.user_defined_path)}번 추가됨."
                    return

        # K key: Create Obstacle (경로 초기화 방지)
        if event.type == pygame.KEYDOWN and event.key == pygame.K_k:
            new_obs = pygame.Rect(mx-25, my-25, 50, 50)
            self.obstacles.append(new_obs)
            # 장애물 추가 시 경로 초기화 방지: path_points와 user_defined_path 유지
            self.success_msg = "장애물이 추가되었습니다. 시작 버튼을 눌러 경로를 다시 탐색하세요."
            return

        # 마우스 우클릭으로 장애물 제거 기능 추가
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            clicked_pos = (mx, my)
            min_dist = float('inf')
            closest_obs = None
            
            for obs in self.obstacles:
                obs_center = obs.center
                dist = math.dist(clicked_pos, obs_center)
                
                if dist < 50: 
                    if dist < min_dist:
                        min_dist = dist
                        closest_obs = obs
            
            if closest_obs: 
                self.obstacles.remove(closest_obs)
                self.path_points = [] 
                self.user_defined_path = []
            return

        # START 버튼/SPACE: 경로 실행
        if start_button.is_clicked(event) or (event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE):
            if self.robot.is_moving:
                return
                
            self.mousing_obstacle = None
            if cut_grass:
                self.mark_obstacle_grass()

            if not self.user_defined_path:
                self.success_msg = "경로를 먼저 지정해주세요 (왼쪽 클릭)."
                return

            current_start = (self.robot.pos.x, self.robot.pos.y)
            self.generate_full_path(current_start, self.user_defined_path)
            
            if sim_pixhawk_active:
                 if "성공" in self.success_msg:
                     self.success_msg = "GPS 미션 시작! A*로 계산된 경로를 따릅니다."
                 elif "실패" in self.success_msg:
                     self.success_msg += " (GPS 경로 생성 실패)"
            

        # R 키: 초기화
        if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
            self.obstacles.clear()
            self.path_points.clear()
            self.user_defined_path.clear() 
            self.robot.finished = False
            self.robot.is_moving = False
            self.current_target_idx = 0
            self.success_msg = ""
            
            grass_cols = self.mowing_area_rect.width // self.grid_size
            grass_rows = self.mowing_area_rect.height // self.grid_size
            self.grass = [[True for _ in range(grass_cols)] for _ in range(grass_rows)]
            self.robot = Robot(WIDTH//2, HEIGHT - 80, self)
            self.goal_point = None
            
    def draw_obstacles(self, surf):
        for obs in self.obstacles:
            pygame.draw.rect(surf, OBSTACLE_COLOR, obs)
            pygame.draw.rect(surf, BLACK, obs, 2)
            
        if not self.robot.is_moving and self.mousing_obstacle:
             pygame.draw.rect(surf, RED, self.mousing_obstacle, 2)
             
# ====================================================================
# Main Loop Setup
# ====================================================================

buttons = [
    Button((WIDTH//2 - 150, HEIGHT//2 - 100, 300, 50), "픽스호크 GPS 자율주행"), # 새 메뉴
    Button((WIDTH//2 - 150, HEIGHT//2 - 30, 300, 50), "자율주행 시뮬 체험"),
    Button((WIDTH//2 - 150, HEIGHT//2 + 40, 300, 50), "잔디깎기 시뮬레이션"),
]

START_BTN_W, START_BTN_H = 150, 40
start_button = Button(
    (WIDTH//2 - START_BTN_W//2, HEIGHT - 50, START_BTN_W, START_BTN_H), 
    "시작", 
    color=START_BUTTON_COLOR, 
    hover_color=(255, 150, 150)
)


menu_active = True
sim_active = False
sim_grass_active = False
sim_pixhawk_active = False # 픽스호크 모드 상태 추가

running = True
sim = Simulation(WIDTH, HEIGHT)

# ====================================================================
# Main Game Loop
# ====================================================================

while running:
    
    # 모드에 따라 배경색 설정
    if menu_active:
        screen.fill(MENU_BG)
    elif sim_grass_active: 
        screen.fill(WHITE)
    elif sim_pixhawk_active or sim_active: 
        screen.fill(GREEN)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
        if menu_active:
            for btn in buttons:
                if btn.is_clicked(event):
                    sim.obstacles.clear() 
                    sim.path_points.clear()
                    sim.user_defined_path.clear() 
                    sim.success_msg = ""
                    sim.mousing_obstacle = None

                    if btn.text == "픽스호크 GPS 자율주행":
                        menu_active = False
                        sim_active = True
                        sim_grass_active = False
                        sim_pixhawk_active = True # 픽스호크 모드 활성화
                        
                        sim.robot = Robot(WIDTH//2, HEIGHT-80, sim)
                        sim.goal_point = None
                        sim.path_points = []
                        sim.current_target_idx = 0
                        sim.robot.is_moving = False 
                        sim.success_msg = "픽스호크 모드: 좌클릭으로 GPS 목표점을 설정하세요."
                        
                    elif btn.text == "자율주행 시뮬 체험":
                        menu_active = False
                        sim_active = True
                        sim_grass_active = False
                        sim_pixhawk_active = False
                        
                        sim.robot = Robot(WIDTH//2, HEIGHT-80, sim)
                        sim.goal_point = None
                        sim.path_points = []
                        sim.current_target_idx = 0
                        sim.robot.is_moving = False 
                        
                    elif btn.text == "잔디깎기 시뮬레이션":
                        menu_active = False
                        sim_active = True
                        sim_grass_active = True
                        sim_pixhawk_active = False
                        
                        grass_cols = sim.mowing_area_rect.width // sim.grid_size
                        grass_rows = sim.mowing_area_rect.height // sim.grid_size
                        sim.grass = [[True for _ in range(grass_cols)] for _ in range(grass_rows)]
                        sim.robot = Robot(WIDTH//2, HEIGHT - 80, sim) 
                        sim.path_points = []
                        sim.current_target_idx = 0
                        sim.robot.is_moving = False
                        sim.goal_point = None
                        
        else:
            sim.handle_events(event, cut_grass=sim_grass_active)

    if sim_active:
        if sim_grass_active:
            sim.draw_grass(screen)

            # 깎인 잔디 계산 (화면 표시용)
            cut_count = 0
            total_cells = 0
            gs = sim.grid_size
            area = sim.mowing_area_rect

            for y, row in enumerate(sim.grass):
                for x, alive in enumerate(row):
                    total_cells += 1
                    
                    is_obstacle_area = False
                    grass_rect = pygame.Rect(x * gs + area.left, y * gs + area.top, gs, gs)
                    for obs in sim.obstacles:
                        if obs.colliderect(grass_rect):
                            is_obstacle_area = True
                            break
                    
                    if not alive or is_obstacle_area:
                          cut_count += 1
            
            if cut_count == total_cells and total_cells > 0:
                  sim.success_msg = "잔디 깎기 성공! (R키로 재시작)"

        sim.draw_obstacles(screen)
        
        # 사용자 지정 경로 그리기 (모든 모드 공통)
        if sim.user_defined_path and not sim.robot.is_moving:
            if len(sim.user_defined_path) > 1:
                pygame.draw.lines(screen, YELLOW, False, sim.user_defined_path, 3) 
            for i, p in enumerate(sim.user_defined_path):
                pygame.draw.circle(screen, YELLOW, p, 8)
                txt_num = font.render(str(i+1), True, BLACK)
                screen.blit(txt_num, (p[0] - txt_num.get_width()//2, p[1] - txt_num.get_height()//2))


        # 실행 중인 경로 그리기
        draw_path = sim.robot.is_moving or sim.path_points
        
        # 경로 색상 설정 (자율주행/픽스호크: LIME_GREEN, 잔디깎기: BLUE)
        path_color = LIME_GREEN if not sim_grass_active else BLUE
        dot_color = DARK_LIME if not sim_grass_active else BLUE
        target_color = LIME_GREEN if not sim_grass_active else YELLOW
        
        if sim.path_points and len(sim.path_points) > 1 and draw_path:
            pygame.draw.lines(screen, path_color, False, sim.path_points, 2)
            if not sim_grass_active:
                for p in sim.path_points:
                    pygame.draw.circle(screen, dot_color, (int(p[0]), int(p[1])), 4)

        if sim.goal_point and draw_path:
            pygame.draw.circle(screen, target_color, (int(sim.goal_point[0]), int(sim.goal_point[1])), 8)
            pygame.draw.circle(screen, BLACK, (int(sim.goal_point[0]), int(sim.goal_point[1])), 3)


        sim.robot.update()
        sim.robot.draw(screen)
        
        sim.update_particles()
        sim.draw_particles(screen)
        
        if not sim.robot.is_moving and not sim.robot.finished:
            start_button.draw(screen)

        if sim.robot.finished and not sim_grass_active:
            sim.success_msg = "목표도착 성공! (좌클릭으로 새로운 경로점 추가)"
            
        # 픽스호크 모드 시 로봇의 GPS 좌표 표시
        if sim_pixhawk_active:
            current_gps = sim.to_gps((sim.robot.pos.x, sim.robot.pos.y))
            gps_txt = font_small.render(
                f"GPS: Lat {current_gps[0]:.5f}, Lon {current_gps[1]:.5f}", True, BLACK
            )
            screen.blit(gps_txt, (WIDTH - gps_txt.get_width() - 8, 8))


        # 안내 메시지 업데이트
        base_guide = "좌클릭: 경로점 추가 | K: 장애물 생성 | 우클릭: 장애물 제거 | R: 모두 초기화 | SPACE: 경로 실행"
        
        if sim_pixhawk_active:
            mode_text = "픽스호크 GPS 모드 (장애물 회피 활성화) | "
        elif sim_grass_active:
             mode_text = "잔디깎기 모드 | "
        else:
             mode_text = "자율주행 시뮬 모드 | "
             
        if sim.robot.is_moving:
            txt1 = font.render(f"{mode_text} 시뮬레이션 중 | R: 초기화", True, BLACK)
        else:
            txt1 = font.render(f"{mode_text} 경로 지정 단계 | {base_guide} ", True, BLACK)
             
        txt2 = font.render("제작자:Dangel(던젤)", True, BLACK)
        txt3 = font.render("제작일:5일", True, BLACK)
        txt4 = font.render("사용언어:파이썬", True, BLACK)
        txt5 = font.render("무단 배포는 금지이며 변조도 금지합니다", True, BLACK)

        screen.blit(txt1, (8, 8))
        screen.blit(txt2, (8, 40))
        screen.blit(txt3, (8, 70))
        screen.blit(txt4, (8, 100))
        screen.blit(txt5, (8, 690))
        
        if sim.success_msg:
            txt_success = font.render(sim.success_msg, True, RED)
            screen.blit(txt_success, (WIDTH//2 - txt_success.get_width()//2, HEIGHT - 40))

    if menu_active:
        for btn in buttons:
            btn.draw(screen)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()