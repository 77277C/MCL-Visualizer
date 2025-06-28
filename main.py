# ======= 1. IMPORTS & EXTERNAL PROCESS SETUP =======
import pygame       # For graphics and event handling
import math         # For mathematical functions
import sys          # For program exit
import threading    # For running background tasks
import subprocess   # For launching and communicating with the external localization program

# Launch the external localization executable, setting up pipes for communication
subprocess.run("g++ -std=c++17 -I. -o main.exe main.cpp")
process = subprocess.Popen(
    ["main.exe"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    text=True  # Treats input/output as text instead of bytes
)

# Dictionary to store particle data received from the localization program
particles = {}

mcl_pose = (0, 0, 0)

# ======= 2. BACKGROUND THREADS FOR PROCESS I/O =======
# Continuously read stdout lines from the localization process.
# Updates 'particles' dictionary on lines starting with 'particle'.
# Prints position on lines starting with 'pose'.
def refresh_particles():
    global predicted_pose, mcl_pose
    while True:
        line = process.stdout.readline()
        if not line:
            continue  # Skip empty reads
        if line.startswith("particle"):
            # Expected format: 'particle index x y weight'
            _, idx, x, y, w = line.split()
            particles[int(idx)] = (float(x), float(y))
        elif line.startswith("pose"):
            _, x, y, theta = line.split()
            mcl_pose = (float(x), float(y), float(theta))
        else:
            # Print other output messages for debugging
            print(line.strip())

# Continuously read stderr from the localization process and print errors.
def print_stderr():
    while True:
        line = process.stderr.readline()
        if not line:
            continue
        print("ERROR:", line.strip())

# Start threads to handle I/O without blocking
stdout_thread = threading.Thread(target=refresh_particles, daemon=True)
stdout_thread.start()
stderr_thread = threading.Thread(target=print_stderr, daemon=True)
stderr_thread.start()

# Request initial particle data
process.stdin.write("get\n")
process.stdin.flush()

# ======= 3. PYGAME INITIALIZATION & CONSTANTS =======
pygame.init()  # Initialize all pygame modules

# Screen dimensions (pixels)
WIDTH, HEIGHT = 600, 600
# Robot display size (pixels)
ROBOT_SIZE = 40
# Background color (RGB)
BG_COLOR = (80, 80, 80)
# Frames per second
FPS = 100
# Robot movement speed (pixels/frame)
SPEED = 2.5

# Real-world dimensions (meters) from wall to wall
WORLD_X_MIN, WORLD_X_MAX = -1.78308, 1.78308
WORLD_WIDTH = WORLD_X_MAX - WORLD_X_MIN

# Conversion factors
PIXELS_TO_METERS = WORLD_WIDTH / WIDTH        # meters per pixel
conv = PIXELS_TO_METERS * 39.3701             # inches per pixel
PIXELS_PER_INCH  = WIDTH / (WORLD_WIDTH * 39.3701)  # pixels per inch conversion

# Drawing colors
ROBOT_COLOR = (150, 255, 150)  # Light green
LINE_COLOR  = (0, 255, 0)      # Bright green
GRID_COLOR  = (120, 120, 120)  # Light gray

# Create the display surface and clock object
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Visualizer")
clock = pygame.time.Clock()

# Initial robot pose (offset from center)
robot_x = WIDTH // 2 + 10
robot_y = HEIGHT // 2 + 10
robot_angle = 10 * (math.pi / 180)
# Keep previous pose for odometry
previous_robot_x = robot_x
previous_robot_y = robot_y
previous_robot_angle = robot_angle

# Grid layout dimensions
GRID_ROWS = 6
GRID_COLS = 6

# ======= 4. HELPER FUNCTIONS =======
# Draw a GRID_ROWS x GRID_COLS grid overlay on the screen.
def draw_grid():
    cell_w = WIDTH / GRID_COLS
    cell_h = HEIGHT / GRID_ROWS
    for i in range(GRID_COLS + 1):
        x = i * cell_w
        pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, HEIGHT), 1)
    for j in range(GRID_ROWS + 1):
        y = j * cell_h
        pygame.draw.line(screen, GRID_COLOR, (0, y), (WIDTH, y), 1)

# Rotate point (px, py) around center (cx, cy) by 'angle' radians.
def rotate_point(cx, cy, px, py, angle):
    dx, dy = px - cx, py - cy
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    rx = dx * cos_a - dy * sin_a
    ry = dx * sin_a + dy * cos_a
    return rx + cx, ry + cy

# Cast a ray from (cx, cy) in direction 'angle' until it reaches the screen edge.
# Draws the line and returns its endpoint.
def draw_sensor_line(cx, cy, angle):
    dx, dy = math.cos(angle), math.sin(angle)
    intersections = []
    if dx != 0:
        t = (0 - cx) / dx
        y = cy + t * dy
        if t > 0 and 0 <= y <= HEIGHT:
            intersections.append((0, y, t))
        t = (WIDTH - cx) / dx
        y = cy + t * dy
        if t > 0 and 0 <= y <= HEIGHT:
            intersections.append((WIDTH, y, t))
    if dy != 0:
        t = (0 - cy) / dy
        x = cx + t * dx
        if t > 0 and 0 <= x <= WIDTH:
            intersections.append((x, 0, t))
        t = (HEIGHT - cy) / dy
        x = cx + t * dx
        if t > 0 and 0 <= x <= WIDTH:
            intersections.append((x, HEIGHT, t))
    if intersections:
        end_x, end_y, _ = min(intersections, key=lambda item: item[2])
    else:
        end_x, end_y = cx + dx * 1000, cy + dy * 1000
    pygame.draw.line(screen, LINE_COLOR, (cx, cy), (end_x, end_y), 2)
    return end_x, end_y

# ======= 5. MAIN VISUALIZATION LOOP =======
running = True
while running:
    screen.fill(BG_COLOR)
    draw_grid()
    dt = clock.tick(FPS) / 1000
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    mx, my = pygame.mouse.get_pos()
    dx, dy = mx - robot_x, my - robot_y
    dist = math.hypot(dx, dy)
    if dist > SPEED:
        theta = math.atan2(dy, dx)
        robot_x += SPEED * math.cos(theta)
        robot_y += SPEED * math.sin(theta)
        robot_angle = theta
    else:
        robot_angle = math.atan2(dy, dx)
    half = ROBOT_SIZE / 2

    # Convert MCL pose (inches) to screen coordinates
    screen_x = WIDTH // 2 + mcl_pose[0] * PIXELS_PER_INCH
    screen_y = HEIGHT // 2 - mcl_pose[1] * PIXELS_PER_INCH  # invert Y axis

    # Draw a semi-transparent quadrilateral for MCL pose
    half = ROBOT_SIZE / 2
    corners = []
    for off_x, off_y in [(-half, -half), (half, -half), (half, half), (-half, half)]:
        rx, ry = rotate_point(0, 0, off_x, off_y, mcl_pose[2])
        corners.append((screen_x + rx, screen_y + ry))
    pygame.draw.polygon(screen, (200, 200, 200), corners)

    corners.clear()
    for off_x, off_y in [(-half, -half), (half, -half), (half, half), (-half, half)]:
        rx, ry = rotate_point(0, 0, off_x, off_y, robot_angle)
        corners.append((robot_x + rx, robot_y + ry))
    pygame.draw.polygon(screen, ROBOT_COLOR, corners)
    pygame.draw.circle(screen, (0, 255, 0), (int(robot_x), int(robot_y)), 5)
    end_f = draw_sensor_line(robot_x, robot_y, robot_angle)
    end_l = draw_sensor_line(robot_x, robot_y, robot_angle - math.pi/2)
    end_r = draw_sensor_line(robot_x, robot_y, robot_angle + math.pi/2)
    for x_m, y_m in particles.values():
        sx = WIDTH//2 + x_m * PIXELS_PER_INCH
        sy = HEIGHT//2 - y_m * PIXELS_PER_INCH
        pygame.draw.circle(screen, (255, 0, 0), (int(sx), int(sy)), 2)
    dist_px_f = math.hypot(end_f[0] - robot_x, end_f[1] - robot_y)
    dist_px_l = math.hypot(end_l[0] - robot_x, end_l[1] - robot_y)
    dist_px_r = math.hypot(end_r[0] - robot_x, end_r[1] - robot_y)
    dist_in_f = dist_px_f * conv
    dist_in_l = dist_px_l * conv
    dist_in_r = dist_px_r * conv
    for name, d in [("front", dist_in_f), ("left", dist_in_l), ("right", dist_in_r)]:
        process.stdin.write(f"{name} {d}\n")
        process.stdin.flush()
    dx_in  = (robot_x - previous_robot_x) * conv
    dy_in  = (robot_y - previous_robot_y) * conv
    dtheta = robot_angle - previous_robot_angle
    process.stdin.write(f"change {dx_in} {-dy_in} {dtheta}\n")
    process.stdin.flush()
    process.stdin.write("get\n")
    process.stdin.write("pose\n")
    process.stdin.flush()
    previous_robot_x, previous_robot_y = robot_x, robot_y
    previous_robot_angle = robot_angle
    pygame.display.flip()
# ======= 6. CLEANUP =======
process.stdin.write("exit\n")
process.stdin.flush()
pygame.quit()
sys.exit()
