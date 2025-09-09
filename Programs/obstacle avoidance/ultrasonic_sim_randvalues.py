import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import random

ROBOT_WIDTH = 3.5
ROBOT_HEIGHT = 5.0
ROBOT_COLOR = "black"

WHEEL_WIDTH = 1.0
WHEEL_HEIGHT = 2.0
WHEEL_OFFSET_X = 1.1
WHEEL_COLOR = "black"

# Sensor
SENSORS = [
    (0, ROBOT_HEIGHT/2, 90, 4, 30),                # center front
    (-ROBOT_WIDTH/2, ROBOT_HEIGHT/2, 110, 4, 30),  # front left angled
    (ROBOT_WIDTH/2, ROBOT_HEIGHT/2, 70, 4, 30),    # front right angled
    (-ROBOT_WIDTH/2, 0, 180, 3, 30),               # left side
    (ROBOT_WIDTH/2, 0, 0, 3, 30),                  # right side
]

FOV_COLOR = "gray"      # sensor FOV background
FOV_ALPHA = 0.2         # transparency
ARC_ALPHA = 0.5         # detection arc transparency

# Distance thresholds for coloring
DIST_THRESHOLDS = {
    "red": 0.3,     # <30% of max range
    "yellow": 0.7,  # <70% of max range
    "green": 1.0    # otherwise
}

# Animation settings
UPDATE_INTERVAL_MS = 50   # 20 Hz (smaller = smoother)
PLOT_LIMIT = 6            # world boundary (x and y)

def random_distance(max_range=4, probability_of_none=0.1):
    """Simulate a random distance measurement or None (no detection)."""
    if random.random() < probability_of_none:
        return None
    return round(random.uniform(0.5, max_range), 2)

def draw_robot(ax):
    # Robot body
    robot = patches.Rectangle(
        (-ROBOT_WIDTH/2, -ROBOT_HEIGHT/2),
        ROBOT_WIDTH, ROBOT_HEIGHT, facecolor=ROBOT_COLOR
    )
    ax.add_patch(robot)

    # Wheels
    wheels = [
        patches.Rectangle((-ROBOT_WIDTH/2 - WHEEL_OFFSET_X, -ROBOT_HEIGHT/2), 
                          WHEEL_WIDTH, WHEEL_HEIGHT, facecolor=WHEEL_COLOR),
        patches.Rectangle((-ROBOT_WIDTH/2 - WHEEL_OFFSET_X, 0.5), 
                          WHEEL_WIDTH, WHEEL_HEIGHT, facecolor=WHEEL_COLOR),
        patches.Rectangle((1.85, -ROBOT_HEIGHT/2), 
                          WHEEL_WIDTH, WHEEL_HEIGHT, facecolor=WHEEL_COLOR),
        patches.Rectangle((1.85 , 0.5), 
                          WHEEL_WIDTH, WHEEL_HEIGHT, facecolor=WHEEL_COLOR),
    ]
    for w in wheels:
        ax.add_patch(w)

def setup_sensors(ax):
    sensor_backgrounds = []
    sensor_arcs = []
    for (x, y, angle, max_range, fov) in SENSORS:
        bg = patches.Wedge((x, y), max_range, angle - fov/2, angle + fov/2,
                           facecolor=FOV_COLOR, alpha=FOV_ALPHA)
        arc = patches.Wedge((x, y), 0.01, angle - fov/2, angle + fov/2,
                            facecolor="none", alpha=ARC_ALPHA)
        ax.add_patch(bg)
        ax.add_patch(arc)
        ax.plot(x, y, "ko")
        sensor_backgrounds.append(bg)
        sensor_arcs.append(arc)
    return sensor_arcs

def update(frame, sensor_arcs):
    for arc, (x, y, angle, max_range, fov) in zip(sensor_arcs, SENSORS):
        dist = random_distance(max_range)
        if dist is None:
            arc.set_visible(False)
        else:
            arc.set_visible(True)
            arc.set_radius(dist)

            # Pick color based on thresholds
            ratio = dist / max_range
            if ratio < DIST_THRESHOLDS["red"]:
                arc.set_facecolor("red")
            elif ratio < DIST_THRESHOLDS["yellow"]:
                arc.set_facecolor("yellow")
            else:
                arc.set_facecolor("green")


# MAIN

fig, ax = plt.subplots(figsize=(8, 8))

draw_robot(ax)
sensor_arcs = setup_sensors(ax)

# Setup plot
ax.set_xlim(-PLOT_LIMIT, PLOT_LIMIT)
ax.set_ylim(-PLOT_LIMIT, PLOT_LIMIT)
ax.set_aspect("equal")
ax.set_title("Ultrasonic Visualization (Modular)")

ani = FuncAnimation(fig, update, fargs=(sensor_arcs,), interval=UPDATE_INTERVAL_MS)
plt.show()
