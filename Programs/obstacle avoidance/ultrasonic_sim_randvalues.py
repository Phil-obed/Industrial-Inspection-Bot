import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
from matplotlib.animation import FuncAnimation

# --- Robot Setup ---
robot_width, robot_height = 3.5, 5.0
robot_x, robot_y = 0, 0

# Sensor positions & orientations
sensors = [
    (0, robot_height/2, 90, 4, 30),                # center front
    (-robot_width/2, robot_height/2, 110, 4, 30),  # front left angled
    (robot_width/2, robot_height/2, 70, 4, 30),    # front right angled
    (-robot_width/2, 0, 180, 3, 30),               # left side
    (robot_width/2, 0, 0, 3, 30),                  # right side
]

# --- Generate Random Test Distances ---
def random_distance(max_range=4, probability_of_none=0.1):
    """Random distance or None (simulate no detection)."""
    if random.random() < probability_of_none:
        return None
    return round(random.uniform(0.5, max_range), 2)

# --- Init Plot ---
fig, ax = plt.subplots(figsize=(8, 8))

# Draw robot body once
robot = patches.Rectangle(
    (robot_x - robot_width/2, robot_y - robot_height/2), 
    robot_width, robot_height, facecolor="black"
)
wheels = [
    patches.Rectangle(((robot_x - robot_width/2)-1.1, robot_y - robot_height/2), 1, 2, facecolor="black"),
    patches.Rectangle(((robot_x - robot_width/2)-1.1, 0.5), 1, 2, facecolor="black"),
    patches.Rectangle((1.85, robot_y - robot_height/2), 1, 2, facecolor="black"),
    patches.Rectangle((1.85, 0.5), 1, 2, facecolor="black"),
]
ax.add_patch(robot)
for w in wheels: ax.add_patch(w)

# Create sensor visuals (background + detection arc)
sensor_backgrounds = []
sensor_arcs = []
for (x, y, angle, max_range, fov) in sensors:
    bg = patches.Wedge((x, y), max_range, angle - fov/2, angle + fov/2,
                       facecolor="gray", alpha=0.2)
    arc = patches.Wedge((x, y), 0.01, angle - fov/2, angle + fov/2,
                        facecolor="none", alpha=0.5)
    ax.add_patch(bg)
    ax.add_patch(arc)
    ax.plot(x, y, "ko")
    sensor_backgrounds.append(bg)
    sensor_arcs.append(arc)

# Setup plot
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_aspect("equal")  
ax.set_title("Ultrasonic Visualization")

# --- Update Function ---
def update(frame):
    for arc, (x, y, angle, max_range, fov) in zip(sensor_arcs, sensors):
        dist = random_distance(max_range)
        if dist is None:
            arc.set_visible(False)
        else:
            arc.set_visible(True)
            arc.set_radius(dist)
            if dist < max_range * 0.3:
                arc.set_facecolor("red")
            elif dist < max_range * 0.7:
                arc.set_facecolor("yellow")
            else:
                arc.set_facecolor("green")

# --- Run Animation ---
ani = FuncAnimation(fig, update, interval=30)  # ~33 FPS (30ms/frame)
plt.show()
