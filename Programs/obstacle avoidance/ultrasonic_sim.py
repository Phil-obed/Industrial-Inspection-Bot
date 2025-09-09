import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Create a figure and axis
fig, ax = plt.subplots(figsize=(10, 10))

# Robot parameters
robot_width = 3.5
robot_height = 5.0
robot_x = 0  # center X
robot_y = 0  # center Y

# Draw robot body
robot = patches.Rectangle(
    (robot_x - robot_width/2, robot_y - robot_height/2), 
    robot_width,
    robot_height,
    linewidth=2,
    edgecolor='none',
    facecolor='black'
)

# Wheels
wheel1 = patches.Rectangle(((robot_x - robot_width/2)-1.1, robot_y - robot_height/2), 1, 2, facecolor='black')
wheel2 = patches.Rectangle(((robot_x - robot_width/2)-1.1, 0.5), 1, 2, facecolor='black')
wheel3 = patches.Rectangle((1.85, robot_y - robot_height/2), 1, 2, facecolor='black')
wheel4 = patches.Rectangle((1.85, 0.5), 1, 2, facecolor='black')

# Add robot + wheels
ax.add_patch(robot)
ax.add_patch(wheel1)
ax.add_patch(wheel2)
ax.add_patch(wheel3)
ax.add_patch(wheel4)

# ---- Ultrasonic Sensor Visuals ----
def add_sensor(ax, x, y, angle_deg, fov=15, max_range=1, color="orange"):
    """
    Adds an ultrasonic sensor visualization as a fan-shaped arc.
    """
    arc = patches.Wedge(
        (x, y),                   # position of sensor
        max_range,                # radius
        angle_deg - fov/2,        # start angle
        angle_deg + fov/2,        # end angle
        facecolor=color,
        alpha=0.3
    )
    ax.add_patch(arc)
    # also draw sensor point
    ax.plot(x, y, "ko")

# Add front sensors
add_sensor(ax, 0, robot_height/2, 90, fov=30, max_range=4)                      # center front
add_sensor(ax, -robot_width/2, robot_height/2, 110, fov=30, max_range=4)        # front left angled outward
add_sensor(ax, robot_width/2, robot_height/2, 70, fov=30, max_range=4)          # front right angled outward

# Add side sensors
add_sensor(ax, -robot_width/2, 0, 180, fov=30, max_range=3)  # left side
add_sensor(ax, robot_width/2, 0, 0, fov=30, max_range=3)     # right side

# Plot setup
ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_aspect('equal', adjustable='box')
plt.title("Ultrasonic Visualization")
plt.show()
