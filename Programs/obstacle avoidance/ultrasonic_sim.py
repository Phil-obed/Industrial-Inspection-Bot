import serial
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

# ---------------- USER CONFIG ----------------
PORT = "/dev/ttyUSB0"
BAUD = 115200
NUM_SENSORS = 5
MAX_RANGE = [400, 400, 400, 300, 300]  # cm, per sensor max range (tweak if needed)

# Sensor layout: (x, y, angle, max_range, fov)
sensors = [
    (0, 2.5, 90, 400, 30),   # front mid
    (-1.75, 2.5, 110, 400, 30), # front left
    (1.75, 2.5, 70, 400, 30),  # front right
    (-1.75, 0, 180, 300, 30),  # left
    (1.75, 0, 0, 300, 30),     # right
]   

# ---------------- SERIAL SETUP ----------------
ser = serial.Serial(PORT, BAUD, timeout=1)

# ---------------- PLOT SETUP ----------------
fig, ax = plt.subplots(figsize=(8, 8))

# Robot body
robot_width, robot_height = 3.5, 5.0
robot = patches.Rectangle((-robot_width/2, -robot_height/2),
                          robot_width, robot_height,
                          facecolor="black")
ax.add_patch(robot)

# Wheels (just visuals)
wheels = [
    patches.Rectangle((-robot_width/2 - 1.1, -robot_height/2), 1, 2, facecolor="black"),
    patches.Rectangle((-robot_width/2 - 1.1, 0.5), 1, 2, facecolor="black"),
    patches.Rectangle((robot_width/2 + 0.1, -robot_height/2), 1, 2, facecolor="black"),
    patches.Rectangle((robot_width/2 + 0.1, 0.5), 1, 2, facecolor="black"),
]
for w in wheels:
    ax.add_patch(w)

# Sensor arcs
sensor_arcs = []
for (x, y, angle, max_range, fov) in sensors:
    bg = patches.Wedge((x, y), max_range, angle - fov/2, angle + fov/2,
                       facecolor="gray", alpha=0.1)
    arc = patches.Wedge((x, y), 0.01, angle - fov/2, angle + fov/2,
                        facecolor="none", alpha=0.6)
    ax.add_patch(bg)
    ax.add_patch(arc)
    ax.plot(x, y, "ko")
    sensor_arcs.append(arc)

ax.set_xlim(-6, 6)
ax.set_ylim(-6, 6)
ax.set_aspect("equal")
ax.set_title("Ultrasonic Sensor Visualization")

# ---------------- UPDATE FUNCTION ----------------
def update(frame):
    try:
        line = ser.readline().decode().strip()
        if not line:
            return
        values = [float(v) for v in line.split(",")]
        if len(values) != NUM_SENSORS:
            return

        for val, arc, (x, y, angle, max_r, fov) in zip(values, sensor_arcs, sensors):
            dist = min(val, max_r)  # clamp
            arc.set_radius(dist)

            # color map
            if dist < max_r * 0.3:
                arc.set_facecolor("red")
            elif dist < max_r * 0.7:
                arc.set_facecolor("yellow")
            else:
                arc.set_facecolor("green")
    except Exception as e:
        print("Error:", e)

# RUN
ani = FuncAnimation(fig, update, interval=50)  # ~20 FPS
plt.show()
