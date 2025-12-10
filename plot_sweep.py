import re
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = "COM9"
BAUD = 115200

angles = []
distances = []
temps = []

plt.style.use("dark_background")
fig = plt.figure(figsize=(14, 6))
ax1 = fig.add_subplot(121, projection="polar")
ax2 = fig.add_subplot(122)


def temp_color(temp):
    if temp < 15:
        return "#3fb1ff"
    if temp < 20:
        return "#7bd7ff"
    if temp < 25:
        return "#8dd68d"
    if temp < 30:
        return "#f4e05a"
    if temp < 35:
        return "#f98d3f"
    return "#f85149"


sweep_active = False


def update_plot(frame):
    global sweep_active
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if "Starting 360° sweep" in line or "Starting 360° environment sweep" in line:
            sweep_active = True
            print("🚀 SWEEP DETECTED! Starting visualization...\n")
            angles.clear()
            distances.clear()
            temps.clear()
            return

        if "360° sweep complete" in line:
            sweep_active = False
            print(f"\n✓ Sweep complete! Captured {len(angles)} points")
            return

        if not sweep_active:
            return

        match = re.search(r"Angle:\s+([\d.]+)°.*Distance:\s+(\d+)\s+cm.*Temp:\s+([\d.]+)°C", line)
        if match:
            angle = float(match.group(1))
            dist = int(match.group(2))
            temp = float(match.group(3))

            if dist == 0:
                return

            angles.append(angle)
            distances.append(dist)
            temps.append(temp)

            ax1.clear()
            ax1.set_theta_zero_location("N")
            ax1.set_theta_direction(-1)
            ax1.set_ylim(0, 250)
            ax1.set_title("Environment Map", fontsize=14, pad=20)

            for i in range(len(angles)):
                color = temp_color(temps[i])
                ax1.scatter(
                    np.radians(angles[i]),
                    distances[i],
                    c=color,
                    s=80,
                    alpha=0.8,
                    edgecolors="white",
                    linewidth=1,
                )

            ax1.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
            ax1.set_xticklabels(["0°", "45°", "90°", "135°", "180°", "225°", "270°", "315°"])

            ax2.clear()
            if len(angles) > 1:
                ax2.plot(angles, temps, "-", color="#555", linewidth=1, alpha=0.5)

            for i in range(len(angles)):
                color = temp_color(temps[i])
                ax2.scatter(
                    angles[i],
                    temps[i],
                    c=color,
                    s=50,
                    edgecolors="white",
                    linewidth=0.5,
                    zorder=3,
                )

            ax2.set_xlabel("Angle (degrees)", fontsize=12)
            ax2.set_ylabel("Temperature (°C)", fontsize=12)
            ax2.set_title(f"Temperature vs Angle ({len(angles)} points)", fontsize=14)
            ax2.grid(True, alpha=0.3, linestyle="--")
            ax2.set_xlim(0, 360)

            print(f"📍 {angle:6.1f}° | 📏 {dist:4d}cm | 🌡️  {temp:5.1f}°C | Total: {len(angles)}")

    except Exception:
        pass


print("🔌 Connecting to robot...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"✓ Connected to {PORT}")
    print("⏳ Waiting for sweep to start...")
    print("   Press 'n' on robot or use WiFi '360° Sweep' button\n")
except Exception:
    print(f"❌ Failed to connect to {PORT}")
    raise SystemExit(1)

ani = FuncAnimation(fig, update_plot, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()
