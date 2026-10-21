
"""
Live Mapping Rover plotter using UDP over Wi-Fi.

Expected records:
    DATA,sequence,distance_mm,angle_raw,z_mm

The receiver sends b"ACK" to the sender after every UDP datagram.
Status records are printed but not plotted.
"""

import math
import queue
import socket
import threading

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ------------------------- USER CONFIGURATION -------------------------
# Listen on all laptop network interfaces.
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

PIPE_DIAMETER_MM = 254.0
PIPE_RADIUS_MM = PIPE_DIAMETER_MM / 2.0
SENSOR_HEIGHT_FROM_BOTTOM_MM = 85.0
SENSOR_X_OFFSET_MM = 0.0
SENSOR_Y_OFFSET_MM = SENSOR_HEIGHT_FROM_BOTTOM_MM - PIPE_RADIUS_MM

SERVO_UNITS_PER_REV = 4096.0
MIN_DISTANCE_MM = 1.0
MAX_DISTANCE_MM = 1000.0
MAX_DISPLAY_POINTS = 30000
PLOT_INTERVAL_MS = 100
HEATMAP_MARKER_SIZE = 12
SHOW_PIPE_REFERENCE = True
# ---------------------------------------------------------------------

records_queue = queue.Queue()
receiver_status = {"message": "Waiting for UDP measurements..."}
stop_event = threading.Event()


def parse_record(line):
    """Return (sequence, distance_mm, angle_raw, z_mm) or None."""
    fields = line.strip().split(",")

    if len(fields) != 5 or fields[0].strip() != "DATA":
        return None

    try:
        sequence = int(fields[1].strip())
        distance_mm = float(fields[2].strip())
        angle_raw = float(fields[3].strip())
        z_mm = float(fields[4].strip())
        
    except ValueError:
        return None

    if sequence < 0 or not all(
        math.isfinite(value)
        for value in (distance_mm, angle_raw, z_mm)
    ):
        return None

    return sequence, distance_mm, angle_raw, z_mm


def udp_receiver():
    """Receive UDP datagrams, process records, and ACK each datagram."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)

    try:
        sock.bind((LISTEN_IP, LISTEN_PORT))
        receiver_status["message"] = (
            f"Listening for UDP on {LISTEN_IP}:{LISTEN_PORT}"
        )
        print(receiver_status["message"])

        while not stop_event.is_set():
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:
                if not stop_event.is_set():
                    print(f"UDP receive error: {exc}")
                break

            # Each UDP datagram is one message from the ESP32.
            # splitlines() also supports multiple records in one datagram.
            text = data.decode("utf-8", errors="replace").strip()

            try:
                lines = text.splitlines()

                for line in lines:
                    line = line.strip()
                    if not line:
                        continue

                    if not line.startswith("DATA,"):
                        print(f"ESP32 {addr[0]}:{addr[1]}: {line}")
                        receiver_status["message"] = line
                        continue

                    parsed = parse_record(line)

                    if parsed is None:
                        print("Ignoring malformed record:", line)
                        continue

                    sequence, distance_mm, angle_raw, z_mm = parsed
                    angle_deg = (
                        angle_raw * 360.0 / SERVO_UNITS_PER_REV
                    ) % 360.0

                    print(
                        f"Sequence: {sequence} | "
                        f"Distance: {distance_mm:.0f} mm | "
                        f"Angle: {angle_deg:.2f} degrees | "
                        f"Z: {z_mm:.0f} mm"
                    )

                    if MIN_DISTANCE_MM <= distance_mm <= MAX_DISTANCE_MM:
                        records_queue.put(
                            (sequence, z_mm, angle_deg, distance_mm)
                        )

            finally:
                # ACK every received UDP datagram, even if its record
                # is a status message or is not valid DATA.
                try:
                    sock.sendto(b"ACK", addr)
                    print(
                        f"ACK sent to {addr[0]}:{addr[1]}"
                    )
                except OSError as exc:
                    print(f"Failed to send ACK: {exc}")

    finally:
        sock.close()


# ----------------------------- PLOT ----------------------------------
plt.ion()
fig = plt.figure(figsize=(15, 7))
ax3d = fig.add_subplot(121, projection="3d")
ax_heat = fig.add_subplot(122)

ax3d.set_title("Live 3D Pipe Scan")
ax3d.set_xlabel("X (mm)")
ax3d.set_ylabel("Y (mm)")
ax3d.set_zlabel("Z along pipe (mm)")

xy_limit = max(PIPE_RADIUS_MM * 1.5, MAX_DISTANCE_MM)
ax3d.set_xlim(-xy_limit, xy_limit)
ax3d.set_ylim(-xy_limit, xy_limit)
ax3d.set_zlim(0, 500)
ax3d.set_box_aspect((1, 1, 2))
ax3d.view_init(elev=25, azim=35)

ax_heat.set_title("LiDAR Distance Heatmap")
ax_heat.set_xlabel("Angle (degrees)")
ax_heat.set_ylabel("Z along pipe (mm)")
ax_heat.set_xlim(0, 360)

heat_scatter = ax_heat.scatter(
    [], [], c=[], cmap="turbo",
    vmin=MIN_DISTANCE_MM, vmax=MAX_DISTANCE_MM,
    s=HEATMAP_MARKER_SIZE
)

colorbar = fig.colorbar(heat_scatter, ax=ax_heat)
colorbar.set_label("Distance (mm)")

if SHOW_PIPE_REFERENCE:
    theta = np.linspace(0, 2 * np.pi, 36)
    z_reference = np.linspace(0, 500, 20)
    theta_grid, z_grid = np.meshgrid(theta, z_reference)

    ax3d.plot_wireframe(
        PIPE_RADIUS_MM * np.cos(theta_grid),
        PIPE_RADIUS_MM * np.sin(theta_grid),
        z_grid,
        color="gray",
        alpha=0.18,
        linewidth=0.5,
        rstride=2,
        cstride=2
    )

point_scatter = ax3d.scatter(
    [], [], [], c=[],
    cmap="turbo",
    vmin=MIN_DISTANCE_MM,
    vmax=MAX_DISTANCE_MM,
    s=8
)

status_text = fig.suptitle("Waiting for UDP measurements...")

z_values = []
angle_values = []
distance_values = []
x_values = []
y_values = []
z_coordinates = []


def update_plot(_frame):
    added = 0

    while True:
        try:
            _sequence, z_mm, angle_deg, distance_mm = (
                records_queue.get_nowait()
            )
        except queue.Empty:
            break

        angle_rad = math.radians(angle_deg)
        x_mm = SENSOR_X_OFFSET_MM + distance_mm * math.cos(angle_rad)
        y_mm = SENSOR_Y_OFFSET_MM + distance_mm * math.sin(angle_rad)

        z_values.append(z_mm)
        angle_values.append(angle_deg)
        distance_values.append(distance_mm)
        x_values.append(x_mm)
        y_values.append(y_mm)
        z_coordinates.append(z_mm)
        added += 1

    if len(distance_values) > MAX_DISPLAY_POINTS:
        excess = len(distance_values) - MAX_DISPLAY_POINTS

        del z_values[:excess]
        del angle_values[:excess]
        del distance_values[:excess]
        del x_values[:excess]
        del y_values[:excess]
        del z_coordinates[:excess]

    if added > 0:
        points_3d = np.column_stack(
            (x_values, y_values, z_coordinates)
        )

        point_scatter._offsets3d = (
            points_3d[:, 0],
            points_3d[:, 1],
            points_3d[:, 2]
        )
        point_scatter.set_array(np.asarray(distance_values))

        heat_scatter.set_offsets(
            np.column_stack((angle_values, z_values))
        )
        heat_scatter.set_array(np.asarray(distance_values))

        z_max = max(z_values) if z_values else 100.0
        display_z_max = max(100.0, z_max + 20.0)

        ax3d.set_zlim(0, display_z_max)
        ax_heat.set_ylim(0, display_z_max)

    latest_z = f"{z_values[-1]:.0f} mm" if z_values else "--"

    status_text.set_text(
        f"Live 3D Pipe Scan | {receiver_status['message']} | "
        f"Points: {len(distance_values)} | Latest Z: {latest_z}"
    )

    return point_scatter, heat_scatter, status_text


def main():
    receiver_thread = threading.Thread(
        target=udp_receiver,
        daemon=True
    )
    receiver_thread.start()

    animation = FuncAnimation(
        fig,
        update_plot,
        interval=PLOT_INTERVAL_MS,
        blit=False,
        cache_frame_data=False
    )

    try:
        plt.show(block=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.close(fig)


if __name__ == "__main__":
    main()
