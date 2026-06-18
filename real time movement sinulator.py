import socket
import threading
import json
import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
from scipy.optimize import least_squares

# ============================================================
# CONFIGURATION — edit these to match your setup
# ============================================================
HOST = "192.168.1.102"   # Must match host= in tag_firmware.ino
PORT = 7007

# Anchor positions in cm, measured from a common room corner.
# Order must match anchor IDs: A1, A2, A3
ANCHOR_POSITIONS = np.array([
    [  0,   0],   # Anchor 1 (x, y) in cm
    [120,   0],   # Anchor 2
    [ 60, 195],   # Anchor 3
])

ROOM_WIDTH  = 120   # cm
ROOM_HEIGHT = 210   # cm

# Background floor plan image (put your floorplan.png next to this script).
# If the file is missing a plain white background is used automatically.
IMAGE_FILE = "floorplan.png"

# FIX: Only include anchors whose filtered_distance > this threshold.
# Anchors that haven't ranged yet arrive as 0.0 — skip them in trilateration.
MIN_VALID_DISTANCE = 1.0   # cm

# Maximum position trail length
TRAIL_LENGTH = 150
# ============================================================


# Global state
latest_distances = {}          # {1: float, 2: float, 3: float}
latest_rssi      = {}          # {1: float, 2: float, 3: float}
data_lock        = threading.Lock()
server_running   = True
recv_buffer      = ""


# ── Trilateration ────────────────────────────────────────────
def trilaterate(distances, positions):
    """
    distances : list of (distance_cm, anchor_index) pairs — only valid anchors
    positions : np.array of all anchor (x,y) coords indexed by anchor_index
    Returns (x, y) or None.
    """
    if len(distances) < 2:
        return None  # Need at least 2 valid anchors

    if len(distances) == 2:
        # With only 2 anchors use the midpoint as a rough estimate
        i0, d0 = distances[0]
        i1, d1 = distances[1]
        p0, p1 = positions[i0], positions[i1]
        return (p0 + p1) / 2  # Very rough — trilateration needs ≥3

    # Full least-squares with ≥3 anchors
    def residuals(p):
        return [
            np.sqrt((p[0] - positions[i][0])**2 + (p[1] - positions[i][1])**2) - d
            for i, d in distances
        ]

    x0 = np.mean([positions[i] for i, _ in distances], axis=0)
    try:
        result = least_squares(residuals, x0, method="lm")
        return result.x if result.success else None
    except Exception as e:
        print(f"[TRILATERATE] Error: {e}")
        return None


# ── TCP server thread ─────────────────────────────────────────
def tcp_server():
    global latest_distances, latest_rssi, server_running, recv_buffer

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.settimeout(1.0)   # Allow clean shutdown
        srv.bind((HOST, PORT))
        srv.listen(1)
        print(f"[SERVER] Listening on {HOST}:{PORT}")

        while server_running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[SERVER] Connected by {addr}")
            conn.settimeout(2.0)

            with conn:
                while server_running:
                    try:
                        chunk = conn.recv(2048)
                        if not chunk:
                            print("[SERVER] Client disconnected")
                            break
                        recv_buffer += chunk.decode("utf-8", errors="replace")

                        # Process all complete lines
                        while "\n" in recv_buffer:
                            line, recv_buffer = recv_buffer.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue
                            try:
                                pkt     = json.loads(line)
                                anchors = pkt["anchors"]

                                new_dist = {}
                                new_rssi = {}
                                for key, val in anchors.items():
                                    # key is "A1", "A2", "A3"
                                    try:
                                        anchor_id = int(key[1:])  # strip 'A'
                                        d = float(val["distance"])
                                        r = float(val.get("rssi", 0.0))
                                        if d >= MIN_VALID_DISTANCE:
                                            new_dist[anchor_id] = d
                                            new_rssi[anchor_id] = r
                                    except (ValueError, KeyError):
                                        pass

                                with data_lock:
                                    latest_distances.update(new_dist)
                                    latest_rssi.update(new_rssi)

                                dist_str = " | ".join(
                                    f"A{k}: {v:.1f}cm" for k, v in sorted(new_dist.items()))
                                print(f"[RX] tag={pkt.get('tag_id')}  {dist_str}")

                            except (json.JSONDecodeError, KeyError) as e:
                                print(f"[RX] Bad packet: {e}  raw={line[:80]}")

                    except socket.timeout:
                        continue
                    except (ConnectionResetError, OSError):
                        print("[SERVER] Connection lost")
                        break


# ── Matplotlib setup ──────────────────────────────────────────
fig, ax = plt.subplots(figsize=(9, 8))

# Background image
if os.path.exists(IMAGE_FILE):
    bg = mpimg.imread(IMAGE_FILE)
    ax.imshow(bg, extent=[0, ROOM_WIDTH, 0, ROOM_HEIGHT],
              origin="lower", alpha=0.5, zorder=-1)
else:
    ax.set_facecolor("#f0f0f0")
    print(f"[INFO] '{IMAGE_FILE}' not found — using plain background")

# Anchor markers & RSSI text labels
anchor_colors  = ["green", "blue", "magenta", "orange", "cyan"]
anchor_markers = []
anchor_texts   = []
for i, (x, y) in enumerate(ANCHOR_POSITIONS):
    col = anchor_colors[i % len(anchor_colors)]
    mkr, = ax.plot(x, y, marker="^", color=col, markersize=14,
                   label=f"Anchor {i+1}", zorder=3)
    ax.annotate(f"A{i+1}", xy=(x, y), xytext=(x, y + 8),
                ha="center", color=col, fontsize=9, fontweight="bold")
    txt = ax.text(x, y - 14, "", color=col, ha="center", va="top",
                  fontsize=7)
    anchor_markers.append(mkr)
    anchor_texts.append(txt)

# Tag dot and trail
tag_dot,  = ax.plot([], [], "ro", markersize=11, label="Tag", zorder=5)
trail_line, = ax.plot([], [], "r-", alpha=0.4, linewidth=1.2, zorder=4)
status_txt  = ax.text(0.02, 0.97, "Waiting for data…",
                      transform=ax.transAxes, va="top",
                      fontsize=9, color="gray")

trail_x, trail_y = [], []

ax.set_xlim(-10, ROOM_WIDTH + 10)
ax.set_ylim(-10, ROOM_HEIGHT + 10)
ax.set_aspect("equal")
ax.grid(True, linestyle="--", alpha=0.5)
ax.legend(loc="upper right", fontsize=8)
ax.set_title("Real-time UWB Indoor Positioning  (3 Anchors)", pad=10)
ax.set_xlabel("X  (cm)")
ax.set_ylabel("Y  (cm)")


# ── Animation ─────────────────────────────────────────────────
def update(_frame):
    global trail_x, trail_y

    with data_lock:
        dist_snap = dict(latest_distances)
        rssi_snap = dict(latest_rssi)

    # Update RSSI labels
    for i, txt in enumerate(anchor_texts):
        aid = i + 1
        if aid in rssi_snap:
            txt.set_text(f"{rssi_snap[aid]:.1f} dBm")
        else:
            txt.set_text("")

    if not dist_snap:
        status_txt.set_text("Waiting for data…")
        return tag_dot, trail_line, status_txt, *anchor_texts

    # Build list of (anchor_index, distance) for valid readings
    valid = []
    for i in range(len(ANCHOR_POSITIONS)):
        aid = i + 1  # anchor IDs are 1-based
        if aid in dist_snap:
            valid.append((i, dist_snap[aid]))

    status_txt.set_text(
        f"Valid anchors: {[i+1 for i, _ in valid]}  |  "
        + "  ".join(f"A{i+1}={d:.0f}cm" for i, d in valid)
    )

    pos = trilaterate(valid, ANCHOR_POSITIONS)
    if pos is not None:
        x, y = float(pos[0]), float(pos[1])
        print(f"[POS] x={x:.1f} cm  y={y:.1f} cm")

        tag_dot.set_data([x], [y])
        trail_x.append(x)
        trail_y.append(y)
        if len(trail_x) > TRAIL_LENGTH:
            trail_x.pop(0)
            trail_y.pop(0)
        trail_line.set_data(trail_x, trail_y)
    else:
        status_txt.set_text(status_txt.get_text() + "  (trilateration failed)")

    return tag_dot, trail_line, status_txt, *anchor_texts


# ── Entry point ───────────────────────────────────────────────
if __name__ == "__main__":
    t = threading.Thread(target=tcp_server, daemon=True)
    t.start()

    ani = animation.FuncAnimation(
        fig, update, interval=100, blit=False, cache_frame_data=False
    )

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down…")
    finally:
        server_running = False
        plt.close()
