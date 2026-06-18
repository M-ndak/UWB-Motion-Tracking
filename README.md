# UWB Indoor Real-Time Location System (RTLS)

A DS-TWR (Double-Sided Two-Way Ranging) indoor positioning system built with ESP32 DevKit boards and Qorvo DWM3000EVB UWB modules. Three fixed anchors range against a mobile tag; computed distances are streamed over Wi-Fi to a Python visualizer that trilaterates and plots the tag's position in real time on a floor plan.

---

## Hardware

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 DevKit v1 | 4 | One per anchor, one for tag |
| Qorvo DWM3000EVB | 4 | UWB radio module |
| Micro-USB cables | 4 | Power & flashing |

### Wiring (identical for all 4 boards)

| DWM3000 Pin | ESP32 GPIO |
|-------------|-----------|
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |
| CS | 5 |
| RST | 27 |
| IRQ | 34 |
| WAKEUP (optional) | 4 |
| 3.3 V | 3.3 V |
| GND | GND |

---

## Repository Structure

```
.
├── anchor_firmware/
│   └── anchor_firmware.ino   # Flash to each anchor (set ANCHOR_ID 1–3)
├── tag_firmware/
│   └── tag_firmware.ino      # Flash to the tag board
├── floor_view.py             # Python visualizer (run on your PC)
├── floorplan.png             # Your room's floor plan image (add this yourself)
└── README.md
```

---

## Quick Start

### 1 — Flash the anchors

Open `anchor_firmware/anchor_firmware.ino` in the Arduino IDE.

Before flashing each board, set its unique ID at the top of the file:

```cpp
#define ANCHOR_ID 1   // Change to 2 for the second anchor, 3 for the third
```

Flash one board at a time and label each physically.

### 2 — Configure and flash the tag

Open `tag_firmware/tag_firmware.ino`. Edit the Wi-Fi and host settings near the top:

```cpp
const char *ssid     = "your_wifi_ssid";
const char *password = "your_wifi_password";
const char *host     = "192.168.x.x";   // Your PC's local IP address
const int   port     = 7007;
```

Flash the board that will act as the mobile tag.

### 3 — Configure the visualizer

Open `floor_view.py`. Set the anchor positions measured from a common room corner, and confirm the host/port match the tag firmware:

```python
HOST = "192.168.x.x"
PORT = 7007

ANCHOR_POSITIONS = np.array([
    [  0,   0],   # Anchor 1 (x, y) in cm
    [120,   0],   # Anchor 2
    [ 60, 195],   # Anchor 3
])

ROOM_WIDTH  = 120   # cm
ROOM_HEIGHT = 210   # cm
```

Place your floor plan image (PNG) in the same directory as the script and name it `floorplan.png`. If the file is absent, a plain grey background is used automatically.

### 4 — Install Python dependencies

```bash
pip install numpy scipy matplotlib
```

### 5 — Run

1. Power all four ESP32 boards.
2. On your PC, run the visualizer first so the TCP server is ready:
   ```bash
   python floor_view.py
   ```
3. The tag will connect automatically once it joins Wi-Fi. Position data appears within a few ranging cycles.

---

## How It Works

### Ranging — DS-TWR

Each ranging cycle follows a four-message exchange between tag and anchor:

```
Tag                          Anchor
 |--- Poll (stage 1) -------->|
 |<-- Response (stage 2) -----|
 |--- Final (stage 3) -------->|
 |<-- RT Info (stage 4) ------|   ← timing data for ToF calculation
```

The tag cycles through anchors sequentially (A1 → A2 → A3 → A1 …). After each complete exchange, it computes a Time-of-Flight distance and updates the filter.

### Distance filtering

Each anchor maintains a sliding window of the last 5 raw distance readings. A median filter over the valid samples in that window produces the `filtered_distance` that is sent to the visualizer. The filter starts producing output after the very first valid exchange — there is no artificial hold-off.

### Trilateration

`floor_view.py` uses a weighted least-squares solver (`scipy.optimize.least_squares`) on whichever anchors have a valid reading. With fewer than 3 anchors it falls back to a midpoint estimate; with 3 or more it solves the full system. The computed position is low-pass smoothed and drawn on the floor plan with a configurable trail.

### Wi-Fi transport

The tag sends one JSON line per anchor cycle over a persistent TCP connection:

```json
{
  "tag_id": 10,
  "anchors": {
    "A1": {"distance": 134.72, "raw": 136.10, "rssi": -78.3, "fp_rssi": -80.1, "round_time": 1234567, "reply_time": 654321, "clock_offset": 0.000012},
    "A2": {"distance": 98.45,  ...},
    "A3": {"distance": 211.30, ...}
  }
}
```

Anchors that have not yet completed a ranging cycle are reported with `distance: 0.0` and are skipped by the visualizer.

---

## Configuration Reference

### anchor_firmware.ino

| `#define` | Default | Description |
|-----------|---------|-------------|
| `ANCHOR_ID` | — | **Must be set per board** (1, 2, or 3) |
| `ANTENNA_DELAY` | 16350 | Antenna delay ticks; calibrate for your hardware |
| `RESPONSE_TIMEOUT_MS` | 50 | Milliseconds before declaring a ranging timeout |
| `MAX_RETRIES` | 3 | Consecutive failures before a soft radio reset |

### tag_firmware.ino

| `#define` / variable | Default | Description |
|----------------------|---------|-------------|
| `NUM_ANCHORS` | 3 | Number of anchors in the system |
| `TAG_ID` | 10 | UWB address of the tag |
| `FIRST_ANCHOR_ID` | 1 | Anchors are assigned IDs 1 … NUM_ANCHORS |
| `FILTER_SIZE` | 5 | Median filter window size |
| `MAX_DISTANCE` | 1500 cm | Readings above this are discarded as invalid |
| `ANTENNA_DELAY` | 16350 | Must match the value used on the anchors |

### floor_view.py

| Variable | Default | Description |
|----------|---------|-------------|
| `HOST` / `PORT` | `192.168.1.102` / `7007` | Must match tag firmware |
| `ANCHOR_POSITIONS` | see file | (x, y) of each anchor in cm from room corner |
| `ROOM_WIDTH` / `ROOM_HEIGHT` | 120 / 210 cm | Used to scale the floor plan |
| `MIN_VALID_DISTANCE` | 1.0 cm | Distances below this are treated as "not yet ranged" |
| `TRAIL_LENGTH` | 150 | Number of historical positions drawn |

---

## Calibration

Accurate ranging requires matching the `ANTENNA_DELAY` constant to your specific hardware. To calibrate:

1. Place the tag at a precisely known distance from a single anchor (e.g. 100 cm).
2. Flash anchor and tag with the default delay (`16350`).
3. Observe the reported distance in the Serial Monitor or visualizer.
4. Adjust `ANTENNA_DELAY` up or down (each tick ≈ 15 ps ≈ 0.45 cm) until the reading matches the true distance.
5. Use the same value in both anchor and tag firmware.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `[ERROR] DEV_ID IS WRONG!` | SPI wiring issue or board not powered | Check wiring; ensure 3.3 V supply is stable |
| `[ERROR] IDLE1/IDLE2 FAILED` | DWM3000 not ready at boot | Increase the `delay(2000)` in `setup()` |
| `[ERROR] PGF calibration failed` | Usually clears on next boot | Power-cycle; persists if LDO supply is noisy |
| Tag never receives a Response | Wrong `ANCHOR_ID` on anchor | Verify each anchor has a unique, correct ID |
| Distances wildly inaccurate | `ANTENNA_DELAY` mismatch | Re-calibrate (see above) |
| Visualizer shows no position | Fewer than 2 valid anchor readings | Wait for more ranging cycles; check Wi-Fi and TCP connection |
| TCP connection refused | PC firewall blocking port 7007 | Add an inbound firewall rule for TCP 7007 |

---

## License

This project is licensed under the **GNU General Public License v3.0**.  
See the [LICENSE](LICENSE) file for details.

A copy of the license is also available at <https://www.gnu.org/licenses/gpl-3.0>.
