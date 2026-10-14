# Hardware Setup, Flashing and Demo Guide

## 1. What the guidelines require

| Node | Guideline role | Sensor required? | This project |
|---|---|---|---|
| **N1** | Sensor Node | **Yes** | MPU6050 / GY-521 accelerometer (vibration) |
| **N2** | Sensor / Relay Node | **Yes** | IR flame sensor module (AO + DO) |
| **N3** | Cluster Head / Aggregator / Router | No | IR obstacle sensor is an **optional extra** |
| **N4** | Sink / Gateway | No | none (Wi-Fi + Firebase + dashboard) |

**Minimum hardware:** 4 × ESP32 DevKit (WROOM-32), 1 accelerometer, 1 flame sensor and 4 USB cables. You also need a phone hotspot set to **2.4 GHz**; the ESP32 has no 5 GHz radio.

- **Two sensors are enough.** N3's IR sensor is optional.
- **Without the IR sensor:** set `#define N3_HAS_IR_SENSOR 0` near the top of `node3_cluster_head.ino`.
- **Keeping it is fine:** it adds N3's own reading to every aggregate.

## 2. Wiring

| Node | Module pin | ESP32 pin | Notes |
|---|---|---|---|
| N1 | VCC / GND | 3V3 / GND | GY-521 boards also accept 5 V on VCC, but 3V3 is safest |
| N1 | SDA / SCL | GPIO21 / GPIO22 | Leave AD0 unconnected or tie it to GND (I2C address 0x68; 0x69 also works) |
| N2 | VCC / GND | **3V3** / GND | **Do not power from 5 V**: AO/DO would exceed the 3.3 V limit of ESP32 inputs |
| N2 | AO / DO | GPIO32 / GPIO16 | Turn the module's potentiometer until DO switches near a flame |
| N3 (optional) | VCC / GND / OUT | 3V3 / GND / GPIO4 | Output is LOW when an obstacle is present |
| N4 | – | – | USB power only |

At boot N1 prints which chip it found, for example `WHO_AM_I=0x70 (MPU6500)`. Clone chips (MPU6500, MPU9250) are supported. If you see `MPU6050 NOT FOUND … I2C devices found: none`, the wiring is wrong.

## 3. Placement (important for BLE and routing)

- Put the boards **1–3 m apart**, never touching. Two boards touching gave −24 dBm (0.03 m) in your test.
- Keep the antenna end of each board (the zig-zag trace) free: no metal, hands or laptops on top.
- Tape-measure each board's position (x, y in metres). Enter the values in `NODE_TRUE_POS` in **`arduino codes/common/wsn_ble.h`**, then copy that file into all four node folders. Keep the four points non-collinear (not all on one line).

## 4. Software

1. Install Arduino IDE 2.x.
2. In **Boards Manager**, install **esp32 by Espressif Systems** 3.x.
3. In **Library Manager**, install **NimBLE-Arduino** by h2zero, version 2.x. That's the only library needed; the Adafruit MPU6050 library is no longer used.
4. Board settings:
   - **Tools → Board:** "ESP32 Dev Module"
   - **Upload speed:** 921600
   - **N4 only:** set **Tools → Partition Scheme → "Huge APP (3MB No OTA/1MB SPIFFS)"**. N4 includes BLE, Wi-Fi and HTTPS, which don't fit the default 1.2 MB partition. N1–N3 can keep the default.

## 5. Get the latest code and flash all 4 boards

```
git clone https://github.com/Eshwarnath24/WSN.git      # first time only
cd WSN
git checkout main
git pull
```

Open each sketch from **this** folder and upload it:

| Board | Sketch |
|---|---|
| N4 | `arduino codes/node4_sink/node4_sink.ino` |
| N3 | `arduino codes/node3_cluster_head/node3_cluster_head.ino` |
| N1 | `arduino codes/node1_vibration/node1_vibration.ino` |
| N2 | `arduino codes/node2_flame/node2_flame.ino` |

**All four boards must run the same version.** After reset, every node prints a line like this:
```
[FW] WSN-FW 2.3 (Wi-Fi stays on mesh channel, BLE in SLEEP epoch) | log level 1 ...
```
If any board doesn't print this line, it is still running old firmware. Re-upload it from the folder above.

Power-on order: **N4 → N3 → N1 → N2**. N4 must first connect to the hotspot, because the hotspot's channel becomes the mesh channel.

## 6. Serial Monitor

- **Baud:** 115200, line ending **"Newline"**.
- Use one Serial Monitor window per board (Arduino IDE 2 can open several), or PuTTY / screen.
- Every 5 s each node prints **one compact status block**:
```
==================== N1 Sensor | t=45 s ====================
 SENSOR: accelerometer @0x68 | AX 0.14 AY -0.09 AZ 9.80 | |a| 9.80 m/s^2 (normal) | readings sent 45
 TDMA  : SYNCED (beacon) ch 6 | frame 61 | schedule v1 by N3 "INIT: default schedule" | my slots S1:TX | duty 43%
 ROUTE : next hop N3 (hops to sink 2, cost 4.3) | N2 up -44 dBm ACK n/a | N3 up -45 dBm ACK 96% | N4 up -42 dBm ACK n/a |
 MAC   : sent 51, ACK ok 49, retransmit 2, ACK timeout 2, dropped 0 | CCA busy 8, backoffs 57, deferred 3 | forwarded 0, dup 0
 BLE   : N2 -64 dBm→1.7 m | N3 -66 dBm→2.2 m | N4 -63 dBm→1.5 m | nearest N4 | position (0.03, -0.05) error 0.06 m
=====================================================================
```
(The example values come from the simulator. Yours will differ.)

The firmware does **not** slow down with `delay()`: that would break the time slots and ACK timing. Instead, only important events are printed between the status blocks.

### Commands (type into the Serial Monitor)

| Command | Node | What it does |
|---|---|---|
| `status` | any | Full neighbour, routing and MAC table plus the BLE table |
| `log 0` / `log 1` / `log 2` | any | Summaries only / summaries + events (default) / full packet trace (CSMA, backoff, TX, RX, ACK). Saved in flash. |
| `ble` | any | BLE status: RSSI, distances, nearest node, position |
| `cal <id> <m>` | any | BLE calibration: put node `<id>` exactly `<m>` metres away, then run, e.g. `cal 2 1.0`. Saved in flash. |
| `setn <n>` | any | BLE path-loss exponent (2.0 = open space … 3.5 = cluttered lab) |
| `collision` | N4 | N1 and N2 transmit at the same moment (contention demo; full trace shown for 3 s) |
| `kill 3 4` / `restore` | N4 | Inject a link failure between N3 and N4 / restore all links |

## 7. BLE calibration (do once, before the demo)

1. Place N2 exactly 1.0 m from N1 and wait 10 s.
2. On N1, type `cal 2 1.0`.
3. Repeat on N2 (`cal 1 1.0`), N3 (`cal 4 1.0` with N4 1 m away) and N4 (`cal 3 1.0`).
4. Put the boards back at their measured positions. After about 30 s, each status block shows the BLE distances and the position error.

A neighbour may occasionally show `not heard yet` or `expired`. BLE shares the 2.4 GHz radio with Wi-Fi, so some adverts are missed; a neighbour is kept for 30 s after it was last heard.

**If a node is never discovered** (for example N3 shows `N1: not discovered by BLE (but Wi-Fi link up at -78 dBm)`), the cause is distance or obstruction, not the code:
- BLE is about 10–15 dB weaker than the ESP-NOW link. In your last test N3 heard N2 at −94 dBm, which is at the limit of BLE sensitivity (≈ −95 dBm).
- Move the boards to **1–3 m apart** with a clear line of sight, antennas pointing up, not lying on a laptop or metal table.
- Each BLE neighbour should then read between −55 and −80 dBm.

Since FW 2.3, synced nodes advertise in a short burst (every 25 ms) inside the SLEEP epoch, exactly while every node scans. Each scan therefore hears about 4 adverts per neighbour instead of about 2, and BLE no longer competes with Wi-Fi during the data slots. If a node has only 2 BLE neighbours, N4 still gives a 2-anchor position estimate.

## 8. Demonstration sequence (follows the guideline flow)

| # | Step | What to show |
|---|---|---|
| 1 | **BLE discovery → RSSI → localization** | Boot messages of N1 (`[BLE] Discovered …`), then the BLE line in its status block |
| 2 | **Wi-Fi initialization** | N4: `[WiFi] CONNECTED … Channel=X`. N1–N3: `[TDMA] Beacon sync … LOCKED` |
| 3 | **Dynamic schedule assignment** | `[TDMA] Schedule v1 from N3 applied … S1=N1:TX S2=N2:TX S3=N3:TX S4=N2:RELAY` |
| 4 | **Channel sensing → random backoff → transmission → ACK** | Type `log 2` on N1 for 5 s, then `log 1` |
| 5 | **Contention** | Type `collision` on N4. One of N1/N2 shows `channel IDLE → ACK OK`, the other `CCA: BUSY … Contention … defer` and is delivered later. |
| 6 | **Unicast + aggregation** | N3's AGG line: `readings in → aggregate packets out | data reduction …%` |
| 7 | **Multi-hop to the sink** | N4's SINK line: `via N1→N3→N4` |
| 8 | **Schedule update** | Shake N1 hard for 3 s. N3 prints `Schedule update v2: ALERT: N1 extra slot S4`; about 10 s later `NORMAL: alert cleared` |
| 9 | **Link failure → detection → alternate route → recovery** | Unplug N3, **or** type `kill 3 4` on N4. Watch for `Link … DEAD`, `Next hop changed`, `Re-routing … via ALTERNATE`, then N4's SINK line changing to `via N1→N4`. Then type `restore` or plug N3 back in. |
| 10 | **Performance analysis** | N4's PERF line (PDR, loss, delay, throughput, retransmissions, data reduction) and LOCAL line (localization error), plus the dashboard |

## 9. Wi-Fi and Firebase

### Firebase rules (lab setting)
In the Firebase console, open **Realtime Database → Rules**, set the rules below and click **Publish**:
```
{ "rules": { ".read": true, ".write": true } }
```
This removes the `HTTP 401` errors. These rules make the database public. That's fine for a lab demo; lock it down or delete the database afterwards.

### How Wi-Fi works since FW 2.3
At boot, N4 scans once for the hotspot and saves its **channel and BSSID**. That channel is the mesh channel.

Before, if the hotspot dropped N4, the reconnect scanned all 13 channels. During the scan N4 was off the mesh channel, so every packet to N4 failed. The old logs showed exactly this: `Link-layer send failed`, ACK timeouts, `Link … DEAD`, then `E wifi: sta is connecting, cannot set config`.

Now:
- N4 reconnects in the background every 15 s, **on the mesh channel only**. The mesh keeps running and data keeps reaching N4.
- While Wi-Fi is down, the `CLOUD` line shows the reason. Common reasons:

| Reason | Fix |
|---|---|
| `hotspot not found` | The hotspot is off or set to 5 GHz. Switch it to 2.4 GHz. |
| `beacon timeout` | The phone is too far away. Keep it within 3 m of N4. |
| `4-way handshake timeout` or `authentication failed` | Wrong password in `wsn_config.h`. |
| `inactivity` | The phone dropped an idle client. Turn off the hotspot's "auto-disconnect idle devices" / power-saving option. |

- If the hotspot is restarted on a different channel, N4 finds it within about 1 minute. N1–N3 then re-sync to the new channel automatically (`Scanning Wi-Fi Channel …` then `Channel X LOCKED`).

**Keep the phone hotspot ON for the whole demo.** On Android, enable "Extend compatibility" / 2.4 GHz band and disable "Turn off hotspot automatically".

## 10. Dashboard

```
cd dashboard
python -m http.server 8080
```
Open http://localhost:8080 while N4 has internet access.
- N4's status block shows `CLOUD : Wi-Fi connected | Firebase uploads ok N, failed 0`.
- If `failed` keeps increasing, send me the `[Cloud] HTTP …` line; it contains the error code and the free memory.
