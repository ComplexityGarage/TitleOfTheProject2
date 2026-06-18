# Running `meshtastic-fpv-drone-rc` — step by step

This guide takes you from a clean machine to driving the drone over the mesh.
Developed against a **Heltec WiFi LoRa 32 V3** + **BetaFPV Matrix 1S**
(Betaflight), transmitter = **T-Deck** / phone app / `meshtastic` CLI.

> ⚠️ **Keep the propellers OFF for the entire procedure.** This is a bench
> proof-of-concept. Throttle is hard-capped in firmware, but a mesh link has
> latency and no delivery guarantee — never treat it as a real RC link, and obey
> your local LoRa band / drone regulations.

Conventions: `WSL$` = a command in your Linux/WSL shell, `PS>` = Windows
PowerShell. If you work entirely in Linux, run the PowerShell steps in your shell
with the matching `/dev/ttyUSB*` port instead of `COMx`.

---

## 0. What you need

**Hardware**

- Heltec WiFi LoRa 32 V3 (ESP32-S3) — the on-drone Meshtastic node.
- BetaFPV Matrix 1S running Betaflight, with the `RX1`/`TX1` (UART1) pads free.
- A second Meshtastic device as the transmitter (T-Deck, phone app, or any node + CLI).
- Soldering iron, thin wire, 2× USB-C cables, a 1S battery.

**Software**

- **WSL (Ubuntu)** or Linux with **PlatformIO** (`pip install platformio`).
- **esptool** and the **meshtastic** CLI (`pip install esptool meshtastic`).
- **Betaflight Configurator** (desktop app, or https://app.betaflight.com in Chrome).
- USB serial driver for the Heltec's **CP2102** (Silicon Labs CP210x) — usually
  auto-installed on Windows 10/11; otherwise install the SiLabs VCP driver.

---

## 1. Install the toolchain

```bash
WSL$ python3 -m pip install --upgrade platformio esptool meshtastic
WSL$ pio --version          # sanity check
```

On Windows, make sure `esptool` is reachable from PowerShell too (it ships with
Python: `PS> python -m esptool version`). Flashing is done from the OS that owns
the COM port — on a typical WSL setup that's **PowerShell**, while the build runs
in **WSL**.

---

## 2. Get the code and apply the overlay

```bash
WSL$ git clone https://github.com/meshtastic/firmware.git
WSL$ git clone https://github.com/<you>/meshtastic-fpv-drone-rc.git

WSL$ chmod +x meshtastic-fpv-drone-rc/install.sh
WSL$ ./meshtastic-fpv-drone-rc/install.sh ./firmware
```

`install.sh` copies `CRSFRcModule.{h,cpp}` into `firmware/src/modules/` and
registers the module in `setupModules()` (idempotent — safe to re-run). To do it
by hand instead, copy the two files and apply
`patches/0001-register-CRSFRcModule.patch`.

> **Pick the command channel now.** By default the module listens on Meshtastic
> channel index **0** (`CRSF_CMD_CHANNEL`). The simplest path is to keep index 0
> and put a private PSK on the primary channel (Step 5). If you prefer a
> dedicated secondary channel at index 1, build with
> `PLATFORMIO_BUILD_FLAGS="-DCRSF_CMD_CHANNEL=1"`.

---

## 3. Build the firmware

```bash
WSL$ cd firmware
WSL$ pio run -e heltec-v3
```

The binary is at `firmware/.pio/build/heltec-v3/firmware.bin`. Copy it somewhere
PowerShell can reach if you flash from Windows:

```bash
WSL$ cp .pio/build/heltec-v3/firmware.bin /mnt/c/dev/flash/
```

---

## 4. Flash the Heltec V3

Plug the Heltec into USB. Find its port (Windows: Device Manager → COMx; if the
chip won't enter download mode, hold **BOOT**, tap **RST**, release **BOOT**).

```powershell
PS> cd C:\dev\flash
PS> python -m esptool --chip esp32s3 --port COM4 --baud 921600 write_flash 0x10000 firmware.bin
```

> First time on a brand-new board (never ran Meshtastic)? Flash the full image
> set once with the official Meshtastic web flasher, then the `0x10000`
> app-only flash above is all you need on every rebuild.

**Verify the module started** — open the serial monitor and look for:

```powershell
PS> python -m serial.tools.miniterm COM4 115200
```
```
CRSFRcModule init: UART2 TX=4 RX=6 @420000 baud, ch=0, 150Hz, ACK=0, FS=60000ms
CRSFRcModule txTask: core=0 prio=5 stack=4096 started=1
```

`txTask ... started=1` confirms the gapless CRSF streaming task is running — this
is what makes arming work reliably.

---

## 5. Set up the Meshtastic radio (do this on BOTH devices)

Both the Heltec and the transmitter must share the **same region, modem preset,
channel name and PSK, at the same channel index**.

Set the region (use your own — example is EU 868 MHz):

```bash
WSL$ meshtastic --port COM4 --set lora.region EU_868
```

**Option A — simplest (channel index 0).** Give the primary channel a private
PSK so random public traffic can't reach the drone, and leave the firmware
default `CRSF_CMD_CHANNEL=0`:

```bash
WSL$ meshtastic --port COM4 --ch-set psk random --ch-index 0
WSL$ meshtastic --port COM4 --ch-set name drone  --ch-index 0
```

**Option B — dedicated channel (index 1).** Build the firmware with
`-DCRSF_CMD_CHANNEL=1` (Step 2), then:

```bash
WSL$ meshtastic --port COM4 --ch-add drone
WSL$ meshtastic --port COM4 --ch-set psk random --ch-index 1
```

**Copy the channel to the transmitter.** Print the node's channel URL and import
it on the other device so the PSK matches exactly:

```bash
WSL$ meshtastic --port COM4 --info        # shows the "Complete URL (...)"
```

On the T-Deck or phone app, scan/paste that URL (or run
`meshtastic --port COM5 --seturl "<the URL>"` on the second node). After this,
both devices list the same channel at the same index.

---

## 6. Wire the Heltec to the Matrix 1S

CRSF goes on **UART1** (the `RX1`/`TX1` pads on the Matrix). Full annotated
overlay on the board photos: open [`docs/wiring/wiring.html`](wiring/wiring.html).

| Signal | Heltec V3 | → | Matrix 1S | Note |
|--------|-----------|---|-----------|------|
| +5 V   | **5V**            | → | **+5V** (BEC) | power — **not** `BATT+`/VBAT |
| GND    | **GND**           | → | **GND**       | common ground |
| CRSF data | **GPIO4** (UART2 TX) | → | **RX1** (UART1) | **required** |
| telemetry | **GPIO6** (UART2 RX) | ← | **TX1** (UART1) | optional |

Notes:

- TX always goes to RX (crossed). Three wires (5V, GND, GPIO4→RX1) are enough;
  the telemetry wire is optional.
- Power the Heltec from the regulated **5V/BEC** pad. On raw battery the voltage
  sags when motors spin → the Heltec browns out → CRSF drops → the FC fails safe.

---

## 7. Configure Betaflight

Connect the FC over USB to Betaflight Configurator:

1. **Ports** → on **UART1** enable **Serial RX**. Save & Reboot.
2. **Receiver** → Receiver Mode **Serial (via UART)**, Provider **CRSF**. Save.
3. **Modes** → add **ARM**, channel **AUX1**, range **1750–2100**. Save.
4. **Configuration / Motors** → ESC protocol **DSHOT** (e.g. DSHOT300). Props off.

> The Configurator holds the FC over MSP, which **blocks arming** (the `MSP`
> flag). Click **Disconnect** before arming from the mesh; reconnect to inspect.

---

## 8. First run on the bench (props OFF)

1. Props off, drone on the bench, fresh 1S battery. The Heltec boots from the FC
   5V and starts streaming CRSF.
2. In Betaflight **Receiver** tab, from the transmitter send `c` then `w`/`s` —
   the channel bars must move. This proves the whole mesh → CRSF path.
   - T-Deck: type on the channel. Phone app: send text on the channel.
   - CLI: `meshtastic --port COM5 --ch-index 0 --sendtext "c"`
     (use `--ch-index 1` if you chose Option B).
3. **Disconnect** the Configurator.
4. Send `x` once (clears any stale state), then `arm`. The FC arms (beep/LED).
5. Send `w` a few times → motors spin to idle. Send `x` → disarm/stop.

---

## 9. Verify it worked

- Boot log shows `txTask ... started=1`.
- Receiver tab channels track your `w`/`s`/`a`/`d`/`arm` commands.
- After `arm`, with Configurator reconnected, the **Arming Disable Flags** are
  clear (no `BAD_RX_RECOVERY`). Motors idle on `w`, stop on `x`.

---

## 10. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `BAD_RX_RECOVERY` on `arm` | Not running this firmware — confirm `txTask ... started=1` in the boot log. |
| `MSP` flag, won't arm | Betaflight Configurator still connected — **Disconnect** it. |
| `THROTTLE` flag | Throttle not at minimum; `arm` forces it to MIN, so don't send `w` before `arm`. |
| `MOTOR_PROTOCOL` / DSHOT flag | ESC side: Configuration tab must use DSHOT; check bidirectional-DSHOT / RPM-filter match your ESC (BLHeli_S may need Bluejay). |
| No movement in Receiver tab | TX/RX swapped, wrong UART set to Serial RX, baud mismatch, or the two nodes aren't on the same channel/PSK/index. |
| Commands ignored but link is up | Channel index mismatch — `--ch-index` must equal `CRSF_CMD_CHANNEL`. |
| Heltec resets when motors spin | It's on VBAT — move its power to the FC **5V/BEC** pad. |

---

## Appendix A — command reference

Send as plain text on the command channel:

| Command(s) | Action |
|------------|--------|
| `arm` | ARM (forces throttle to MIN first) |
| `x` / `z` / `stop` / `disarm` | DISARM, throttle to MIN |
| `w` / `s` | throttle up / down |
| `a` / `d` | roll left / right |
| `f` / `b` | pitch forward / back |
| `q` / `e` | yaw left / right |
| `c` / `center` | center roll/pitch/yaw |
| `?` / `status` | report current channel state |

## Appendix B — build-time options

Override any `#define` from `src/modules/CRSFRcModule.h` with a build flag, e.g.:

```bash
WSL$ PLATFORMIO_BUILD_FLAGS="-DCRSF_CMD_CHANNEL=1 -DCRSF_MAX_THROTTLE_US=1250" pio run -e heltec-v3
```

| Macro | Default | Meaning |
|-------|---------|---------|
| `CRSF_CMD_CHANNEL` | 0 | Meshtastic channel index to listen on |
| `CRSF_UART_NUM` / `CRSF_TX_PIN` / `CRSF_RX_PIN` | 2 / 4 / 6 | UART + pins to the FC |
| `CRSF_BAUD` | 420000 | CRSF link baud |
| `CRSF_TX_HZ` | 150 | RC frame rate to the FC |
| `CRSF_TX_TASK_CORE` / `_PRIO` / `_STACK` | 0 / 5 / 4096 | dedicated streaming task |
| `CRSF_MAX_THROTTLE_US` | 1300 | hard throttle ceiling (safety) |
| `CRSF_FAILSAFE_MS` | 60000 | no-command timeout before throttle ramps down |
| `CRSF_SEND_ACK` | 0 | mesh confirmation replies (off by default) |
