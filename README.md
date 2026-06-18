# Controlling an FPV drone over a Meshtastic LoRa mesh (`meshtastic-fpv-drone-rc`)

A proof-of-concept: arm and fly a small FPV drone over a **Meshtastic (LoRa)**
mesh **with no extra radio hardware** — no ELRS/Crossfire transmitter, no
separate RC receiver. The only radio on the drone is an ordinary Meshtastic node
(**Heltec WiFi LoRa 32 V3**) that already exists in the mesh.

## Authors

- Wojciech Pałka

## Description of the project

Normally an FPV drone needs a dedicated radio control link: a transmitter module
and a matching receiver soldered to the flight controller. This project removes
that requirement. Custom Meshtastic firmware on the Heltec V3 (`CRSFRcModule`)
translates short text messages received over the mesh (`arm`, `w`, `x`, …) into a
**CRSF** RC-channels stream on a UART, feeding it straight into the flight
controller. In other words, **the Meshtastic node itself becomes the RC
receiver**. Any other Meshtastic device — a LILYGO **T-Deck**, the Meshtastic
phone app, or the `meshtastic` CLI — acts as the "transmitter" simply by sending
text on a shared, private channel.

```
  T-Deck   (or phone app, or `meshtastic` CLI)            <-- transmitter
      |
      |   LoRa / Meshtastic   --   text: "arm", "w", "x", ...
      v
  Heltec V3  --  CRSFRcModule   (Meshtastic node = the RC receiver)
      |
      |   UART CRSF @ 420000 baud   --   RC channel frames @ 150 Hz
      v
  BetaFPV FC (Betaflight)   +   ESC   +   motors          <-- the drone
```

It was developed and validated on the bench against a **BetaFPV Matrix 1S**
(Betaflight). It is an **educational proof-of-concept, not a flight-ready control
link**: a mesh is store-and-forward, with latency and no delivery guarantee.

> ⚠️ **Safety.** Keep the **propellers off** for all testing. Throttle is
> hard-capped in firmware (`CRSF_MAX_THROTTLE_US`, default 1300 µs). You are
> responsible for safe and legal operation (LoRa band/duty-cycle rules, drone
> regulations).

## Science and tech used

**Building blocks**

- **Meshtastic / LoRa** — an open-source long-range, low-bandwidth mesh. Used
  here as the command transport between transmitter and the on-drone node.
- **CRSF (Crossfire/ELRS protocol)** — the serial RC protocol Betaflight speaks
  at 420000 baud: a 16-channel packed frame (11 bits/channel) with a CRC-8. The
  module synthesises these frames from the current stick/aux state.
- **ESP32-S3 + FreeRTOS** — the heart of the implementation. The CRSF stream runs
  in a **dedicated, preemptive FreeRTOS task pinned to the second core**, fed
  from a mutex-guarded channel snapshot, so it is never starved by Meshtastic's
  cooperative scheduler.
- **Betaflight / DSHOT** — the flight-controller side; arming is mapped to AUX1.

**Key engineering result — the BAD_RX_RECOVERY trap.** A naive bridge emits CRSF
from Meshtastic's cooperative loop. Then, processing an inbound `arm` packet
(decrypt, route, nodeDB, flash) blocks the sender for hundreds of milliseconds;
the FC sees an RC dropout → failsafe → recovers with the arm switch already high
→ Betaflight latches `BAD_RX_RECOVERY` ("RX recovered from failsafe while the arm
switch is on"). The `arm` command was itself causing the failsafe that blocked
arming. Moving the CRSF stream to its own preemptive task makes the stream
gapless and arming reliable.

**How to reproduce** — condensed below; see [`docs/RUNNING.md`](docs/RUNNING.md) for the full step-by-step run guide.

1. *Build* — clone the upstream firmware, overlay this module, compile:
   ```bash
   git clone https://github.com/meshtastic/firmware.git
   ./meshtastic-fpv-drone-rc/install.sh ./firmware
   cd firmware && pio run -e heltec-v3
   ```
2. *Flash the Heltec V3* with esptool:
   ```bash
   python -m esptool --chip esp32s3 --port COM4 --baud 921600 \
       write_flash 0x10000 .pio/build/heltec-v3/firmware.bin
   ```
   Confirm the boot log shows `CRSFRcModule txTask: ... started=1`.
3. *Wire Heltec → Matrix 1S* (CRSF on **UART1** / the `RX1`/`TX1` pads). See the
   annotated overlay in [`docs/wiring/wiring.html`](docs/wiring/wiring.html):

   | Signal | Heltec V3 | → | Matrix 1S | Note |
   |--------|-----------|---|-----------|------|
   | +5 V   | 5V        | → | +5V (BEC) | power (not VBAT) |
   | GND    | GND       | → | GND       | common ground |
   | CRSF data | GPIO4 (UART2 TX) | → | RX1 (UART1) | **required** |
   | telemetry | GPIO6 (UART2 RX) | ← | TX1 (UART1) | optional |

4. *Configure Betaflight* — Ports: `UART1 = Serial RX`; Receiver: `Serial / CRSF`;
   Modes: `ARM` on `AUX1` (1750–2100). Disconnect the Configurator before arming
   (the `MSP` flag blocks arming while connected).
5. *Set up Meshtastic* — same region and a shared private channel (name + PSK) at
   the same channel index on both nodes (`CRSF_CMD_CHANNEL`, default 0).
6. *Fly the bench* (props off) — from the transmitter send `c`, watch the
   Receiver tab move; then `x`, `arm`, `w` to spin, `x` to stop.

**Command set:** `arm`, `x`/`z`/`stop`/`disarm`, `w`/`s` (throttle), `a`/`d`
(roll), `f`/`b` (pitch), `q`/`e` (yaw), `c` (center), `?` (status). Tunable
`#define`s (channel, pins, baud, frame rate, throttle cap, failsafe) live in
`src/modules/CRSFRcModule.h`.

## State of the art

- **Dedicated RC links (ELRS, TBS Crossfire).** The norm for FPV: 2.4/900 MHz
  links with millisecond latency and hundreds of packets/second, but they need a
  bound TX module and a dedicated on-craft receiver.
- **Meshtastic.** An open LoRa mesh aimed at text/telemetry over kilometres, with
  relaying and encryption — but high latency and strict duty cycle, never
  intended for real-time control.
- **LoRa for drones.** Most prior work uses LoRa for long-range *telemetry* or
  MAVLink bridging, not as the primary control link.

This project sits at that intersection: it reuses an existing Meshtastic node as
a CRSF RC receiver, trading latency and reliability for **zero extra radio
hardware** plus the mesh's range and relaying. The novel piece is treating a
general-purpose mesh node as a real-time-ish RC source and solving the
firmware-timing problem that otherwise prevents arming.

## What next?

- **Characterise latency & reliability** — command round-trip and packet loss vs.
  distance / mesh hops.
- **Bidirectional telemetry** — use the optional FC→node UART to show battery,
  attitude and arming flags back on the T-Deck.
- **Flight-grade failsafe** — replace the bench "sticky-arm" behaviour with a
  proper RC-loss failsafe before any real flight.
- **Command authentication** — harden the private channel; reject malformed or
  replayed commands.
- **Higher-level commands** — `goto` / return-to-home over the mesh instead of raw
  stick nudges, potentially bridging GPS/MAVLink.
- **Controlled flight tests** — move beyond the bench within legal and safety
  limits.

## Sources

- [Meshtastic firmware](https://github.com/meshtastic/firmware)
- [Meshtastic documentation](https://meshtastic.org/docs/)
- [ExpressLRS / CRSF protocol](https://www.expresslrs.org/)
- [Betaflight — Arming Sequence & Safety](https://betaflight.com/docs/wiki/guides/current/Arming-Sequence-And-Safety)
- [Betaflight arming disable flags explained (Oscar Liang)](https://oscarliang.com/quad-arming-issue-fix/)
- [BetaFPV Matrix 1S flight controller](https://betafpv.com/products/matrix-1s-brushless-flight-controller)
- [Heltec WiFi LoRa 32 V3](https://heltec.org/project/wifi-lora-32-v3/)
- [Writing on GitHub](https://docs.github.com/en/get-started/writing-on-github)

## License

GPL-3.0-or-later — a derivative work of the Meshtastic firmware, distributed under
the same license. See [LICENSE](LICENSE).
