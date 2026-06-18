// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Wojciech Pałka <wpalka@icecodegames.com>
// CRSFRcModule — Meshtastic firmware overlay. See LICENSE (GPL-3.0-or-later).
#pragma once

/*
 * CRSFRcModule
 * ------------
 * Custom Meshtastic firmware module for piloting a CRSF-capable flight
 * controller (Betaflight / iNAV / EmuFlight) from incoming text messages.
 *
 * Specifically tested against the BetaFPV Matrix 1S (STM32G473 + Betaflight,
 * internal ELRS RX using CRSF). The same code works for any FC accepting
 * CRSF on a UART.
 *
 * Architecture:
 *   T-Deck ──LoRa──> Heltec V3 (this module) ──UART CRSF 420000──> FC
 *
 * - SinglePortModule listens for TEXT_MESSAGE_APP and mutates channel state.
 * - A DEDICATED FreeRTOS task (crsfTxTask), pinned to the second core and run
 *   preemptively, streams a CRSF RC-channels frame every 1000/CRSF_TX_HZ ms.
 *   This is the critical design point: CRSF must NOT share the cooperative
 *   Meshtastic OSThread/main-loop, otherwise inbound LoRa packet processing
 *   (decrypt, route, nodeDB, flash) starves the sender for hundreds of ms ->
 *   the FC sees an RC dropout -> failsafe -> on the next 'arm' it latches
 *   BAD_RX_RECOVERY ("RX recovered from failsafe while arm switch is ON").
 *   A separate preemptive task keeps the stream gapless regardless of LoRa.
 * - Channel state shared between the main thread (command parsing + failsafe)
 *   and the TX task is guarded by a FreeRTOS mutex.
 * - OSThread runOnce() now only manages the throttle-down failsafe.
 * - Safety: throttle is hard-capped, failsafe ramps throttle after timeout.
 *
 * Drop-in location: src/modules/CRSFRcModule.h / .cpp
 * Registration:     add  new CRSFRcModule();  inside setupModules().
 */

#include "SinglePortModule.h"
#include "concurrency/OSThread.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

// ---- CONFIG ---------------------------------------------------------------

// Channel index on the Meshtastic side used to receive commands. Use a
// private channel with its own PSK for any non-bench testing.
#ifndef CRSF_CMD_CHANNEL
#define CRSF_CMD_CHANNEL 0
#endif

// ESP32 UART number (0 = console, 1 = often GPS, 2 = free on Heltec V3).
#ifndef CRSF_UART_NUM
#define CRSF_UART_NUM 2
#endif

// Heltec V3 free pins for the UART to the FC.
// TX_PIN is required (FC's RX pad). RX_PIN is for optional FC->ESP telemetry.
#ifndef CRSF_TX_PIN
#define CRSF_TX_PIN 4
#endif
#ifndef CRSF_RX_PIN
#define CRSF_RX_PIN 6
#endif

// CRSF link baud — standard for Crossfire / ELRS.
#ifndef CRSF_BAUD
#define CRSF_BAUD 420000
#endif

// Frame rate to the FC (Hz). Higher = more resilience to brief gaps when
// the LoRa radio is busy transmitting ACKs.
#ifndef CRSF_TX_HZ
#define CRSF_TX_HZ 150
#endif

// ---- Dedicated CRSF TX task -----------------------------------------------
// The CRSF stream runs in its own preemptive FreeRTOS task so it is never
// blocked by Meshtastic's cooperative scheduler during LoRa RX/TX.
//
// CRSF_TX_TASK_CORE: pin to the core NOT running the Arduino main loop.
//   On ESP32(-S3) the Arduino loop (and thus Meshtastic) runs on core 1, so
//   we put CRSF on core 0. Core 0 also hosts BLE/system tasks, but those run
//   in short bursts and won't cause the multi-frame gaps the main loop did.
#ifndef CRSF_TX_TASK_CORE
#define CRSF_TX_TASK_CORE 0
#endif
// Priority above app-level tasks, below ESP-IDF system/BLE tasks.
#ifndef CRSF_TX_TASK_PRIO
#define CRSF_TX_TASK_PRIO 5
#endif
#ifndef CRSF_TX_TASK_STACK
#define CRSF_TX_TASK_STACK 4096   // bytes (ESP-IDF FreeRTOS); margin for Serial.write
#endif

// ---- Safety / range limits (microseconds) --------------------------------

// Hard ceiling on throttle. 1S whoop will hover around 1250..1400 us with
// the stock 0802 motors. 1300 us is a safe POC ceiling — adjust upward only
// after you understand what you are doing.
#ifndef CRSF_MAX_THROTTLE_US
#define CRSF_MAX_THROTTLE_US 1300
#endif

#ifndef CRSF_MIN_THROTTLE_US
#define CRSF_MIN_THROTTLE_US 988
#endif

#ifndef CRSF_STICK_STEP_US
#define CRSF_STICK_STEP_US 50  // per 'w'/'s'/'a'/'d' command
#endif

#ifndef CRSF_STICK_MIN_US
#define CRSF_STICK_MIN_US 1300
#endif

#ifndef CRSF_STICK_MAX_US
#define CRSF_STICK_MAX_US 1700
#endif

// AUX1 high = ARM, low = DISARM.
#ifndef CRSF_AUX1_LOW_US
#define CRSF_AUX1_LOW_US 988
#endif
#ifndef CRSF_AUX1_HIGH_US
#define CRSF_AUX1_HIGH_US 2012
#endif

// Failsafe: if no command arrives within this many milliseconds, force
// throttle to MIN. AUX1/arm is NOT touched (sticky arm — explicit 'z' to disarm).
// For real flight reduce to ~2000.
#ifndef CRSF_FAILSAFE_MS
#define CRSF_FAILSAFE_MS 60000
#endif

// ACK disabled — LoRa transmission burst was causing RX_FAILSAFE on FC.
// Re-enable to 1 if you want T-Deck confirmation messages.
#ifndef CRSF_SEND_ACK
#define CRSF_SEND_ACK 0
#endif

// ---- Channel slot layout (Betaflight default AETR1234) -------------------
// Channel index 0..15 in the CRSF frame.
#define CRSF_CH_ROLL     0
#define CRSF_CH_PITCH    1
#define CRSF_CH_THROTTLE 2
#define CRSF_CH_YAW      3
#define CRSF_CH_AUX1     4   // arm
#define CRSF_CH_AUX2     5
#define CRSF_CH_AUX3     6
#define CRSF_CH_AUX4     7

// --------------------------------------------------------------------------

class CRSFRcModule : public SinglePortModule, private concurrency::OSThread
{
  public:
    CRSFRcModule();

  protected:
    // Mesh side
    virtual ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;
    // Periodic CRSF sender — returns ms until next call
    virtual int32_t runOnce() override;

  private:
    // Channel state in microseconds (more readable than CRSF integer units).
    // Shared between the main thread (writer) and crsfTxTask (reader) -> guard
    // every access with chMutex.
    uint16_t channelsUs[16];
    uint32_t lastCmdMs = 0;
    bool armed = false;

    // Dedicated CRSF streaming task + state lock.
    TaskHandle_t txTaskHandle = nullptr;
    SemaphoreHandle_t chMutex = nullptr;
    static void crsfTxTask(void *arg);

    // Helpers
    void resetToSafeDefaults();
    void clampStick(uint16_t &v) const;

    // Returns true if the text was a recognized command. Fills `reply` with
    // a human-readable status string. Pure (no I/O) so we can unit test it.
    static bool parseCommand(const char *text, uint16_t channelsUs[16],
                              bool &armed, char *reply, size_t replyLen);

    // CRSF protocol — see crsf_test.cpp for unit tests of these primitives.
    static uint16_t usToCrsf(uint16_t us);
    static void packChannels(const uint16_t channelsUs[16], uint8_t out22[22]);
    static uint8_t crsfCrc8(const uint8_t *ptr, uint8_t len);
    // Builds and writes one CRSF RC frame from the supplied channel snapshot.
    void sendFrame(const uint16_t ch[16]);

    void sendMeshReply(const char *msg);
    static void normalizeText(const uint8_t *src, size_t srcLen, char *dst, size_t dstLen);
};

extern CRSFRcModule *crsfRcModule;
