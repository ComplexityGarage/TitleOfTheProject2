// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Wojciech Pałka <wpalka@icecodegames.com>
// CRSFRcModule — Meshtastic firmware overlay. See LICENSE (GPL-3.0-or-later).
#include "CRSFRcModule.h"
#include "MeshService.h"
#include "configuration.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#include <cctype>
#include <cstdio>
#include <cstring>

CRSFRcModule *crsfRcModule = nullptr;

static HardwareSerial CrsfSerial(CRSF_UART_NUM);

// --- CRSF primitives --------------------------------------------------------

uint16_t CRSFRcModule::usToCrsf(uint16_t us)
{
    if (us < CRSF_MIN_THROTTLE_US) us = CRSF_MIN_THROTTLE_US;
    if (us > 2012) us = 2012;
    // val = (us - 1500) * 8 / 5 + 992
    int32_t v = ((int32_t)us - 1500) * 8 / 5 + 992;
    if (v < 0) v = 0;
    if (v > 2047) v = 2047;
    return (uint16_t)v;
}

void CRSFRcModule::packChannels(const uint16_t channelsUs[16], uint8_t out22[22])
{
    memset(out22, 0, 22);
    uint32_t bits = 0;
    int bitcount = 0;
    int outIdx = 0;
    for (int ch = 0; ch < 16; ch++) {
        uint16_t v = usToCrsf(channelsUs[ch]);
        bits |= (uint32_t)(v & 0x7FF) << bitcount;
        bitcount += 11;
        while (bitcount >= 8 && outIdx < 22) {
            out22[outIdx++] = (uint8_t)(bits & 0xFF);
            bits >>= 8;
            bitcount -= 8;
        }
    }
}

uint8_t CRSFRcModule::crsfCrc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

void CRSFRcModule::sendFrame(const uint16_t ch[16])
{
    uint8_t frame[26];
    frame[0] = 0xC8;   // CRSF_ADDR_FC
    frame[1] = 24;     // length = type + 22 payload + crc
    frame[2] = 0x16;   // RC channels packed
    packChannels(ch, frame + 3);
    frame[25] = crsfCrc8(frame + 2, 23);
    CrsfSerial.write(frame, 26);
}

// --- dedicated CRSF streaming task ------------------------------------------
// Runs preemptively on its own core. Takes a brief snapshot of the shared
// channel state under chMutex, then writes one CRSF frame, on a fixed period.
// Because it is NOT on the cooperative Meshtastic thread, LoRa RX/TX bursts
// can never open a multi-frame gap in the stream -> no spurious FC failsafe,
// no BAD_RX_RECOVERY when arming.
void CRSFRcModule::crsfTxTask(void *arg)
{
    CRSFRcModule *self = static_cast<CRSFRcModule *>(arg);

    uint32_t periodMs = 1000 / CRSF_TX_HZ;
    if (periodMs == 0) periodMs = 1;
    const TickType_t period = pdMS_TO_TICKS(periodMs) ? pdMS_TO_TICKS(periodMs) : 1;

    TickType_t last = xTaskGetTickCount();
    uint16_t snap[16];

    for (;;) {
        if (self->chMutex) xSemaphoreTake(self->chMutex, portMAX_DELAY);
        memcpy(snap, self->channelsUs, sizeof(snap));
        if (self->chMutex) xSemaphoreGive(self->chMutex);

        self->sendFrame(snap);

        vTaskDelayUntil(&last, period);
    }
}

// --- module lifecycle -------------------------------------------------------

CRSFRcModule::CRSFRcModule()
    : SinglePortModule("crsfrc", meshtastic_PortNum_TEXT_MESSAGE_APP),
      concurrency::OSThread("CRSFRcModule")
{
    resetToSafeDefaults();

    CrsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
    lastCmdMs = millis();

    // State lock must exist before the TX task or handleReceived can run.
    chMutex = xSemaphoreCreateMutex();

    // Start the dedicated, preemptive CRSF streaming task on the second core.
    BaseType_t taskOk = xTaskCreatePinnedToCore(
        &CRSFRcModule::crsfTxTask, "crsfTx", CRSF_TX_TASK_STACK, this,
        CRSF_TX_TASK_PRIO, &txTaskHandle, CRSF_TX_TASK_CORE);

    LOG_INFO("CRSFRcModule init: UART%d TX=%d RX=%d @%d baud, ch=%d, %dHz, ACK=%d, FS=%dms",
             (int)CRSF_UART_NUM, (int)CRSF_TX_PIN, (int)CRSF_RX_PIN,
             (int)CRSF_BAUD, (int)CRSF_CMD_CHANNEL,
             (int)CRSF_TX_HZ, (int)CRSF_SEND_ACK, (int)CRSF_FAILSAFE_MS);
    LOG_INFO("CRSFRcModule txTask: core=%d prio=%d stack=%d started=%d",
             (int)CRSF_TX_TASK_CORE, (int)CRSF_TX_TASK_PRIO,
             (int)CRSF_TX_TASK_STACK, (taskOk == pdPASS));
}

void CRSFRcModule::resetToSafeDefaults()
{
    for (int i = 0; i < 16; i++) channelsUs[i] = 1500;
    channelsUs[CRSF_CH_THROTTLE] = CRSF_MIN_THROTTLE_US;
    channelsUs[CRSF_CH_AUX1]     = CRSF_AUX1_LOW_US;
    armed = false;
}

void CRSFRcModule::clampStick(uint16_t &v) const
{
    if (v < CRSF_STICK_MIN_US) v = CRSF_STICK_MIN_US;
    if (v > CRSF_STICK_MAX_US) v = CRSF_STICK_MAX_US;
}

// --- periodic sender + failsafe --------------------------------------------

int32_t CRSFRcModule::runOnce()
{
    // NOTE: the CRSF frame stream is emitted by crsfTxTask (preemptive, own
    // core). runOnce() now only manages the throttle-down failsafe so it can
    // safely run on the cooperative scheduler — being starved here no longer
    // affects the RC link.
    uint32_t now = millis();
    if ((now - lastCmdMs) > CRSF_FAILSAFE_MS) {
        // Failsafe: ramp throttle down only. AUX1 stays where it is — explicit
        // 'z' / 'disarm' from the pilot is the only way to disarm in test mode.
        // For real flight, uncomment the AUX1 LOW + armed=false lines below.
        if (chMutex) xSemaphoreTake(chMutex, portMAX_DELAY);
        if (channelsUs[CRSF_CH_THROTTLE] > CRSF_MIN_THROTTLE_US) {
            uint16_t newT = channelsUs[CRSF_CH_THROTTLE] - 10;
            if (newT < CRSF_MIN_THROTTLE_US) newT = CRSF_MIN_THROTTLE_US;
            channelsUs[CRSF_CH_THROTTLE] = newT;
        }
        // channelsUs[CRSF_CH_AUX1] = CRSF_AUX1_LOW_US;  // sticky arm — disabled for bench
        // armed = false;                                 // sticky arm — disabled for bench
        if (chMutex) xSemaphoreGive(chMutex);
    }
    return 20;  // failsafe-management cadence (ms); RC stream handled by crsfTxTask
}

// --- text -> command --------------------------------------------------------

void CRSFRcModule::normalizeText(const uint8_t *src, size_t srcLen, char *dst, size_t dstLen)
{
    if (dstLen == 0) return;
    size_t n = (srcLen < dstLen - 1) ? srcLen : dstLen - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
    size_t start = 0;
    while (dst[start] == ' ' || dst[start] == '\t') start++;
    if (start > 0) memmove(dst, dst + start, strlen(dst + start) + 1);
    size_t L = strlen(dst);
    while (L > 0 && (dst[L-1] == ' ' || dst[L-1] == '\t' ||
                     dst[L-1] == '\n' || dst[L-1] == '\r')) dst[--L] = '\0';
    for (char *c = dst; *c; c++) *c = (char)tolower((unsigned char)*c);
}

static inline void bumpThrottle(uint16_t &t, int delta) {
    int32_t v = (int32_t)t + delta;
    if (v < CRSF_MIN_THROTTLE_US) v = CRSF_MIN_THROTTLE_US;
    if (v > CRSF_MAX_THROTTLE_US) v = CRSF_MAX_THROTTLE_US;
    t = (uint16_t)v;
}

static inline void bumpStick(uint16_t &s, int delta) {
    int32_t v = (int32_t)s + delta;
    if (v < CRSF_STICK_MIN_US) v = CRSF_STICK_MIN_US;
    if (v > CRSF_STICK_MAX_US) v = CRSF_STICK_MAX_US;
    s = (uint16_t)v;
}

bool CRSFRcModule::parseCommand(const char *text, uint16_t channelsUs[16],
                                bool &armed, char *reply, size_t replyLen)
{
    if (!text || !*text) return false;

    // Throttle up / down
    if (!strcmp(text, "w") || !strcmp(text, "up") || !strcmp(text, "gora") || !strcmp(text, "góra")) {
        bumpThrottle(channelsUs[CRSF_CH_THROTTLE], CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "THR %u us", (unsigned)channelsUs[CRSF_CH_THROTTLE]);
        return true;
    }
    if (!strcmp(text, "s") || !strcmp(text, "down") || !strcmp(text, "dol") || !strcmp(text, "dół")) {
        bumpThrottle(channelsUs[CRSF_CH_THROTTLE], -CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "THR %u us", (unsigned)channelsUs[CRSF_CH_THROTTLE]);
        return true;
    }
    // Roll (a/d)
    if (!strcmp(text, "a") || !strcmp(text, "left")) {
        bumpStick(channelsUs[CRSF_CH_ROLL], -CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "ROLL %u us", (unsigned)channelsUs[CRSF_CH_ROLL]);
        return true;
    }
    if (!strcmp(text, "d") || !strcmp(text, "right")) {
        bumpStick(channelsUs[CRSF_CH_ROLL], CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "ROLL %u us", (unsigned)channelsUs[CRSF_CH_ROLL]);
        return true;
    }
    // Pitch (f/b)
    if (!strcmp(text, "f") || !strcmp(text, "fwd") || !strcmp(text, "forward")) {
        bumpStick(channelsUs[CRSF_CH_PITCH], CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "PITCH %u us", (unsigned)channelsUs[CRSF_CH_PITCH]);
        return true;
    }
    if (!strcmp(text, "b") || !strcmp(text, "back")) {
        bumpStick(channelsUs[CRSF_CH_PITCH], -CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "PITCH %u us", (unsigned)channelsUs[CRSF_CH_PITCH]);
        return true;
    }
    // Yaw (q/e)
    if (!strcmp(text, "q") || !strcmp(text, "yawl")) {
        bumpStick(channelsUs[CRSF_CH_YAW], -CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "YAW %u us", (unsigned)channelsUs[CRSF_CH_YAW]);
        return true;
    }
    if (!strcmp(text, "e") || !strcmp(text, "yawr")) {
        bumpStick(channelsUs[CRSF_CH_YAW], CRSF_STICK_STEP_US);
        snprintf(reply, replyLen, "YAW %u us", (unsigned)channelsUs[CRSF_CH_YAW]);
        return true;
    }
    // Arm / disarm
    if (!strcmp(text, "arm")) {
        // Never arm with throttle raised — otherwise Betaflight blocks arming
        // with the THROTTLE flag. Force min before raising AUX1.
        channelsUs[CRSF_CH_THROTTLE] = CRSF_MIN_THROTTLE_US;
        channelsUs[CRSF_CH_AUX1]     = CRSF_AUX1_HIGH_US;
        armed = true;
        snprintf(reply, replyLen, "ARMED");
        return true;
    }
    if (!strcmp(text, "disarm") || !strcmp(text, "stop") || !strcmp(text, "x") || !strcmp(text, "z")) {
        channelsUs[CRSF_CH_AUX1]     = CRSF_AUX1_LOW_US;
        channelsUs[CRSF_CH_THROTTLE] = CRSF_MIN_THROTTLE_US;
        armed = false;
        snprintf(reply, replyLen, "DISARMED");
        return true;
    }
    // Center sticks (throttle untouched)
    if (!strcmp(text, "c") || !strcmp(text, "center")) {
        channelsUs[CRSF_CH_ROLL]  = 1500;
        channelsUs[CRSF_CH_PITCH] = 1500;
        channelsUs[CRSF_CH_YAW]   = 1500;
        snprintf(reply, replyLen, "CENTERED");
        return true;
    }
    // Status
    if (!strcmp(text, "?") || !strcmp(text, "status")) {
        snprintf(reply, replyLen, "%s THR=%u R=%u P=%u Y=%u",
                 armed ? "ARM" : "DIS",
                 (unsigned)channelsUs[CRSF_CH_THROTTLE],
                 (unsigned)channelsUs[CRSF_CH_ROLL],
                 (unsigned)channelsUs[CRSF_CH_PITCH],
                 (unsigned)channelsUs[CRSF_CH_YAW]);
        return true;
    }
    return false;
}

// --- mesh side --------------------------------------------------------------

ProcessMessage CRSFRcModule::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (mp.channel != CRSF_CMD_CHANNEL) return ProcessMessage::CONTINUE;

    char buf[64];
    normalizeText(mp.decoded.payload.bytes, mp.decoded.payload.size, buf, sizeof(buf));

    char reply[64];
    // Guard channel mutation against the concurrent CRSF TX task.
    if (chMutex) xSemaphoreTake(chMutex, portMAX_DELAY);
    bool wasCmd = parseCommand(buf, channelsUs, armed, reply, sizeof(reply));
    if (chMutex) xSemaphoreGive(chMutex);
    if (!wasCmd) {
        LOG_DEBUG("CRSFRcModule: '%s' not a command", buf);
        return ProcessMessage::CONTINUE;
    }

    lastCmdMs = millis();
    LOG_INFO("CRSFRcModule: %s", reply);

#if CRSF_SEND_ACK
    sendMeshReply(reply);
#endif
    return ProcessMessage::CONTINUE;
}

void CRSFRcModule::sendMeshReply(const char *msg)
{
    if (!msg || !*msg || !service) return;
    meshtastic_MeshPacket *p = allocDataPacket();
    if (!p) return;
    p->to = NODENUM_BROADCAST;
    p->channel = CRSF_CMD_CHANNEL;
    p->decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP;
    p->want_ack = false;
    size_t len = strlen(msg);
    size_t maxLen = sizeof(p->decoded.payload.bytes);
    if (len > maxLen) len = maxLen;
    memcpy(p->decoded.payload.bytes, msg, len);
    p->decoded.payload.size = (uint16_t)len;
    service->sendToMesh(p, RX_SRC_LOCAL, true);
}
