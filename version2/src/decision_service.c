/*
 * Copyright (c) 2024, BlackBerry Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * decision_service — ADAS risk classification.
 *
 * Receives adas_sensor_frame_t from sensor_service via QNX message passing,
 * classifies risk level (NORMAL / CAUTION / EMERGENCY), and forwards
 * adas_vehicle_state_t to safety_service.
 *
 * Registered with HAM: YES — supervisor restarts it on death.
 * On restart, it re-attaches its channel and reconnects to safety.
 *
 * CPU:      Core 1  (same as safety — tight pipeline latency)
 * Priority: 52      (SCHED_FIFO — below safety, above non-critical)
 */

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "ipc_protocol.h"
#include "service_common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DECISION_CPU   1
#define DECISION_PRIO  52

static adas_risk_level_t classify(const adas_sensor_frame_t *f,
                                   float roll, float pitch)
{
    adas_risk_level_t lvl = ADAS_LEVEL_NORMAL;

    if (fabsf(roll) > 35.f || fabsf(pitch) > 35.f) {
        lvl = ADAS_LEVEL_CAUTION;
    }
    if (f->ultrasonic_ok) {
        if (f->distance_mm > 0 && f->distance_mm < 280) {
            return ADAS_LEVEL_EMERGENCY;
        }
        if (f->distance_mm > 0 && f->distance_mm < 550
            && lvl != ADAS_LEVEL_EMERGENCY) {
            lvl = ADAS_LEVEL_CAUTION;
        }
    } else {
        /* Ultrasonic failed — treat as caution (unknown obstacle) */
        if (lvl == ADAS_LEVEL_NORMAL) lvl = ADAS_LEVEL_CAUTION;
    }
    return lvl;
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* --- RT setup ---------------------------------------------------- */
    svc_set_priority(DECISION_PRIO, LOG_TAG_DEC);
    svc_bind_cpu(DECISION_CPU, LOG_TAG_DEC);

    printf("%sStarting decision_service  CPU=%d  PRIO=%d  SCHED_FIFO\n",
           LOG_TAG_DEC, DECISION_CPU, DECISION_PRIO);

    /* --- IPC: register receive channel ------------------------------- */
    name_attach_t *attach = name_attach(NULL, ADAS_CHANNEL_DECISION, 0);
    if (attach == NULL) {
        perror(LOG_TAG_DEC "name_attach");
        return EXIT_FAILURE;
    }
    printf("%sListening on channel '%s'\n", LOG_TAG_DEC, ADAS_CHANNEL_DECISION);

    /* HAM registration done by supervisor — do not self-register */

    /* --- Connect to supervisor --------------------------------------- */
    int sup_coid = -1;
    {
        int tries = 0;
        while (sup_coid == -1 && tries < 20) {
            sup_coid = name_open(ADAS_CHANNEL_SUPERVISOR, 0);
            if (sup_coid == -1) { usleep(100000); tries++; }
        }
        if (sup_coid == -1) {
            fprintf(stderr, "%sWarning: supervisor not found\n", LOG_TAG_DEC);
        }
    }

    /* --- Connect to safety_service ----------------------------------- */
    int saf_coid = -1;
    printf("%sWaiting for safety_service...\n", LOG_TAG_DEC);
    while (saf_coid == -1) {
        saf_coid = name_open(ADAS_CHANNEL_SAFETY, 0);
        if (saf_coid == -1) usleep(100000);
    }
    printf("%sConnected to safety_service\n", LOG_TAG_DEC);

    uint16_t out_seq = 0;

    while (1) {
        adas_sensor_frame_t frame;
        struct _msg_info info;
        memset(&frame, 0, sizeof(frame));

        int rcvid = MsgReceive(attach->chid, &frame, sizeof(frame), &info);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            continue;
        }
        if (rcvid == 0) continue; /* pulse — ignore */

        /* Reply to sensor immediately to unblock it */
        adas_simple_ack_t ack = {0};
        MsgReply(rcvid, EOK, &ack, sizeof(ack));

        /* Compute roll/pitch from accelerometer */
        const float ax = (float)frame.ax;
        const float ay = (float)frame.ay;
        const float az = (float)frame.az;
        const float roll  = atan2f(ay, az) * 180.0f / (float)M_PI;
        const float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

        /* Build vehicle state */
        adas_vehicle_state_t state;
        memset(&state, 0, sizeof(state));
        state.type        = ADAS_MSG_VEHICLE_STATE;
        state.seq         = ++out_seq;
        state.roll_deg    = roll;
        state.pitch_deg   = pitch;
        state.distance_mm = frame.distance_mm;
        state.risk_level  = (uint8_t)classify(&frame, roll, pitch);

        /* Forward to safety_service */
        adas_simple_ack_t s_ack = {0};
        if (MsgSend(saf_coid, &state, sizeof(state),
                    &s_ack, sizeof(s_ack)) == -1) {
            /*
             * Safety channel gone — this should not happen (safety never dies).
             * Log and attempt reconnect.
             */
            fprintf(stderr, "%sLost connection to safety_service: %s\n",
                    LOG_TAG_DEC, strerror(errno));
            name_close(saf_coid);
            saf_coid = -1;
            while (saf_coid == -1) {
                saf_coid = name_open(ADAS_CHANNEL_SAFETY, 0);
                if (saf_coid == -1) usleep(100000);
            }
            printf("%sReconnected to safety_service\n", LOG_TAG_DEC);
        }

        /* Heartbeat to supervisor */
        if (sup_coid != -1) {
            svc_send_heartbeat(sup_coid, PULSE_HB_DECISION);
        }

        /* HAM heartbeat */
        svc_ham_heartbeat();
    }

    name_close(saf_coid);
    if (sup_coid != -1) name_close(sup_coid);
    name_detach(attach, 0);
    return EXIT_SUCCESS;
}
