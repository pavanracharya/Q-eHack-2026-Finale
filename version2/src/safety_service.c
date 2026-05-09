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
 * safety_service — HIGHEST PRIORITY process. NEVER registered with HAM.
 *
 * CPU:      Core 1  (isolated from non-critical load on Core 3)
 * Priority: 58      (SCHED_FIFO)
 * IPC:      name_attach("adas_safety") — receives adas_vehicle_state_t
 *
 * Uses a QNX pulse-based timer (SIGEV_PULSE + timer_create) so that
 * MsgReceive returns periodically even when decision_service is dead,
 * allowing the fail-safe LED pattern to keep running.
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/siginfo.h>

#include "ipc_protocol.h"
#include "rpi_gpio.h"
#include "service_common.h"

#define LED_GPIO        GPIO16
#define SAFETY_CPU      1
#define SAFETY_PRIO     58

/* Pulse code for the watchdog timer — must not clash with PULSE_HB_* */
#define PULSE_CODE_TIMER  (_PULSE_CODE_MINAVAIL + 20)

static void failsafe_blink(void)
{
    int i;
    for (i = 0; i < 2; i++) {
        rpi_gpio_output(LED_GPIO, GPIO_HIGH); usleep(60000);
        rpi_gpio_output(LED_GPIO, GPIO_LOW);  usleep(60000);
    }
    usleep(300000);
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* --- RT setup ---------------------------------------------------- */
    svc_set_priority(SAFETY_PRIO, LOG_TAG_SAF);
    svc_bind_cpu(SAFETY_CPU, LOG_TAG_SAF);

    printf("%sStarting safety_service  CPU=%d  PRIO=%d  SCHED_FIFO\n",
           LOG_TAG_SAF, SAFETY_CPU, SAFETY_PRIO);

    /* --- GPIO --------------------------------------------------------- */
    if (rpi_gpio_setup(LED_GPIO, GPIO_OUT) != GPIO_SUCCESS) {
        fprintf(stderr, "%sError: GPIO setup failed\n", LOG_TAG_SAF);
        return EXIT_FAILURE;
    }

    /* --- IPC channel -------------------------------------------------- */
    name_attach_t *attach = name_attach(NULL, ADAS_CHANNEL_SAFETY, 0);
    if (attach == NULL) {
        perror(LOG_TAG_SAF "name_attach");
        return EXIT_FAILURE;
    }
    printf("%sListening on channel '%s'\n", LOG_TAG_SAF, ADAS_CHANNEL_SAFETY);

    /* --- Connect to supervisor --------------------------------------- */
    int sup_coid = -1;
    {
        int tries = 0;
        while (sup_coid == -1 && tries < 20) {
            sup_coid = name_open(ADAS_CHANNEL_SUPERVISOR, 0);
            if (sup_coid == -1) { usleep(100000); tries++; }
        }
        if (sup_coid == -1) {
            fprintf(stderr, "%sWarning: supervisor not found\n", LOG_TAG_SAF);
        }
    }

    /*
     * QNX-native pulse timer: delivers a pulse to our channel every 500 ms.
     * This unblocks MsgReceive so we can run the fail-safe pattern and send
     * heartbeats even when no vehicle state messages are arriving.
     */
    struct sigevent timer_ev;
    timer_t timer_id;
    struct itimerspec its;

    SIGEV_PULSE_INIT(&timer_ev, attach->chid,
                     SIGEV_PULSE_PRIO_INHERIT, PULSE_CODE_TIMER, 0);
    if (timer_create(CLOCK_MONOTONIC, &timer_ev, &timer_id) == -1) {
        perror(LOG_TAG_SAF "timer_create");
        return EXIT_FAILURE;
    }

    its.it_value.tv_sec     = 0;
    its.it_value.tv_nsec    = 500000000L; /* 500 ms first fire */
    its.it_interval.tv_sec  = 0;
    its.it_interval.tv_nsec = 500000000L; /* 500 ms repeat */
    timer_settime(timer_id, 0, &its, NULL);

    printf("%sSafety controller ONLINE\n", LOG_TAG_SAF);

    uint32_t tick = 0;
    int data_received = 0;

    while (1) {
        adas_vehicle_state_t v;
        struct _msg_info info;
        memset(&v, 0, sizeof(v));

        int rcvid = MsgReceive(attach->chid, &v, sizeof(v), &info);

        if (rcvid == -1) {
            if (errno == EINTR) continue;
            continue;
        }

        /* Pulse (timer or other) */
        if (rcvid == 0) {
            if (!data_received) {
                failsafe_blink();
            }
            if (sup_coid != -1) {
                svc_send_heartbeat(sup_coid, PULSE_HB_SAFETY);
            }
            continue;
        }

        /* Real message from decision_service */
        adas_simple_ack_t ack = {0};
        MsgReply(rcvid, EOK, &ack, sizeof(ack));
        data_received = 1;
        tick++;

        switch ((adas_risk_level_t)v.risk_level) {

            default:
            case ADAS_LEVEL_NORMAL:
                if ((tick % 20u) == 0u) {
                    printf("%sNORMAL  seq=%u  mm=%ld  R=%.1f  P=%.1f\n",
                           LOG_TAG_SAF, v.seq, (long)v.distance_mm,
                           v.roll_deg, v.pitch_deg);
                }
                rpi_gpio_output(LED_GPIO, GPIO_HIGH); usleep(80000);
                rpi_gpio_output(LED_GPIO, GPIO_LOW);  usleep(400000);
                break;

            case ADAS_LEVEL_CAUTION:
                printf("%sCAUTION seq=%u  mm=%ld\n",
                       LOG_TAG_SAF, v.seq, (long)v.distance_mm);
                {
                    int i;
                    for (i = 0; i < 4; i++) {
                        rpi_gpio_output(LED_GPIO, GPIO_HIGH); usleep(100000);
                        rpi_gpio_output(LED_GPIO, GPIO_LOW);  usleep(100000);
                    }
                }
                break;

            case ADAS_LEVEL_EMERGENCY:
                printf("%sEMERGENCY seq=%u  mm=%ld\n",
                       LOG_TAG_SAF, v.seq, (long)v.distance_mm);
                {
                    int i;
                    for (i = 0; i < 6; i++) {
                        rpi_gpio_output(LED_GPIO, GPIO_HIGH); usleep(50000);
                        rpi_gpio_output(LED_GPIO, GPIO_LOW);  usleep(50000);
                    }
                }
                break;
        }

        if (sup_coid != -1) {
            svc_send_heartbeat(sup_coid, PULSE_HB_SAFETY);
        }
    }

    timer_delete(timer_id);
    name_detach(attach, 0);
    return EXIT_SUCCESS;
}
