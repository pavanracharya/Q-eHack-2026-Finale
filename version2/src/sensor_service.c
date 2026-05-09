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
 * sensor_service — HC-SR04 ultrasonic + LSM6DSOX IMU data collection.
 *
 * Reads sensors at 20 Hz and sends adas_sensor_frame_t to decision_service
 * via QNX MsgSend. If decision_service dies and is restarted by the
 * supervisor, sensor_service detects the broken connection (MsgSend returns
 * -1 / ESRCH) and reconnects via name_open loop.
 *
 * Registered with HAM: YES — supervisor restarts it on death.
 *
 * CPU:      Core 2  (dedicated sensor collection core)
 * Priority: 54      (SCHED_FIFO — high, below safety)
 *
 * Hardware:
 *   HC-SR04: TRIG=GPIO23, ECHO=GPIO24  (see ultrasonic_hcsr04.c)
 *   LSM6DSOX: /dev/i2c1, addr 0x6A/0x6B (see imu_lsm6dsox.c)
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>

#include "imu_lsm6dsox.h"
#include "ipc_protocol.h"
#include "service_common.h"
#include "ultrasonic_hcsr04.h"

#define SENSOR_CPU   2
#define SENSOR_PRIO  54

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    /* --- RT setup ---------------------------------------------------- */
    svc_set_priority(SENSOR_PRIO, LOG_TAG_SNS);
    svc_bind_cpu(SENSOR_CPU, LOG_TAG_SNS);

    printf("%sStarting sensor_service  CPU=%d  PRIO=%d  SCHED_FIFO\n",
           LOG_TAG_SNS, SENSOR_CPU, SENSOR_PRIO);

    /* --- Hardware init ----------------------------------------------- */
    if (ultra_init() != 0) {
        fprintf(stderr, "%sError: Ultrasonic init failed\n", LOG_TAG_SNS);
        return EXIT_FAILURE;
    }

    uint8_t imu_addr = 0;
    if (imu_lsm6dsox_init(&imu_addr) != 0) {
        fprintf(stderr, "%sError: IMU init failed\n", LOG_TAG_SNS);
        return EXIT_FAILURE;
    }
    printf("%sHardware ready  IMU=0x%02X\n", LOG_TAG_SNS, imu_addr);

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
            fprintf(stderr, "%sWarning: supervisor not found\n", LOG_TAG_SNS);
        }
    }

    /* --- Connect to decision_service --------------------------------- */
    int dec_coid = -1;
    printf("%sWaiting for decision_service...\n", LOG_TAG_SNS);
    while (dec_coid == -1) {
        dec_coid = name_open(ADAS_CHANNEL_DECISION, 0);
        if (dec_coid == -1) usleep(100000);
    }
    printf("%sConnected to decision_service\n", LOG_TAG_SNS);

    uint16_t seq = 0;

    while (1) {
        /* --- Read ultrasonic ----------------------------------------- */
        float mm = -1.f;
        const int urc = ultra_read_mm(&mm);
        const uint8_t ultra_ok = (urc == 0) ? 1u : 0u;

        /* --- Read IMU ------------------------------------------------ */
        int16_t gx = 0, gy = 0, gz = 0, ax = 0, ay = 0, az = 0;
        (void)imu_lsm6dsox_read(imu_addr, &gx, &gy, &gz, &ax, &ay, &az);

        /* --- Build frame --------------------------------------------- */
        adas_sensor_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.type         = ADAS_MSG_SENSOR_FRAME;
        frame.seq          = ++seq;
        frame.distance_mm  = ultra_ok ? (int32_t)mm : -1;
        frame.ultrasonic_ok = ultra_ok;
        frame.ax = ax; frame.ay = ay; frame.az = az;
        frame.gx = gx; frame.gy = gy; frame.gz = gz;

        /* --- Send to decision_service -------------------------------- */
        adas_simple_ack_t ack = {0};
        if (MsgSend(dec_coid, &frame, sizeof(frame),
                    &ack, sizeof(ack)) == -1) {
            /*
             * decision_service died (SIGKILL from judge or crash).
             * The kernel has already cleaned up its channel.
             * Reconnect loop — supervisor is restarting decision in parallel.
             */
            fprintf(stderr, "%sLost connection to decision_service (%s) — reconnecting\n",
                    LOG_TAG_SNS, strerror(errno));
            name_close(dec_coid);
            dec_coid = -1;
            while (dec_coid == -1) {
                dec_coid = name_open(ADAS_CHANNEL_DECISION, 0);
                if (dec_coid == -1) usleep(100000);
            }
            printf("%sReconnected to decision_service\n", LOG_TAG_SNS);
            continue;
        }

        /* --- Heartbeat to supervisor --------------------------------- */
        if (sup_coid != -1) {
            svc_send_heartbeat(sup_coid, PULSE_HB_SENSOR);
        }

        /* HAM heartbeat */
        svc_ham_heartbeat();

        usleep(50000); /* 20 Hz */
    }

    name_close(dec_coid);
    if (sup_coid != -1) name_close(sup_coid);
    return EXIT_SUCCESS;
}
