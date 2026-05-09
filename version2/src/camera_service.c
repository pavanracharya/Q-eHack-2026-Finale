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
 * camera_service — launches and monitors camera_example3_viewfinder.
 *
 * Pattern taken directly from camera_test/src/camera_test.c (QNX reference).
 * Uses posix_spawnp() to start the QNX viewfinder binary, then monitors it
 * with waitpid(WNOHANG). If it exits, restarts it.
 *
 * This process is the fault injection target for judges:
 *   kill -9 <camera_service_pid>
 * → supervisor detects death, HAM restarts camera_service
 * → camera_service restarts camera_example3_viewfinder
 * → safety_service LED never stops (separate process, CPU1)
 *
 * CPU:      Core 0  (isolated — highest priority workload)
 * Priority: 63      (SCHED_FIFO)
 */

#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ipc_protocol.h"
#include "service_common.h"

#define CAMERA_CPU   0
#define CAMERA_PRIO  63

extern char **environ;

int main(int argc, char *argv[])
{
    /* --- RT setup ---------------------------------------------------- */
    svc_set_priority(CAMERA_PRIO, LOG_TAG_CAM);
    svc_bind_cpu(CAMERA_CPU, LOG_TAG_CAM);

    printf("%sStarting camera_service  CPU=%d  PRIO=%d  SCHED_FIFO\n",
           LOG_TAG_CAM, CAMERA_CPU, CAMERA_PRIO);

    /*
     * HAM registration is done by adas_supervisor after spawning this process.
     * Services do NOT self-register — supervisor owns all HAM entities.
     */

    printf("%sOnline — simulating camera pipeline workload\n", LOG_TAG_CAM);
    int sup_coid = -1;
    {
        int tries = 0;
        while (sup_coid == -1 && tries < 20) {
            sup_coid = name_open(ADAS_CHANNEL_SUPERVISOR, 0);
            if (sup_coid == -1) { usleep(100000); tries++; }
        }
        if (sup_coid == -1) {
            fprintf(stderr, "%sWarning: supervisor not found\n", LOG_TAG_CAM);
        }
    }

    /*
     * Launch camera_example3_viewfinder — the QNX-provided viewfinder binary.
     * Pattern from camera_test/src/camera_test.c (QNX reference code).
     */
    pid_t cam_pid = -1;
    {
        char *const child_argv[] = { "camera_example3_viewfinder", NULL };
        int rc = posix_spawnp(&cam_pid, "camera_example3_viewfinder",
                               NULL, NULL, child_argv, environ);
        if (rc != 0) {
            fprintf(stderr, "%sFailed to start camera_example3_viewfinder: %s\n",
                    LOG_TAG_CAM, strerror(rc));
            /*
             * Camera hardware may not be connected — continue running so
             * the service stays alive for fault injection demo purposes.
             */
            cam_pid = -1;
        } else {
            printf("%scamera_example3_viewfinder started  pid=%ld\n",
                   LOG_TAG_CAM, (long)cam_pid);
        }
    }

    uint32_t tick = 0;

    while (1) {
        tick++;

        /* Monitor viewfinder process — restart if it exits */
        if (cam_pid > 0) {
            int st = 0;
            pid_t p = waitpid(cam_pid, &st, WNOHANG);
            if (p == cam_pid) {
                printf("%scamera_example3_viewfinder exited (status=0x%x) — restarting\n",
                       LOG_TAG_CAM, st);
                cam_pid = -1;

                /* Restart viewfinder */
                char *const child_argv[] = { "camera_example3_viewfinder", NULL };
                int rc = posix_spawnp(&cam_pid, "camera_example3_viewfinder",
                                       NULL, NULL, child_argv, environ);
                if (rc != 0) {
                    fprintf(stderr, "%sRestart failed: %s\n",
                            LOG_TAG_CAM, strerror(rc));
                    cam_pid = -1;
                } else {
                    printf("%scamera_example3_viewfinder restarted  pid=%ld\n",
                           LOG_TAG_CAM, (long)cam_pid);
                }
            }
        }

        if ((tick % 40u) == 0u) {
            printf("%sAlive  viewfinder_pid=%ld\n",
                   LOG_TAG_CAM, (long)cam_pid);
        }

        /* HAM heartbeat */
        svc_ham_heartbeat();

        /* Supervisor pulse heartbeat */
        if (sup_coid != -1) {
            svc_send_heartbeat(sup_coid, PULSE_HB_CAMERA);
        }

        usleep(50000); /* 20 Hz monitor loop */
    }

    /* Cleanup (unreachable in normal operation) */
    if (cam_pid > 0) {
        kill(cam_pid, SIGTERM);
        waitpid(cam_pid, NULL, 0);
    }
    if (sup_coid != -1) name_close(sup_coid);
    return EXIT_SUCCESS;
}
