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

#ifndef SERVICE_COMMON_H
#define SERVICE_COMMON_H

#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/syspage.h>

/* -----------------------------------------------------------------------
 * Log tags
 * --------------------------------------------------------------------- */
#define LOG_TAG_SUP  "[SUPERVISOR] "
#define LOG_TAG_CAM  "[CAMERA]     "
#define LOG_TAG_SNS  "[SENSOR]     "
#define LOG_TAG_DEC  "[DECISION]   "
#define LOG_TAG_SAF  "[SAFETY]     "

/* -----------------------------------------------------------------------
 * Monotonic millisecond clock
 * --------------------------------------------------------------------- */
static inline int64_t svc_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

/* -----------------------------------------------------------------------
 * CPU affinity — QNX-native ThreadCtl runmask
 * Matches rt_affinity.h pattern from gpio_test reference.
 * --------------------------------------------------------------------- */
static inline int svc_bind_cpu(int cpu, const char *who)
{
    const int num_cpu = _syspage_ptr->num_cpu;
    if (cpu < 0 || cpu >= num_cpu) {
        fprintf(stderr, "%s: invalid cpu=%d (num_cpu=%d)\n", who, cpu, num_cpu);
        return -1;
    }

    const int rsize = RMSK_SIZE(num_cpu);
    const size_t bytes = sizeof(struct _thread_runmask)
                         + (size_t)(2 * rsize) * sizeof(unsigned);

    struct _thread_runmask *rm = (struct _thread_runmask *)calloc(1, bytes);
    if (!rm) { perror("calloc(runmask)"); return -1; }

    rm->size = rsize;
    unsigned *runmask = (unsigned *)(rm + 1);
    unsigned *inherit  = runmask + rsize;
    RMSK_SET(cpu, runmask);
    RMSK_SET(cpu, inherit);

    if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT, rm) == -1) {
        fprintf(stderr, "%s: ThreadCtl(runmask cpu=%d) failed: %s\n",
                who, cpu, strerror(errno));
        free(rm);
        return -1;
    }
    fprintf(stderr, "%s: pinned to CPU%d/%d\n", who, cpu, num_cpu);
    free(rm);
    return 0;
}

/* -----------------------------------------------------------------------
 * SCHED_FIFO priority — matches rt_prio.h pattern from gpio_test reference
 * --------------------------------------------------------------------- */
static inline void svc_set_priority(int prio, const char *who)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;
    if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
        fprintf(stderr, "%s: sched_setscheduler(FIFO, %d) failed: %s\n",
                who, prio, strerror(errno));
    }
}

/* -----------------------------------------------------------------------
 * Pulse heartbeat to supervisor channel
 * --------------------------------------------------------------------- */
static inline int svc_send_heartbeat(int coid, int pulse_code)
{
    return MsgSendPulse(coid, -1, pulse_code, 0);
}

/* -----------------------------------------------------------------------
 * HAM self-attach + CONDDEATH restart + heartbeat monitoring.
 *
 * Each restartable service calls this once at startup.
 * Uses ham_attach_self() — works for ANY process (not just session-1).
 * HAM monitors via heartbeats (CONDHBEATMISSEDLOW/HIGH) AND death (CONDDEATH).
 *
 * binary_path: FULL absolute path to this binary (e.g. "/tmp/adas/camera_service")
 * hb_interval_ns: heartbeat interval in nanoseconds (e.g. 500000000 = 500ms)
 *                 pass 0 to disable heartbeat monitoring (death-only)
 * --------------------------------------------------------------------- */
#ifdef USE_HAM
#include <ha/ham.h>

/* Global entity handle — used by svc_ham_heartbeat() */
static ham_entity_t *g_ham_entity = NULL;

static inline int svc_ham_init(const char *entity_name,
                                const char *binary_path,
                                uint64_t    hb_interval_ns)
{
    /*
     * ham_attach_self() — self-attached entity.
     * Works for any process regardless of session.
     * hp = heartbeat interval in ns (0 = no heartbeat monitoring)
     * hpdl = missed heartbeats before CONDHBEATMISSEDLOW (2)
     * hpdh = missed heartbeats before CONDHBEATMISSEDHIGH (4)
     */
    g_ham_entity = ham_attach_self(entity_name,
                                    hb_interval_ns,
                                    2,   /* hpdl */
                                    4,   /* hpdh */
                                    0);
    if (g_ham_entity == NULL) {
        fprintf(stderr, "ham_attach_self(%s) failed: %s\n",
                entity_name, strerror(errno));
        return -1;
    }

    /*
     * CONDDEATH — triggered when this process dies (SIGKILL, crash, etc.)
     * HREARMAFTERRESTART — condition survives across restarts so HAM
     * keeps monitoring after each restart.
     */
    ham_condition_t *hc_death = ham_condition(g_ham_entity,
                                               CONDDEATH,
                                               "death",
                                               HREARMAFTERRESTART);
    if (hc_death == NULL) {
        fprintf(stderr, "ham_condition(CONDDEATH, %s) failed: %s\n",
                entity_name, strerror(errno));
        return -1;
    }

    /*
     * ham_action_restart() — 4 args: (condition, action_name, path, flags)
     * path = FULL path to binary so HAM can re-spawn it.
     * HREARMAFTERRESTART — action survives across restarts.
     */
    ham_action_t *ha = ham_action_restart(hc_death,
                                           "restart",
                                           binary_path,
                                           HREARMAFTERRESTART);
    if (ha == NULL) {
        fprintf(stderr, "ham_action_restart(%s) failed: %s\n",
                entity_name, strerror(errno));
        return -1;
    }

    /*
     * CONDHBEATMISSEDLOW — triggered when heartbeats are missed (hung process).
     * Action: restart the process.
     */
    if (hb_interval_ns > 0) {
        ham_condition_t *hc_hb = ham_condition(g_ham_entity,
                                                CONDHBEATMISSEDLOW,
                                                "hb_missed",
                                                HREARMAFTERRESTART);
        if (hc_hb != NULL) {
            ham_action_restart(hc_hb, "hb_restart", binary_path,
                               HREARMAFTERRESTART);
        }
    }

    fprintf(stderr, "HAM: entity '%s' registered  binary='%s'\n",
            entity_name, binary_path);
    return 0;
}

/* Send heartbeat to HAM — call this in the main service loop */
static inline void svc_ham_heartbeat(void)
{
    if (g_ham_entity != NULL) {
        ham_heartbeat();
    }
}

#else /* !USE_HAM */

static inline int svc_ham_init(const char *entity_name,
                                const char *binary_path,
                                uint64_t    hb_interval_ns)
{
    (void)entity_name; (void)binary_path; (void)hb_interval_ns;
    return 0;
}

static inline void svc_ham_heartbeat(void) {}

#endif /* USE_HAM */

#endif /* SERVICE_COMMON_H */
