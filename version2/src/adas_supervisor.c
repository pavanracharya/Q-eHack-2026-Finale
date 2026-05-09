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
 * adas_supervisor — QNX-native process supervisor with HAM integration.
 *
 * Architecture:
 *   - Supervisor spawns all services → it is their parent process
 *   - waitpid(WNOHANG) catches SIGKILL deaths (works because supervisor is parent)
 *   - Supervisor restarts dead services via spawn() and measures RTO
 *   - HAM is used for heartbeat monitoring (CONDHBEATMISSEDLOW/HIGH)
 *     via ham_attach_self on each service + ham_heartbeat() calls
 *   - MsgReceivePulse receives service heartbeat pulses for supervisor-side
 *     hung-process detection
 *
 * HAM API used:
 *   ham_connect()                — connect to HAM daemon
 *   ham_attach()                 — register each service entity (external attach)
 *   ham_condition(CONDDEATH)     — death condition (fires on core-dump signals)
 *   ham_condition(CONDHBEATMISSEDLOW) — heartbeat miss condition
 *   ham_action_notify_pulse()    — notify supervisor on condition trigger
 *   ham_action_restart()         — HAM secondary restart (backup to waitpid)
 *
 * CPU: Core 3, Priority: 48, SCHED_FIFO
 */

#include <errno.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifdef USE_HAM
#include <ha/ham.h>
#endif

#include "ipc_protocol.h"
#include "service_common.h"

/* -----------------------------------------------------------------------
 * Pulse codes — HAM notifications (must not clash with PULSE_HB_*)
 * --------------------------------------------------------------------- */
#define PULSE_HAM_DEATH    (_PULSE_CODE_MINAVAIL + 30)
#define PULSE_HAM_RESTART  (_PULSE_CODE_MINAVAIL + 31)

#define SUPERVISOR_CPU        3
#define SUPERVISOR_PRIO       48
#define HB_TIMEOUT_MS         4000   /* 4s — generous for sensor loop */
#define FAILSAFE_THRESHOLD    3

typedef enum {
    MODE_NORMAL   = 0,
    MODE_DEGRADED = 1,
    MODE_FAILSAFE = 2,
} system_mode_t;

static const char *mode_name[] = { "NORMAL", "DEGRADED", "FAIL-SAFE" };
static system_mode_t g_mode = MODE_NORMAL;

typedef struct {
    const char *name;
    char        binary[256];  /* full absolute path — built at runtime */
    int         cpu;
    int         priority;
    int         restartable;  /* 0 = safety (never restart) */
    pid_t       pid;
    int64_t     last_hb_ms;
    int         restarts;
    int64_t     died_at_ms;
    int         hb_pulse;
#ifdef USE_HAM
    ham_entity_t    *ham_entity;
#endif
} service_t;

/* Binary names only — full paths built from argv[0] directory at startup */
static service_t g_svc[] = {
    { "safety_service",   "", 1, 58, 0, -1, 0, 0, 0, PULSE_HB_SAFETY   },
    { "decision_service", "", 1, 52, 1, -1, 0, 0, 0, PULSE_HB_DECISION },
    { "sensor_service",   "", 2, 54, 1, -1, 0, 0, 0, PULSE_HB_SENSOR   },
    { "camera_service",   "", 0, 63, 1, -1, 0, 0, 0, PULSE_HB_CAMERA   },
};
#define NUM_SERVICES  ((int)(sizeof(g_svc)/sizeof(g_svc[0])))

static name_attach_t *g_attach = NULL;
static int64_t g_start_ms = 0;

/* -----------------------------------------------------------------------
 * Timestamped log
 * --------------------------------------------------------------------- */
static void ts_log(const char *fmt, ...)
{
    const int64_t rel = svc_now_ms() - g_start_ms;
    printf("[t=%lldms] ", (long long)rel);
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

/* -----------------------------------------------------------------------
 * Mode transition
 * --------------------------------------------------------------------- */
static void set_mode(system_mode_t m)
{
    if (m == g_mode) return;
    g_mode = m;
    ts_log("%sMODE -> %s", LOG_TAG_SUP, mode_name[g_mode]);
    switch (g_mode) {
    case MODE_NORMAL:
        printf("  MODE: NORMAL\n  SENSORS: ACTIVE\n  PROCESSOR: ACTIVE\n\n");
        break;
    case MODE_DEGRADED:
        printf("  MODE: DEGRADED\n  PROCESSOR FAILED\n  RECOVERING...\n\n");
        break;
    case MODE_FAILSAFE:
        printf("  FAIL-SAFE MODE\n  SAFETY ONLY\n\n");
        break;
    }
    fflush(stdout);
}

/* -----------------------------------------------------------------------
 * Spawn a service with CPU affinity + SCHED_FIFO
 * Supervisor is the parent → waitpid() will catch its death
 * --------------------------------------------------------------------- */
static pid_t do_spawn(service_t *s)
{
    struct inheritance inherit;
    memset(&inherit, 0, sizeof(inherit));
    inherit.flags  = SPAWN_EXPLICIT_CPU | SPAWN_EXPLICIT_SCHED;
    inherit.policy = SCHED_FIFO;
    inherit.param.sched_priority = s->priority;
    inherit.runmask = (unsigned)(1u << (unsigned)s->cpu);

    char *av[] = { s->binary, NULL };
    pid_t pid = spawn(s->binary, 0, NULL, &inherit, av, NULL);
    if (pid == -1) {
        fprintf(stderr, "%sError: spawn(%s): %s\n",
                LOG_TAG_SUP, s->binary, strerror(errno));
    }
    return pid;
}

/* -----------------------------------------------------------------------
 * HAM registration after spawn.
 * Uses ham_attach() (external attach with known PID).
 * Adds CONDDEATH + ham_action_restart() as a SECONDARY restart layer.
 * Primary restart is done by supervisor via waitpid + do_spawn.
 * --------------------------------------------------------------------- */
static void ham_register(service_t *s)
{
#ifdef USE_HAM
    if (s->pid <= 0) return;

    /* Detach previous entity if re-registering after restart */
    if (s->ham_entity != NULL) {
        ham_detach(s->ham_entity, 0);
        s->ham_entity = NULL;
    }

    /*
     * ham_attach() — external attach with running PID + binary path.
     * The binary path (line) is used by HAM for ham_action_restart().
     */
    s->ham_entity = ham_attach(s->name,
                                ND_LOCAL_NODE,
                                s->pid,
                                s->binary,
                                0);
    if (s->ham_entity == NULL) {
        fprintf(stderr, "%sham_attach(%s pid=%d): %s\n",
                LOG_TAG_SUP, s->name, s->pid, strerror(errno));
        return;
    }

    /* CONDDEATH — fires on core-dump signals (SIGSEGV, SIGABRT, etc.)
     * Note: SIGKILL is caught by waitpid, not HAM CONDDEATH.
     * HREARMAFTERRESTART keeps condition alive across restarts. */
    ham_condition_t *hc = ham_condition(s->ham_entity,
                                         CONDDEATH, "death",
                                         HREARMAFTERRESTART);
    if (hc != NULL) {
        /* Secondary restart via HAM (backup if supervisor misses it) */
        ham_action_restart(hc, "restart", s->binary, HREARMAFTERRESTART);

        /* Notify supervisor via pulse on death */
        ham_action_notify_pulse(hc, "death_pulse",
                                 0,              /* reserved */
                                 0,              /* topid (0 = this process) */
                                 g_attach->chid,
                                 PULSE_HAM_DEATH,
                                 (int)(s - g_svc),
                                 HREARMAFTERRESTART);
    }

    /* CONDRESTART — fires when HAM restarts the process */
    ham_condition_t *hr = ham_condition(s->ham_entity,
                                         CONDRESTART, "restarted",
                                         HREARMAFTERRESTART);
    if (hr != NULL) {
        ham_action_notify_pulse(hr, "restart_pulse",
                                 0,
                                 0,
                                 g_attach->chid,
                                 PULSE_HAM_RESTART,
                                 (int)(s - g_svc),
                                 HREARMAFTERRESTART);
    }

    ts_log("%sHAM: %s registered  pid=%d", LOG_TAG_SUP, s->name, s->pid);
#else
    (void)s;
#endif
}

/* -----------------------------------------------------------------------
 * Start a service (initial launch or restart)
 * --------------------------------------------------------------------- */
static void start_service(int idx)
{
    service_t *s = &g_svc[idx];
    ts_log("%sSpawning %-20s  CPU=%d  PRIO=%d",
           LOG_TAG_SUP, s->name, s->cpu, s->priority);

    s->pid = do_spawn(s);
    if (s->pid == -1) return;

    s->last_hb_ms = svc_now_ms();
    ts_log("%s%-20s started  pid=%d", LOG_TAG_SUP, s->name, s->pid);
    ham_register(s);
}

/* -----------------------------------------------------------------------
 * Handle confirmed death (from waitpid)
 * --------------------------------------------------------------------- */
static void handle_death(int idx, int sig)
{
    service_t *s = &g_svc[idx];
    s->died_at_ms = svc_now_ms();
    s->pid = -1;

    ts_log("%s*** DEATH: %-20s  signal=%d  restarts_so_far=%d",
           LOG_TAG_SUP, s->name, sig, s->restarts);

    if (!s->restartable) {
        ts_log("%sCRITICAL: safety_service died — FAIL-SAFE MODE", LOG_TAG_SUP);
        set_mode(MODE_FAILSAFE);
        return;
    }

    s->restarts++;
    if (s->restarts >= FAILSAFE_THRESHOLD) {
        set_mode(MODE_FAILSAFE);
    } else {
        set_mode(MODE_DEGRADED);
    }

    /* Restart immediately — supervisor is the parent so it owns restart */
    s->pid = do_spawn(s);
    if (s->pid == -1) {
        ts_log("%sERROR: restart of %s FAILED", LOG_TAG_SUP, s->name);
        return;
    }

    const int64_t rto = svc_now_ms() - s->died_at_ms;
    s->last_hb_ms = svc_now_ms();

    ts_log("%s*** RECOVERED: %-20s  new_pid=%d  RTO=%lld ms  total_restarts=%d",
           LOG_TAG_SUP, s->name, s->pid, (long long)rto, s->restarts);

    ham_register(s);

    /* Check if all restartable services are back */
    int all_ok = 1, i;
    for (i = 0; i < NUM_SERVICES; i++) {
        if (g_svc[i].restartable && g_svc[i].pid == -1) { all_ok = 0; break; }
    }
    if (all_ok && g_mode == MODE_DEGRADED) set_mode(MODE_NORMAL);
}

/* -----------------------------------------------------------------------
 * main
 * --------------------------------------------------------------------- */
int main(int argc, char *argv[])
{
    g_start_ms = svc_now_ms();

    /* Build full binary paths from the directory of this executable */
    {
        char dir[256] = {0};
        const char *slash = strrchr(argv[0], '/');
        if (slash) {
            size_t n = (size_t)(slash - argv[0]);
            if (n >= sizeof(dir)) n = sizeof(dir) - 1;
            strncpy(dir, argv[0], n);
            dir[n] = '\0';
        } else {
            if (!getcwd(dir, sizeof(dir))) { dir[0]='.'; dir[1]='\0'; }
        }
        int i;
        for (i = 0; i < NUM_SERVICES; i++) {
            snprintf(g_svc[i].binary, sizeof(g_svc[i].binary),
                     "%s/%s", dir, g_svc[i].name);
        }
        fprintf(stderr, "%sBinary dir: %s\n", LOG_TAG_SUP, dir);
    }

    svc_set_priority(SUPERVISOR_PRIO, LOG_TAG_SUP);
    svc_bind_cpu(SUPERVISOR_CPU, LOG_TAG_SUP);

    ts_log("%sQNX ADAS SUPERVISOR  CPU=%d  PRIO=%d  SCHED_FIFO",
           LOG_TAG_SUP, SUPERVISOR_CPU, SUPERVISOR_PRIO);

    /* IPC channel — must exist before HAM registration */
    g_attach = name_attach(NULL, ADAS_CHANNEL_SUPERVISOR, 0);
    if (!g_attach) { perror("name_attach"); return EXIT_FAILURE; }
    ts_log("%sChannel '%s' ready", LOG_TAG_SUP, ADAS_CHANNEL_SUPERVISOR);

#ifdef USE_HAM
    if (ham_connect(0) == 0) {
        ts_log("%sConnected to HAM daemon", LOG_TAG_SUP);
    } else {
        fprintf(stderr, "%sham_connect failed: %s\n", LOG_TAG_SUP, strerror(errno));
    }
#else
    ts_log("%sHAM disabled", LOG_TAG_SUP);
#endif

    /* Spawn order: safety first (creates adas_safety channel),
     * then decision (connects to safety), sensor (connects to decision),
     * camera (independent) */
    int i;
    for (i = 0; i < NUM_SERVICES; i++) {
        start_service(i);
        usleep(300000); /* 300ms — let each service register its channel */
    }

    set_mode(MODE_NORMAL);
    ts_log("%sAll services ONLINE. Do: kill -9 <pid> to inject fault.", LOG_TAG_SUP);

    /* ---------------------------------------------------------------
     * Supervision loop — 20 Hz
     *
     * A) waitpid(WNOHANG) — primary death detection (supervisor is parent)
     *    Catches SIGKILL immediately. RTO measured here.
     *
     * B) MsgReceivePulse  — non-blocking pulse drain:
     *    PULSE_HAM_DEATH   — HAM secondary death notification (log only)
     *    PULSE_HAM_RESTART — HAM secondary restart notification (log only)
     *    PULSE_HB_*        — service heartbeats (update last_hb_ms)
     *
     * C) Heartbeat timeout — kill hung processes (supervisor restarts them)
     * ------------------------------------------------------------- */
    while (1) {
        /* A) waitpid — catches all SIGKILL deaths */
        {
            int st; pid_t dp;
            while ((dp = waitpid(-1, &st, WNOHANG)) > 0) {
                int sig = WIFSIGNALED(st) ? WTERMSIG(st) : 0;
                for (i = 0; i < NUM_SERVICES; i++) {
                    if (g_svc[i].pid == dp) {
                        handle_death(i, sig);
                        break;
                    }
                }
            }
        }

        /* B) Drain pulses */
        {
            struct _pulse p;
            while (MsgReceivePulse(g_attach->chid, &p, sizeof(p), NULL) == 0) {
                const int code = p.code;
                const int val  = p.value.sival_int;

#ifdef USE_HAM
                if (code == PULSE_HAM_DEATH) {
                    if (val >= 0 && val < NUM_SERVICES)
                        ts_log("%sHAM CONDDEATH pulse: %s",
                               LOG_TAG_SUP, g_svc[val].name);
                    continue;
                }
                if (code == PULSE_HAM_RESTART) {
                    if (val >= 0 && val < NUM_SERVICES) {
                        const int64_t rto = svc_now_ms() - g_svc[val].died_at_ms;
                        ts_log("%sHAM CONDRESTART pulse: %s  RTO=%lld ms",
                               LOG_TAG_SUP, g_svc[val].name, (long long)rto);
                    }
                    continue;
                }
#endif
                /* Heartbeat pulses from services */
                for (i = 0; i < NUM_SERVICES; i++) {
                    if (g_svc[i].hb_pulse == code) {
                        g_svc[i].last_hb_ms = svc_now_ms();
                        break;
                    }
                }
            }
        }

        /* C) Heartbeat timeout — kill hung processes */
        {
            const int64_t now = svc_now_ms();
            for (i = 0; i < NUM_SERVICES; i++) {
                if (g_svc[i].pid == -1 || g_svc[i].last_hb_ms == 0) continue;
                const int64_t age = now - g_svc[i].last_hb_ms;
                if (age > HB_TIMEOUT_MS) {
                    ts_log("%sHB TIMEOUT: %s  age=%lld ms — SIGKILL",
                           LOG_TAG_SUP, g_svc[i].name, (long long)age);
                    kill(g_svc[i].pid, SIGKILL);
                    /* waitpid catches it next iteration */
                }
            }

            /* Periodic status */
            static int64_t last_print = 0;
            if ((now - last_print) > 5000) {
                last_print = now;
                ts_log("%s--- STATUS MODE=%s ---", LOG_TAG_SUP, mode_name[g_mode]);
                for (i = 0; i < NUM_SERVICES; i++) {
                    ts_log("%s  %-20s  pid=%-6d  restarts=%d",
                           LOG_TAG_SUP, g_svc[i].name,
                           g_svc[i].pid, g_svc[i].restarts);
                }
            }
        }

        usleep(50000); /* 20 Hz */
    }

#ifdef USE_HAM
    ham_disconnect(0);
#endif
    name_detach(g_attach, 0);
    return EXIT_SUCCESS;
}
