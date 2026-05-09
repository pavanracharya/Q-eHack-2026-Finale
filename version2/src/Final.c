/*
 * Single-binary ADAS runtime with QNX-native clustering + failover.
 *
 * Primary flow preserved:
 *   sensor_service -> adas_sensor_frame_t -> decision_engine -> safety_controller
 *
 * CPU clustering:
 *   C0: camera_critical_sim (highest)
 *   C1: safety + decision
 *   C2: sensor
 *   C3: multimedia + monitor/fallback helpers
 *
 * Fault tolerance:
 *   - heartbeat pulses from every main thread
 *   - timestamped FAILED/RECOVERED logs
 *   - backup/fallback paths activated during failure windows
 */

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "imu_lsm6dsox.h"
#include "ipc_protocol.h"
#include "rpi_gpio.h"
#include "rt_affinity.h"
#include "rt_prio.h"
#include "ultrasonic_hcsr04.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define LED_GPIO GPIO16
#define AIRBAG_GPIO GPIO19
#define PULSE_BASE (_PULSE_CODE_MINAVAIL + 10)
/* Airbag trigger tuning (raw IMU units, tune on target). */
#define AIRBAG_GYRO_RAW_THRESHOLD 18000
#define AIRBAG_ROLL_DEG_THRESHOLD 25.0f
#define AIRBAG_ROLL_STEP_DEG_THRESHOLD 10.0f
#define AIRBAG_ROLL_RATE_DPS_THRESHOLD 180.0f
#define AIRBAG_ACCEL_DELTA_THRESHOLD 4500.0f
#define IMU_1G_RAW 16384.0f

static volatile sig_atomic_t g_running = 1;

/* Channel graph */
static int g_decision_chid = -1;
static int g_safety_chid = -1;
static pthread_mutex_t g_chan_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_chan_cv = PTHREAD_COND_INITIALIZER;
static int g_decision_ready = 0;
static int g_safety_ready = 0;

/* monitor channel for heartbeat pulses */
static int g_monitor_chid = -1;
static pthread_mutex_t g_monitor_mu = PTHREAD_MUTEX_INITIALIZER;

enum mod_id {
    MOD_CAMERA = 0,
    MOD_SENSOR,
    MOD_DECISION,
    MOD_SAFETY,
    MOD_MEDIA,
    MOD_COUNT
};

static const char *MOD_NAME[MOD_COUNT] = {
    "camera_realtime",
    "sensor_service",
    "decision_engine",
    "safety_controller",
    "multimedia_sim"
};

static int64_t g_hb_ms[MOD_COUNT];
static int g_failed[MOD_COUNT];
static int64_t g_failed_at_ms[MOD_COUNT];
static pthread_mutex_t g_hb_mu = PTHREAD_MUTEX_INITIALIZER;

/* fallback flags */
static volatile int g_camera_fallback_active = 0;
static volatile int g_sensor_fallback_active = 0;
static volatile int g_decision_fallback_active = 0;
static volatile int g_safety_guard_active = 0;

/* shared data for fallback paths */
typedef struct {
    uint32_t seq;
    int64_t ts_ms;
    int source_fallback;
} camera_packet_t;

static camera_packet_t g_camera_pkt;
static pthread_mutex_t g_camera_mu = PTHREAD_MUTEX_INITIALIZER;

static adas_sensor_frame_t g_last_sensor_frame;
static pthread_mutex_t g_sensor_mu = PTHREAD_MUTEX_INITIALIZER;

/* thread handles for restart */
static pthread_t g_th_camera, g_th_sensor, g_th_decision, g_th_safety, g_th_media;
static pthread_t g_th_monitor_pulse, g_th_watchdog, g_th_sensor_fb, g_th_decision_fb, g_th_safety_guard, g_th_camera_fb;
static pthread_t g_th_airbag_guard;
static pthread_t g_th_fault_injector;
static pthread_mutex_t g_restart_mu = PTHREAD_MUTEX_INITIALIZER;
static int64_t g_start_ms = 0;

static volatile int g_force_imu_fail = 0;
static volatile int g_force_ultra_fail = 0;
static volatile int g_airbag_trigger = 0;
static int64_t g_airbag_trigger_ms = 0;
static pthread_mutex_t g_airbag_mu = PTHREAD_MUTEX_INITIALIZER;
static float g_airbag_roll_deg_thr = AIRBAG_ROLL_DEG_THRESHOLD;
static float g_airbag_roll_step_deg_thr = AIRBAG_ROLL_STEP_DEG_THRESHOLD;
static float g_airbag_roll_rate_dps_thr = AIRBAG_ROLL_RATE_DPS_THRESHOLD;
static float g_airbag_accel_delta_thr = AIRBAG_ACCEL_DELTA_THRESHOLD;

typedef struct {
    int kill_enabled;
    enum mod_id kill_mod;
    unsigned kill_after_s;

    int hog_enabled;
    int hog_core;
    unsigned hog_after_s;
    unsigned hog_dur_s;

    int storm_enabled;
    unsigned storm_after_s;
    unsigned storm_rate_hz;
    unsigned storm_dur_s;

    int sensor_fail_enabled;
    int sensor_fail_mode; /* 1=imu,2=ultra,3=both */
    unsigned sensor_fail_after_s;
    unsigned sensor_fail_dur_s;
} inject_cfg_t;

static inject_cfg_t g_inj;

/* timing */
static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static void msleep(unsigned ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000U;
    ts.tv_nsec = (long)(ms % 1000U) * 1000000L;
    (void)nanosleep(&ts, NULL);
}

static void ts_log(const char *fmt, ...)
{
    const int64_t rel_ms = now_ms() - g_start_ms;
    printf("[t=%lldms] ", (long long)rel_ms);
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

static void on_sig(int sig)
{
    (void)sig;
    g_running = 0;
}

static enum mod_id parse_mod_name(const char *s)
{
    if (!s) return MOD_COUNT;
    if (strcmp(s, "camera") == 0) return MOD_CAMERA;
    if (strcmp(s, "sensor") == 0) return MOD_SENSOR;
    if (strcmp(s, "decision") == 0) return MOD_DECISION;
    if (strcmp(s, "safety") == 0) return MOD_SAFETY;
    if (strcmp(s, "media") == 0) return MOD_MEDIA;
    return MOD_COUNT;
}

static int parse_u32(const char *s, unsigned *out)
{
    char *end = NULL;
    unsigned long v;
    if (!s || !out) return -1;
    v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') return -1;
    if (v == 0UL) return -1;
    *out = (unsigned)v;
    return 0;
}

static int parse_f32_pos(const char *s, float *out)
{
    char *end = NULL;
    float v;
    if (!s || !out) return -1;
    v = strtof(s, &end);
    if (end == s || *end != '\0' || v <= 0.0f) return -1;
    *out = v;
    return 0;
}

static void print_usage(const char *prog)
{
    fprintf(stderr,
            "Usage:\n"
            "  %s\n"
            "  %s --kill <camera|sensor|decision|safety|media> --after <sec>\n"
            "  %s --hog --core <0..3> --after <sec> --dur <sec>\n"
            "  %s --storm --after <sec> --rate <hz> --dur <sec>\n"
            "  %s --sensor-fail <imu|ultra|both> --after <sec> --dur <sec>\n"
            "  %s --airbag-roll <deg> --airbag-step <deg> --airbag-rate <dps> --airbag-accel <raw>\n",
            prog, prog, prog, prog, prog, prog);
}

static int parse_cli(int argc, char **argv)
{
    int i = 1;
    memset(&g_inj, 0, sizeof(g_inj));
    g_inj.kill_mod = MOD_COUNT;
    g_inj.hog_core = 3;
    g_inj.storm_rate_hz = 1000;

    while (i < argc) {
        if (strcmp(argv[i], "--kill") == 0) {
            if (i + 1 >= argc) return -1;
            g_inj.kill_mod = parse_mod_name(argv[i + 1]);
            if (g_inj.kill_mod == MOD_COUNT) return -1;
            g_inj.kill_enabled = 1;
            i += 2;
        } else if (strcmp(argv[i], "--after") == 0) {
            unsigned v = 0;
            if (i + 1 >= argc || parse_u32(argv[i + 1], &v) != 0) return -1;
            if (g_inj.storm_enabled && g_inj.storm_after_s == 0) g_inj.storm_after_s = v;
            else if (g_inj.sensor_fail_enabled && g_inj.sensor_fail_after_s == 0) g_inj.sensor_fail_after_s = v;
            else if (g_inj.hog_enabled && g_inj.hog_after_s == 0) g_inj.hog_after_s = v;
            else g_inj.kill_after_s = v;
            i += 2;
        } else if (strcmp(argv[i], "--hog") == 0) {
            g_inj.hog_enabled = 1;
            i += 1;
        } else if (strcmp(argv[i], "--core") == 0) {
            unsigned v = 0;
            if (i + 1 >= argc || parse_u32(argv[i + 1], &v) != 0) return -1;
            g_inj.hog_core = (int)v;
            i += 2;
        } else if (strcmp(argv[i], "--dur") == 0) {
            unsigned v = 0;
            if (i + 1 >= argc || parse_u32(argv[i + 1], &v) != 0) return -1;
            if (g_inj.storm_enabled && g_inj.storm_dur_s == 0) g_inj.storm_dur_s = v;
            else if (g_inj.sensor_fail_enabled && g_inj.sensor_fail_dur_s == 0) g_inj.sensor_fail_dur_s = v;
            else g_inj.hog_dur_s = v;
            i += 2;
        } else if (strcmp(argv[i], "--storm") == 0) {
            g_inj.storm_enabled = 1;
            i += 1;
        } else if (strcmp(argv[i], "--rate") == 0) {
            unsigned v = 0;
            if (i + 1 >= argc || parse_u32(argv[i + 1], &v) != 0) return -1;
            g_inj.storm_rate_hz = v;
            i += 2;
        } else if (strcmp(argv[i], "--sensor-fail") == 0) {
            if (i + 1 >= argc) return -1;
            if (strcmp(argv[i + 1], "imu") == 0) g_inj.sensor_fail_mode = 1;
            else if (strcmp(argv[i + 1], "ultra") == 0) g_inj.sensor_fail_mode = 2;
            else if (strcmp(argv[i + 1], "both") == 0) g_inj.sensor_fail_mode = 3;
            else return -1;
            g_inj.sensor_fail_enabled = 1;
            i += 2;
        } else if (strcmp(argv[i], "--airbag-roll") == 0) {
            if (i + 1 >= argc || parse_f32_pos(argv[i + 1], &g_airbag_roll_deg_thr) != 0) return -1;
            i += 2;
        } else if (strcmp(argv[i], "--airbag-step") == 0) {
            if (i + 1 >= argc || parse_f32_pos(argv[i + 1], &g_airbag_roll_step_deg_thr) != 0) return -1;
            i += 2;
        } else if (strcmp(argv[i], "--airbag-rate") == 0) {
            if (i + 1 >= argc || parse_f32_pos(argv[i + 1], &g_airbag_roll_rate_dps_thr) != 0) return -1;
            i += 2;
        } else if (strcmp(argv[i], "--airbag-accel") == 0) {
            if (i + 1 >= argc || parse_f32_pos(argv[i + 1], &g_airbag_accel_delta_thr) != 0) return -1;
            i += 2;
        } else {
            return -1;
        }
    }

    if (g_inj.kill_enabled && g_inj.kill_after_s == 0) return -1;
    if (g_inj.hog_enabled && (g_inj.hog_after_s == 0 || g_inj.hog_dur_s == 0)) return -1;
    if (g_inj.storm_enabled && (g_inj.storm_after_s == 0 || g_inj.storm_dur_s == 0 || g_inj.storm_rate_hz == 0)) return -1;
    if (g_inj.sensor_fail_enabled && (g_inj.sensor_fail_after_s == 0 || g_inj.sensor_fail_dur_s == 0 || g_inj.sensor_fail_mode == 0)) return -1;
    if (g_inj.hog_enabled && (g_inj.hog_core < 0 || g_inj.hog_core > 3)) return -1;

    return 0;
}

static enum adas_risk_level classify(const adas_sensor_frame_t *in, float roll, float pitch)
{
    enum adas_risk_level lvl = ADAS_LEVEL_NORMAL;

    if (fabsf(roll) > 35.f || fabsf(pitch) > 35.f) lvl = ADAS_LEVEL_CAUTION;
    if (in->ultrasonic_ok) {
        if (in->distance_mm > 0 && in->distance_mm < 280) return ADAS_LEVEL_EMERGENCY;
        if (in->distance_mm > 0 && in->distance_mm < 550 && lvl != ADAS_LEVEL_EMERGENCY) lvl = ADAS_LEVEL_CAUTION;
    } else if (lvl == ADAS_LEVEL_NORMAL) {
        lvl = ADAS_LEVEL_CAUTION;
    }
    return lvl;
}

/* heartbeat pulses */
static int hb_connect_coid(void)
{
    int coid = -1;
    pthread_mutex_lock(&g_monitor_mu);
    if (g_monitor_chid != -1) {
        coid = ConnectAttach(ND_LOCAL_NODE, 0, g_monitor_chid, _NTO_SIDE_CHANNEL, 0);
    }
    pthread_mutex_unlock(&g_monitor_mu);
    return coid;
}

static void hb_pulse(int coid, enum mod_id mod)
{
    if (coid != -1) {
        (void)MsgSendPulse(coid, 10, (int)(PULSE_BASE + mod), 0);
    }
}

extern char **environ;

static int imu_airbag_condition(float roll_deg, float droll_deg, float roll_rate_dps, float accel_delta,
                                int16_t gx, int16_t gy, int16_t gz)
{
    const int agx = (gx < 0) ? -gx : gx;
    const int agy = (gy < 0) ? -gy : gy;
    const int agz = (gz < 0) ? -gz : gz;

    /* Mimic rollover/crash: significant roll + dynamic change + high acceleration delta. */
    if (fabsf(roll_deg) > g_airbag_roll_deg_thr &&
        (fabsf(droll_deg) > g_airbag_roll_step_deg_thr || fabsf(roll_rate_dps) > g_airbag_roll_rate_dps_thr) &&
        accel_delta > g_airbag_accel_delta_thr) {
        return 1;
    }

    /* Hard emergency fallback for violent angular spikes with high acceleration change. */
    if ((agx > AIRBAG_GYRO_RAW_THRESHOLD || agy > AIRBAG_GYRO_RAW_THRESHOLD || agz > AIRBAG_GYRO_RAW_THRESHOLD) &&
        accel_delta > g_airbag_accel_delta_thr) return 1;
    return 0;
}

/* ==== primary threads ==== */
static int spawn_camera_worker(pid_t *out_pid)
{
    pid_t pid = -1;
    int rc = 0;
    char *const child_argv[] = { "camera_example3_viewfinder", NULL };
    rc = posix_spawnp(&pid, "camera_example3_viewfinder", NULL, NULL, child_argv, environ);
    if (rc != 0) {
        errno = rc;
        return -1;
    }
    *out_pid = pid;
    return 0;
}

static void camera_child_cleanup(void *arg)
{
    pid_t *pidp = (pid_t *)arg;
    if (!pidp || *pidp <= 0) return;
    (void)kill(*pidp, SIGTERM);
    (void)waitpid(*pidp, NULL, 0);
    *pidp = -1;
}

static void *camera_primary_fn(void *arg)
{
    (void)arg;
    adas_try_priority(63, "camera_realtime");
    (void)adas_bind_to_cpu(0, "camera_realtime");
    int hb = hb_connect_coid();

    uint32_t seq = 0;
    while (g_running) {
        pid_t cam_pid = -1;
        if (spawn_camera_worker(&cam_pid) != 0) {
            perror("[camera] spawn(camera_example3_viewfinder)");
            msleep(1000);
            continue;
        }

        ts_log("[camera] realtime primary online pid=%ld (camera_example3_viewfinder)", (long)cam_pid);
        pthread_cleanup_push(camera_child_cleanup, &cam_pid);
        while (g_running) {
            int st = 0;
            const pid_t p = waitpid(cam_pid, &st, WNOHANG);
            if (p == cam_pid) {
                ts_log("[camera] camera process exited status=0x%x", st);
                break; /* retry spawn in outer loop */
            }

            pthread_mutex_lock(&g_camera_mu);
            g_camera_pkt.seq = ++seq;
            g_camera_pkt.ts_ms = now_ms();
            g_camera_pkt.source_fallback = 0;
            pthread_mutex_unlock(&g_camera_mu);

            hb_pulse(hb, MOD_CAMERA);
            msleep(50);
        }
        pthread_cleanup_pop(1);
        msleep(100);
    }

    if (hb != -1) (void)ConnectDetach(hb);
    return NULL;
}

static void *sensor_primary_fn(void *arg)
{
    (void)arg;
    adas_try_priority(54, "sensor_service");
    (void)adas_bind_to_cpu(2, "sensor_service");
    int hb = hb_connect_coid();

    if (ultra_init() != 0) {
        ts_log("[sensor] ultra_init failed");
        g_running = 0;
        return NULL;
    }
    uint8_t imu_addr = 0;
    if (imu_lsm6dsox_init(&imu_addr) != 0) {
        ts_log("[sensor] imu init failed");
        g_running = 0;
        return NULL;
    }

    pthread_mutex_lock(&g_chan_mu);
    while (!g_decision_ready && g_running) pthread_cond_wait(&g_chan_cv, &g_chan_mu);
    const int decision_chid = g_decision_chid;
    pthread_mutex_unlock(&g_chan_mu);
    if (!g_running) return NULL;

    const int decision_coid = ConnectAttach(ND_LOCAL_NODE, 0, decision_chid, _NTO_SIDE_CHANNEL, 0);
    if (decision_coid == -1) {
        perror("[sensor] ConnectAttach(decision)");
        g_running = 0;
        return NULL;
    }

    ts_log("[sensor] primary online IMU=0x%02X", imu_addr);
    uint16_t seq = 0;
    uint32_t imu_log_tick = 0;
    int have_prev_roll = 0;
    float prev_roll_deg = 0.0f;
    int64_t prev_roll_ms = 0;
    while (g_running) {
        float mm = -1.f;
        const int urc = g_force_ultra_fail ? -1 : ultra_read_mm(&mm);
        const uint8_t ultra_ok = (urc == 0) ? 1u : 0u;
        int16_t gx=0, gy=0, gz=0, ax=0, ay=0, az=0;
        if (g_force_imu_fail || imu_lsm6dsox_read(imu_addr, &gx, &gy, &gz, &ax, &ay, &az) != 0) {
            msleep(50);
            continue;
        }
        const float fax = (float)ax, fay = (float)ay, faz = (float)az;
        const float roll_deg = atan2f(fay, faz) * 180.0f / (float)M_PI;
        const float accel_mag = sqrtf(fax*fax + fay*fay + faz*faz);
        const float accel_delta = fabsf(accel_mag - IMU_1G_RAW);
        const int64_t t_ms = now_ms();
        float droll_deg = 0.0f;
        float roll_rate_dps = 0.0f;
        if (have_prev_roll) {
            const int64_t dt_ms = t_ms - prev_roll_ms;
            if (dt_ms > 0) {
                droll_deg = roll_deg - prev_roll_deg;
                roll_rate_dps = (droll_deg * 1000.0f) / (float)dt_ms;
            }
        }
        prev_roll_deg = roll_deg;
        prev_roll_ms = t_ms;
        have_prev_roll = 1;

        if ((++imu_log_tick % 20u) == 0u) {
            ts_log("[imu] roll=%.1f droll=%.1f rate=%.1f accel_delta=%.0f gx=%d gy=%d gz=%d ax=%d ay=%d az=%d",
                   roll_deg, droll_deg, roll_rate_dps, accel_delta, gx, gy, gz, ax, ay, az);
        }

        if (imu_airbag_condition(roll_deg, droll_deg, roll_rate_dps, accel_delta, gx, gy, gz)) {
            pthread_mutex_lock(&g_airbag_mu);
            if (!g_airbag_trigger) {
                g_airbag_trigger = 1;
                g_airbag_trigger_ms = now_ms();
                ts_log("[airbag] roll trigger roll=%.1f droll=%.1f rate=%.1f accel_delta=%.0f gx=%d gy=%d gz=%d",
                       roll_deg, droll_deg, roll_rate_dps, accel_delta, gx, gy, gz);
            }
            pthread_mutex_unlock(&g_airbag_mu);
        }

        adas_sensor_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.type = ADAS_MSG_SENSOR_FRAME;
        frame.seq = ++seq;
        frame.distance_mm = ultra_ok ? (int32_t)mm : -1;
        frame.ultrasonic_ok = ultra_ok;
        frame.ax=ax; frame.ay=ay; frame.az=az;
        frame.gx=gx; frame.gy=gy; frame.gz=gz;

        pthread_mutex_lock(&g_sensor_mu);
        g_last_sensor_frame = frame;
        pthread_mutex_unlock(&g_sensor_mu);

        adas_simple_ack_t ack = {0};
        if (MsgSend(decision_coid, &frame, sizeof(frame), &ack, sizeof(ack)) == -1) {
            perror("[sensor] MsgSend(decision)");
            g_running = 0;
            break;
        }
        hb_pulse(hb, MOD_SENSOR);
        msleep(50);
    }
    (void)ConnectDetach(decision_coid);
    if (hb != -1) (void)ConnectDetach(hb);
    return NULL;
}

static void *decision_primary_fn(void *arg)
{
    (void)arg;
    adas_try_priority(52, "decision_engine");
    (void)adas_bind_to_cpu(1, "decision_engine");
    int hb = hb_connect_coid();

    const int chid = ChannelCreate(0);
    if (chid == -1) {
        perror("[decision] ChannelCreate");
        g_running = 0;
        return NULL;
    }

    pthread_mutex_lock(&g_chan_mu);
    g_decision_chid = chid;
    g_decision_ready = 1;
    pthread_cond_broadcast(&g_chan_cv);
    while (!g_safety_ready && g_running) pthread_cond_wait(&g_chan_cv, &g_chan_mu);
    const int safety_chid = g_safety_chid;
    pthread_mutex_unlock(&g_chan_mu);
    if (!g_running) { (void)ChannelDestroy(chid); return NULL; }

    int safety_coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_chid, _NTO_SIDE_CHANNEL, 0);
    if (safety_coid == -1) {
        perror("[decision] ConnectAttach(safety)");
        g_running = 0;
        (void)ChannelDestroy(chid);
        return NULL;
    }

    ts_log("[decision] primary online");
    uint16_t out_seq = 0;
    while (g_running) {
        adas_sensor_frame_t frame;
        struct _msg_info info;
        memset(&frame, 0, sizeof(frame));
        const int rcvid = MsgReceive(chid, &frame, sizeof(frame), &info);
        if (rcvid == -1) { if (errno == EINTR) continue; continue; }
        if (rcvid <= 0) continue;

        adas_simple_ack_t ack = {0};
        (void)MsgReply(rcvid, EOK, &ack, sizeof(ack));

        const float ax=(float)frame.ax, ay=(float)frame.ay, az=(float)frame.az;
        const float roll_deg = atan2f(ay, az) * 180.0f / (float)M_PI;
        const float pitch_deg = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / (float)M_PI;

        adas_vehicle_state_t state;
        memset(&state, 0, sizeof(state));
        state.type = ADAS_MSG_VEHICLE_STATE;
        state.seq = ++out_seq;
        state.roll_deg = roll_deg;
        state.pitch_deg = pitch_deg;
        state.distance_mm = frame.distance_mm;
        state.risk_level = (uint8_t)classify(&frame, roll_deg, pitch_deg);

        adas_simple_ack_t s_ack = {0};
        if (MsgSend(safety_coid, &state, sizeof(state), &s_ack, sizeof(s_ack)) == -1) {
            perror("[decision] MsgSend(safety)");
            g_running = 0;
            break;
        }
        hb_pulse(hb, MOD_DECISION);
    }
    (void)ConnectDetach(safety_coid);
    (void)ChannelDestroy(chid);
    if (hb != -1) (void)ConnectDetach(hb);
    return NULL;
}

static void *safety_primary_fn(void *arg)
{
    (void)arg;
    adas_try_priority(58, "safety_controller");
    (void)adas_bind_to_cpu(1, "safety_controller");
    int hb = hb_connect_coid();

    if (rpi_gpio_setup(LED_GPIO, GPIO_OUT) != GPIO_SUCCESS) {
        perror("[safety] rpi_gpio_setup");
        g_running = 0;
        return NULL;
    }

    const int chid = ChannelCreate(0);
    if (chid == -1) {
        perror("[safety] ChannelCreate");
        g_running = 0;
        return NULL;
    }

    pthread_mutex_lock(&g_chan_mu);
    g_safety_chid = chid;
    g_safety_ready = 1;
    pthread_cond_broadcast(&g_chan_cv);
    pthread_mutex_unlock(&g_chan_mu);

    ts_log("[safety] primary online");
    uint32_t tick = 0;
    while (g_running) {
        adas_vehicle_state_t v;
        struct _msg_info info;
        memset(&v, 0, sizeof(v));
        const int rcvid = MsgReceive(chid, &v, sizeof(v), &info);
        if (rcvid == -1) { if (errno == EINTR) continue; continue; }
        if (rcvid <= 0) continue;

        adas_simple_ack_t ack = {0};
        (void)MsgReply(rcvid, EOK, &ack, sizeof(ack));

        tick++;
        switch ((enum adas_risk_level)v.risk_level) {
            default:
            case ADAS_LEVEL_NORMAL:
                if ((tick % 15u) == 0u) ts_log("[safety] seq=%u R=%.1f P=%.1f mm=%ld NORMAL",
                                                v.seq, v.roll_deg, v.pitch_deg, (long)v.distance_mm);
                (void)rpi_gpio_output(LED_GPIO, GPIO_HIGH); msleep(80);
                (void)rpi_gpio_output(LED_GPIO, GPIO_LOW);  msleep(400);
                break;
            case ADAS_LEVEL_CAUTION:
                ts_log("[safety] CAUTION seq=%u mm=%ld", v.seq, (long)v.distance_mm);
                for (int i=0;i<4;i++){ (void)rpi_gpio_output(LED_GPIO, GPIO_HIGH); msleep(100); (void)rpi_gpio_output(LED_GPIO, GPIO_LOW); msleep(100); }
                break;
            case ADAS_LEVEL_EMERGENCY:
                ts_log("[safety] EMERGENCY seq=%u mm=%ld", v.seq, (long)v.distance_mm);
                for (int i=0;i<6;i++){ (void)rpi_gpio_output(LED_GPIO, GPIO_HIGH); msleep(50); (void)rpi_gpio_output(LED_GPIO, GPIO_LOW); msleep(50); }
                break;
        }
        hb_pulse(hb, MOD_SAFETY);
    }
    (void)ChannelDestroy(chid);
    if (hb != -1) (void)ConnectDetach(hb);
    return NULL;
}

static void *media_primary_fn(void *arg)
{
    (void)arg;
    adas_try_priority(12, "multimedia_sim");
    (void)adas_bind_to_cpu(3, "multimedia_sim");
    int hb = hb_connect_coid();

    ts_log("[media] primary online");
    volatile double m = 1.0;
    uint32_t tick = 0;
    while (g_running) {
        for (unsigned i = 0; i < 1200000U; i++) {
            m = m * 1.00000005 + 0.0000001;
            if (m > 2.0) m = 1.0;
        }
        tick++;
        if ((tick % 40u) == 0u) ts_log("[media] stream decode simulation active");
        hb_pulse(hb, MOD_MEDIA);
        msleep(20);
    }
    if (hb != -1) (void)ConnectDetach(hb);
    return NULL;
}

/* ==== fallback helpers ==== */
static void *camera_fallback_fn(void *arg)
{
    (void)arg;
    adas_try_priority(45, "camera_fallback");
    (void)adas_bind_to_cpu(3, "camera_fallback");
    while (g_running) {
        if (g_camera_fallback_active) {
            pthread_mutex_lock(&g_camera_mu);
            g_camera_pkt.seq++;
            g_camera_pkt.ts_ms = now_ms();
            g_camera_pkt.source_fallback = 1;
            pthread_mutex_unlock(&g_camera_mu);
        }
        msleep(20);
    }
    return NULL;
}

static void *sensor_fallback_fn(void *arg)
{
    (void)arg;
    adas_try_priority(44, "sensor_fallback");
    (void)adas_bind_to_cpu(3, "sensor_fallback");

    while (g_running) {
        if (!g_sensor_fallback_active) { msleep(20); continue; }

        pthread_mutex_lock(&g_chan_mu);
        const int decision_chid = g_decision_chid;
        pthread_mutex_unlock(&g_chan_mu);
        if (decision_chid == -1) { msleep(20); continue; }

        const int coid = ConnectAttach(ND_LOCAL_NODE, 0, decision_chid, _NTO_SIDE_CHANNEL, 0);
        if (coid == -1) { msleep(20); continue; }

        adas_sensor_frame_t frame;
        pthread_mutex_lock(&g_sensor_mu);
        frame = g_last_sensor_frame;
        pthread_mutex_unlock(&g_sensor_mu);

        frame.seq += 1000U;
        frame.ultrasonic_ok = 0;
        frame.distance_mm = (frame.distance_mm > 0) ? frame.distance_mm : 999;

        adas_simple_ack_t ack = {0};
        (void)MsgSend(coid, &frame, sizeof(frame), &ack, sizeof(ack));
        (void)ConnectDetach(coid);
        msleep(60);
    }
    return NULL;
}

static void *decision_fallback_fn(void *arg)
{
    (void)arg;
    adas_try_priority(43, "decision_fallback");
    (void)adas_bind_to_cpu(3, "decision_fallback");

    uint16_t seq = 0;
    while (g_running) {
        if (!g_decision_fallback_active) { msleep(20); continue; }

        pthread_mutex_lock(&g_chan_mu);
        const int safety_chid = g_safety_chid;
        pthread_mutex_unlock(&g_chan_mu);
        if (safety_chid == -1) { msleep(20); continue; }

        const int coid = ConnectAttach(ND_LOCAL_NODE, 0, safety_chid, _NTO_SIDE_CHANNEL, 0);
        if (coid == -1) { msleep(20); continue; }

        adas_vehicle_state_t st;
        memset(&st, 0, sizeof(st));
        st.type = ADAS_MSG_VEHICLE_STATE;
        st.seq = ++seq;
        st.distance_mm = 200; /* conservative fallback */
        st.risk_level = ADAS_LEVEL_EMERGENCY;

        adas_simple_ack_t ack = {0};
        (void)MsgSend(coid, &st, sizeof(st), &ack, sizeof(ack));
        (void)ConnectDetach(coid);
        msleep(80);
    }
    return NULL;
}

static void *safety_guard_fn(void *arg)
{
    (void)arg;
    adas_try_priority(57, "safety_guard");
    (void)adas_bind_to_cpu(1, "safety_guard");

    while (g_running) {
        if (g_safety_guard_active) {
            /* fail-safe blink pattern */
            (void)rpi_gpio_output(LED_GPIO, GPIO_HIGH); msleep(70);
            (void)rpi_gpio_output(LED_GPIO, GPIO_LOW);  msleep(70);
        } else {
            msleep(60);
        }
    }
    return NULL;
}

static void *airbag_guard_fn(void *arg)
{
    (void)arg;
    /* Keep camera as the only top-priority real-time thread. */
    adas_try_priority(40, "airbag_guard");
    (void)adas_bind_to_cpu(1, "airbag_guard");

    if (rpi_gpio_setup(AIRBAG_GPIO, GPIO_OUT) != GPIO_SUCCESS) {
        perror("[airbag] rpi_gpio_setup(GPIO19)");
        return NULL;
    }
    (void)rpi_gpio_output(AIRBAG_GPIO, GPIO_LOW);
    ts_log("[airbag] guard online gpio=%d", AIRBAG_GPIO);

    while (g_running) {
        static int was_active = 0;
        int active = 0;
        int64_t at_ms = 0;
        pthread_mutex_lock(&g_airbag_mu);
        active = g_airbag_trigger;
        at_ms = g_airbag_trigger_ms;
        pthread_mutex_unlock(&g_airbag_mu);

        if (active) {
            (void)rpi_gpio_output(AIRBAG_GPIO, GPIO_HIGH);
            if (!was_active) {
                ts_log("[airbag] DEPLOYED (LED GPIO19 ON at t=%lldms)", (long long)at_ms);
            }
            was_active = 1;
            msleep(5);
        } else {
            (void)rpi_gpio_output(AIRBAG_GPIO, GPIO_LOW);
            was_active = 0;
            msleep(10);
        }
    }
    (void)rpi_gpio_output(AIRBAG_GPIO, GPIO_LOW);
    return NULL;
}

/* ==== monitor + recovery ==== */
static void *monitor_pulse_rx_fn(void *arg)
{
    (void)arg;
    const int chid = ChannelCreate(0);
    if (chid == -1) {
        perror("[monitor] ChannelCreate");
        g_running = 0;
        return NULL;
    }

    pthread_mutex_lock(&g_monitor_mu);
    g_monitor_chid = chid;
    pthread_mutex_unlock(&g_monitor_mu);

    ts_log("[monitor] pulse receiver online");

    while (g_running) {
        struct _pulse p;
        const int rcvid = MsgReceivePulse(chid, &p, sizeof(p), NULL);
        if (rcvid == -1) {
            if (errno == EINTR) continue;
            continue;
        }
        if (rcvid != 0) continue;

        const int code = p.code;
        if (code >= PULSE_BASE && code < (PULSE_BASE + MOD_COUNT)) {
            const int mod = code - PULSE_BASE;
            pthread_mutex_lock(&g_hb_mu);
            g_hb_ms[mod] = now_ms();
            if (g_failed[mod]) {
                g_failed[mod] = 0;
                const int64_t rec = g_hb_ms[mod] - g_failed_at_ms[mod];
                ts_log("[monitor] %s RECOVERED at %lld ms (recovery_ms=%lld)",
                       MOD_NAME[mod], (long long)g_hb_ms[mod], (long long)rec);
                if (mod == MOD_CAMERA) g_camera_fallback_active = 0;
                if (mod == MOD_SENSOR) g_sensor_fallback_active = 0;
                if (mod == MOD_DECISION) g_decision_fallback_active = 0;
                if (mod == MOD_SAFETY) g_safety_guard_active = 0;
            }
            pthread_mutex_unlock(&g_hb_mu);
        }
    }

    (void)ChannelDestroy(chid);
    return NULL;
}

static int restart_primary(enum mod_id mod)
{
    pthread_mutex_lock(&g_restart_mu);
    int rc = 0;

    switch (mod) {
    case MOD_CAMERA:
        (void)pthread_cancel(g_th_camera); (void)pthread_join(g_th_camera, NULL);
        rc = pthread_create(&g_th_camera, NULL, camera_primary_fn, NULL);
        break;
    case MOD_SENSOR:
        (void)pthread_cancel(g_th_sensor); (void)pthread_join(g_th_sensor, NULL);
        rc = pthread_create(&g_th_sensor, NULL, sensor_primary_fn, NULL);
        break;
    case MOD_DECISION:
        (void)pthread_cancel(g_th_decision); (void)pthread_join(g_th_decision, NULL);
        rc = pthread_create(&g_th_decision, NULL, decision_primary_fn, NULL);
        break;
    case MOD_SAFETY:
        (void)pthread_cancel(g_th_safety); (void)pthread_join(g_th_safety, NULL);
        rc = pthread_create(&g_th_safety, NULL, safety_primary_fn, NULL);
        break;
    case MOD_MEDIA:
        (void)pthread_cancel(g_th_media); (void)pthread_join(g_th_media, NULL);
        rc = pthread_create(&g_th_media, NULL, media_primary_fn, NULL);
        break;
    default:
        rc = -1;
        break;
    }

    pthread_mutex_unlock(&g_restart_mu);
    return rc;
}

static void *fault_injector_fn(void *arg)
{
    (void)arg;
    adas_try_priority(15, "fault_injector");
    (void)adas_bind_to_cpu(3, "fault_injector");

    if (g_inj.kill_enabled) {
        ts_log("[injector] kill armed target=%s after=%u sec",
               MOD_NAME[g_inj.kill_mod], g_inj.kill_after_s);
        for (unsigned i = 0; g_running && i < g_inj.kill_after_s; i++) msleep(1000);
        if (g_running) {
            pthread_mutex_lock(&g_restart_mu);
            switch (g_inj.kill_mod) {
            case MOD_CAMERA: ts_log("[injector] cancel camera primary"); (void)pthread_cancel(g_th_camera); break;
            case MOD_SENSOR: ts_log("[injector] cancel sensor primary"); (void)pthread_cancel(g_th_sensor); break;
            case MOD_DECISION: ts_log("[injector] cancel decision primary"); (void)pthread_cancel(g_th_decision); break;
            case MOD_SAFETY: ts_log("[injector] cancel safety primary"); (void)pthread_cancel(g_th_safety); break;
            case MOD_MEDIA: ts_log("[injector] cancel media primary"); (void)pthread_cancel(g_th_media); break;
            default: break;
            }
            pthread_mutex_unlock(&g_restart_mu);
        }
    }

    if (g_inj.hog_enabled && g_running) {
        ts_log("[injector] cpu_hog armed core=%d after=%u sec dur=%u sec",
               g_inj.hog_core, g_inj.hog_after_s, g_inj.hog_dur_s);
        for (unsigned i = 0; g_running && i < g_inj.hog_after_s; i++) msleep(1000);
        if (g_running) {
            const int64_t end_ms = now_ms() + (int64_t)g_inj.hog_dur_s * 1000LL;
            volatile double x = 1.0;
            (void)adas_bind_to_cpu(g_inj.hog_core, "fault_injector_hog");
            ts_log("[injector] cpu_hog START");
            while (g_running && now_ms() < end_ms) {
                for (unsigned i = 0; i < 5000000U; i++) {
                    x = x * 1.0000001 + 0.0000001;
                    if (x > 2.0) x = 1.0;
                }
            }
            ts_log("[injector] cpu_hog END");
        }
    }

    if (g_inj.storm_enabled && g_running) {
        ts_log("[injector] storm armed after=%u sec rate=%u hz dur=%u sec",
               g_inj.storm_after_s, g_inj.storm_rate_hz, g_inj.storm_dur_s);
        for (unsigned i = 0; g_running && i < g_inj.storm_after_s; i++) msleep(1000);
        if (g_running) {
            int coid = -1;
            pthread_mutex_lock(&g_chan_mu);
            if (g_decision_chid != -1) coid = ConnectAttach(ND_LOCAL_NODE, 0, g_decision_chid, _NTO_SIDE_CHANNEL, 0);
            pthread_mutex_unlock(&g_chan_mu);

            if (coid != -1) {
                const int64_t end_ms = now_ms() + (int64_t)g_inj.storm_dur_s * 1000LL;
                const unsigned sleep_us = (g_inj.storm_rate_hz >= 1000000U) ? 0U : (1000000U / g_inj.storm_rate_hz);
                uint16_t seq = 50000U;
                ts_log("[injector] storm START");
                while (g_running && now_ms() < end_ms) {
                    adas_sensor_frame_t frame;
                    adas_simple_ack_t ack = {0};
                    memset(&frame, 0, sizeof(frame));
                    frame.type = ADAS_MSG_SENSOR_FRAME;
                    frame.seq = ++seq;
                    frame.distance_mm = 250;
                    frame.ultrasonic_ok = 1;
                    frame.az = 1000;
                    (void)MsgSend(coid, &frame, sizeof(frame), &ack, sizeof(ack));
                    if (sleep_us > 0) usleep(sleep_us);
                }
                ts_log("[injector] storm END");
                (void)ConnectDetach(coid);
            } else {
                ts_log("[injector] storm skipped: decision channel unavailable");
            }
        }
    }

    if (g_inj.sensor_fail_enabled && g_running) {
        ts_log("[injector] sensor_fail armed mode=%d after=%u sec dur=%u sec",
               g_inj.sensor_fail_mode, g_inj.sensor_fail_after_s, g_inj.sensor_fail_dur_s);
        for (unsigned i = 0; g_running && i < g_inj.sensor_fail_after_s; i++) msleep(1000);
        if (g_running) {
            ts_log("[injector] sensor_fail START");
            if (g_inj.sensor_fail_mode & 1) g_force_imu_fail = 1;
            if (g_inj.sensor_fail_mode & 2) g_force_ultra_fail = 1;
            for (unsigned i = 0; g_running && i < g_inj.sensor_fail_dur_s; i++) msleep(1000);
            g_force_imu_fail = 0;
            g_force_ultra_fail = 0;
            ts_log("[injector] sensor_fail END");
        }
    }

    return NULL;
}

static void *watchdog_fn(void *arg)
{
    (void)arg;
    adas_try_priority(20, "health_watchdog");
    (void)adas_bind_to_cpu(3, "health_watchdog");

    ts_log("[watchdog] online");
    const int64_t timeout_ms[MOD_COUNT] = {
        1200, /* camera */
        1800, /* sensor */
        1800, /* decision */
        2200, /* safety */
        3000  /* multimedia */
    };

    while (g_running) {
        msleep(250);
        const int64_t t = now_ms();

        pthread_mutex_lock(&g_hb_mu);
        for (int i = 0; i < MOD_COUNT; i++) {
            const int64_t age = t - g_hb_ms[i];
            if (!g_failed[i] && age > timeout_ms[i]) {
                g_failed[i] = 1;
                g_failed_at_ms[i] = t;
                ts_log("[watchdog] %s FAILED at %lld ms (age=%lld ms)",
                       MOD_NAME[i], (long long)t, (long long)age);

                if (i == MOD_CAMERA) g_camera_fallback_active = 1;
                if (i == MOD_SENSOR) g_sensor_fallback_active = 1;
                if (i == MOD_DECISION) g_decision_fallback_active = 1;
                if (i == MOD_SAFETY) g_safety_guard_active = 1;

                pthread_mutex_unlock(&g_hb_mu);
                if (restart_primary((enum mod_id)i) != 0) {
                    ts_log("[watchdog] restart FAILED for %s", MOD_NAME[i]);
                } else {
                    ts_log("[watchdog] restart issued for %s", MOD_NAME[i]);
                }
                pthread_mutex_lock(&g_hb_mu);
            }
        }
        pthread_mutex_unlock(&g_hb_mu);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    (void)signal(SIGINT, on_sig);
    (void)signal(SIGTERM, on_sig);

    g_start_ms = now_ms();

    if (argc > 1 && parse_cli(argc, argv) != 0) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    ts_log("[Final] one-binary ADAS runtime with CPU clustering + failover");
    ts_log("[Final] core map: C0=camera, C1=safety+decision, C2=sensor, C3=media+monitor");
    ts_log("[Final] airbag thresholds roll=%.1f step=%.1f rate=%.1f accel_delta=%.0f",
           g_airbag_roll_deg_thr, g_airbag_roll_step_deg_thr, g_airbag_roll_rate_dps_thr, g_airbag_accel_delta_thr);

    const int64_t t0 = now_ms();
    for (int i = 0; i < MOD_COUNT; i++) {
        g_hb_ms[i] = t0;
        g_failed[i] = 0;
        g_failed_at_ms[i] = 0;
    }

    if (pthread_create(&g_th_monitor_pulse, NULL, monitor_pulse_rx_fn, NULL) != 0) {
        perror("[Final] monitor_pulse_rx");
        return EXIT_FAILURE;
    }

    /* wait until monitor channel is ready */
    while (g_running) {
        pthread_mutex_lock(&g_monitor_mu);
        const int ready = (g_monitor_chid != -1);
        pthread_mutex_unlock(&g_monitor_mu);
        if (ready) break;
        msleep(20);
    }

    if (!g_running) return EXIT_FAILURE;

    if (pthread_create(&g_th_camera, NULL, camera_primary_fn, NULL) != 0 ||
        pthread_create(&g_th_safety, NULL, safety_primary_fn, NULL) != 0 ||
        pthread_create(&g_th_decision, NULL, decision_primary_fn, NULL) != 0 ||
        pthread_create(&g_th_sensor, NULL, sensor_primary_fn, NULL) != 0 ||
        pthread_create(&g_th_media, NULL, media_primary_fn, NULL) != 0 ||
        pthread_create(&g_th_airbag_guard, NULL, airbag_guard_fn, NULL) != 0 ||
        pthread_create(&g_th_camera_fb, NULL, camera_fallback_fn, NULL) != 0 ||
        pthread_create(&g_th_sensor_fb, NULL, sensor_fallback_fn, NULL) != 0 ||
        pthread_create(&g_th_decision_fb, NULL, decision_fallback_fn, NULL) != 0 ||
        pthread_create(&g_th_safety_guard, NULL, safety_guard_fn, NULL) != 0 ||
        pthread_create(&g_th_watchdog, NULL, watchdog_fn, NULL) != 0) {
        perror("[Final] pthread_create");
        g_running = 0;
    }

    if (g_running && (g_inj.kill_enabled || g_inj.hog_enabled || g_inj.storm_enabled || g_inj.sensor_fail_enabled)) {
        if (pthread_create(&g_th_fault_injector, NULL, fault_injector_fn, NULL) != 0) {
            perror("[Final] fault_injector");
            g_running = 0;
        }
    }

    while (g_running) msleep(200);

    pthread_cond_broadcast(&g_chan_cv);

    (void)pthread_cancel(g_th_sensor); (void)pthread_join(g_th_sensor, NULL);
    (void)pthread_cancel(g_th_decision); (void)pthread_join(g_th_decision, NULL);
    (void)pthread_cancel(g_th_safety); (void)pthread_join(g_th_safety, NULL);
    (void)pthread_cancel(g_th_camera); (void)pthread_join(g_th_camera, NULL);
    (void)pthread_cancel(g_th_media); (void)pthread_join(g_th_media, NULL);
    (void)pthread_cancel(g_th_airbag_guard); (void)pthread_join(g_th_airbag_guard, NULL);
    (void)pthread_cancel(g_th_camera_fb); (void)pthread_join(g_th_camera_fb, NULL);
    (void)pthread_cancel(g_th_sensor_fb); (void)pthread_join(g_th_sensor_fb, NULL);
    (void)pthread_cancel(g_th_decision_fb); (void)pthread_join(g_th_decision_fb, NULL);
    (void)pthread_cancel(g_th_safety_guard); (void)pthread_join(g_th_safety_guard, NULL);
    (void)pthread_cancel(g_th_watchdog); (void)pthread_join(g_th_watchdog, NULL);
    (void)pthread_cancel(g_th_monitor_pulse); (void)pthread_join(g_th_monitor_pulse, NULL);
    if (g_inj.kill_enabled || g_inj.hog_enabled || g_inj.storm_enabled || g_inj.sensor_fail_enabled) {
        (void)pthread_cancel(g_th_fault_injector);
        (void)pthread_join(g_th_fault_injector, NULL);
    }

    ts_log("[Final] stopped");
    return EXIT_SUCCESS;
}

