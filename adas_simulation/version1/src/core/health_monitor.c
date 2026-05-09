#include <stdio.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include <sys/netmgr.h>
#include <time.h>
#include "../common/adas_ipc.h"

// Health Monitor (Watchdog) - Safety-Critical Monitoring
void* health_monitor_start(void* arg) {
    pthread_setname_np(pthread_self(), "Health_Mon_V3");
    int chid, rcvid;
    struct _pulse pulse;
    
    // Channel creation for monitoring pulses
    chid = ChannelCreate(0);
    
    printf("[HEALTH] Watchdog Active. Monitoring pulses on CHID: %d\n", chid);

    // Track last heartbeat for each sensor
    time_t last_heartbeat[5] = {0};

    while (1) {
        // Core 0 (System Core)
        ThreadCtl(_NTO_TCTL_RUNMASK, (void*)0x01);

        // Pulse reception (Non-blocking IPC)
        rcvid = MsgReceive(chid, &pulse, sizeof(pulse), NULL);
        
        if (rcvid == 0) { // It's a pulse
            if (pulse.code == PULSE_CODE_HEARTBEAT) {
                int sensor_id = pulse.value.sival_int;
                last_heartbeat[sensor_id] = time(NULL);
                printf("[HEALTH] Heartbeat received from Sensor %d\n", sensor_id);
            }
        }

        // Fault detection scan
        time_t now = time(NULL);
        for (int i = 1; i <= 3; i++) {
            if (last_heartbeat[i] != 0 && (now - last_heartbeat[i]) > 2) {
                printf("[HEALTH] CRITICAL: Sensor %d FAILED (No heartbeat for 2s!)\n", i);
                // In real world, we'd trigger a Fail-Safe (AEB Disable)
            }
        }

        sleep(1); 
    }

    return NULL;
}
