#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <time.h>
#include "../common/adas_ipc.h"
#include "../common/sensor_protocol.h"

// Fusion Engine Server - Central Processing
void* fusion_engine_start(void* arg) {
    int chid, rcvid;
    adas_metadata_msg_t msg;
    adas_reply_t reply;

    // Channel creation for IPC synchronization
    chid = ChannelCreate(0);
    if (chid == -1) {
        perror("[FUSION] ChannelCreate failed");
        return NULL;
    }

    printf("[FUSION] Server Ready. CHID: %d\n", chid);

    // Reliability State Tracking
    float sensor_weights[5] = {0.0, 1.0, 0.8, 0.7, 0.5}; // ID-indexed

    while (1) {
        // Blocking receive (WAITING state)
        rcvid = MsgReceive(chid, &msg, sizeof(msg), NULL);

        if (rcvid == -1) break;

        // Pulse Handling
        if (rcvid == 0) {
            struct _pulse* pulse = (struct _pulse*)&msg;
            if (pulse->code == PULSE_CODE_HEARTBEAT) {
                // Heartbeat handled by Monitor, but server can log it
            }
            continue;
        }

        // Sensor Fusion Logic
        if (msg.type == MSG_SENSOR_DATA) {
            uint64_t now_ns;
            ThreadCtl(_NTO_TCTL_RUNMASK, (void*)0x02); // Ensure on Core 1

            // Simple "Time Misalignment" check
            // In real QNX, we'd use ClockTime(CLOCK_REALTIME, NULL)
            now_ns = (uint64_t)time(NULL) * 1000000000ULL;
            int64_t delay_ms = (now_ns - msg.timestamp_ns) / 1000000;

            if (delay_ms > 100) {
                printf("[FUSION] WARNING: Sensor %d data stale (%lld ms delay). Reducing weight.\n", 
                       msg.sensor_id, delay_ms);
                sensor_weights[msg.sensor_id] *= 0.9f;
            }

            // Perform "Fusion" based on weights
            printf("[FUSION] RECV Sensor %d | Reliability: %.2f | Action: Fusion Applied\n", 
                   msg.sensor_id, sensor_weights[msg.sensor_id]);

            // Synchronous Reply
            reply.status = 0;
            MsgReply(rcvid, 0, &reply, sizeof(reply));
        }
    }

    ChannelDestroy(chid);
    return NULL;
}
