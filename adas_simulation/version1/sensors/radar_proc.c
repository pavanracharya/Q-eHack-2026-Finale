#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <time.h>
#include "../common/adas_ipc.h"
#include "../common/sensor_protocol.h"

// Radar Sensor Client - Object Detection
void* radar_proc_start(void* arg) {
    int coid;
    int chid = 1; // Assuming CHID 1 for the Fusion Engine Server in this demo
    
    // Connect to Fusion Engine (Assumes central server availability)
    coid = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) {
        perror("[RADAR] ConnectAttach failed");
        return NULL;
    }

    printf("[RADAR] Connected. Initializing detection loop...\n");

    adas_metadata_msg_t msg;
    adas_reply_t reply;
    
    msg.type = MSG_SENSOR_DATA;
    msg.sensor_id = 2; // SENSOR_RADAR

    while (1) {
        // Pin to Core 1
        ThreadCtl(_NTO_TCTL_RUNMASK, (void*)0x02);

        // Simulate Radar Scan
        msg.timestamp_ns = (uint64_t)time(NULL) * 1000000000ULL;
        msg.reliability = 0.95; // Radar is trusted for AEB

        // Send Detection Metadata
        if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            perror("[RADAR] MsgSend failed");
            break;
        }

        // Radar runs fast (100ms)
        usleep(100000); 
    }

    ConnectDetach(coid);
    return NULL;
}
