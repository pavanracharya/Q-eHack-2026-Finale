#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <time.h>
#include "../common/adas_ipc.h"
#include "../common/sensor_protocol.h"

// Vision Sensor Client - Shared Memory Producer
void* vision_proc_start(void* arg) {
    int coid, shm_fd;
    vision_frame_t* shm_ptr;

    // Map Shared Memory for high-bandwidth data
    shm_fd = shm_open(SHM_VISION_PATH, O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("[VISION] shm_open failed");
        return NULL;
    }

    shm_ptr = mmap(NULL, sizeof(vision_frame_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    close(shm_fd);

    // Connect to Fusion Engine
    coid = ConnectAttach(0, 0, 1, _NTO_SIDE_CHANNEL, 0); 

    printf("[VISION] Shm Mapped and Connected. Starting Heavy Processing...\n");

    adas_metadata_msg_t msg;
    adas_reply_t reply;
    msg.type = MSG_SENSOR_DATA;
    msg.sensor_id = 3; // SENSOR_CAMERA

    while (1) {
        // Pin to Core 2
        ThreadCtl(_NTO_TCTL_RUNMASK, (void*)0x04);

        // Simulate image frame capture
        shm_ptr->detected_objects_count = 1;
        shm_ptr->objects[0].x = 100;
        shm_ptr->objects[0].y = 50;
        strcpy(shm_ptr->objects[0].label, "Pedestrian");

        // Metadata Preparation
        msg.timestamp_ns = (uint64_t)time(NULL) * 1000000000ULL;
        msg.data_size = sizeof(vision_frame_t);

        // Send Notification (Zero-copy notification via IPC)
        if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            break;
        }

        // Camera runs slower (200ms)
        usleep(200000);
    }

    munmap(shm_ptr, sizeof(vision_frame_t));
    ConnectDetach(coid);
    return NULL;
}
