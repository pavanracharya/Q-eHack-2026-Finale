#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include <time.h>
#include "../common/adas_ipc.h"
#include "../common/sensor_protocol.h"

// LiDAR Sensor Client - Shared Memory Producer
void* lidar_proc_start(void* arg) {
    pthread_setname_np(pthread_self(), "LiDAR_V3");
    int coid, shm_fd;
    lidar_cloud_t* shm_ptr;

    // Map Shared Memory
    shm_fd = shm_open(SHM_LIDAR_PATH, O_RDWR, 0666);
    if (shm_fd == -1) return NULL;

    shm_ptr = mmap(NULL, sizeof(lidar_cloud_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    close(shm_fd);

    // Connect to Fusion Engine
    coid = ConnectAttach(0, 0, 1, _NTO_SIDE_CHANNEL, 0);

    printf("[LIDAR] Shm Mapped. Spinning 360 Scan...\n");

    adas_metadata_msg_t msg;
    adas_reply_t reply;
    msg.type = MSG_SENSOR_DATA;
    msg.sensor_id = 1; // SENSOR_LIDAR

    while (1) {
        // Pin to Core 3
        ThreadCtl(_NTO_TCTL_RUNMASK, (void*)0x08);

        // Update shared memory data
        shm_ptr->points[0].x = 10.5f;
        shm_ptr->points[0].z = 0.2f;

        // Send Synchronization Message
        msg.timestamp_ns = (uint64_t)time(NULL) * 1000000000ULL;
        if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) break;

        // LiDAR scans (500ms)
        usleep(500000);
    }

    munmap(shm_ptr, sizeof(lidar_cloud_t));
    return NULL;
}
