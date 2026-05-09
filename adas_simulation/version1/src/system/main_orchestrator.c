#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <pthread.h>
#include "../common/adas_ipc.h"
#include "../common/sensor_protocol.h"

// Forward declarations of modular entry points
void* fusion_engine_start(void* arg);
void* health_monitor_start(void* arg);
void* radar_proc_start(void* arg);
void* vision_proc_start(void* arg);
void* lidar_proc_start(void* arg);

// Shared memory initialization for Vision and LiDAR streams
void setup_shared_memory() {
    int v_fd = shm_open(SHM_VISION_PATH, O_RDWR | O_CREAT | O_TRUNC, 0666);
    ftruncate(v_fd, sizeof(vision_frame_t));
    close(v_fd);

    int l_fd = shm_open(SHM_LIDAR_PATH, O_RDWR | O_CREAT | O_TRUNC, 0666);
    ftruncate(l_fd, sizeof(lidar_cloud_t));
    close(l_fd);

    printf("[SYSTEM] Shared Memory Initialized (Vision/LiDAR)\n");
}

// Main System Entry Point
int main() {
    pthread_setname_np(pthread_self(), "Main_Orch_V3");
    printf("--- QNX ADAS SYSTEM V3 --- \n");
    printf("[SYSTEM] Initializing Microkernel Components...\n");

    setup_shared_memory();

    pthread_t fusion_th, health_th, radar_th, vision_th, lidar_th;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // Health Monitor (Core 0, High Priority)
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 25;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&health_th, &attr, health_monitor_start, NULL);
    
    // Pin to Core 0 (0x01)
    // In a real process, we'd use ThreadCtl inside the thread, but we'll show it here too.
    
    // Fusion Engine (Core 1, High Priority)
    param.sched_priority = 22;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&fusion_th, &attr, fusion_engine_start, NULL);

    // Wait for server to stabilize
    sleep(1);

    // Sensor Modules
    param.sched_priority = 21;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&radar_th, &attr, radar_proc_start, NULL);

    pthread_attr_setschedpolicy(&attr, SCHED_RR);
    param.sched_priority = 18;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&vision_th, &attr, vision_proc_start, NULL);

    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = 20;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&lidar_th, &attr, lidar_proc_start, NULL);

    printf("[SYSTEM] All modules launched. System Monitoring Active.\n");

    // Wait indefinitely
    pthread_join(health_th, NULL);

    return 0;
}
