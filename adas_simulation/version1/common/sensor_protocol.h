#ifndef SENSOR_PROTOCOL_H
#define SENSOR_PROTOCOL_H

#include <stdint.h>

// Radar Data Structure
typedef struct {
    float distance;    // Meters
    float velocity;    // m/s
    float angle;       // Degrees
    uint8_t confidence; // 0-100
} radar_data_t;

// Vision Data Structure (Shared Memory)
#define VISION_WIDTH  640
#define VISION_HEIGHT 480

typedef struct {
    uint8_t frame_buffer[VISION_WIDTH * VISION_HEIGHT]; 
    uint16_t detected_objects_count;
    struct {
        int x, y, w, h;
        char label[16];
    } objects[10];
} vision_frame_t;

// LiDAR Data Structure (Shared Memory)
#define LIDAR_POINTS 1024

typedef struct {
    struct {
        float x, y, z;
        float intensity;
    } points[LIDAR_POINTS];
} lidar_cloud_t;

// Fused Object State
typedef struct {
    float x, y;
    float vx, vy;
    float reliability;
    uint32_t last_update_ns;
    uint8_t sensor_mask; // Bitmask: 1=Radar, 2=Vision, 4=LiDAR
} fused_object_t;

#endif // SENSOR_PROTOCOL_H
