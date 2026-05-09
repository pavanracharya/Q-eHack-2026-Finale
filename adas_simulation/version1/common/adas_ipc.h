#ifndef ADAS_IPC_H
#define ADAS_IPC_H

#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/iomsg.h>

// ---------------------------------------------------------
// IPC CHANNEL & SHM NAMES
// ---------------------------------------------------------
#define ADAS_FUSION_CHID_PATH "/dev/name/local/fusion_engine"
#define SHM_VISION_PATH       "/adas_vision_shm"
#define SHM_LIDAR_PATH        "/adas_lidar_shm"

// ---------------------------------------------------------
// MESSAGE TYPES
// ---------------------------------------------------------
typedef enum {
    MSG_SENSOR_DATA = 0x100,
    MSG_HEALTH_HEARTBEAT,
    MSG_ADAPTIVE_CONTROL  // From Fusion -> Sensor (e.g., "Slow down")
} AdasMsgType_e;

// ---------------------------------------------------------
// PULSE CODES (Non-blocking)
// ---------------------------------------------------------
#define PULSE_CODE_HEARTBEAT  10
#define PULSE_CODE_THROTTLE   11
#define PULSE_CODE_SHUTDOWN   12

// ---------------------------------------------------------
// MESSAGE STRUCTURES
// ---------------------------------------------------------

// Metadata sent via MsgSend for Shm-based sensors
typedef struct {
    uint16_t type;
    uint16_t sensor_id;
    uint64_t timestamp_ns;
    uint32_t shm_offset;  // Where in Shm the data is
    uint32_t data_size;
    float    reliability; // Initially set by sensor, adjusted by fusion
} adas_metadata_msg_t;

typedef struct {
    int status;           // 0 = OK, 1 = Throttle active
} adas_reply_t;

#endif // ADAS_IPC_H
