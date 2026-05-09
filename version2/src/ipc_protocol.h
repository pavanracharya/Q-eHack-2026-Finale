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

#ifndef ADAS_IPC_PROTOCOL_H
#define ADAS_IPC_PROTOCOL_H

#include <stdint.h>

/*
 * QNX name_attach / name_open channel names.
 * Each service registers its receive channel under one of these names.
 */
#define ADAS_CHANNEL_DECISION    "adas_decision"
#define ADAS_CHANNEL_SAFETY      "adas_safety"
#define ADAS_CHANNEL_SUPERVISOR  "adas_supervisor"

/* Message type discriminators */
#define ADAS_MSG_SENSOR_FRAME    1u
#define ADAS_MSG_VEHICLE_STATE   2u

/*
 * Pulse codes sent to supervisor channel.
 * Range: _PULSE_CODE_MINAVAIL .. _PULSE_CODE_MAXAVAIL
 */
#define PULSE_HB_CAMERA    (_PULSE_CODE_MINAVAIL + 0)
#define PULSE_HB_SENSOR    (_PULSE_CODE_MINAVAIL + 1)
#define PULSE_HB_DECISION  (_PULSE_CODE_MINAVAIL + 2)
#define PULSE_HB_SAFETY    (_PULSE_CODE_MINAVAIL + 3)

/* Risk levels produced by decision engine */
typedef enum {
    ADAS_LEVEL_NORMAL    = 0,
    ADAS_LEVEL_CAUTION   = 1,
    ADAS_LEVEL_EMERGENCY = 2,
} adas_risk_level_t;

/* Sensor frame: sensor_service -> decision_service */
typedef struct {
    uint16_t type;          /* ADAS_MSG_SENSOR_FRAME */
    uint16_t seq;
    int32_t  distance_mm;   /* -1 if ultrasonic not valid */
    uint8_t  ultrasonic_ok;
    uint8_t  reserved[3];
    int16_t  ax, ay, az;    /* accelerometer raw */
    int16_t  gx, gy, gz;    /* gyroscope raw */
} adas_sensor_frame_t;

/* Vehicle state: decision_service -> safety_service */
typedef struct {
    uint16_t type;          /* ADAS_MSG_VEHICLE_STATE */
    uint16_t seq;
    float    roll_deg;
    float    pitch_deg;
    int32_t  distance_mm;
    uint8_t  risk_level;    /* adas_risk_level_t */
    uint8_t  reserved[3];
} adas_vehicle_state_t;

/* Generic acknowledgement reply */
typedef struct {
    int32_t status;         /* EOK on success */
} adas_simple_ack_t;

#endif /* ADAS_IPC_PROTOCOL_H */
