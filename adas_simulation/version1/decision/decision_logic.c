#include <stdio.h>
#include <stdlib.h>
#include "../common/sensor_protocol.h"

// ADAS Decision Engine Logic
void process_adas_decisions(fused_object_t* obj) {
    printf("\n[DECISION] Analyzing Fused Frame...\n");

    // AEB (Autonomous Emergency Braking)
    if (obj->x < 5.0f && obj->reliability > 0.90f) {
        printf("[FEATURE] !!! EMERGENCY BRAKING ACTIVATED !!! (Target at %.2fm)\n", obj->x);
    }

    // FCW (Forward Collision Warning)
    else if (obj->x < 15.0f) {
        printf("[FEATURE] WARNING: Obstacle Ahead (%.2fm)\n", obj->x);
    }

    // ACC (Adaptive Cruise Control)
    if (obj->vx < 0) {
        printf("[FEATURE] ACC: Decelerating to match lead vehicle speed\n");
    }

    // BSD (Blind Spot Detection)
    if (obj->y > 3.0f || obj->y < -3.0f) {
        printf("[FEATURE] BSD: Object in lateral lane detected\n");
    }

    // LDW (Lane Departure Warning)
    if (obj->sensor_mask & 0x02) { // Vision Bit
        printf("[FEATURE] LDW: Lane markers detected. Steering center.\n");
    }
    
    // Phantom Braking Protection
    if (obj->reliability < 0.40f) {
        printf("[SAFETY] PHANTOM BRAKING PROTECTED: High noise detected, suppressing AEB.\n");
    }
}
