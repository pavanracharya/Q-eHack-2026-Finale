#ifndef ULTRASONIC_HCSR04_H
#define ULTRASONIC_HCSR04_H

/* TRIG GPIO23 (Pin 16), ECHO GPIO24 (Pin 18) */
int  ultra_init(void);
/* Returns 0 on success (valid echo), 1 on timeout, -1 on GPIO error. */
int  ultra_read_mm(float *mm_out);

#endif
