#include <stdint.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include <unistd.h>

#include "rpi_gpio.h"
#include "ultrasonic_hcsr04.h"

#define TRIG_GPIO GPIO23
#define ECHO_GPIO GPIO24

#define US_TO_MM(us) ((float)(us) / 58.0f * 10.0f)

static uint64_t cycles_per_sec(void)
{
    const struct qtime_entry *qtime = SYSPAGE_ENTRY(qtime);
    return qtime ? qtime->cycles_per_sec : 0;
}

static uint64_t cycles_now(void)
{
    return ClockCycles();
}

static int wait_for_level(int gpio, unsigned target_level, uint32_t timeout_us)
{
    const uint64_t cps = cycles_per_sec();
    if (cps == 0) {
        return -1;
    }

    const uint64_t start = cycles_now();
    const uint64_t timeout_cycles = (uint64_t)timeout_us * cps / 1000000ULL;

    for (;;) {
        unsigned level = 0;
        if (rpi_gpio_input(gpio, &level) != GPIO_SUCCESS) {
            return -1;
        }
        if (level == target_level) {
            return 0;
        }
        if ((cycles_now() - start) > timeout_cycles) {
            return 1;
        }
    }
}

static int pulse_trig_10us(void)
{
    if (rpi_gpio_output(TRIG_GPIO, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }
    nanospin_ns(2000);

    if (rpi_gpio_output(TRIG_GPIO, GPIO_HIGH) != GPIO_SUCCESS) {
        return -1;
    }
    nanospin_ns(10000);

    if (rpi_gpio_output(TRIG_GPIO, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }
    return 0;
}

int ultra_init(void)
{
    if (rpi_gpio_setup(TRIG_GPIO, GPIO_OUT) != GPIO_SUCCESS) {
        return -1;
    }
    if (rpi_gpio_setup_pull(ECHO_GPIO, GPIO_IN, GPIO_PUD_OFF) != GPIO_SUCCESS) {
        return -1;
    }
    if (rpi_gpio_output(TRIG_GPIO, GPIO_LOW) != GPIO_SUCCESS) {
        return -1;
    }
    return 0;
}

int ultra_read_mm(float *mm_out)
{
    const uint64_t cps = cycles_per_sec();
    if (cps == 0 || mm_out == NULL) {
        return -1;
    }

    if (pulse_trig_10us() != 0) {
        return -1;
    }

    int rc = wait_for_level(ECHO_GPIO, GPIO_HIGH, 30000);
    if (rc != 0) {
        return (rc < 0) ? -1 : 1;
    }

    const uint64_t t_start = cycles_now();

    rc = wait_for_level(ECHO_GPIO, GPIO_LOW, 30000);
    if (rc != 0) {
        return (rc < 0) ? -1 : 1;
    }

    const uint64_t t_end = cycles_now();
    const double dt_us = (double)(t_end - t_start) * 1e6 / (double)cps;
    *mm_out = US_TO_MM((float)dt_us);
    /*
     * Do not use isfinite()/math.h here: debug builds use -fno-builtin so
     * isfinite pulls __isfinitef from libm. NaN rejects without libm via (x != x).
     */
    if ((*mm_out != *mm_out) || *mm_out < 0.0f || *mm_out > 5000.0f) {
        return 1;
    }
    return 0;
}
