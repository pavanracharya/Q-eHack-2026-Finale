#ifndef RT_PRIO_H
#define RT_PRIO_H

#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>

static inline void adas_try_priority(int prio, const char *who)
{
    struct sched_param sp;
    memset(&sp, 0, sizeof(sp));
    sp.sched_priority = prio;

    if (sched_setscheduler(0, SCHED_RR, &sp) != 0) {
        fprintf(stderr, "%s: sched_setscheduler failed: %s\n", who, strerror(errno));
    }
}

#endif
