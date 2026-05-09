#ifndef RT_AFFINITY_H
#define RT_AFFINITY_H

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>

/*
 * QNX-native CPU isolation:
 * ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT, ...)
 */
static inline int adas_bind_to_cpu(int cpu, const char *who)
{
    const int num_cpu = _syspage_ptr->num_cpu;
    if (cpu < 0 || cpu >= num_cpu) {
        fprintf(stderr, "%s: invalid cpu=%d (num_cpu=%d)\n", who, cpu, num_cpu);
        return -1;
    }

    const int rsize = RMSK_SIZE(num_cpu);
    const size_t bytes = sizeof(struct _thread_runmask) + (size_t)(2 * rsize) * sizeof(unsigned);

    struct _thread_runmask *rm = (struct _thread_runmask *)calloc(1, bytes);
    if (!rm) {
        perror("calloc(runmask)");
        return -1;
    }

    rm->size = rsize;
    unsigned *runmask = (unsigned *)(rm + 1);
    unsigned *inherit = runmask + rsize;

    RMSK_SET(cpu, runmask);
    RMSK_SET(cpu, inherit);

    if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET_INHERIT, rm) == -1) {
        fprintf(stderr, "%s: ThreadCtl(runmask cpu=%d) failed: %s\n",
                who, cpu, strerror(errno));
        free(rm);
        return -1;
    }

    fprintf(stderr, "%s: pinned to CPU%d/%d\n", who, cpu, num_cpu);
    free(rm);
    return 0;
}

#endif

