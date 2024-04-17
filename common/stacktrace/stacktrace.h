#ifndef _STACKTRACE_H_
#define _STACKTRACE_H_

#include <stdint.h>

#define STACKTRACE_DEPTH 10

struct stackframe{
    struct stackframe* fp;
    uint32_t lr;
};

void get_stacktrace(struct stackframe* st);

#endif /* _STACKTRACE_H_ */
