#include <stdint.h>
#include <stddef.h>

#include "stacktrace.h"

void get_stacktrace(struct stackframe* st)
{
    uint32_t fp;
    struct stackframe* stack;

    __asm__("mov %0, fp\n" : "=r"(fp));

    for (int i = 0; (i < STACKTRACE_DEPTH) && (stack->fp != NULL); i++) {
        st[i] = *stack;
        stack = stack->fp;
    }
}
