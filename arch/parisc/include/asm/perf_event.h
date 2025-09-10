#ifndef __ASM_PARISC_PERF_EVENT_H
#define __ASM_PARISC_PERF_EVENT_H

#include <asm/psw.h>

/* Empty, just to avoid compiling error */
/* Perf callbacks */
// struct pt_regs;
// #define perf_arch_bpf_user_pt_regs(regs) &regs->user_regs
//
#define perf_arch_fetch_caller_regs(regs, __ip) { \
	(regs)->gr[0] = KERNEL_PSW; \
        (regs)->iaoq[0] = (__ip); \
        asm volatile("copy %%sp, %0\n":"=r"((regs)->gr[30])); \
}

#endif /* __ASM_PARISC_PERF_EVENT_H */
