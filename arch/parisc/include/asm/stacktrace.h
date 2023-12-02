/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __ASM_PARISC_STACKTRACE_H
#define __ASM_PARISC_STACKTRACE_H

#include <linux/percpu.h>
#include <linux/sched.h>
#include <linux/sched/task_stack.h>

#include <asm/ptrace.h>

static inline bool on_thread_stack(void)
{
	return !(((unsigned long)(current->stack) ^ current_stack_pointer) & ~(THREAD_SIZE - 1));
}

#endif	/* __ASM_PARISC_STACKTRACE_H */
