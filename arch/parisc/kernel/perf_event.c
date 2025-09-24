// SPDX-License-Identifier: GPL-2.0
/*
 * Performance event support for parisc
 *
 * Copyright (C) 2025 by Helge Deller <deller@gmx.de>
 */
#define KMSG_COMPONENT	"perf"
#define pr_fmt(fmt)	KMSG_COMPONENT ": " fmt

#include <linux/kernel.h>
#include <linux/perf_event.h>
#include <linux/percpu.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/sysfs.h>
#include <linux/ptrace.h>
#include <asm/irq.h>
#include <asm/processor.h>
#include <asm/unwind.h>

void perf_event_print_debug(void)
{
	unsigned long flags;

	local_irq_save(flags);
	local_irq_restore(flags);
}

void perf_callchain_kernel(struct perf_callchain_entry_ctx *entry,
			   struct pt_regs *regs)
{

	struct unwind_frame_info info;

	unwind_frame_init_task(&info, current, NULL);
	while (1) {
		if (unwind_once(&info) < 0 || info.ip == 0)
			break;

		if (!__kernel_text_address(info.ip) ||
			perf_callchain_store(entry, info.ip))
				return;
	}
}

#if 0
void perf_callchain_user(struct perf_callchain_entry_ctx *entry,
			 struct pt_regs *regs)
{
	arch_stack_walk_user_common(NULL, NULL, entry, regs, true);
}
#endif
