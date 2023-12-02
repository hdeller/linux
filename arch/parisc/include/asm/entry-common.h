/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _ASM_PARISC_ENTRY_COMMON_H
#define _ASM_PARISC_ENTRY_COMMON_H

#include <asm/stacktrace.h>

void handle_page_fault(struct pt_regs *regs, unsigned long code,
		       unsigned long address);
void handle_break(struct pt_regs *regs);

#endif /* _ASM_PARISC_ENTRY_COMMON_H */
