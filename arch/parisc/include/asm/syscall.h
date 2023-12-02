/* SPDX-License-Identifier: GPL-2.0 */
/* syscall.h */

#ifndef _ASM_PARISC_SYSCALL_H_
#define _ASM_PARISC_SYSCALL_H_

#include <uapi/linux/audit.h>
#include <linux/compat.h>
#include <linux/err.h>
#include <asm/ptrace.h>

#define NR_syscalls (__NR_Linux_syscalls)

static inline long syscall_get_nr(struct task_struct *tsk,
				  struct pt_regs *regs)
{
	return regs->gr[20];
}

static inline void syscall_get_arguments(struct task_struct *tsk,
					 struct pt_regs *regs,
					 unsigned long *args)
{
	args[5] = regs->gr[21];
	args[4] = regs->gr[22];
	args[3] = regs->gr[23];
	args[2] = regs->gr[24];
	args[1] = regs->gr[25];
	args[0] = regs->gr[26];
}

static inline long syscall_get_error(struct task_struct *task,
				     struct pt_regs *regs)
{
	unsigned long error = regs->gr[28];
	return IS_ERR_VALUE(error) ? error : 0;
}

static inline long syscall_get_return_value(struct task_struct *task,
						struct pt_regs *regs)
{
	return regs->gr[28];
}

static inline void syscall_set_return_value(struct task_struct *task,
					    struct pt_regs *regs,
					    int error, long val)
{
	regs->gr[28] = error ? error : val;
}

static inline void syscall_rollback(struct task_struct *task,
				    struct pt_regs *regs)
{
	/* do nothing */
}

static inline int syscall_get_arch(struct task_struct *task)
{
	int arch = AUDIT_ARCH_PARISC;
#ifdef CONFIG_64BIT
	if (!__is_compat_task(task))
		arch = AUDIT_ARCH_PARISC64;
#endif
	return arch;
}

extern void * const sys_call_table[];
extern void * const sys_call_table64[];

typedef long (*syscall_t)(ulong, ulong, ulong, ulong, ulong, ulong, ulong);
static inline void syscall_handler(struct pt_regs *regs, ulong syscall)
{
	syscall_t fn;

#ifdef CONFIG_COMPAT
	/* W hidden in bottom bit of sp */
	if (regs->gr[30] & 1)
		fn = sys_call_table64[syscall];
	else
#endif
		fn = sys_call_table[syscall];

	regs->gr[28] = fn(regs->gr[26], regs->gr[25], regs->gr[24], regs->gr[23],
			regs->gr[22], regs->gr[21], regs->gr[20]); // XXX for
								   // 32/64bit
}

static inline bool arch_syscall_is_vdso_sigreturn(struct pt_regs *regs)
{
	return false;
}

#endif /*_ASM_PARISC_SYSCALL_H_*/
