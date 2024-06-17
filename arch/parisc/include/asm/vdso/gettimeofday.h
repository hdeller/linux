/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ASM_VDSO_GETTIMEOFDAY_H
#define ASM_VDSO_GETTIMEOFDAY_H

/* Enable PIC since need to save GR4 across syscall, see asm/unistd.h */
#define PIC
#include <asm/unistd.h>
#include <asm/unistd_32.h>

#define VDSO_HAS_TIME 1

#define VDSO_HAS_CLOCK_GETRES 1

#include <asm/barrier.h>
#include <asm/processor.h>
#include <asm/timex.h>
#include <linux/compiler.h>

#if !defined(PAGE_SIZE)
#define PAGE_SIZE	4096
#endif

static inline unsigned long get_vdso_base(void)
{
	unsigned long addr, offs;

	__asm__(
	"	b,l,n	1f,%%r2		\n"
	"	.import vdso_start,data	\n"
	"	.word	vdso_start - .	\n"
	"1:	copy	%%r2,%0		\n"
	"	ldw	-4-%2(%0),%1	\n"
	: "=r" (addr), "=r" (offs) : "i" (PRIV_USER) : "r2");

	return (addr + offs) & ~(PAGE_SIZE-1);
}

#define vdso_calc_delta __arch_vdso_calc_delta
static __always_inline u64 __arch_vdso_calc_delta(u64 cycles, u64 last, u64 mask, u32 mult)
{
	return (cycles - last) * mult;
}

static __always_inline const struct vdso_data *__arch_get_vdso_data(void)
{
	return (const struct vdso_data *) get_vdso_base() - 2 * PAGE_SIZE;
}

static inline bool parisc_vdso_hres_capable(void)
{
	if (!IS_ENABLED(CONFIG_SMP))
		return true;
	// if (on_qemu) true
	return false;
	return true;
}
#define __arch_vdso_hres_capable parisc_vdso_hres_capable

static inline u64 __arch_get_hw_counter(s32 clock_mode, const struct vdso_data *vd)
{
	// asm(".word 0xdeadbeef");
	return get_cycles();
}

static __always_inline
long clock_gettime_fallback(clockid_t clkid, struct __kernel_timespec *ts)
{
	/* we are 32-bit binary, so need to call gettime64 syscall */
	return syscall2(__NR_clock_gettime64, (long)clkid, (long)ts);
}

static __always_inline
long clock_gettime32_fallback(clockid_t clkid, struct old_timespec32 *ts)
{
	/* calls compat function when built as 32-bit binary */
	return syscall2(__NR_clock_gettime, (long)clkid, (long)ts);
}

static __always_inline
long gettimeofday_fallback(register struct __kernel_old_timeval *tv,
			   register struct timezone *tz)
{
	return syscall2(__NR_gettimeofday, (long)tv, (long)tz);
}

static __always_inline
long clock_getres_fallback(clockid_t clkid, struct __kernel_timespec *ts)
{
	return syscall2(__NR_clock_getres_time64, (long)clkid, (long)ts);
}

static __always_inline
long clock_getres32_fallback(clockid_t clkid, struct old_timespec32 *ts)
{
	return syscall2(__NR_clock_getres, (long)clkid, (long)ts);
}

#ifdef CONFIG_TIME_NS
static __always_inline
const struct vdso_data *__arch_get_timens_vdso_data(const struct vdso_data *vd)
{
	return (const struct vdso_data *) get_vdso_base() - 1 * PAGE_SIZE;
}
#endif

#endif
