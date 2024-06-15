/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ASM_VDSO_GETTIMEOFDAY_H
#define ASM_VDSO_GETTIMEOFDAY_H

/* need to save GR4 across syscall, see asm/unistd.h */
#define PIC
#include <asm/unistd.h>

#define VDSO_HAS_TIME 1

#define VDSO_HAS_CLOCK_GETRES 1

// #include <asm/syscall.h>
#include <asm/barrier.h>
#include <asm/timex.h>
#include <linux/compiler.h>

#define PAGE_SIZE	4096

static inline unsigned long get_vdso_base(void)
{
	unsigned long addr, offs;

	/*
	 * Get the base load address of the VDSO. We have to avoid generating
	 * relocations and references to the GOT because ld.so does not perform
	 * relocations on the VDSO. We use the current offset from the VDSO base
	 * and perform a PC-relative branch which gives the absolute address in
	 *
	 */

	__asm__(
	"	b,l,n	1f,%%r2		\n"
	"	.import vdso_start,data	\n"
	"	.word	vdso_start - .	\n"
	"1:	copy	%%r2,%0		\n"
	"	ldw	-4(%0),%1	\n"
	: "=r" (addr), "=r" (offs) : : "r2");

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

static inline u64 __arch_get_hw_counter(s32 clock_mode, const struct vdso_data *vd)
{
#if 0
	u64 adj, now;

	now = get_tod_clock();
	adj = vd->arch_data.tod_steering_end - now;
	if (unlikely((s64) adj > 0))
		now += (vd->arch_data.tod_steering_delta < 0) ? (adj >> 15) : -(adj >> 15);
	return now;
#else
	return -1ULL;
#endif
}

static __always_inline
long clock_gettime_fallback(clockid_t clkid, struct __kernel_timespec *ts)
{
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
