// SPDX-License-Identifier: GPL-2.0

#define BUILD_VDSO32
#include "fake_32bit_build.h"

#include "../../../../lib/vdso/gettimeofday.c"
#include "vdso.h"

int __vdso_gettimeofday(struct __kernel_old_timeval *tv,
			     struct timezone *tz)
{
	return __cvdso_gettimeofday(tv, tz);
}

int __vdso_clock_gettime32(clockid_t clock, struct old_timespec32 *ts)
{
	return __cvdso_clock_gettime32(clock, ts);
}

int __vdso_clock_gettime64(clockid_t clock, struct __kernel_timespec *ts)
{
	return __cvdso_clock_gettime(clock, ts);
}
