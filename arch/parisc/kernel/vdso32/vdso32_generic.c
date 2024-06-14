// SPDX-License-Identifier: GPL-2.0
#include "../../../../lib/vdso/gettimeofday.c"
#include "vdso.h"

int __kernel_gettimeofday(struct __kernel_old_timeval *tv,
			     struct timezone *tz)
{
	return __cvdso_gettimeofday(tv, tz);
}

int __kernel_clock_gettime(clockid_t clock, struct __kernel_timespec *ts)
{
	return __cvdso_clock_gettime(clock, ts);
}

int __kernel_clock_getres(clockid_t clock, struct __kernel_timespec *ts)
{
	return __cvdso_clock_getres(clock, ts);
}
