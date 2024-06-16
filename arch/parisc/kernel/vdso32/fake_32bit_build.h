/* SPDX-License-Identifier: GPL-2.0 */
#ifdef CONFIG_64BIT

/*
 * in case of a 32 bit VDSO for a 64 bit kernel fake a 32 bit kernel
 * configuration
 */
#undef CONFIG_64BIT
#undef CONFIG_COMPAT

#define BUILD_VDSO32_64

#endif
