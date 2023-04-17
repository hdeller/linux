// SPDX-License-Identifier: GPL-2.0+
/*
 * Framebuffer driver for Visualize FX cards commonly found in PA-RISC machines
 *
 * Copyright (c) 2021-2023 Sven Schnelle <svens@stackframe.org>
 * Copyright (c) 2022-2023 Helge Deller <deller@gmx.de>
 *
 *
 * insmod ~/visfxfb.ko mode_option=1366x768
 * Tools:
 * con2fbmap 1 1
 * fbset  -fb /dev/fb1 1280x1024-60 -depth 8
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rational.h>

#include <asm/grfioctl.h>

#include "sticore.h"
#include "visualizefx.h"

#define VISFX_CARDTYPE_FX5	CRT_ID_LEGO

#define UP_CONTROL_TO		BIT(13)
#define UP_CONTROL_TCE		BIT(12)
#define UP_CONTROL_SOFT_RST	BIT(8)

#define UB_STATUS_FAULT		BIT(27)
#define UB_STATUS_WPNE		BIT(24)

#define VISFX_DFP_ENABLE		0x10000
#define VISFX_HSYNC_POSITIVE		0x40000
#define VISFX_VSYNC_POSITIVE		0x80000

#define VISFX_SYNC_PLL_BASE		49383 /* 20.25MHz in ps */

#define VISFX_FB_LENGTH			0x01000000
#define VISFX_FB_OFFSET			0x01000000

#define MIN_XRES	640
#define MIN_YRES	480

#define DEFAULT_BPP	8 // 32

static char *mode_option; /* empty means take video mode from ROM */

/* const struct in ROM */
struct visfx_default {
	u8 __pad0[0x48];
	u32 pll_core_0;
	u32 pll_core_1;
	u8 __pad1[0x08];
	u32 pll_ram_0;
	u32 pll_ram_1;
	u8 __pad2[0x18];
	u32 pll_cb4038_0;
	u32 pll_cb4038_1;
	u8 __pad3[0x08];
	u32 pll_ga_ipll_0;
	u32 pll_ga_ipll_1;
	u8 __pad4[0x08];
};

struct visfx_par {
	void __iomem *reg_base;
	unsigned long reg_size;
	u32 abmap, ibmap0, bmap_z, ibmap1, obmap0;
	u32 dba, bmap_dba;
	u32 pseudo_palette[16];
	struct visfx_default *defaults;
	struct device *dev;

	struct fb_info *info;
};

static void visfx_setup(struct fb_info *info);

static u32 visfx_readl(struct fb_info *info, int reg)
{
	struct visfx_par *par = info->par;

	return le32_to_cpu(readl(par->reg_base + reg));
}

static void visfx_writel(struct fb_info *info, int reg, u32 val)
{
	struct visfx_par *par = info->par;

	return writel(cpu_to_le32(val), par->reg_base + reg);
}

static void visfx_write_vram(struct fb_info *info, int reg, u32 val)
{
	struct visfx_par *par = info->par;

	return writel(val, par->reg_base + reg);
}

static int visfx_wait_write_pipe_empty(struct fb_info *info)
{
	u32 status;
	int i;

	for(i = 0; i < 1000000; i++) {
		status = visfx_readl(info, UB_STATUS);
		if (WARN_ON_ONCE(status & UB_STATUS_FAULT))
			return -EIO;

		if (!(status & UB_STATUS_WPNE))
			return 0;
		udelay(1);
	}
	WARN_ON_ONCE(1);
	return -ETIMEDOUT;
}

static void visfx_set_vram_addr(struct fb_info *info, int x, int y)
{
	visfx_writel(info, B2_DWA, (y << 16) | x);
}

static u32 visfx_cmap_entry(struct fb_info *info, int color)
{
	struct fb_cmap *cmap;

	if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
		return color;

	cmap = &info->cmap;
	return (((cmap->blue[color] & 0xff) << 0) |
		((cmap->green[color] & 0xff) << 8) |
		(cmap->red[color] & 0xff) << 16);
}

static void visfx_set_bmove_color(struct fb_info *info, int fg, int bg)
{
	visfx_writel(info, B2_IBC, visfx_cmap_entry(info, bg));
	visfx_writel(info, B2_IFC, visfx_cmap_entry(info, fg));
}

static void visfx_wclip(struct fb_info *info, int x1, int y1, int x2, int y2)
{
	visfx_writel(info, B2_WCLIP1UL, (x1 << 16) | y1);
	visfx_writel(info, B2_WCLIP1LR, (x2 << 16) | y2);
}

#define LINESIZE(x) (((x-1)/8)+1)
static void visfx_copyline(struct fb_info *info, const char *data, int x,
			   int width, int height, int len)
{
	u32 tmp;
	int y;

	for (y = 0; y < height; y++) {
		memcpy(&tmp, &data[y * LINESIZE(width) + x], len);
		visfx_write_vram(info, B2_BTDstObj_Yi, tmp);
	}
}

static void visfx_imageblit_mono(struct fb_info *info, const char *data, int dx, int dy,
				 int width, int height, int fg_color, int bg_color)
{
	int _width, x;

	// B2_DBA_BIN8F | B2_DBA_OTC01 | B2_DBA_DIRECT | B2_DBA_D;  -> B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	// B2_DBA_BIN8I | B2_DBA_OTC04 | B2_DBA_DIRECT | B2_DBA_D;
	// visfx_writel(info, B2_DBA, par->dba | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fg_color, bg_color);
	visfx_writel(info, B2_BPM, 0xffffffff);

	for (x = 0, _width = width; _width > 0; _width -= 32, x += 4) {
		visfx_set_vram_addr(info, dx + x * 32, dy);
		if (_width >= 32)
			visfx_copyline(info, data, x, width, height, 4);
		else {
			visfx_writel(info, B2_BPM, GENMASK(31, 31 - _width));
			visfx_copyline(info, data, x, width, height, LINESIZE(_width));
		}
	}
	visfx_writel(info, B2_BPM, 0xffffffff);
}

static void visfx_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct visfx_par *par = info->par;
	int x, y;

	visfx_wait_write_pipe_empty(info);

	switch (image->depth) {
	case 1:
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
			return cfb_imageblit(info, image);

		visfx_imageblit_mono(info, image->data, image->dx, image->dy,
				     image->width, image->height,
				     image->fg_color, image->bg_color);
		break;
	case 8:
		if (info->fix.visual == FB_VISUAL_PSEUDOCOLOR)
			return cfb_imageblit(info, image);

		visfx_writel(info, B2_DBA, B2_DBA_OTC01 | B2_DBA_DIRECT);
		visfx_writel(info, B2_BPM, 0xffffffff);

		for (y = 0; y < image->height; y++) {
			visfx_set_vram_addr(info, image->dx, image->dy + y);
			for (x = 0; x < image->width; x++) {
				unsigned int c = ((unsigned char *)image->data)[y * image->width + x];

				visfx_write_vram(info, B2_BTDstObj_Xi, ((u32*)info->pseudo_palette)[c]);
			}
		}
		break;

	default:
		return cfb_imageblit(info, image);
	}

	visfx_writel(info, B2_DBA, par->dba);
}

static void visfx_fillrect(struct fb_info *info, const struct fb_fillrect *fr)
{
	struct visfx_par *par = info->par;

	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fr->color, 0);
	visfx_writel(info, B2_MNOOP_R0R1, (fr->dx << 16) | fr->dy);
	visfx_writel(info, B2_SOLIDFILL_R2R3, (fr->width << 16) | fr->height);

	visfx_writel(info, B2_DBA, par->dba);
}

static void visfx_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	visfx_writel(info, B2_MNOOP_R4R5, (area->sx << 16) | area->sy);
	visfx_writel(info, B2_MNOOP_R2R3, (area->width << 16) | area->height);
	visfx_writel(info, B2_BLT_R0R1, (area->dx << 16) | area->dy);
}

static int visfx_setcolreg(unsigned regno, unsigned red, unsigned green,
                             unsigned blue, unsigned transp,
			     struct fb_info *info)
{
	u32 r, g, b;

	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue = (red * 77 + green * 151 + blue * 28) >> 8;
	}

	switch (info->fix.visual) {
	case FB_VISUAL_PSEUDOCOLOR:
		if (regno >= 256)
			return -EINVAL;
		r = (red >> 8) << 16;
		g = (green >> 8) << 8;
		b = (blue >> 8);
		set_clut:
		visfx_writel(info, B2_LLCA, regno);
		visfx_writel(info, B2_LUTD, r | g | b);
		break;
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_DIRECTCOLOR:
		r = (red >> (16 - info->var.red.length))
			<< info->var.red.offset;
		b = (blue >> (16 - info->var.blue.length))
			<< info->var.blue.offset;
		g = (green >> (16 - info->var.green.length))
			<< info->var.green.offset;
		if (regno < 16)
			((u32 *) info->pseudo_palette)[regno] = r | g | b;
		if (info->fix.visual == FB_VISUAL_DIRECTCOLOR)
			goto set_clut;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int visfx_blank(int blank_mode, struct fb_info *info)
{
	u32 mpc;

	switch (blank_mode) {
	case FB_BLANK_POWERDOWN:
		mpc = 3;
		break;
	case FB_BLANK_NORMAL:
		mpc = 0;
		break;
	case FB_BLANK_UNBLANK:
		mpc = 3 << 2;
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		mpc = 1 << 1;
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		mpc = 1 << 0;
		break;
	default:
		return 1;
	}
	visfx_writel(info, B2_MPC, mpc);

	return 0;
}

int visfx_sync(struct fb_info *info)
{
	visfx_wait_write_pipe_empty(info);
	return 0;
}

static void visfx_get_video_mode(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;
	unsigned long n, d;
	u32 tmp;

	tmp = visfx_readl(info, B2_VHAL);
	var->xres = (tmp & 0xffff) + 1;
	var->yres = (tmp >> 16) + 1;

// printk("x %d  y %d\n", var->xres, var->yres);

	tmp = visfx_readl(info, B2_PLL_DOT_CTL);
	n = (tmp & 0xff) + 1;
	d = ((tmp >> 8) & 0xff) + 1;
	var->pixclock = (VISFX_SYNC_PLL_BASE / d) * n;

	tmp = visfx_readl(info, B2_HTG);
	var->left_margin = ((tmp >> 20) & 0x1ff) + 1;
	var->hsync_len = (((tmp >> 12) & 0xff) + 1) * 4;
	var->right_margin = (tmp & 0x1ff) + 1;

	tmp = visfx_readl(info, B2_VTG);
	var->upper_margin = ((tmp >> 16) & 0xff) + 1;
	var->vsync_len = ((tmp >> 8) & 0xff) + 1;
	var->lower_margin = (tmp & 0xff) + 1;

	tmp = visfx_readl(info, B2_CFG);
	if (tmp & VISFX_HSYNC_POSITIVE)
		var->sync |= FB_SYNC_HOR_HIGH_ACT;
	if (tmp & VISFX_VSYNC_POSITIVE)
		var->sync |= FB_SYNC_VERT_HIGH_ACT;
}

static int visfx_wait_pll(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	u32 status, pll_stat;
	int i = 0;

	visfx_writel(info, B2_PLL_REF_CNT, BIT(31) | 202500);

	for (i = 0; i < 100000; i++) {
		status = visfx_readl(info, UB_STATUS);
		pll_stat = visfx_readl(info, B2_PLL_REF_CNT) & 0xffffff;
		if (pll_stat == 0 && !(pll_stat & (UB_STATUS_FAULT | UB_STATUS_WPNE)))
			break;
		udelay(1);
	}

	if (status & (UB_STATUS_FAULT | UB_STATUS_WPNE)) {
		dev_err(par->dev, "error %x\n", status);
		return -EIO;
	}

	if (i == 100000) {
		dev_err(par->dev, "timeout (status %x)\n",
		       visfx_readl(info, B2_PLL_REF_CNT));
		return -ETIMEDOUT;
	}
	return 0;
}

static int visfx_init_pll(struct fb_info *info, u32 reg, u32 value1, u32 value2)
{
	int ret;

	visfx_writel(info, reg, value1);
	ret = visfx_wait_pll(info);
	if (ret)
		return ret;

	visfx_writel(info, reg, value2);
	ret = visfx_wait_pll(info);
	if (ret)
		return ret;

	visfx_writel(info, reg, value2 | 0x10000);
        return visfx_wait_pll(info);
}

static int visfx_set_pll(struct fb_info *info, u32 reg, unsigned long clock)
{
	unsigned long n, d, tmp;

	rational_best_approximation(clock, VISFX_SYNC_PLL_BASE, 0x3f, 0x3f, &n, &d);
	tmp = 0x520000 | ((((d * 4) - 1) << 8) | ((n * 4) - 1));
	return visfx_init_pll(info, reg, tmp, tmp);
}

static int visfx_set_par(struct fb_info *info)
{
	u32 xres, yres, hbp, hsw, hfp, vbp, vsw, vfp, tmp;
	struct fb_var_screeninfo *var = &info->var;
	int ret;

	xres = var->xres;
	yres = var->yres;
	hsw = var->hsync_len / 4 - 1;
	hfp = (var->right_margin - 1) & 0x1ff;
	hbp = var->left_margin - 1;
	vsw = var->vsync_len - 1;
	vfp = var->lower_margin - 1;
	vbp = var->upper_margin - 1;

	ret = visfx_set_pll(info, B2_PLL_DOT_CTL, var->pixclock);
	if (ret)
		return ret;

	visfx_writel(info, B2_HTG, (hbp << 20) | (hsw << 12) | (0xc << 8) | hfp);
	visfx_writel(info, B2_VTG, (vbp << 0) | (vsw << 8) | (vfp << 16));
	visfx_writel(info, B2_VHAL,((yres - 1) << 16) | (xres - 1));
	visfx_writel(info, B2_ETG, 0x12260);
	visfx_writel(info, B2_SCR, 0);
	visfx_wclip(info, 0, 0, xres, yres);

	tmp = VISFX_DFP_ENABLE;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		tmp |= VISFX_HSYNC_POSITIVE;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		tmp |= VISFX_VSYNC_POSITIVE;
	visfx_writel(info, B2_CFG, tmp);
	visfx_writel(info, B2_MPC, 0xc);

	visfx_setup(info);

	return visfx_wait_write_pipe_empty(info);
}

static int visfx_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct fb_var_screeninfo *v = &info->var;

	if (var->pixclock > VISFX_SYNC_PLL_BASE ||
		var->left_margin > 512 ||
		var->right_margin > 512 ||
		var->hsync_len > 512 ||
		var->lower_margin > 256 ||
		var->upper_margin > 256 ||
		var->vsync_len > 256 ||
		var->xres > 2048 ||
		var->yres > 2048)
		return -EINVAL;

	if (var->bits_per_pixel == 24)
		var->bits_per_pixel = 32;
	if (var->bits_per_pixel != 8 && var->bits_per_pixel != 32)
		return -EINVAL;

	if (var->xres & 0x7)
		return -EINVAL;
	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = v->red;
	var->green  = v->green;
	var->blue   = v->blue;
	var->transp = v->transp;

	var->nonstd = 0;
	var->height = -1;
	var->width = -1;
	var->vmode = FB_VMODE_NONINTERLACED;
	var->accel_flags = info->flags;

	return 0;
}

static void visfx_update_cursor_image_line(struct fb_info *info,
					   struct fb_cursor *cursor, int y)
{
	unsigned int x, bytecnt;
	u32 data[2] = { 0 };
	u8 d, m;

	bytecnt = ((cursor->image.width - 1) / 8) + 1;

	for (x = 0; x < bytecnt && x < 8; x++) {
		m = cursor->mask[y * bytecnt + x];
		d = cursor->image.data[y * bytecnt + x];

		if (cursor->rop == ROP_XOR)
			((u8 *)data)[x] = d ^ m;
		else
			((u8 *)data)[x] = d & m;
	}

	if (cursor->image.width < 32)
		data[0] &= GENMASK(31, 31 - cursor->image.width + 1);
	visfx_writel(info, UB_CD, data[0]);
	if (cursor->image.width < 64)
		data[0] &= GENMASK(31, 63 - cursor->image.width + 1);
	visfx_writel(info, UB_CD, data[1]);
}

static void visfx_update_cursor_image(struct fb_info *info,
				      struct fb_cursor *cursor)
{
	int y, height = cursor->image.height;

	WARN_ONCE(1, "HALLO2");

	if (height > 128)
		height = 128;

	visfx_writel(info, UB_CA, 0);
	for (y = 0; y < height; y++)
		visfx_update_cursor_image_line(info, cursor, y);

	for (; y < 256; y++)
		visfx_writel(info, UB_CD, 0);
}

static int visfx_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	u32 cp, color;

	cp = (cursor->image.dx << 16) | (cursor->image.dy & 0xffff);
	visfx_writel(info, UB_CP, cp);

	if (cursor->set & (FB_CUR_SETIMAGE|FB_CUR_SETSHAPE))
		visfx_update_cursor_image(info, cursor);

	if (cursor->set & FB_CUR_SETCMAP) {
		color = visfx_cmap_entry(info, cursor->image.fg_color);
		visfx_writel(info, UB_CB, color);
	}

	if (cursor->enable) {
		cp |= UB_CP_CURSOR_ENABLE;
		visfx_writel(info, UB_CP, cp);
	}
	return 0;
}

static const struct fb_ops visfx_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= visfx_check_var,
	.fb_set_par	= visfx_set_par,
	.fb_setcolreg	= visfx_setcolreg,
	.fb_blank	= visfx_blank,
	.fb_fillrect	= visfx_fillrect,
	.fb_copyarea    = visfx_copyarea,
	.fb_imageblit	= visfx_imageblit,
	// .fb_cursor	= visfx_cursor,
	.fb_sync	= visfx_sync,
};

static void visfx_bus_error_timer_enable(struct fb_info *info, bool enable)
{
	u32 tmp = visfx_readl(info, UP_CONTROL);

	if (enable)
		tmp |= 0x1000;
	else
		tmp &= ~0x1000;
	visfx_writel(info, UP_CONTROL, tmp);
}

static int visfx_soft_reset(struct fb_info *info)
{
	int i = 0;
	u32 old;

	old = visfx_readl(info, UP_CONTROL);
	visfx_writel(info, UP_CONTROL, UP_CONTROL_SOFT_RST);
	while (visfx_readl(info, UP_CONTROL) & UP_CONTROL_SOFT_RST) {
		if (i++ > 10000)
			return -ETIMEDOUT;
		udelay(10);
	}
	visfx_writel(info, UP_TCP, 0x00030000);
	visfx_writel(info, UP_CONTROL, old & ~UP_CONTROL_TO);
	visfx_writel(info, B2_WORG, 0);
	if (visfx_readl(info, UB_STATUS) & UB_STATUS_FAULT)
		return -EIO;
	return 0;
}

static int visfx_reset(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	int ret;
	u32 tmp;

	ret = visfx_soft_reset(info);
	if (ret)
		return ret;

	ret = visfx_wait_write_pipe_empty(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_SWEN, 0xffffffff);
	visfx_writel(info, B2_SREN, 0xffffffff);

	ret = visfx_wait_write_pipe_empty(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_MV, 0x2001db0);
	visfx_writel(info, UB_UIRC, 0x04040404);
	visfx_writel(info, B2_IRC, 0x04040404);
	visfx_writel(info, B2_DMA_BSCSAV, 0);
	visfx_writel(info, UB_DMA_UBSCSAV, 0);
	visfx_writel(info, B2_DMA_BSCBLK, 0);
	visfx_writel(info, UB_DMA_UBSCBLK, 0);
	visfx_writel(info, UP_TCP, 0x30000);
	visfx_writel(info, UP_DC, 0x30301);
	visfx_writel(info, BP_CF_DRC, 0x300);
	visfx_writel(info, BP_CF_HWC, 0x104);
	visfx_writel(info, BP_CF_LWC, 0x3e8);
	visfx_writel(info, BP_CD_HWC, 0x4);
	visfx_writel(info, B2_CD_BUFFER_CTL, 0x4040404);
	visfx_writel(info, B2_PDU_BSCFB, 0x1b);
	visfx_writel(info, UB_PDU_UBSCFB, 0x1b);
	visfx_writel(info, B2_DMA_BSCFB, 0x1b);
	visfx_writel(info, UB_DMA_UBSCFB, 0x1b);
	visfx_writel(info, B2_FBC_RBS, 0xe4);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_MFU_BSCCTL, 0x1b);
	visfx_writel(info, B2_MFU_BSCTD, 0x1b);
	visfx_writel(info, BP_ICR, 0);
	visfx_writel(info, B2_SWEN, 0xffffffff);
	visfx_writel(info, B2_SREN, 0x1000001);
	visfx_writel(info, B2_FPR, 0);

	visfx_writel(info, B3_BBC_CONFIG, 0);
	tmp = visfx_readl(info, B3_HUNGRY);

	switch(tmp) {
	case 0x2a:
		visfx_writel(info, B3_NUM_GA, 0x00000003);
		visfx_writel(info, B3_GA_CONFIG, 0x00000002);
		break;
	case 0xaaa:
		visfx_writel(info, B3_NUM_GA, 0x00000006);
		visfx_writel(info, B3_GA_CONFIG, 0x00000005);
		break;
	default:
		dev_err(par->dev, "unknown value %x\n", tmp);
		break;
	}
	visfx_writel(info, B3_BBC_CONFIG, 4);
	return 0;
}

int visfx_set_default_mode(struct fb_info *info)
{
	u32 tmp;
	int ret;

	ret = visfx_init_pll(info, B2_PLL_DOT_CTL,
			     visfx_readl(info, 0x50028),
			     visfx_readl(info, 0x5002c));
	if (ret)
		return ret;

	visfx_writel(info, B2_HTG, visfx_readl(info, 0x50038));
	visfx_writel(info, B2_VTG, visfx_readl(info, 0x5003c));
	visfx_writel(info, B2_CFG, visfx_readl(info, 0x50040));

	tmp = (visfx_readl(info, 0x50024) & 0xffff0000) |
		(visfx_readl(info, 0x50020) & 0xffff);

	visfx_writel(info, B2_VHAL, tmp - 0x10001);

	visfx_writel(info, B2_ETG, visfx_readl(info, 0x50044));
	visfx_writel(info, B2_SCR, visfx_readl(info, 0x5004c));
	visfx_writel(info, B2_IMD, 2);
	return 0;
}

static void visfx_buffer_setup(struct fb_info *info, int num,
			       u32 imc, u32 ibs, u32 iclr)
{
	visfx_writel(info, B2_ICLR0 + (num << 2), iclr);
	visfx_writel(info, B2_IMC0 + (num << 2), imc);
	visfx_writel(info, B2_IBS0 + (num << 2), ibs);
}

static void visfx_setup_and_wait(struct fb_info *info, u32 reg, u32 val)
{
	visfx_writel(info, UP_CF_STATE, (1 << 25));
	visfx_writel(info, B2_VBS, 1);
	visfx_writel(info, reg, val);
	visfx_writel(info, UP_CF_STATE, 0);
	visfx_wait_write_pipe_empty(info);
}

static void visfx_clear_buffer(struct fb_info *info, u32 dba, u32 bmap_dba, u32 mbwb, u32 ifc)
{
	struct fb_var_screeninfo *var = &info->var;

	visfx_writel(info, B2_DBA, dba);
	visfx_writel(info, B2_BMAP_DBA, bmap_dba);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);
	visfx_writel(info, B2_IFC, ifc);
	visfx_writel(info, B2_FOE, 1);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_MBWB, mbwb);
	visfx_writel(info, B2_MNOOP_R0R1, 0);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, var->xres<<16 | var->yres);
}

static void visfx_setup(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int i;

	visfx_wait_write_pipe_empty(info);

#ifdef __BIG_ENDIAN
	// SEite 238
	if (var->bits_per_pixel == 32) {
		visfx_writel(info, B2_DMA_BSCFB, DSM_NO_BYTE_SWAP);
		visfx_writel(info, B2_PDU_BSCFB, DSM_NO_BYTE_SWAP);
	} else {
		// 0 am Besten, zeigt aber 4 gleiche Pixel nebeneinander (wegen OTC04, aber OTC01 ist falsch bei 32bit zugriff)
		// Seite 222
		// DSM_NO_BYTE_SWAP (e4) -> console ist OK, aber pinguin farben falsch
		// 0 -> console kaputt, aber pinguin richtig.
		u32 val = DSM_NO_BYTE_SWAP;  // XXX
		visfx_writel(info, B2_DMA_BSCFB, val);
		visfx_writel(info, B2_PDU_BSCFB, val);
		visfx_writel(info, B2_FBC_RBS, DSM_NO_BYTE_SWAP);
	}
	visfx_writel(info, B2_MFU_BSCTD, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
#if 0
#define DSM_NO_BYTE_SWAP                        0xe4e4e4e4  // Seite 238
#define DSM_RGBA_TO_ARGB_BYTE_SWAP              0x39393939
#define DSM_ARGB_TO_RGBA_BYTE_SWAP              0x93939393
#define DSM_PDU_BYTE_SWAP_DEFAULT               0x1b1b1b1b
#define DSM_HOST_IS_LITTLE_ENDIAN               0x01010101
#define DSM_HOST_IS_BIG_ENDIAN                  0x00000000
#endif
#else
	visfx_writel(info, B2_DMA_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_PDU_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_MFU_BSCTD, DSM_PDU_BYTE_SWAP_DEFAULT);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
#endif

	visfx_writel(info, B2_DMA_BSCBLK, 0);
	visfx_writel(info, B2_VTB, 0x35);
	visfx_writel(info, B2_TTB, 0x99);
	visfx_writel(info, B2_EN2D, 0);
	visfx_writel(info, B2_FOE, 1);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_SB, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_ZBO, 0x00080000);
	visfx_writel(info, B2_RTG_MEM_LOAD, 0xc9200); // bits 21:2 of host address XXX
	visfx_writel(info, B2_TM_TSS, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_BMAP_Z, 0);
	visfx_writel(info, B2_FSRMWB, 0);
	visfx_readl(info, UB_CONTROL);
	visfx_wait_write_pipe_empty(info);

	visfx_writel(info, UP_CF_STATE, 0x02000000);
	visfx_writel(info, B2_VBS, 1);

	// ABMAP: 0x20200=ohne Overlay,
	par->abmap  = var->xres / 4;	/* XXX: x-resolution divided by 4 */
	par->ibmap0 = 0x02681002;
	par->bmap_z = 0x13090006;
	par->obmap0 = 0x23a80000 | var->xres / 2;
	par->ibmap1 = 0x27e00002;

	visfx_setup_and_wait(info, B2_ABMAP, par->abmap);
	visfx_setup_and_wait(info, B2_IBMAP0, par->ibmap0);
	visfx_setup_and_wait(info, B2_BMAP_Z, par->bmap_z);
	visfx_setup_and_wait(info, B2_OBMAP0, par->obmap0);
	visfx_setup_and_wait(info, UP_CF_STATE, 0);
	visfx_setup_and_wait(info, B2_IBMAP1, par->ibmap1);
	visfx_setup_and_wait(info, B2_DUM, 0x81030002); // keine Auswirkung?
	visfx_setup_and_wait(info, B2_OXYO, 0);

	visfx_writel(info, B2_PMASK, 0xff);
	visfx_writel(info, B2_MPC, 0x0c);

	for (i = 0; i < 7; i++)
		visfx_buffer_setup(info, i, 0, 0, 0);

	visfx_buffer_setup(info, 0, (var->bits_per_pixel == 32) ? 0x09000004 : 0x2000000, 0, 0);
	visfx_buffer_setup(info, 1, (var->bits_per_pixel == 32) ? 0x09000004 : 0x2000000, 1, 0);
	visfx_setup_and_wait(info, B2_OMC, 0x2000000);
	visfx_writel(info, B2_OTLS, 2);
	visfx_writel(info, B2_OBS, 0);

	visfx_wclip(info, 0, 0, var->xres, var->yres);
	visfx_clear_buffer(info, 0x05000880, par->ibmap0, 0x03f00000, 0);
	visfx_clear_buffer(info, 0x05000880, par->obmap0, 0x00fc0000, (var->bits_per_pixel == 32) ? 0xffffffff : 0);
	visfx_clear_buffer(info, 0x05000880, par->abmap,  0x00900000, 0);

	visfx_writel(info, B2_PMASK, 0xff);
	visfx_writel(info, B2_FATTR, 0);
	visfx_writel(info, B2_OTLS, (var->bits_per_pixel == 8) ? 0 : 0x10002);  // OTLS_Type -> auf 0 ?? statt OTR oder 1
	visfx_writel(info, B2_CKEY_HI, 0xffffff);
	visfx_writel(info, B2_CKEY_LO, 0xffffff);

	for (i = 0; i < 256; i++) {
		visfx_writel(info, B2_LLCA, 0x40003000 | i);
		visfx_writel(info, B2_LUTD, 0x010101 * i);
	}

	// visfx_clear_buffer(info, 0x00000a00, par->ibmap0, 0x03f00000, 0);
	visfx_buffer_setup(info, 2, 0x08000084, 0, 0);  // 8000084 für Overlay

	visfx_wait_write_pipe_empty(info);
	visfx_writel(info, B2_BMAP_DBA, par->ibmap0);

	info->fix.accel = FB_ACCEL_NONE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.line_length = 2048 * var->bits_per_pixel / 8;

	switch (var->bits_per_pixel) {
	default:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
		var->red.length = 8;
		var->red.offset = 8;
		var->green.length = 8;
		var->green.offset = 16;
		var->blue.length = 8;
		var->blue.offset = 24;
		var->transp.length = 8;
		var->transp.offset = 0;
		break;
	case 8:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		var->red.length = 8;
		var->red.offset = 0;
		var->green.length = 8;
		var->green.offset = 0;
		var->blue.length = 8;
		var->blue.offset = 0;
		var->transp.length = 0;
		var->transp.offset = 0;
	}

	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	visfx_wclip(info, 0, 0, var->xres, var->yres);

	/* set DBA */
	if (info->var.bits_per_pixel == 32) {
		visfx_writel(info, B2_EN2D, B2_EN2D_WORD_MODE);
		par->dba = B2_DBA_BIN8F | B2_DBA_OTC01 | B2_DBA_DIRECT | B2_DBA_D;
		par->bmap_dba = par->ibmap0;
		visfx_writel(info, B2_BPM, 0xffffffff);
		visfx_writel(info, B2_OTR, 1<<16 | 1<<8 | 0); // ???  Seite 382, Overlay immer durchsichtig !!
		visfx_writel(info, B2_FATTR, 0);
	} else { /* 8-bit indexed: */
		visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);
		par->dba = B2_DBA_BIN8I | B2_DBA_OTC04 | B2_DBA_DIRECT | B2_DBA_D;  // doch OCT01 oder OTC04 ???
		par->bmap_dba = par->obmap0;
		visfx_writel(info, B2_BPM, 0xff << 24);  // oder 0xff << 24
		// visfx_writel(info, B2_OTR, 1<<16 | 3); // ???  Seite 382, 3=immer undurchsichtig !!
		visfx_writel(info, B2_OTR, 2 ); // ???  Seite 382, 3=immer undurchsichtig !!
		visfx_writel(info, B2_CFS16, 0x40); // mode=4, LUT=3 LUT auswählen
		// visfx_writel(info, B2_CFS16, 0x0); // mode=4, LUT=3 LUT auswählen // Seite 396
		// visfx_writel(info, B2_FATTR, 1<<7 | 1<<4); // -> CFS16  -> force Overlay!
		// visfx_writel(info, B2_FATTR, 0);
		visfx_writel(info, B2_FATTR, 1<<7 | 1<<4); // -> CFS16  -> force Overlay!
	}

	visfx_writel(info, B2_BMAP_BABoth, par->bmap_dba);  // WOW !!!!!
	visfx_writel(info, B2_BABoth, par->dba);
	visfx_writel(info, B2_IPM, 0xffffffff); /* all bits/planes relevant, incl. A-mask */
}

static int __init visfx_initialize(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	int i, ret;


	visfx_bus_error_timer_enable(info, false);
	ret = visfx_reset(info);
	if (ret)
		return ret;
	visfx_bus_error_timer_enable(info, true);

	ret = visfx_init_pll(info, B2_PLL_CORE_CTL,
			     par->defaults->pll_core_0,
			     par->defaults->pll_core_1);
	if (ret)
		return ret;

	ret = visfx_init_pll(info, B2_PLL_RAM_CTL,
			     par->defaults->pll_ram_0,
			     par->defaults->pll_ram_1);
	if (ret)
		return ret;

	visfx_writel(info, B2_RC_CONFIG1, 0);
	visfx_writel(info, B2_RC_CONFIG0, 0x04444649);
	visfx_writel(info, B2_RC_CONFIG2, 0x0119F051);
	visfx_writel(info, B2_RC_CONFIG1, 0xE3901212);

	ret = visfx_init_pll(info, 0xcb4038,
			     par->defaults->pll_cb4038_0,
			     par->defaults->pll_cb4038_1);
	if (ret)
		return ret;

	for(i = 0; i < 6; i++)
		visfx_writel(info, B3_GA_NOP_EOC, 0);

	ret = visfx_init_pll(info, B3_GA_IPLL,
			     par->defaults->pll_ga_ipll_0,
			     par->defaults->pll_ga_ipll_1);
	if (ret)
		return ret;

	for(i = 0; i < 6; i++)
		visfx_writel(info, B3_GA_NOP_EOC, 0);

	visfx_writel(info, B2_FOE, 0);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);

	visfx_wclip(info, 0, 0, 2048, 2048);
	visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);
	visfx_writel(info, B2_BABoth, 0x2000000);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x80000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x40000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, B2_MV, 0);

	// setup video regs
	ret = visfx_set_default_mode(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_MFU_BSCTD, 0x1b);
	visfx_writel(info, B2_MFU_BSCCTL, 0x1b);
	visfx_writel(info, B2_FBC_RBS, 0xe4);
	visfx_writel(info, B2_FOE, 0);

	visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);

	return 0;
}

static int __init visfx_get_rom_defaults(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	u32 offset, id;

	if (visfx_readl(info, 0) != 0x55aa0000)
		return -EINVAL;

	offset = le32_to_cpu(visfx_readl(info, 0x08)) + 0x08;
	if (offset > par->reg_size)
		return -EINVAL;

	id = visfx_readl(info, offset);

	if (id != VISFX_CARDTYPE_FX5) {
		dev_err(par->dev, "Unsupported card: ID %08x\n", id);
		return -EINVAL;
	}

	offset = le32_to_cpu(visfx_readl(info, 0x08)) + 0x28;
	if (offset > par->reg_size)
		return -EINVAL;

	offset = visfx_readl(info, offset);
	if (offset > par->reg_size)
		return -EINVAL;

	par->defaults = par->reg_base + offset;
	return 0;
}

/* setup */

static void __init visfx_setup_info(struct pci_dev *pdev, struct fb_info *info)
{
	struct visfx_par *par = info->par;

	info->fbops = &visfx_ops;
	info->flags = FBINFO_HWACCEL_FILLRECT | FBINFO_HWACCEL_IMAGEBLIT |
			FBINFO_HWACCEL_COPYAREA | FBINFO_READS_FAST;
	info->pseudo_palette = par->pseudo_palette;
	info->screen_base = par->reg_base + VISFX_FB_OFFSET;

	strscpy(info->fix.id, "Visualize-FX", sizeof(info->fix.id));
	info->fix.mmio_start = pci_resource_start(pdev, 0);
	info->fix.smem_start = pci_resource_start(pdev, 0) + VISFX_FB_OFFSET;
	info->fix.smem_len = VISFX_FB_LENGTH;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	if (DEFAULT_BPP == 32)
		info->fix.visual = FB_VISUAL_TRUECOLOR;
	else
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	info->fix.line_length = 2048 * DEFAULT_BPP / 8;
	info->fix.accel = FB_ACCEL_NONE;
}

static int __init visfx_init_device(struct pci_dev *pdev, struct sti_struct *sti)
{
	struct visfx_par *par;
	struct fb_info *info;
	char mode[64];
	int ret;

	info = framebuffer_alloc(sizeof(struct visfx_par), &pdev->dev);
	if (!info)
		return -ENOMEM;

	pci_set_drvdata(pdev, info);
	par = info->par;
	par->info = info;
	par->reg_size = pci_resource_len(pdev, 0);
	par->dev = &pdev->dev;

	if (sti) {
		sti->info = info;
		par->reg_base = pci_iomap(pdev, 0, VISFX_FB_OFFSET + VISFX_FB_LENGTH);
	} else
		par->reg_base = pcim_iomap_table(pdev)[0];

	ret = visfx_get_rom_defaults(info);
	if (ret)
		goto err_out_free;

	visfx_setup_info(pdev, info);
	if (sti)
		strscpy(info->fix.id, sti->sti_data->inq_outptr.dev_name,
			sizeof(info->fix.id));

	ret = visfx_initialize(info);
	if (ret)
		goto err_out_free;

	info->var.bits_per_pixel = DEFAULT_BPP;
	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto err_out_free;

	visfx_get_video_mode(info);

	if (!mode_option) {
		scnprintf(mode, sizeof(mode), "%dx%d@60",
			info->var.xres, info->var.yres);
		mode_option = mode;
	}

// printk("MODE OPTION %s\n", mode_option);

	fb_find_mode(&info->var, info, mode_option, NULL, 0, NULL, DEFAULT_BPP);
	visfx_check_var(&info->var, info);
	visfx_set_par(info);

	ret = register_framebuffer(info);
	if (ret)
		goto err_out_dealloc_cmap;

        fb_info(info, "visfxfb %dx%d-%d frame buffer device, %s, mmio: 0x%04lx\n",
                info->var.xres,
                info->var.yres,
                info->var.bits_per_pixel,
                info->fix.id,
                info->fix.mmio_start);

	return 0;

err_out_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
err_out_free:
	framebuffer_release(info);
	return ret;
}

static int visfx_probe_pci(struct pci_dev *pdev,
	       const struct pci_device_id *ent)
{
	int ret;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable PCI device: %d\n", ret);
		return ret;
	}

	ret = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
	if (ret) {
		dev_err(&pdev->dev, "Cannot map PCI resources: %d\n", ret);
		return ret;
	}

	return visfx_init_device(pdev, NULL);
}

static void visfx_probe_sti(struct sti_struct *sti, int enable)
{
	if (!sti || !sti->pd)
		return;

	if (sti->graphics_id[0] != VISFX_CARDTYPE_FX5)
		return;

	if (enable)
		visfx_init_device(sti->pd, sti);
	else {
		struct fb_info *info;
		struct visfx_par *par;

		info = pci_get_drvdata(sti->pd);
		par = info->par;
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
		pci_iounmap(sti->pd, par->reg_base);
	}
}

static int __init visfx_options(char *options)
{
	if (!options || !*options)
		return 0;

	mode_option = options;

	return 0;
}

static void __exit visfx_remove(struct pci_dev *pdev)
{
	struct fb_info *info = pci_get_drvdata(pdev);

	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	framebuffer_release(info);
}

static const struct pci_device_id visfx_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_HP, PCI_DEVICE_ID_HP_VISUALIZE_FX4) },
	{ 0 },
};
MODULE_DEVICE_TABLE(pci, visfx_pci_tbl);

static struct pci_driver visfx_driver = {
	.name      = KBUILD_MODNAME,
	.id_table  = visfx_pci_tbl,
	.probe     = visfx_probe_pci,
	.remove    = visfx_remove,
};

static int __init visfx_init(void)
{
	char *option = NULL;
	int i;

	if (fb_get_options(KBUILD_MODNAME, &option))
		return -ENODEV;

	visfx_options(option);

	if (IS_ENABLED(CONFIG_STI_CONSOLE) || IS_ENABLED(CONFIG_FB_STI))
		for (i = 1; i <= MAX_STI_ROMS; i++)
			visfx_probe_sti(sti_get_rom(i), 1);

	return pci_register_driver(&visfx_driver);
}

static void __exit visfx_exit(void)
{
	int i;

	if (IS_ENABLED(CONFIG_STI_CONSOLE) || IS_ENABLED(CONFIG_FB_STI))
		for (i = 1; i <= MAX_STI_ROMS; i++)
			visfx_probe_sti(sti_get_rom(i), 0);

	pci_unregister_driver(&visfx_driver);
}

module_init(visfx_init);
module_exit(visfx_exit);

module_param(mode_option, charp, 0);
MODULE_PARM_DESC(mode_option, "Initial video mode or empty for ROM defaults");

MODULE_AUTHOR("Sven Schnelle <svens@stackframe.org>");
MODULE_AUTHOR("Helge Deller <deller@gmx.de>");
MODULE_DESCRIPTION("Framebuffer driver for HP Visualize FX cards");
MODULE_LICENSE("GPL");
