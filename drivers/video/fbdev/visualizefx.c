// SPDX-License-Identifier: GPL-2.0+
/*
 * Framebuffer driver for Visualize FX cards commonly found in PA-RISC machines
 *
 * Copyright (c) 2021 Sven Schnelle <svens@stackframe.org>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s:%d: " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include "visfx_regs.h"

#define VISFX_CARDTYPE_FX5		0x35acda30

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
#define NR_PALETTE 256

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
	struct visfx_default *defaults;
	struct device *dev;
	u32 pseudo_palette[256];
	unsigned long debug_reg;
	void __iomem *reg_base;
	unsigned long reg_size;
	int open_count;
};

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

static u32 visfx_cmap_entry(struct fb_cmap *cmap, int color)
{
	return (((cmap->blue[color] & 0xff)) |
		((cmap->green[color] & 0xff) << 8) |
		(cmap->red[color] & 0xff) << 16);
}

static void visfx_set_bmove_color(struct fb_info *info, int fg, int bg)
{
	visfx_writel(info, B2_IBC, visfx_cmap_entry(&info->cmap, bg));
	visfx_writel(info, B2_IFC, visfx_cmap_entry(&info->cmap, fg));
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

	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fg_color, bg_color);
	visfx_writel(info, B2_BPM, 0xffffffff);

	for (x = 0, _width = width; _width > 0; _width -= 32, x += 4) {
		visfx_set_vram_addr(info, dx + x * 8, dy);
		if (_width >= 32)
			visfx_copyline(info, data, x, width, height, 4);
		else {
			visfx_writel(info, B2_BPM, GENMASK(31, 31 - _width));
			visfx_copyline(info, data, x, width, height, LINESIZE(_width));
		}
	}
}

static void visfx_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct visfx_par *par = info->par;
	int x, y;

	visfx_wait_write_pipe_empty(info);

	switch (image->depth) {
	case 1:
		visfx_imageblit_mono(info, image->data, image->dx, image->dy,
				     image->width, image->height,
				     image->fg_color, image->bg_color);
		break;
	case 8:
		visfx_writel(info, B2_DBA, B2_DBA_OTC(0) | B2_DBA_DIRECT);
		visfx_writel(info, B2_BPM, 0xffffffff);

		visfx_wclip(info, 0, 0, info->var.xres, info->var.yres);

		for (y = 0; y < image->height; y++) {
			visfx_set_vram_addr(info, image->dx, image->dy + y);
			for (x = 0; x < image->width; x++) {
				unsigned int c = ((unsigned char *)image->data)[y * image->width + x];

				visfx_write_vram(info, B2_BTDstObj_Xi, ((u32*)info->pseudo_palette)[c]);
			}
		}
		break;

	default:
		dev_err(par->dev, "depth %d not supported\n", image->depth);
		break;
	}
}

static void visfx_fillrect(struct fb_info *info, const struct fb_fillrect *fr)
{
	visfx_wait_write_pipe_empty(info);
	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fr->color, 0);
	visfx_writel(info, B2_MNOOP_R0R1, (fr->dx << 16) | fr->dy);
	visfx_writel(info, B2_SOLIDFILL_R2R3, (fr->width << 16) | fr->height);
}

static int visfx_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	unsigned int i;

	visfx_writel(info, B2_LLCA, cmap->start);

	for (i = 0; i < cmap->len; i++) {
		u32 rgb = visfx_cmap_entry(cmap, i);

		visfx_writel(info, B2_LUTD, rgb);
		((u32*)info->pseudo_palette)[cmap->start + i] = rgb;
	}

	visfx_writel(info, B2_PMASK, 0xffffff);
	visfx_writel(info, B2_CP, 0);
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

	var->red.length = 8;
	var->red.offset = 16;
	var->green.length = 8;
	var->green.offset = 8;
	var->blue.length = 8;
	var->blue.offset = 0;
	var->bits_per_pixel = 24;
	var->grayscale = 0;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	info->screen_size = 2048 * var->yres;
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
	hfp = var->right_margin - 1;
	hbp = var->left_margin - 1;
	vsw = var->vsync_len - 1;
	vfp = var->lower_margin - 1;
	vbp = var->upper_margin - 1;

	ret = visfx_set_pll(info, B2_PLL_DOT_CTL, var->pixclock);
	if (ret)
		return ret;

	visfx_writel(info, B2_VHAL, ((yres - 1) << 16) | (xres - 1));
	visfx_writel(info, B2_HTG, (hbp << 20) | (hsw << 12) | (0xc << 8) | hfp);
	visfx_writel(info, B2_VTG, (vbp << 16) | (vsw << 8) | vfp);
	visfx_wclip(info, 0, 0, xres, yres);

	tmp = VISFX_DFP_ENABLE;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		tmp |= VISFX_HSYNC_POSITIVE;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		tmp |= VISFX_VSYNC_POSITIVE;
	visfx_writel(info, B2_CFG, tmp);
	visfx_writel(info, B2_MPC, 0xc);
	visfx_get_video_mode(info);
	return 0;
}

static int visfx_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
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
		color = visfx_cmap_entry(&info->cmap, cursor->image.fg_color);
		visfx_writel(info, UB_CB, color);
	}

	if (cursor->enable) {
		cp |= UB_CP_CURSOR_ENABLE;
		visfx_writel(info, UB_CP, cp);
	}
	return 0;
}

static int visfx_open(struct fb_info *info, int user)
{
	struct visfx_par *par = info->par;

	if (user && par->open_count++ == 0) {
		visfx_writel(info, B2_BMAP_DBA, 0x02680e02);
		visfx_writel(info, B2_DBA, B2_DBA_OTC(0) | B2_DBA_DIRECT);
	}

	return 0;
}

static int visfx_release(struct fb_info *info, int user)
{
	struct visfx_par *par = info->par;

	if (user)
		par->open_count--;

	return 0;
}

static const struct fb_ops visfx_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= visfx_open,
	.fb_release	= visfx_release,
	.fb_setcmap	= visfx_setcmap,
	.fb_fillrect	= visfx_fillrect,
	.fb_imageblit	= visfx_imageblit,
	.fb_set_par	= visfx_set_par,
	.fb_check_var	= visfx_check_var,
	.fb_cursor	= visfx_cursor,
};

static struct fb_fix_screeninfo visfx_fix = {
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_PSEUDOCOLOR,
	.id = "Visualize FX",
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
	tmp = visfx_readl(info, 0x50054);
	visfx_writel(info, B2_IBMAP0, tmp);
	visfx_writel(info, B2_IMD, tmp);
	visfx_writel(info, B2_BMAP_BABoth, tmp);
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
	visfx_writel(info, B2_DBA, dba);
	visfx_writel(info, B2_BMAP_DBA, bmap_dba);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);
	visfx_writel(info, B2_IFC, ifc);
	visfx_writel(info, B2_FOE, 1);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_WORG, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_MBWB, mbwb);
	visfx_writel(info, B2_DBA, dba | 0x1000);
	visfx_writel(info, B2_MNOOP_R0R1, 0);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, 0x06400240);
	visfx_writel(info, B2_DBA, 0x05000880);
	visfx_writel(info, B2_MNOOP_R0R1, 0x480);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, 0x06400030);
}

static void visfx_setup_x11_pattern(struct fb_info *info)
{
	int i;

	visfx_writel(info, B2_IPM, 0xffffffff);
	for (i = 0; i < 256; i += 4)
		visfx_writel(info, B2_PDR0 + i, 0);
	visfx_writel(info, B2_PCR, 0xc034);
}

static void visfx_setup(struct fb_info *info)
{
	int i;
	visfx_wait_write_pipe_empty(info);

#ifdef __BIG_ENDIAN
	visfx_writel(info, B2_DMA_BSCFB, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_PDU_BSCFB, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_MFU_BSCTD, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
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
	visfx_writel(info, B2_WORG, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_ZBO, 0x00080000);
	visfx_writel(info, B2_RTG_MEM_LOAD, 0xc9200);
	visfx_writel(info, B2_TM_TSS, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_BMAP_Z, 0);
	visfx_writel(info, B2_FSRMWB, 0);
	visfx_readl(info, UB_CONTROL);
	visfx_wait_write_pipe_empty(info);

	visfx_writel(info, UP_CF_STATE, 0x02000000);
	visfx_writel(info, B2_VBS, 1);

	visfx_setup_and_wait(info, B2_ABMAP, 0x20200);
	visfx_setup_and_wait(info, B2_IBMAP0, 0x02680e02);
	visfx_setup_and_wait(info, B2_BMAP_Z, 0x13080e06);
	visfx_setup_and_wait(info, B2_OBMAP0, 0x23a80380);
	visfx_setup_and_wait(info, UP_CF_STATE, 0x00000000);
	visfx_setup_and_wait(info, B2_IBMAP1, 0x27d00e02);
	visfx_setup_and_wait(info, B2_DUM, 0x81030002);
	visfx_setup_and_wait(info, B2_OXYO, 0);

	visfx_writel(info, B2_PMASK, 0xff);
	visfx_writel(info, B2_MPC, 0x0c);

	for (i = 0; i < 7; i++)
		visfx_buffer_setup(info, i, 0, 0, 0);

	visfx_buffer_setup(info, 0, 0x09000004, 0, 0);
	visfx_buffer_setup(info, 1, 0x09000004, 1, 0);
	visfx_setup_and_wait(info, B2_OMC, (1 << 25));
	visfx_writel(info, B2_OTLS, 2);
	visfx_writel(info, B2_OBS, 0);

	visfx_clear_buffer(info, 0x05000880, 0x02680e02, 0x03f00000, 0);
	visfx_clear_buffer(info, 0x05000880, 0x23a80380, 0x00fc0000, 0);
	visfx_clear_buffer(info, 0x05000880, 0x00020200, 0x00900000, 0);

	visfx_writel(info, B2_PMASK, 0xff);
	visfx_writel(info, B2_FATTR, 0);
	visfx_writel(info, B2_OTLS, 0x10002);
	visfx_writel(info, B2_CKEY_HI, 0xffffff);
	visfx_writel(info, B2_CKEY_LO, 0xffffff);

	for (i = 0; i < 256; i++) {
		visfx_writel(info, B2_LLCA, 0x40003000 | i);
		visfx_writel(info, B2_LUTD, 0x010101 * i);
	}

	visfx_setup_x11_pattern(info);

	visfx_clear_buffer(info, 0x00000a00, 0x02680e02, 0x03f00000, 0);
	visfx_wclip(info, 0, 0, 1600, 1200); // FIXME
	visfx_clear_buffer(info, 0x05000880, 0x23a80380, 0x00fc0000, 0xffffffff);
	visfx_buffer_setup(info, 2, 0, 0x08000084, 0);

	visfx_writel(info, B2_WORG, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_SOV, 0);
	visfx_writel(info, B2_DBA, 0x05000880);
	visfx_writel(info, B2_BMAP_DBA, 0x20200);

	visfx_writel(info, B2_IFC, 0x00000000);
	visfx_writel(info, B2_IPM, 0xffffffff);

	visfx_writel(info, B2_MBWB, 0x00900000);
	visfx_writel(info, B2_DBA, 0x05001880);
	visfx_writel(info, B2_MNOOP_R0R1, 0);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, 0x03200240);
	visfx_writel(info, B2_DBA, 0x05000880);
	visfx_writel(info, B2_MNOOP_R0R1, 0x480);
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, 0x03200030);
	visfx_wait_write_pipe_empty(info);
	visfx_writel(info, B2_BMAP_DBA, 0x02680e02);
}

static int visfx_initialize(struct fb_info *info)
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

	visfx_writel(info, B2_WORG, 0);
	visfx_writel(info, B2_FOE, 0);
	visfx_writel(info, B2_IBO, 0);
	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_IPM, 0xffffffff);

	visfx_wclip(info, 0, 0, 2048, 2048);
	visfx_writel(info, B2_EN2D, 0xb0);
	visfx_writel(info, B2_BABoth, 0x2000000);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x80000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x40000200);
	visfx_writel(info, 0x1000000, 0);
	visfx_writel(info, B2_BMAP_BABoth, 0x200);
	visfx_writel(info, 0x800040, 0);

	// setup video regs
	ret = visfx_set_default_mode(info);
	if (ret)
		return ret;

	visfx_writel(info, B2_MFU_BSCTD, 0x1b);
	visfx_writel(info, B2_MFU_BSCCTL, 0x1b);
	visfx_writel(info, B2_FBC_RBS, 0xe4);
	visfx_writel(info, B2_WORG, 0);
	visfx_writel(info, B2_FOE, 0);

	visfx_writel(info, B2_FCDA, 0);
	visfx_writel(info, B2_WCE, 0);
	visfx_writel(info, B2_CPE, 0);
	visfx_writel(info, B2_WCLIP1UL, 0);
	visfx_writel(info, B2_EN2D, 0xb0);

	visfx_writel(info, B2_DBA, B2_DBA_OTC(2));
	visfx_writel(info, B2_IPM, 0xffffffff);
	visfx_writel(info, B2_BPM, 0xffffffff);

	visfx_setup(info);
	return 0;
}

static int visfx_check_defaults(struct fb_info *info)
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

static void visfx_setup_info(struct pci_dev *pdev, struct fb_info *info)
{
	struct visfx_par *par = info->par;

	info->fbops = &visfx_ops;
	info->flags = FBINFO_DEFAULT | \
		      FBINFO_HWACCEL_COPYAREA | \
		      FBINFO_HWACCEL_FILLRECT | \
		      FBINFO_HWACCEL_IMAGEBLIT;
	info->pseudo_palette = par->pseudo_palette;
	info->screen_base = par->reg_base + VISFX_FB_OFFSET;

	info->fix = visfx_fix;
	info->fix.smem_start = pci_resource_start(pdev, 0) + VISFX_FB_OFFSET;
	info->fix.smem_len = VISFX_FB_LENGTH;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = 2048;
	info->fix.accel = FB_ACCEL_NONE;
}

static int visfx_probe(struct pci_dev *pdev,
		       const struct pci_device_id *ent)
{
	struct visfx_par *par;
	struct fb_info *info;
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

	info = framebuffer_alloc(sizeof(struct visfx_par), &pdev->dev);
	if (!info)
		return -ENOMEM;

	pci_set_drvdata(pdev, info);

	par = info->par;
	par->reg_base = pcim_iomap_table(pdev)[0];
	par->reg_size = pci_resource_len(pdev, 0);
	par->dev = &pdev->dev;

	ret = visfx_check_defaults(info);
	if (ret)
		goto err_out_free;

	visfx_setup_info(pdev, info);

	ret = visfx_initialize(info);
	if (ret)
		goto err_out_free;

	visfx_get_video_mode(info);
	info->var.accel_flags = info->flags;

	ret = fb_alloc_cmap(&info->cmap, NR_PALETTE, 0);
	if (ret)
		goto err_out_free;

	ret = register_framebuffer(info);
	if (ret)
		goto err_out_dealloc_cmap;
	return 0;

err_out_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
err_out_free:
	framebuffer_release(info);
	return ret;
}

static void __exit visfx_remove(struct pci_dev *pdev)
{
	struct fb_info *info = pci_get_drvdata(pdev);

	unregister_framebuffer(info);
	framebuffer_release(info);
}

static const struct pci_device_id visfx_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_HP, 0x1008) },
	{ 0 },
};
MODULE_DEVICE_TABLE(pci, visfx_pci_tbl);

static struct pci_driver visfx_driver = {
	.name      = KBUILD_MODNAME,
	.id_table  = visfx_pci_tbl,
	.probe     = visfx_probe,
	.remove    = visfx_remove,
};

module_pci_driver(visfx_driver);

MODULE_AUTHOR("Sven Schnelle <svens@stackframe.org>");
MODULE_DESCRIPTION("Framebuffer driver for HP Visualize FX cards");
MODULE_LICENSE("GPL");
