// SPDX-License-Identifier: GPL-2.0+
/*
 * Framebuffer driver for Visualize FX cards commonly found in PA-RISC machines
 *
 * Copyright (c) 2021-2023 Sven Schnelle <svens@stackframe.org>
 * Copyright (c)      2022 Helge Deller <deller@gmx.de>
 *
 *
 * insmod ~/visfxfb.ko mode_option=1366x768
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/rational.h>
#include <linux/i2c-algo-bit.h>

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

#define NR_PALETTE	256

#define DEFAULT_BPP	32

#define SYSFS		0

static char *mode_option; /* empty means take video mode from ROM */

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
	u32 pseudo_palette[NR_PALETTE];
	struct visfx_default *defaults;
	struct device *dev;
	int open_count;
	unsigned long debug_reg;

	struct fb_info *info;
	struct i2c_adapter adapter;
	struct i2c_algo_bit_data algo;
	u8 *edid;
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

static void visfx_writel_dump(struct fb_info *info, int reg, u32 val, u32 reference)
{
	// printk("DUMP  reg %08x <- %08x,   reference %08x\n", reg, val, reference);
	return visfx_writel(info, reg, val);
}

static ssize_t visfx_sysfs_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct fb_info *info = pci_get_drvdata(to_pci_dev(dev));
	struct visfx_par *par = info->par;

	return sprintf(buf, "%08x\n", visfx_readl(info, par->debug_reg));
}

static ssize_t visfx_sysfs_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct fb_info *info = pci_get_drvdata(to_pci_dev(dev));
	struct visfx_par *par = info->par;
	unsigned long data;
	char *p;

	p = strchr(buf, '=');
	if (p)
		*p = '\0';

	if (kstrtoul(buf, 16, &par->debug_reg))
		return -EINVAL;

	if (par->debug_reg >= VISFX_FB_LENGTH)
		return -EINVAL;

	if (p) {
		if (kstrtoul(p+1, 16, &data))
			return -EINVAL;
		visfx_writel(info, par->debug_reg, data);
	}
	return count;
}

// static DEVICE_ATTR(reg, 0600, visfx_sysfs_show_reg, visfx_sysfs_store_reg);
static DEVICE_ATTR_ADMIN_RW(visfx_sysfs);


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
	return (((cmap->blue[color] & 0xff) << 0) |
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

	// B2_DBA_BIN8F | B2_DBA_OTC01 | B2_DBA_DIRECT | B2_DBA_D;  -> B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	// B2_DBA_BIN8I | B2_DBA_OTC04 | B2_DBA_DIRECT | B2_DBA_D;
	// visfx_writel(info, B2_DBA, par->dba | B2_DBA_S | B2_DBA_IND_BG_FG);
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
	visfx_writel(info, B2_BPM, 0xffffffff);
}

static void visfx_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct visfx_par *par = info->par;
	int x, y;

	if (info->var.bits_per_pixel == 8)
		return cfb_imageblit(info, image);

	visfx_wait_write_pipe_empty(info);

	switch (image->depth) {
	case 1:
		visfx_imageblit_mono(info, image->data, image->dx, image->dy,
				     image->width, image->height,
				     image->fg_color, image->bg_color);
		break;
	case 8:
		visfx_writel(info, B2_DBA, B2_DBA_OTC01 | B2_DBA_DIRECT);
		visfx_writel(info, B2_BPM, 0xffffffff);
		// visfx_wclip(info, 0, 0, info->var.xres, info->var.yres);

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

	visfx_writel(info, B2_DBA, par->dba);
}

static void visfx_fillrect(struct fb_info *info, const struct fb_fillrect *fr)
{
	struct visfx_par *par = info->par;

	visfx_wait_write_pipe_empty(info);

	visfx_writel(info, B2_DBA, B2_DBA_OTC(5) | B2_DBA_S | B2_DBA_IND_BG_FG);
	visfx_set_bmove_color(info, fr->color, 0);
	visfx_writel(info, B2_MNOOP_R0R1, (fr->dx << 16) | fr->dy);
	visfx_writel(info, B2_SOLIDFILL_R2R3, (fr->width << 16) | fr->height);

	visfx_writel(info, B2_DBA, par->dba);
}

static int visfx_setcmap(struct fb_cmap *cmap, struct fb_info *info)
{
	unsigned int i;

	if (info->fix.visual != FB_VISUAL_PSEUDOCOLOR)
		return -EINVAL;

// printk("visfx_setcmap start %d  len = %d\n", cmap->start, cmap->len);
	visfx_writel(info, B2_LLCA, cmap->start);

	for (i = 0; i < cmap->len; i++) {
		u32 rgb = visfx_cmap_entry(cmap, i);

		visfx_writel(info, B2_LUTD, rgb);
		if (cmap->start + i < NR_PALETTE)
			((u32*)info->pseudo_palette)[cmap->start + i] = rgb;
	}

	// visfx_writel(info, B2_PMASK, 0xffffffff);
	visfx_writel(info, B2_CP, 0);

	return 0;
}

/**
 * visfx_blank() - blank or unblank the given window
 * @blank_mode: The blank state from FB_BLANK_*
 * @info: The framebuffer to blank.
 *
 * Framebuffer layer request to change the power state.
 */
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
// printk("visfx_set_par %dx%d-%d\n", xres, yres, var->bits_per_pixel);

	ret = visfx_set_pll(info, B2_PLL_DOT_CTL, var->pixclock); // 00527b0c
	if (ret)
		return ret;

	visfx_writel_dump(info, B2_HTG, (hbp << 20) | (hsw << 12) | (0xc << 8) | hfp, 0x14f33c7f);
	visfx_writel_dump(info, B2_VTG, (vbp << 0) | (vsw << 8) | (vfp << 16), 0x260200); // vertauscht!!
	visfx_writel_dump(info, B2_VHAL,((yres - 1) << 16) | (xres - 1), 0x437077f);
	visfx_writel_dump(info, B2_ETG, 0x12260, 0);
	visfx_writel(info, B2_SCR, 0);
	visfx_wclip(info, 0, 0, xres, yres);

	tmp = VISFX_DFP_ENABLE;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		tmp |= VISFX_HSYNC_POSITIVE;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		tmp |= VISFX_VSYNC_POSITIVE;
	visfx_writel_dump(info, B2_CFG, tmp, 0);
	visfx_writel(info, B2_MPC, 0xc);
	// visfx_get_video_mode(info);

	visfx_setup(info);

	return 0;
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
// printk("visfx_check_var  bpp %d\n", var->bits_per_pixel);
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

static const struct fb_ops visfx_ops = {
	.owner		= THIS_MODULE,
	.fb_setcmap	= visfx_setcmap,
	.fb_fillrect	= visfx_fillrect,
        // .fb_copyarea    = stifb_copyarea,
	.fb_imageblit	= visfx_imageblit,
	.fb_set_par	= visfx_set_par,
	.fb_blank	= visfx_blank,
	.fb_check_var	= visfx_check_var,
	// .fb_cursor	= visfx_cursor,
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
	visfx_writel(info, B2_CFG, visfx_readl(info, 0x50040)); // 0 ??

	tmp = (visfx_readl(info, 0x50024) & 0xffff0000) |
		(visfx_readl(info, 0x50020) & 0xffff);

	visfx_writel(info, B2_VHAL, tmp - 0x10001);

	visfx_writel(info, B2_ETG, visfx_readl(info, 0x50044));
	visfx_writel(info, B2_SCR, visfx_readl(info, 0x5004c));
	tmp = visfx_readl(info, 0x50054);
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
	visfx_writel(info, B2_SOLIDFILL_R2R3_REMAP, var->xres<<16 | var->yres); // 0x06400240);
}

static void visfx_setup(struct fb_info *info)
{
	struct visfx_par *par = info->par;
	struct fb_var_screeninfo *var = &info->var;
	int i;

	visfx_wait_write_pipe_empty(info);

#ifdef __BIG_ENDIAN
	// SEite 238
	if (1 || var->bits_per_pixel == 32) {
		visfx_writel(info, B2_DMA_BSCFB, DSM_NO_BYTE_SWAP);
		visfx_writel(info, B2_PDU_BSCFB, DSM_NO_BYTE_SWAP);
	} else {
		visfx_writel(info, B2_DMA_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
		visfx_writel(info, B2_PDU_BSCFB, DSM_PDU_BYTE_SWAP_DEFAULT);
	}
	visfx_writel(info, B2_MFU_BSCTD, DSM_NO_BYTE_SWAP);
	visfx_writel(info, B2_MFU_BSCCTL, DSM_PDU_BYTE_SWAP_DEFAULT);
#if 0
#define DSM_NO_BYTE_SWAP                        0xe4e4e4e4
#define DSM_RGBA_TO_ARGB_BYTE_SWAP              0x39393939
#define DSM_ARGB_TO_RGBA_BYTE_SWAP              0x93939393
#define DSM_PDU_BYTE_SWAP_DEFAULT               0x1b1b1b1b
#define DSM_DMA_BYTE_SWAP_DEFAULT               0x1b1b1b1b
#define DSM_MFU_BYTE_SWAP_DEFAULT               0x1b1b1b1b
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
	visfx_writel(info, B2_ZBO, 0x00080000); // oder 0x66666666 oder 0x3010030f
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
	par->ibmap1 = 0x27e00002; // 0x27d00e02

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
	visfx_clear_buffer(info, 0x05000880, par->obmap0, 0x00fc0000, 0);
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

	visfx_clear_buffer(info, 0x00000a00, par->ibmap0, 0x03f00000, 0);
	visfx_clear_buffer(info, 0x05000880, par->obmap0, 0x00fc0000, 0xffffffff);
	visfx_buffer_setup(info, 2, 0x08000084, 0, 0);  // 8000084 für Overlay

	visfx_wait_write_pipe_empty(info);
	visfx_writel(info, B2_BMAP_DBA, par->ibmap0);

	info->fix.accel = FB_ACCEL_NONE;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.line_length = 2048 * var->bits_per_pixel / 8;
	info->fix.smem_len = PAGE_ALIGN(info->fix.line_length * var->yres);

	switch (var->bits_per_pixel) {
	case 32:
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
	default:
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

	var->grayscale = 0;
	var->xres_virtual = var->xres;
	var->yres_virtual = var->yres;
	visfx_wclip(info, 0, 0, var->xres, var->yres);

	/* set DBA */
	if (info->var.bits_per_pixel == 32) {
		visfx_writel(info, B2_EN2D, B2_EN2D_WORD_MODE);
		par->dba = B2_DBA_BIN8F | B2_DBA_OTC01 | B2_DBA_DIRECT | B2_DBA_D;
		// visfx_writel(info, B2_SBA, B2_DBA_BIN8F | B2_DBA_OTC01);
		par->bmap_dba = par->ibmap0;
		visfx_writel(info, B2_OTR, 1<<16 | 1<<8 | 0); // ???  Seite 382, Overlay immer durchsichtig !!
		visfx_writel(info, B2_FATTR, 0);
	} else { /* 8-bit indexed: */
		visfx_writel(info, B2_EN2D, B2_EN2D_BYTE_MODE);
		// visfx_writel(info, B2_SBA, B2_DBA_BIN8I | B2_DBA_OTC04);
		par->dba = B2_DBA_BIN8I | B2_DBA_OTC04 | B2_DBA_DIRECT | B2_DBA_D;  // doch OCT01 ???
		visfx_writel(info, B2_BPM, 0xffffffff);
		// visfx_writel(info, B2_OTR, 1<<16 | 3); // ???  Seite 382, 3=immer undurchsichtig !!
		visfx_writel(info, B2_OTR, 2 ); // ???  Seite 382, 3=immer undurchsichtig !!
		// IAA0 -> Seite 383 -> 
		visfx_writel(info, B2_CFS16, 0x40); // mode=4, LUT=3 LUT auswählen
		// visfx_writel(info, B2_CFS16, 0x0); // mode=4, LUT=3 LUT auswählen // Seite 396
		// visfx_writel(info, B2_FATTR, 1<<7 | 1<<4); // -> CFS16  -> force Overlay!
		// visfx_writel(info, B2_FATTR, 0);
		visfx_writel(info, B2_FATTR, 1<<7 | 1<<4); // -> CFS16  -> force Overlay!
		par->bmap_dba = par->obmap0;
	}
// con2fbmap 1 1
// fbset  -fb /dev/fb1 1280x1024-60 -depth 8

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

static int __init visfx_check_defaults(struct fb_info *info)
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


/* DDC support */

#define SCL_PIN		BIT(3)
#define SDA_PIN		BIT(2)
#define SCL_REG		BIT(1)
#define SDA_REG		BIT(0)
#define DDC_MASK	(SCL_REG | SDA_REG);

static void visfx_setscl(void *data, int state)
{
	struct visfx_par *par = data;
	u32 val;

	val = visfx_readl(par->info, B2_DDC) & DDC_MASK;
	if (state)
		val |= SCL_REG;
	else
		val &= ~SCL_REG;
	visfx_writel(par->info, B2_DDC, val);
}

static void visfx_setsda(void *data, int state)
{
	struct visfx_par *par = data;
	u32 val;

	val = visfx_readl(par->info, B2_DDC) & DDC_MASK;
	if (state)
		val |= SDA_REG;
	else
		val &= ~SDA_REG;
	visfx_writel(par->info, B2_DDC, val);
}

static int visfx_getscl(void *data)
{
	struct visfx_par *par = data;
	u32 val = 0;

	val = visfx_readl(par->info, B2_DDC);
	val = (val & SCL_PIN) ? 1 : 0;
//  printk("scl = %d\n", val);

	return val;
}

static int visfx_getsda(void *data)
{
	struct visfx_par *par = data;
	u32 val = 0;

	val = visfx_readl(par->info, B2_DDC);
	val = (val & SDA_PIN) ? 1 : 0;
// printk("sda = %d\n", val);

	return val;
}

static int visfx_setup_i2c_bus(struct visfx_par *par,
				unsigned int i2c_class)
{
	int rc;
	static const char name[] = "VisFX-DDC";

	strscpy(par->adapter.name, name, sizeof(par->adapter.name));
	par->adapter.owner = THIS_MODULE;
	par->adapter.class = i2c_class;
	par->adapter.algo_data = &par->algo;
	par->adapter.dev.parent = par->dev;
	par->algo.setsda = visfx_setsda;
	par->algo.setscl = visfx_setscl;
	par->algo.getsda = visfx_getsda;
	par->algo.getscl = visfx_getscl;
	par->algo.udelay = 10;
	par->algo.timeout = msecs_to_jiffies(2);
	par->algo.data = par;

	i2c_set_adapdata(&par->adapter, par);

	/* Raise SCL and SDA */
	visfx_setsda(par, 1);
	visfx_setscl(par, 1);
	udelay(20);

	rc = i2c_bit_add_bus(&par->adapter);
	if (rc == 0)
		dev_dbg(par->dev,
			"I2C bus %s registered.\n", name);
	else {
		dev_warn(par->dev,
			 "Failed to register I2C bus %s.\n", name);
	}

	return rc;
}

static void __init visfx_create_i2c_busses(struct visfx_par *par)
{
	visfx_setup_i2c_bus(par, I2C_CLASS_DDC);
}

static void visfx_delete_i2c_busses(struct visfx_par *par)
{
	i2c_del_adapter(&par->adapter);
}

static int __init visfx_probe_i2c_connector(struct fb_info *info, u8 **out_edid)
{
	struct visfx_par *par = info->par;
	u8 *edid;

	edid = fb_ddc_read(&par->adapter);
// printk("edid1 %px    %*ph\n", edid, 20, edid);

	*out_edid = edid;

	return (edid) ? 0 : 1;
}

static void __init visfx_find_init_mode(struct fb_info *info)
{
	struct fb_videomode mode;
	struct fb_var_screeninfo var;
	struct fb_monspecs *specs = &info->monspecs;
	int found = 0;
	struct visfx_par *par = info->par;

	INIT_LIST_HEAD(&info->modelist);
	memset(&mode, 0, sizeof(struct fb_videomode));
	var = info->var;

	visfx_create_i2c_busses(par);

	if (!visfx_probe_i2c_connector(info, &par->edid))
		printk("visfxfb_init_pci: DDC probe successful\n");
printk("bbbbbbb  par->edid = %px\n", par->edid);

	fb_edid_to_monspecs(par->edid, specs);

printk("Monitor:  %s\n", specs->monitor);

	if (specs->modedb == NULL)
		printk("visfxfb_init_pci: Unable to get Mode Database\n");

	fb_videomode_to_modelist(specs->modedb, specs->modedb_len,
				 &info->modelist);
	if (specs->modedb != NULL) {
		const struct fb_videomode *m;

		if (1) {   // xres && yres) {
			if ((m = fb_find_best_mode(&var, &info->modelist))) {
				mode = *m;
				found  = 1;
			}
		}

		if (!found) {
			m = fb_find_best_display(&info->monspecs, &info->modelist);
			mode = *m;
			found = 1;
		}

		fb_videomode_to_var(&var, &mode);
	}

if (!mode_option)
		mode_option="1366x768";
	if (mode_option)
		fb_find_mode(&var, info, mode_option, specs->modedb,
			     specs->modedb_len, (found) ? &mode : NULL,
			     info->var.bits_per_pixel);

	info->var = var;
	fb_destroy_modedb(specs->modedb);
	specs->modedb = NULL;
}

/* setup */

static void __init visfx_setup_info(struct pci_dev *pdev, struct fb_info *info)
{
	struct visfx_par *par = info->par;

	info->fbops = &visfx_ops;
	info->flags = FBINFO_HWACCEL_FILLRECT | FBINFO_HWACCEL_IMAGEBLIT;
		/* TODO: FBINFO_HWACCEL_COPYAREA */
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
	int ret;

	info = framebuffer_alloc(sizeof(struct visfx_par), &pdev->dev);
	if (!info)
		return -ENOMEM;

	pci_set_drvdata(pdev, info);
	if (sti)
		sti->info = info;

	par = info->par;
	par->info = info;

	if (sti)
		par->reg_base = pci_iomap(pdev, 0, VISFX_FB_OFFSET + VISFX_FB_LENGTH);
	else
		par->reg_base = pcim_iomap_table(pdev)[0];

	par->reg_size = pci_resource_len(pdev, 0);
	par->dev = &pdev->dev;

	ret = visfx_check_defaults(info);
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

printk("MODE OPTION %s\n", mode_option);
	visfx_setup(info);
	visfx_find_init_mode(info);
	if (visfx_check_var(&info->var, info) == 0)
		visfx_set_par(info);
	else
#if 0
	if (mode_option &&
	    fb_find_mode(&info->var, info, mode_option, NULL, 0, NULL, DEFAULT_BPP) &&
#endif

	visfx_get_video_mode(info);
	info->var.accel_flags = info->flags;

	ret = fb_alloc_cmap(&info->cmap, NR_PALETTE, 0);
	if (ret)
		goto err_out_free;

	ret = register_framebuffer(info);
	if (ret)
		goto err_out_dealloc_cmap;

        fb_info(info, "visfxfb %dx%d-%d frame buffer device, %s, mmio: 0x%04lx\n",
                info->var.xres,
                info->var.yres,
                info->var.bits_per_pixel,
                info->fix.id,
                info->fix.mmio_start);

	if (SYSFS && device_create_file(&pdev->dev, &dev_attr_visfx_sysfs))
		dev_err(&pdev->dev, "Can't create sysfs regdump file\n");

	return 0;

err_out_dealloc_cmap:
	fb_dealloc_cmap(&info->cmap);
err_out_free:
	visfx_delete_i2c_busses(par);
	kfree(par->edid);
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
		pci_iounmap(sti->pd, par->reg_base);
		fb_dealloc_cmap(&info->cmap);
		unregister_framebuffer(info);
		framebuffer_release(info);
		if (SYSFS) device_remove_file(info->dev, &dev_attr_visfx_sysfs);
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
	struct visfx_par *par = info->par;

	visfx_delete_i2c_busses(par);
	unregister_framebuffer(info);
	fb_dealloc_cmap(&info->cmap);
	kfree(par->edid);
	framebuffer_release(info);
	if (SYSFS) device_remove_file(&pdev->dev, &dev_attr_visfx_sysfs);
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
