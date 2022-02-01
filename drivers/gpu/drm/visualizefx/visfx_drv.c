// SPDX-License-Identifier: GPL-2.0+
/*
 * Framebuffer driver for Visualize FX cards commonly found in PA-RISC machines
 *
 * Copyright (c) 2021 Sven Schnelle <svens@stackframe.org>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/rational.h>

#include <drm/drm_drv.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_format_helper.h>

#define VISFX_VRAM_ENDIANESS_WRITE	0xa4303c
#define VISFX_VRAM_ENDIANESS_READ	0xaa0408
#define VISFX_VRAM_ENDIANESS_BIG	0xe4e4e4e4
#define VISFX_VRAM_ENDIANESS_LITTLE	0x1b1b1b1b

#define VISFX_STATUS			0x641400

#define VISFX_COLOR_MASK		0x800018
#define VISFX_COLOR_INDEX		0x800020
#define VISFX_COLOR_VALUE		0x800024

#define VISFX_SYNC_POLARITY		0x800044
#define VISFX_SYNC_VISIBLE_SIZE		0x80005c
#define VISFX_SYNC_HORIZ_CONF		0x800060
#define VISFX_SYNC_VERT_CONF		0x800068
#define VISFX_SYNC_MASTER_PLL		0x8000a0
#define VISFX_SYNC_PLL_STATUS		0x8000b8

#define VISFX_VRAM_WRITE_MODE		0xa00808
#define VISFX_VRAM_MASK			0xa0082c
#define VISFX_FGCOLOR			0xa0083c
#define VISFX_BGCOLOR			0xa00844
#define VISFX_WRITE_MASK		0xa0084c
#define VISFX_VRAM_WRITE_DATA_INCRX	0xa60000
#define VISFX_VRAM_WRITE_DATA_INCRY	0xa68000
#define VISFX_SCREEN_SIZE		0xac1054
#define VISFX_VRAM_WRITE_DEST		0xac1000

#define VISFX_START			0xb3c000
#define VISFX_SIZE			0xb3c808
#define VISFX_HEIGHT			0xb3c008
#define VISFX_DST			0xb3cc00

#define VISFX_DFP_ENABLE		0x10000
#define VISFX_HSYNC_POSITIVE		0x40000
#define VISFX_VSYNC_POSITIVE		0x80000

#define VISFX_SYNC_PLL_BASE		20250 /* 20.25MHz in Khz */

#define VISFX_CURSOR_POS		0x400000
#define VISFX_CURSOR_INDEX		0x400004
#define VISFX_CURSOR_DATA		0x400008
#define VISFX_CURSOR_COLOR		0x400010
#define VISFX_CURSOR_ENABLE		0x80000000

#define VISFX_VRAM_WRITE_MODE_BITMAP	0x02000000
#define VISFX_VRAM_WRITE_MODE_COLOR	0x050004c0
#define VISFX_VRAM_WRITE_MODE_FILL	0x05000080

#define VISFX_FB_LENGTH			0x01000000
#define VISFX_FB_OFFSET			0x01000000
#define NR_PALETTE 256

struct visfx_device {
	struct drm_device		dev;
	struct drm_connector		conn;
	struct drm_simple_display_pipe	pipe;
	unsigned long			debug_reg;
	void __iomem			*mmio;
	unsigned long			mmio_size;
	unsigned int			cpp;
};

#define to_visfx(_dev) container_of(_dev, struct visfx_device, dev)

static u32 visfx_readl(struct visfx_device *dev, int reg)
{
	return le32_to_cpu(readl(dev->mmio + reg));
}

static void visfx_writel(struct visfx_device *dev, int reg, u32 val)
{
	return writel(cpu_to_le32(val), dev->mmio + reg);
}

static ssize_t visfx_sysfs_show_reg(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct visfx_device *visfx = pci_get_drvdata(container_of(dev, struct pci_dev, dev));

	return sprintf(buf, "%08x\n", visfx_readl(visfx, visfx->debug_reg));
}

static ssize_t visfx_sysfs_store_reg(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct visfx_device *visfx = pci_get_drvdata(container_of(dev, struct pci_dev, dev));
	unsigned long data;
	char *p;

	p = strchr(buf, '=');
	if (p)
		*p = '\0';

	if (kstrtoul(buf, 16, &visfx->debug_reg))
		return -EINVAL;

	if (visfx->debug_reg > visfx->mmio_size)
		return -EINVAL;

	if (p) {
		if (kstrtoul(p+1, 16, &data))
			return -EINVAL;
		visfx_writel(visfx, visfx->debug_reg, data);
	}
	return count;
}

static DEVICE_ATTR(reg, 0600, visfx_sysfs_show_reg, visfx_sysfs_store_reg);

DEFINE_DRM_GEM_FOPS(visfx_drm_fops);

static int visfx_dumb_create(struct drm_file *file, struct drm_device *dev,
			     struct drm_mode_create_dumb *args)
{
	args->pitch = 2048;
	args->size = PAGE_ALIGN(args->pitch * args->height);
	return drm_gem_shmem_dumb_create(file, dev, args);
}

static const struct drm_driver visfx_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM | DRIVER_ATOMIC,
	.name				= "visualizefx",
	.desc				= "DRM driver for HP Visualize FX cards",
	.date				= "01.11.2021",
	.major				= 1,
	.minor				= 0,
	.fops				= &visfx_drm_fops,
	.prime_handle_to_fd		= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle		= drm_gem_prime_fd_to_handle,
	.gem_prime_import_sg_table	= drm_gem_shmem_prime_import_sg_table,
	.gem_prime_mmap			= drm_gem_prime_mmap,
	.dumb_create			= visfx_dumb_create
};

static void visfx_set_pll(struct visfx_device *dev, unsigned long clock)
{
	unsigned long n, d;
	u32 tmp;

	rational_best_approximation(VISFX_SYNC_PLL_BASE, clock, 0x3f, 0x3f, &n, &d);
	tmp = (((d * 4) - 1) << 8) | ((n * 4) - 1);
	visfx_writel(dev, VISFX_SYNC_MASTER_PLL, 0x520000 | tmp);
	while (visfx_readl(dev, VISFX_SYNC_PLL_STATUS) & 0xffffff)
		udelay(10);
	visfx_writel(dev, VISFX_SYNC_MASTER_PLL, 0x530000 | tmp);
	while (visfx_readl(dev, VISFX_SYNC_PLL_STATUS) & 0xffffff)
		udelay(10);
}

static int visfx_mode_set(struct visfx_device *visfx,
			  struct drm_display_mode *mode,
			  struct drm_framebuffer *fb)
{
	u32 xres, yres, hbp, hsw, hfp, vbp, vsw, vfp, tmp;
	int idx;

	if (!drm_dev_enter(&visfx->dev, &idx))
		return -1;

	xres = mode->hdisplay;
	yres = mode->vdisplay;

	hsw = (mode->hsync_end - mode->hsync_start) / 4 - 1;
	hfp = (mode->hsync_start - mode->hdisplay) - 1;
	hbp = (mode->htotal - mode->hsync_end) - 1;
	vsw = (mode->vsync_end - mode->vsync_start) - 1;
	vfp = (mode->vsync_start - mode->vdisplay) - 1;
	vbp = (mode->vtotal - mode->vsync_end) - 1;

	visfx_set_pll(visfx, mode->clock);
	visfx_writel(visfx, VISFX_SYNC_VISIBLE_SIZE, ((yres - 1) << 16) | (xres - 1));
	visfx_writel(visfx, VISFX_SYNC_HORIZ_CONF, (hbp << 20) | (hsw << 12) | (0xc << 8) | hfp);
	visfx_writel(visfx, VISFX_SYNC_VERT_CONF, (vbp << 16) | (vsw << 8) | vfp);
	visfx_writel(visfx, VISFX_SCREEN_SIZE, (xres << 16) | yres);

	tmp = VISFX_DFP_ENABLE;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		tmp |= VISFX_HSYNC_POSITIVE;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		tmp |= VISFX_VSYNC_POSITIVE;
	visfx_writel(visfx, VISFX_SYNC_POLARITY, tmp);
	visfx_writel(visfx, VISFX_VRAM_WRITE_MODE, VISFX_VRAM_WRITE_MODE_BITMAP);
	return 0;
}

static int visfx_fb_blit_rect(struct drm_framebuffer *fb,
			       const struct dma_buf_map *map,
			       struct drm_rect *rect)
{
	struct visfx_device *visfx = to_visfx(fb->dev);
	void *vmap = map->vaddr; /* TODO: Use mapping abstraction properly */
	uint8_t *dst;
	int idx;

	if (!drm_dev_enter(&visfx->dev, &idx))
		return -ENODEV;
	visfx_writel(visfx, VISFX_VRAM_WRITE_MODE, VISFX_VRAM_WRITE_MODE_BITMAP);
	dst = visfx->mmio + 0x01000000;
	dst += drm_fb_clip_offset(fb->pitches[0], fb->format, rect);
	drm_fb_memcpy_toio(dst, fb->pitches[0], vmap, fb, rect);
	drm_dev_exit(idx);
	return 0;
}

static int visfx_fb_blit_fullscreen(struct drm_framebuffer *fb, const struct dma_buf_map *map)
{
	struct drm_rect fullscreen = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	return visfx_fb_blit_rect(fb, map, &fullscreen);
}

static int visfx_check_size(int width, int height, struct drm_framebuffer *fb)
{
	if (width > 2048 || height > 2048)
		return -EINVAL;
	return 0;
}

static enum drm_mode_status visfx_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
						   const struct drm_display_mode *mode)
{
	if (visfx_check_size(mode->hdisplay, mode->vdisplay, NULL) < 0)
		return MODE_BAD;
	return MODE_OK;
}

static int visfx_pipe_check(struct drm_simple_display_pipe *pipe,
			     struct drm_plane_state *plane_state,
			     struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = plane_state->fb;

	if (!fb)
		return 0;
	return visfx_check_size(fb->width, fb->height, fb);
}

static void visfx_pipe_enable(struct drm_simple_display_pipe *pipe,
			      struct drm_crtc_state *crtc_state,
			      struct drm_plane_state *plane_state)
{
	struct visfx_device *visfx = to_visfx(pipe->crtc.dev);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);

	visfx_mode_set(visfx, &crtc_state->mode, plane_state->fb);
	visfx_fb_blit_fullscreen(plane_state->fb, &shadow_plane_state->data[0]);
}

static void visfx_color_set(struct visfx_device *visfx,
			    struct drm_crtc *crtc)
{
	struct drm_color_lut *lut = crtc->state->gamma_lut->data;
	int i;

	visfx_writel(visfx, VISFX_COLOR_INDEX, 0);

	for (i = 0; i < crtc->gamma_size; ++i) {
		u32 entry = drm_color_lut_extract(lut[i].red, 8) << 16
			  | drm_color_lut_extract(lut[i].green, 8) << 8
			  | drm_color_lut_extract(lut[i].blue, 8);

		visfx_writel(visfx, VISFX_COLOR_VALUE, entry);
	}

	visfx_writel(visfx, VISFX_COLOR_MASK, 0xff);
	visfx_writel(visfx, 0x80004c, 0xc);
	visfx_writel(visfx, 0x800000, 0);
}

static void visfx_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct visfx_device *visfx = to_visfx(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(state);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_rect rect;

	if (crtc->state->mode_changed)
		visfx_mode_set(visfx, &crtc->mode, state->fb);

	if (crtc->state->color_mgmt_changed)
		visfx_color_set(visfx, crtc);

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		visfx_fb_blit_rect(state->fb, &shadow_plane_state->data[0], &rect);
}

static const struct drm_simple_display_pipe_funcs visfx_pipe_funcs = {
	.mode_valid = visfx_pipe_mode_valid,
	.check	    = visfx_pipe_check,
	.enable	    = visfx_pipe_enable,
	.update	    = visfx_pipe_update,
	DRM_GEM_SIMPLE_DISPLAY_PIPE_SHADOW_PLANE_FUNCS,
};

static const uint32_t visfx_formats[] = {
	DRM_FORMAT_C8,
};

static const uint64_t visfx_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int visfx_pipe_init(struct visfx_device *visfx)
{

	int ret = drm_simple_display_pipe_init(&visfx->dev,
					    &visfx->pipe,
					    &visfx_pipe_funcs,
					    visfx_formats,
					    ARRAY_SIZE(visfx_formats),
					    visfx_modifiers,
					    &visfx->conn);
	drm_crtc_enable_color_mgmt(&visfx->pipe.crtc, 0, false, 256);
	drm_mode_crtc_set_gamma_size(&visfx->pipe.crtc, 256);
	return ret;
}

static struct drm_framebuffer*
visfx_fb_create(struct drm_device *dev, struct drm_file *file_priv,
		const struct drm_mode_fb_cmd2 *mode_cmd)
{
	if (mode_cmd->pixel_format != DRM_FORMAT_C8)
		return ERR_PTR(-EINVAL);
	if (visfx_check_size(mode_cmd->width, mode_cmd->height, NULL) < 0)
		return ERR_PTR(-EINVAL);
	return drm_gem_fb_create_with_dirty(dev, file_priv, mode_cmd);
}

static const struct drm_mode_config_funcs visfx_mode_config_funcs = {
	.fb_create = visfx_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int visfx_mode_config_init(struct visfx_device *visfx)
{
	struct drm_device *dev = &visfx->dev;
	int ret;

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;
	dev->mode_config.preferred_depth = 32;
	dev->mode_config.prefer_shadow = 0;
	dev->mode_config.funcs = &visfx_mode_config_funcs;
	return 0;
}

static int visfx_conn_get_modes(struct drm_connector *conn)
{
	int count;

	count = drm_add_modes_noedid(conn,
				     conn->dev->mode_config.max_width,
				     conn->dev->mode_config.max_height);
	drm_set_preferred_mode(conn, 1600, 1200);
	return count;
}

static const struct drm_connector_funcs visfx_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_connector_helper_funcs visfx_conn_helper_funcs = {
	.get_modes = visfx_conn_get_modes,
};

static int visfx_conn_init(struct visfx_device *visfx)
{
	drm_connector_helper_add(&visfx->conn, &visfx_conn_helper_funcs);
	return drm_connector_init(&visfx->dev, &visfx->conn,
				  &visfx_conn_funcs, DRM_MODE_CONNECTOR_VGA);

}

static void visfx_setup_unknown(struct visfx_device *visfx)
{
	visfx_writel(visfx, 0xb08044, 0x1b);
	visfx_writel(visfx, 0xb08048, 0x1b);
	visfx_writel(visfx, 0x920860, 0xe4);
	visfx_writel(visfx, 0xa00818, 0);
	visfx_writel(visfx, 0xa00404, 0);
	visfx_writel(visfx, 0x921110, 0);
	visfx_writel(visfx, 0x9211d8, 0);
	visfx_writel(visfx, 0xa0086c, 0);
	visfx_writel(visfx, 0x921114, 0);
	visfx_writel(visfx, 0xac1050, 0);
	visfx_writel(visfx, 0xa00858, 0xb0);

	visfx_writel(visfx, VISFX_VRAM_WRITE_MODE, VISFX_VRAM_WRITE_MODE_BITMAP);
	visfx_writel(visfx, VISFX_WRITE_MASK, 0xffffffff);
	visfx_writel(visfx, VISFX_VRAM_MASK, 0xffffffff);
#ifdef __BIG_ENDIAN
	visfx_writel(visfx, VISFX_VRAM_ENDIANESS_READ, VISFX_VRAM_ENDIANESS_BIG);
	visfx_writel(visfx, VISFX_VRAM_ENDIANESS_WRITE, VISFX_VRAM_ENDIANESS_BIG);
#else
	visfx_writel(visfx, VISFX_VRAM_ENDIANESS_READ, VISFX_VRAM_ENDIANESS_LITTLE);
	visfx_writel(visfx, VISFX_VRAM_ENDIANESS_WRITE, VISFX_VRAM_ENDIANESS_LITTLE);
#endif
}

static int visfx_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct drm_device *dev;
	struct visfx_device *visfx;
	int ret;

	visfx = devm_drm_dev_alloc(&pdev->dev, &visfx_drm_driver,
				   struct visfx_device, dev);
	if (!visfx)
		return -ENOMEM;

	dev = &visfx->dev;

	ret = pci_enable_device(pdev);
	if (ret)
		goto err_out_free;

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret)
		goto err_out_disable;

	visfx->mmio = pci_ioremap_bar(pdev, 0);
	visfx->mmio_size = pci_resource_len(pdev, 0);

	if (!visfx->mmio) {
		ret = -ENOMEM;
		goto err_out_release;
	}

	pci_set_drvdata(pdev, dev);

	visfx_setup_unknown(visfx);

	ret = device_create_file(&pdev->dev, &dev_attr_reg);
	if (ret)
		goto err_out_iounmap;

	ret = visfx_mode_config_init(visfx);
	if (ret)
		goto err_out_remove;

	ret = visfx_conn_init(visfx);
	if (ret)
		goto err_out_remove;

	ret = visfx_pipe_init(visfx);
	if (ret)
		goto err_out_remove;

	drm_mode_config_reset(dev);
	ret = drm_dev_register(dev, 0);
	if (ret)
		goto err_out_remove;
	drm_fbdev_generic_setup(dev, 8);
	return 0;

err_out_remove:
	device_remove_file(&pdev->dev, &dev_attr_reg);
err_out_iounmap:
	pci_iounmap(pdev, visfx->mmio);
err_out_release:
	pci_release_regions(pdev);
err_out_disable:
	pci_disable_device(pdev);
err_out_free:
	drm_dev_unplug(dev); // TODO: check
	return ret;
}

static void __exit visfx_pci_remove(struct pci_dev *pdev)
{
	struct visfx_device *dev = pci_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_reg);
	drm_dev_unplug(&dev->dev);
	pci_iounmap(pdev, dev->mmio);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static const struct pci_device_id visfx_pci_tbl[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_HP, 0x1008) },
	{ 0 },
};
MODULE_DEVICE_TABLE(pci, visfx_pci_tbl);

static struct pci_driver visfx_pci_driver = {
	.name      = KBUILD_MODNAME,
	.id_table  = visfx_pci_tbl,
	.probe     = visfx_pci_probe,
	.remove    = visfx_pci_remove,
};

module_pci_driver(visfx_pci_driver);

MODULE_AUTHOR("Sven Schnelle <svens@stackframe.org>");
MODULE_DESCRIPTION("DRM driver for HP Visualize FX cards");
MODULE_LICENSE("GPL");
