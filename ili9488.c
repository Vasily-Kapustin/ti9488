// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Ilitek ILI9488 panels
 *
 * Copyright 2023 VASILY KAPUSTIN <vasilykap@live.com>
 *
 * Based on mi0283qt.c:
 * Copyright 2016 Noralf Trønnes
 * Based on ili9488.c:
 * Copyright 2019 Bird Techstep
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_modeset_helper.h>
#include <video/mipi_display.h>

/* Level 1 Commands (from the display Datasheet) */
#define ILI9488_CMD_NOP					0x00
#define ILI9488_CMD_SOFTWARE_RESET			0x01
#define ILI9488_CMD_READ_DISP_ID			0x04
#define ILI9488_CMD_READ_ERROR_DSI			0x05
#define ILI9488_CMD_READ_DISP_STATUS			0x09
#define ILI9488_CMD_READ_DISP_POWER_MODE		0x0A
#define ILI9488_CMD_READ_DISP_MADCTRL			0x0B
#define ILI9488_CMD_READ_DISP_PIXEL_FORMAT		0x0C
#define ILI9488_CMD_READ_DISP_IMAGE_MODE		0x0D
#define ILI9488_CMD_READ_DISP_SIGNAL_MODE		0x0E
#define ILI9488_CMD_READ_DISP_SELF_DIAGNOSTIC		0x0F
#define ILI9488_CMD_ENTER_SLEEP_MODE			0x10
#define ILI9488_CMD_SLEEP_OUT				0x11
#define ILI9488_CMD_PARTIAL_MODE_ON			0x12
#define ILI9488_CMD_NORMAL_DISP_MODE_ON			0x13
#define ILI9488_CMD_DISP_INVERSION_OFF			0x20
#define ILI9488_CMD_DISP_INVERSION_ON			0x21
#define ILI9488_CMD_PIXEL_OFF				0x22
#define ILI9488_CMD_PIXEL_ON				0x23
#define ILI9488_CMD_DISPLAY_OFF				0x28
#define ILI9488_CMD_DISPLAY_ON				0x29
#define ILI9488_CMD_COLUMN_ADDRESS_SET			0x2A
#define ILI9488_CMD_PAGE_ADDRESS_SET			0x2B
#define ILI9488_CMD_MEMORY_WRITE			0x2C
#define ILI9488_CMD_MEMORY_READ				0x2E
#define ILI9488_CMD_PARTIAL_AREA			0x30
#define ILI9488_CMD_VERT_SCROLL_DEFINITION		0x33
#define ILI9488_CMD_TEARING_EFFECT_LINE_OFF		0x34
#define ILI9488_CMD_TEARING_EFFECT_LINE_ON		0x35
#define ILI9488_CMD_MEMORY_ACCESS_CONTROL		0x36
#define ILI9488_CMD_VERT_SCROLL_START_ADDRESS		0x37
#define ILI9488_CMD_IDLE_MODE_OFF			0x38
#define ILI9488_CMD_IDLE_MODE_ON			0x39
#define ILI9488_CMD_COLMOD_PIXEL_FORMAT_SET		0x3A
#define ILI9488_CMD_WRITE_MEMORY_CONTINUE		0x3C
#define ILI9488_CMD_READ_MEMORY_CONTINUE		0x3E
#define ILI9488_CMD_SET_TEAR_SCANLINE			0x44
#define ILI9488_CMD_GET_SCANLINE			0x45
#define ILI9488_CMD_WRITE_DISPLAY_BRIGHTNESS		0x51
#define ILI9488_CMD_READ_DISPLAY_BRIGHTNESS		0x52
#define ILI9488_CMD_WRITE_CTRL_DISPLAY			0x53
#define ILI9488_CMD_READ_CTRL_DISPLAY			0x54
#define ILI9488_CMD_WRITE_CONTENT_ADAPT_BRIGHTNESS	0x55
#define ILI9488_CMD_READ_CONTENT_ADAPT_BRIGHTNESS	0x56
#define ILI9488_CMD_WRITE_MIN_CAB_LEVEL			0x5E
#define ILI9488_CMD_READ_MIN_CAB_LEVEL			0x5F
#define ILI9488_CMD_READ_ABC_SELF_DIAG_RES		0x68
#define ILI9488_CMD_READ_ID1				0xDA
#define ILI9488_CMD_READ_ID2				0xDB
#define ILI9488_CMD_READ_ID3				0xDC

/* Level 2 Commands (from the display Datasheet) */
#define ILI9488_CMD_INTERFACE_MODE_CONTROL		0xB0
#define ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL		0xB1
#define ILI9488_CMD_FRAME_RATE_CONTROL_IDLE_8COLOR	0xB2
#define ILI9488_CMD_FRAME_RATE_CONTROL_PARTIAL		0xB3
#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL		0xB4
#define ILI9488_CMD_BLANKING_PORCH_CONTROL		0xB5
#define ILI9488_CMD_DISPLAY_FUNCTION_CONTROL		0xB6
#define ILI9488_CMD_ENTRY_MODE_SET			0xB7
#define ILI9488_CMD_BACKLIGHT_CONTROL_1			0xB9
#define ILI9488_CMD_BACKLIGHT_CONTROL_2			0xBA
#define ILI9488_CMD_HS_LANES_CONTROL			0xBE
#define ILI9488_CMD_POWER_CONTROL_1			0xC0
#define ILI9488_CMD_POWER_CONTROL_2			0xC1
#define ILI9488_CMD_POWER_CONTROL_NORMAL_3		0xC2
#define ILI9488_CMD_POWER_CONTROL_IDEL_4		0xC3
#define ILI9488_CMD_POWER_CONTROL_PARTIAL_5		0xC4
#define ILI9488_CMD_VCOM_CONTROL_1			0xC5
#define ILI9488_CMD_CABC_CONTROL_1			0xC6
#define ILI9488_CMD_CABC_CONTROL_2			0xC8
#define ILI9488_CMD_CABC_CONTROL_3			0xC9
#define ILI9488_CMD_CABC_CONTROL_4			0xCA
#define ILI9488_CMD_CABC_CONTROL_5			0xCB
#define ILI9488_CMD_CABC_CONTROL_6			0xCC
#define ILI9488_CMD_CABC_CONTROL_7			0xCD
#define ILI9488_CMD_CABC_CONTROL_8			0xCE
#define ILI9488_CMD_CABC_CONTROL_9			0xCF
#define ILI9488_CMD_NVMEM_WRITE				0xD0
#define ILI9488_CMD_NVMEM_PROTECTION_KEY		0xD1
#define ILI9488_CMD_NVMEM_STATUS_READ			0xD2
#define ILI9488_CMD_READ_ID4				0xD3
#define ILI9488_CMD_ADJUST_CONTROL_1			0xD7
#define ILI9488_CMD_READ_ID_VERSION			0xD8
#define ILI9488_CMD_POSITIVE_GAMMA_CORRECTION		0xE0
#define ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION		0xE1
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_1		0xE2
#define ILI9488_CMD_DIGITAL_GAMMA_CONTROL_2		0xE3
#define ILI9488_CMD_SET_IMAGE_FUNCTION			0xE9
#define ILI9488_CMD_ADJUST_CONTROL_2			0xF2
#define ILI9488_CMD_ADJUST_CONTROL_3			0xF7
#define ILI9488_CMD_ADJUST_CONTROL_4			0xF8
#define ILI9488_CMD_ADJUST_CONTROL_5			0xF9
#define ILI9488_CMD_SPI_READ_SETTINGS			0xFB
#define ILI9488_CMD_ADJUST_CONTROL_6			0xFC
#define ILI9488_CMD_ADJUST_CONTROL_7			0xFF

#define ILI9488_MADCTL_BGR	BIT(3)
#define ILI9488_MADCTL_MV	BIT(5)
#define ILI9488_MADCTL_MX	BIT(6)
#define ILI9488_MADCTL_MY	BIT(7)


static void mipi_dbi18_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect);
		
int mipi_dbi18_buf_copy( void *dst, struct drm_framebuffer *fb, struct drm_rect *clip, bool swap);

static const uint32_t mipi_dbi_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_XRGB8888
};

static void mipi_dbi_set_window_address(struct mipi_dbi_dev *dbidev,
					unsigned int xs, unsigned int xe,
					unsigned int ys, unsigned int ye)
{
	struct mipi_dbi *dbi = &dbidev->dbi;

	xs += dbidev->left_offset;
	xe += dbidev->left_offset;
	ys += dbidev->top_offset;
	ye += dbidev->top_offset;

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS, (xs >> 8) & 0xff,
			 xs & 0xff, (xe >> 8) & 0xff, xe & 0xff);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS, (ys >> 8) & 0xff,
			 ys & 0xff, (ye >> 8) & 0xff, ye & 0xff);
}

static unsigned int clip_offset(const struct drm_rect *clip, unsigned int pitch, unsigned int cpp)
{
	return clip->y1 * pitch + clip->x1 * cpp;
}

static int __drm_fb_xfrm(void *dst, unsigned long dst_pitch, unsigned long dst_pixsize,
			 const void *vaddr, const struct drm_framebuffer *fb,
			 const struct drm_rect *clip, bool vaddr_cached_hint,
			 void (*xfrm_line)(void *dbuf, const void *sbuf, unsigned int npixels))
{
	unsigned long linepixels = drm_rect_width(clip);
	unsigned long lines = drm_rect_height(clip);
	size_t sbuf_len = linepixels * fb->format->cpp[0];
	void *stmp = NULL;
	unsigned long i;
	const void *sbuf;

	/*
	 * Some source buffers, such as DMA memory, use write-combine
	 * caching, so reads are uncached. Speed up access by fetching
	 * one line at a time.
	 */
	if (!vaddr_cached_hint) {
		stmp = kmalloc(sbuf_len, GFP_KERNEL);
		if (!stmp)
			return -ENOMEM;
	}

	if (!dst_pitch)
		dst_pitch = drm_rect_width(clip) * dst_pixsize;
	vaddr += clip_offset(clip, fb->pitches[0], fb->format->cpp[0]);

	for (i = 0; i < lines; ++i) {
		if (stmp)
			sbuf = memcpy(stmp, vaddr, sbuf_len);
		else
			sbuf = vaddr;
		xfrm_line(dst, sbuf, linepixels);
		vaddr += fb->pitches[0];
		dst += dst_pitch;
	}

	kfree(stmp);

	return 0;
}

/* TODO: Make this function work with multi-plane formats. */
static int __drm_fb_xfrm_toio(void __iomem *dst, unsigned long dst_pitch, unsigned long dst_pixsize,
			      const void *vaddr, const struct drm_framebuffer *fb,
			      const struct drm_rect *clip, bool vaddr_cached_hint,
			      void (*xfrm_line)(void *dbuf, const void *sbuf, unsigned int npixels))
{
	unsigned long linepixels = drm_rect_width(clip);
	unsigned long lines = drm_rect_height(clip);
	size_t dbuf_len = linepixels * dst_pixsize;
	size_t stmp_off = round_up(dbuf_len, ARCH_KMALLOC_MINALIGN); /* for sbuf alignment */
	size_t sbuf_len = linepixels * fb->format->cpp[0];
	void *stmp = NULL;
	unsigned long i;
	const void *sbuf;
	void *dbuf;

	if (vaddr_cached_hint) {
		dbuf = kmalloc(dbuf_len, GFP_KERNEL);
	} else {
		dbuf = kmalloc(stmp_off + sbuf_len, GFP_KERNEL);
		stmp = dbuf + stmp_off;
	}
	if (!dbuf)
		return -ENOMEM;

	if (!dst_pitch)
		dst_pitch = linepixels * dst_pixsize;
	vaddr += clip_offset(clip, fb->pitches[0], fb->format->cpp[0]);

	for (i = 0; i < lines; ++i) {
		if (stmp)
			sbuf = memcpy(stmp, vaddr, sbuf_len);
		else
			sbuf = vaddr;
		xfrm_line(dbuf, sbuf, linepixels);
		memcpy_toio(dst, dbuf, dbuf_len);
		vaddr += fb->pitches[0];
		dst += dst_pitch;
	}

	kfree(dbuf);

	return 0;
}

static int drm_fb_xfrm(struct iosys_map *dst,
		       const unsigned int *dst_pitch, const u8 *dst_pixsize,
		       const struct iosys_map *src, const struct drm_framebuffer *fb,
		       const struct drm_rect *clip, bool vaddr_cached_hint,
		       void (*xfrm_line)(void *dbuf, const void *sbuf, unsigned int npixels))
{
	static const unsigned int default_dst_pitch[DRM_FORMAT_MAX_PLANES] = {
		0, 0, 0, 0
	};

	if (!dst_pitch)
		dst_pitch = default_dst_pitch;

	/* TODO: handle src in I/O memory here */
	if (dst[0].is_iomem)
		return __drm_fb_xfrm_toio(dst[0].vaddr_iomem, dst_pitch[0], dst_pixsize[0],
					  src[0].vaddr, fb, clip, vaddr_cached_hint, xfrm_line);
	else
		return __drm_fb_xfrm(dst[0].vaddr, dst_pitch[0], dst_pixsize[0],
				     src[0].vaddr, fb, clip, vaddr_cached_hint, xfrm_line);
}

static void drm_fb_xrgb8888_to_rgb666_line(void *dbuf, const void *sbuf, unsigned int pixels)
{
	u8 *dbuf8 = dbuf;
	const __le32 *sbuf32 = sbuf;
	unsigned int x;
	u32 pix;
	for(x = 0; x < pixels; x++){
		pix = le32_to_cpu(sbuf32[x]);
		*dbuf8++ = (pix & 0x000000FC)>>0;
		*dbuf8++ = (pix & 0x0000FC00)>>0;
		*dbuf8++ = (pix & 0x00FC0000)>>0;
	}
}

void drm_fb_xrgb8888_to_rgb666(struct iosys_map *dst, const unsigned int *dst_pitch,
			       const struct iosys_map *src, const struct drm_framebuffer *fb,
			       const struct drm_rect *clip)
{
	static const u8 dst_pixsize[DRM_FORMAT_MAX_PLANES] = {3};
	drm_fb_xfrm(dst,dst_pitch,dst_pixsize,src,fb,clip,false,drm_fb_xrgb8888_to_rgb666_line);
}



int mipi_dbi18_buf_copy(void *dst, struct drm_framebuffer *fb,
		      struct drm_rect *clip, bool swap)
{
	struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	struct iosys_map dst_map = IOSYS_MAP_INIT_VADDR(dst);
	int ret;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return ret;

	ret = drm_gem_fb_vmap(fb, map, data);
	if (ret)
		goto out_drm_gem_fb_end_cpu_access;

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		if (swap)
			drm_fb_swab(&dst_map, NULL, data, fb, clip, !gem->import_attach);
		else
			drm_fb_memcpy(&dst_map, NULL, data, fb, clip);
		break;
	case DRM_FORMAT_XRGB8888:
		drm_fb_xrgb8888_to_rgb666(&dst_map, NULL, data, fb, clip);
		break;
	default:
		drm_err_once(fb->dev, "Format is not supported: %p4cc\n",
			     &fb->format->format);
		ret = -EINVAL;
	}

	drm_gem_fb_vunmap(fb, map);
out_drm_gem_fb_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);

	return ret;
}

static void mipi_dbi18_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct iosys_map map[DRM_FORMAT_MAX_PLANES];
	struct iosys_map data[DRM_FORMAT_MAX_PLANES];
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;
	struct mipi_dbi *dbi = &dbidev->dbi;
	bool swap = dbi->swap_bytes;
	int idx, ret = 0;
	bool full;
	void *tr;

	if (WARN_ON(!fb))
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	ret = drm_gem_fb_vmap(fb, map, data);
	if (ret)
		goto err_drm_dev_exit;

	full = width == fb->width && height == fb->height;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

	if (!dbi->dc || !full || swap ||
	    fb->format->format == DRM_FORMAT_XRGB8888) {
		tr = dbidev->tx_buf;
		ret = mipi_dbi18_buf_copy(dbidev->tx_buf, fb, rect, swap);
		if (ret)
			goto err_msg;
	} else {
		tr = data[0].vaddr; /* TODO: Use mapping abstraction properly */
	}

	mipi_dbi_set_window_address(dbidev, 
		rect->x1, rect->x2 - 1,
		rect->y1, rect->y2 - 1);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
				   width * height * 3);
err_msg:
	if (ret)
		drm_err_once(fb->dev, "Failed to update display %d\n", ret);

	drm_gem_fb_vunmap(fb, map);

err_drm_dev_exit:
	drm_dev_exit(idx);
}

void mipi_dbi18_pipe_update(struct drm_simple_display_pipe *pipe,
			  struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_rect rect;

	if (!pipe->crtc.state->active)
		return;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		mipi_dbi18_fb_dirty(state->fb, &rect);
}

void mipi_dbi18_enable_flush(struct mipi_dbi_dev *dbidev,
			   struct drm_crtc_state *crtc_state,
			   struct drm_plane_state *plane_state)
{
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	int idx;

	if (!drm_dev_enter(&dbidev->drm, &idx))
		return;

	mipi_dbi18_fb_dirty(fb, &rect);
	backlight_enable(dbidev->backlight);

	drm_dev_exit(idx);
}

int mipi_dbi18_dev_init(struct mipi_dbi_dev *dbidev,
		      const struct drm_simple_display_pipe_funcs *funcs,
		      const struct drm_display_mode *mode, unsigned int rotation)
{
	size_t bufsize = mode->vdisplay * mode->hdisplay * sizeof(u32);

	dbidev->drm.mode_config.preferred_depth = 24;

	return mipi_dbi_dev_init_with_formats(dbidev, funcs, mipi_dbi_formats,
					      ARRAY_SIZE(mipi_dbi_formats), mode,
					      rotation, bufsize);
}


static void sx035hv006_enable(struct drm_simple_display_pipe *pipe,
		struct drm_crtc_state *crtc_state,
		struct drm_plane_state *plane_state)
{

	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	u8 addr_mode;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");
	
	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;
	if (ret == 1)
		goto out_enable;
	
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);

	/* Positive Gamma Control */
	mipi_dbi_command(dbi, ILI9488_CMD_POSITIVE_GAMMA_CORRECTION,
			 0x00, 0x03, 0x09, 0x08, 0x16,
			 0x0a, 0x3f, 0x78, 0x4c, 0x09,
			 0x0a, 0x08, 0x16, 0x1a, 0x0f);

	/* Negative Gamma Control */
	mipi_dbi_command(dbi, ILI9488_CMD_NEGATIVE_GAMMA_CORRECTION,
			 0x00, 0x16, 0x19, 0x03, 0x0f,
			 0x05, 0x32, 0x45, 0x46, 0x04,
			 0x0e, 0x0d, 0x35, 0x37, 0x0f);

	/* Power Control 1,2 */
	mipi_dbi_command(dbi, ILI9488_CMD_POWER_CONTROL_1, 0x17, 0x15);
	mipi_dbi_command(dbi, ILI9488_CMD_POWER_CONTROL_2, 0x41);

	/* VCOM Control 1 */
	mipi_dbi_command(dbi, ILI9488_CMD_VCOM_CONTROL_1, 0x00, 0x12, 0x80);

	/* Memory Access Contorl */
	mipi_dbi_command(dbi, ILI9488_CMD_MEMORY_ACCESS_CONTROL, 0x48);

    /* Pixel Format */
	mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_18BIT<<1 | MIPI_DCS_PIXEL_FMT_18BIT);
	//mipi_dbi_command(dbi, MIPI_DCS_SET_PIXEL_FORMAT, MIPI_DCS_PIXEL_FMT_18BIT);
	
	mipi_dbi_command(dbi, ILI9488_CMD_INTERFACE_MODE_CONTROL, 0x00);

	/* Frame Rate Control */
	/*	Frame rate = 60.76Hz.*/
	//mipi_dbi_command(dbi, ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, 0xa1);
	mipi_dbi_command(dbi, ILI9488_CMD_FRAME_RATE_CONTROL_NORMAL, 0xA0);

	/* Display Inversion Control */
	/*	2 dot inversion */
	mipi_dbi_command(dbi, ILI9488_CMD_DISPLAY_INVERSION_CONTROL, 0x02);

	/* Set Image Function */
	//mipi_dbi_command(dbi, ILI9488_CMD_SET_IMAGE_FUNCTION, 0x00);

	/* Set Display Function Control */
	mipi_dbi_command(dbi, ILI9488_CMD_DISPLAY_FUNCTION_CONTROL, 0x02, 0x02, 0x3B);

	/* Set Entry Mode */
	mipi_dbi_command(dbi, ILI9488_CMD_ENTRY_MODE_SET, 0xC6);

	/* Adjust Control 3 */
	mipi_dbi_command(dbi, ILI9488_CMD_ADJUST_CONTROL_3,
			 0xa9, 0x51, 0x2c, 0x82);

	/* CABC control 2 */
	//mipi_dbi_command(dbi, ILI9488_CMD_CABC_CONTROL_2, 0xb0);

	/* Sleep OUT */
	mipi_dbi_command(dbi, ILI9488_CMD_SLEEP_OUT);

	msleep(120);

	mipi_dbi_command(dbi, ILI9488_CMD_NORMAL_DISP_MODE_ON);

	/* Display ON */
	mipi_dbi_command(dbi, ILI9488_CMD_DISPLAY_ON);
	msleep(100);    

out_enable:
	switch (dbidev->rotation) {
	default:
		addr_mode = ILI9488_MADCTL_MX;
		break;
	case 90:
		addr_mode = ILI9488_MADCTL_MV;
		break;
	case 180:
		addr_mode = ILI9488_MADCTL_MY;
		break;
	case 270:
		addr_mode = ILI9488_MADCTL_MV | ILI9488_MADCTL_MY |
			    ILI9488_MADCTL_MX;
		break;
	}
	//addr_mode |= ILI9488_MADCTL_BGR;
	mipi_dbi_command(dbi, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);
	mipi_dbi18_enable_flush(dbidev, crtc_state, plane_state);
out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs ili9488_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = sx035hv006_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi18_pipe_update,
};

static const struct drm_display_mode sx035hv006_mode = {
	DRM_SIMPLE_MODE(320, 480, 49, 73),
};

static const struct file_operations ili9488_fops = {
		.owner		= THIS_MODULE,
		.open		= drm_open,
		.release	= drm_release,
		.unlocked_ioctl	= drm_ioctl,
		.compat_ioctl	= drm_compat_ioctl,
		.poll		= drm_poll,
		.read		= drm_read,
		.llseek		= noop_llseek,
		.mmap		= drm_gem_mmap,
		DRM_GEM_DMA_UNMAPPED_AREA_FOPS
	};

//DEFINE_DRM_GEM_CMA_FOPS(ili9488_fops);

static struct drm_driver ili9488_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &ili9488_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "ili9488",
	.desc			= "Ilitek ILI9488",
	.date			= "20230414",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id ili9488_of_match[] = {
	{ .compatible = "makerlab,sx035hv006" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili9488_of_match);

static const struct spi_device_id ili9488_id[] = {
	{ "sx035hv006", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, ili9488_id);

static int ili9488_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	int ret;
	
	dbidev = devm_drm_dev_alloc(dev, &ili9488_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;
	
	dbi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
		return PTR_ERR(dbidev->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	ret = mipi_dbi18_dev_init(dbidev, &ili9488_pipe_funcs, &sx035hv006_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static void ili9488_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void ili9488_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver ili9488_spi_driver = {
	.driver = {
		.name = "ili9488",
		.of_match_table = ili9488_of_match,
	},
	.id_table = ili9488_id,
	.probe = ili9488_probe,
	.remove = ili9488_remove,
	.shutdown = ili9488_shutdown,
};
module_spi_driver(ili9488_spi_driver);
	
MODULE_DESCRIPTION("Ilitek ILI9488 DRM driver");
MODULE_AUTHOR("VASILY KAPUSTIN <vasilykap@live.com>");
MODULE_LICENSE("GPL");

