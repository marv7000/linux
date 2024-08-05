/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DRM driver for S3 Trio
 *
 * Copyright 2024 Marvin Friedrich <contact@marvinf.com>
 *
 * This code has been adapted from the fbdev version in
 * linux/drivers/video/fbdev/s3fb.c
 */

#include "drm/drm_print.h"
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_module.h>
#include <drm/drm_client.h>
#include <drm/drm_aperture.h>
#include <drm/drm_managed.h>
#include <drm/drm_mode.h>
#include <drm/drm_gem.h>
#include <drm/drm_drv.h>
#include <drm/drm_file.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_generic.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_atomic.h>

#include <linux/types.h>
#include <linux/pci.h>
#include <linux/array_size.h>
#include <linux/i2c-algo-bit.h>
#include <linux/console.h>
#include <linux/aperture.h>
#include <linux/init.h>
#include <linux/svga.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/units.h>

/* ------------------------------------------------------------------------- */
/* Defines                                                                   */

#define DRIVER_AUTHOR		"Marvin Friedrich <contact@marvinf.com>"
#define DRIVER_NAME		"s3drm"
#define DRIVER_DESC		"DRM driver for S3 Trio"
#define DRIVER_LICENSE		"GPL"
#define DRIVER_DATE		"20240610"
#define DRIVER_MAJOR		0
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	1

#define MMIO_OFFSET		0x1000000
#define MMIO_SIZE		0x10000

/* ------------------------------------------------------------------------- */
/* Device                                                                    */

enum s3drm_chip {
	CHIP_UNKNOWN		= 0x00,
	CHIP_732_TRIO32		= 0x01,
	CHIP_764_TRIO64		= 0x02,
	CHIP_765_TRIO64VP	= 0x03,
	CHIP_767_TRIO64UVP	= 0x04,
	CHIP_775_TRIO64V2_DX	= 0x05,
	CHIP_785_TRIO64V2_GX	= 0x06,
	CHIP_551_PLATO_PX	= 0x07,
	CHIP_M65_AURORA64VP	= 0x08,
	CHIP_325_VIRGE		= 0x09,
	CHIP_988_VIRGE_VX	= 0x0A,
	CHIP_375_VIRGE_DX	= 0x0B,
	CHIP_385_VIRGE_GX	= 0x0C,
	CHIP_357_VIRGE_GX2	= 0x0D,
	CHIP_359_VIRGE_GX2P	= 0x0E,
	CHIP_360_TRIO3D_1X	= 0x10,
	CHIP_362_TRIO3D_2X	= 0x11,
	CHIP_368_TRIO3D_2X	= 0x12,
	CHIP_365_TRIO3D		= 0x13,
	CHIP_260_VIRGE_MX	= 0x14,

	CHIP_UNDECIDED_FLAG	= 0x80,

	CHIP_XXX_TRIO		= CHIP_UNDECIDED_FLAG | 0x00,
	CHIP_XXX_TRIO64V2_DXGX	= CHIP_UNDECIDED_FLAG | 0x01,
	CHIP_XXX_VIRGE_DXGX	= CHIP_UNDECIDED_FLAG | 0x02,
	CHIP_36X_TRIO3D_1X_2X	= CHIP_UNDECIDED_FLAG | 0x03,

	CHIP_MASK = 0xFF,
};

static const struct vga_regset s3_h_total_regs[]        = {{0x00, 0, 7}, {0x5D, 0, 0}, VGA_REGSET_END};
static const struct vga_regset s3_h_display_regs[]      = {{0x01, 0, 7}, {0x5D, 1, 1}, VGA_REGSET_END};
static const struct vga_regset s3_h_blank_start_regs[]  = {{0x02, 0, 7}, {0x5D, 2, 2}, VGA_REGSET_END};
static const struct vga_regset s3_h_blank_end_regs[]    = {{0x03, 0, 4}, {0x05, 7, 7}, VGA_REGSET_END};
static const struct vga_regset s3_h_sync_start_regs[]   = {{0x04, 0, 7}, {0x5D, 4, 4}, VGA_REGSET_END};
static const struct vga_regset s3_h_sync_end_regs[]     = {{0x05, 0, 4}, VGA_REGSET_END};

static const struct vga_regset s3_v_total_regs[]        = {{0x06, 0, 7}, {0x07, 0, 0}, {0x07, 5, 5}, {0x5E, 0, 0}, VGA_REGSET_END};
static const struct vga_regset s3_v_display_regs[]      = {{0x12, 0, 7}, {0x07, 1, 1}, {0x07, 6, 6}, {0x5E, 1, 1}, VGA_REGSET_END};
static const struct vga_regset s3_v_blank_start_regs[]  = {{0x15, 0, 7}, {0x07, 3, 3}, {0x09, 5, 5}, {0x5E, 2, 2}, VGA_REGSET_END};
static const struct vga_regset s3_v_blank_end_regs[]    = {{0x16, 0, 7}, VGA_REGSET_END};
static const struct vga_regset s3_v_sync_start_regs[]   = {{0x10, 0, 7}, {0x07, 2, 2}, {0x07, 7, 7}, {0x5E, 4, 4}, VGA_REGSET_END};
static const struct vga_regset s3_v_sync_end_regs[]     = {{0x11, 0, 3}, VGA_REGSET_END};

static const struct vga_regset s3_line_compare_regs[]   = {{0x18, 0, 7}, {0x07, 4, 4}, {0x09, 6, 6}, {0x5E, 6, 6}, VGA_REGSET_END};
static const struct vga_regset s3_start_address_regs[]  = {{0x0d, 0, 7}, {0x0c, 0, 7}, {0x69, 0, 4}, VGA_REGSET_END};
static const struct vga_regset s3_offset_regs[]         = {{0x13, 0, 7}, {0x51, 4, 5}, VGA_REGSET_END}; /* set 0x43 bit 2 to 0 */

static const struct vga_regset s3_dtpc_regs[]		= {{0x3B, 0, 7}, {0x5D, 6, 6}, VGA_REGSET_END};

static const struct svga_timing_regs s3_timing_regs     = {
	s3_h_total_regs, s3_h_display_regs, s3_h_blank_start_regs,
	s3_h_blank_end_regs, s3_h_sync_start_regs, s3_h_sync_end_regs,
	s3_v_total_regs, s3_v_display_regs, s3_v_blank_start_regs,
	s3_v_blank_end_regs, s3_v_sync_start_regs, s3_v_sync_end_regs,
};

static const struct svga_fb_format s3drm_formats[] = {
	{ 0,  {0, 6, 0},  {0, 6, 0},  {0, 6, 0}, {0, 0, 0}, 0,
		FB_TYPE_TEXT, FB_AUX_TEXT_SVGA_STEP4,	FB_VISUAL_PSEUDOCOLOR, 8, 16},
	{ 4,  {0, 4, 0},  {0, 4, 0},  {0, 4, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_PSEUDOCOLOR, 8, 16},
	{ 4,  {0, 4, 0},  {0, 4, 0},  {0, 4, 0}, {0, 0, 0}, 1,
		FB_TYPE_INTERLEAVED_PLANES, 1,		FB_VISUAL_PSEUDOCOLOR, 8, 16},
	{ 8,  {0, 8, 0},  {0, 8, 0},  {0, 8, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_PSEUDOCOLOR, 4, 8},
	{16,  {10, 5, 0}, {5, 5, 0},  {0, 5, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_TRUECOLOR, 2, 4},
	{16,  {11, 5, 0}, {5, 6, 0},  {0, 5, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_TRUECOLOR, 2, 4},
	{24,  {16, 8, 0}, {8, 8, 0},  {0, 8, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_TRUECOLOR, 1, 2},
	{32,  {16, 8, 0}, {8, 8, 0},  {0, 8, 0}, {0, 0, 0}, 0,
		FB_TYPE_PACKED_PIXELS, 0,		FB_VISUAL_TRUECOLOR, 1, 2},
	SVGA_FORMAT_END
};

static const struct svga_pll s3_pll = {3, 129, 3, 33, 0, 3,
	35000, 240000, 14318};
static const struct svga_pll s3_trio3d_pll = {3, 129, 3, 31, 0, 4,
	230000, 460000, 14318};

static const int s3_memsizes[] = {4096, 0, 3072, 8192, 2048, 6144, 1024, 512};

static const char * const s3_names[] = {
	"S3 Unknown", "S3 Trio32", "S3 Trio64", "S3 Trio64V+",
	"S3 Trio64UV+", "S3 Trio64V2/DX", "S3 Trio64V2/GX",
	"S3 Plato/PX", "S3 Aurora64V+", "S3 Virge",
	"S3 Virge/VX", "S3 Virge/DX", "S3 Virge/GX",
	"S3 Virge/GX2", "S3 Virge/GX2+", "",
	"S3 Trio3D/1X", "S3 Trio3D/2X", "S3 Trio3D/2X",
	"S3 Trio3D", "S3 Virge/MX"
};

struct s3_device {
	enum s3drm_chip chip;
	int rev;
	int mclk_freq;
	int wc_cookie;
	struct vgastate state;
	struct mutex open_lock;
	unsigned int ref_count;
	u32 pseudo_palette[16];
	u8 __iomem *mmio;
	bool ddc_registered;
	struct i2c_adapter ddc_adapter;
	struct i2c_algo_bit_data ddc_algo;

	struct drm_device dev;
	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
	void __iomem *screen_base;
};

/* ------------------------------------------------------------------------- */
/* Utils                                                                     */

/* Image data is MSB-first, fb structure is MSB-first too */
static inline u32 expand_color(u32 c)
{
	return ((c & 1) | ((c & 2) << 7) | ((c & 4) << 14) | ((c & 8) << 21)) * 0xFF;
}

/* Image data is MSB-first, fb structure is high-nibble-in-low-byte-first */
static inline u32 expand_pixel(u32 c)
{
	return (((c &  1) << 24) | ((c &  2) << 27) | ((c &  4) << 14) | ((c &   8) << 17) |
		((c & 16) <<  4) | ((c & 32) <<  7) | ((c & 64) >>  6) | ((c & 128) >>  3)) * 0xF;
}

static inline struct s3_device* s3_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct s3_device, dev);
}

/* Identifies the chip. */
static int s3_identification(struct s3_device *dev)
{
	int chip = dev->chip;

	if (chip == CHIP_XXX_TRIO) {
		u8 cr30 = vga_rcrt(dev->state.vgabase, 0x30);
		u8 cr2e = vga_rcrt(dev->state.vgabase, 0x2e);
		u8 cr2f = vga_rcrt(dev->state.vgabase, 0x2f);

		if ((cr30 == 0xE0) || (cr30 == 0xE1)) {
			if (cr2e == 0x10)
				return CHIP_732_TRIO32;
			if (cr2e == 0x11) {
				if (! (cr2f & 0x40))
					return CHIP_764_TRIO64;
				else
					return CHIP_765_TRIO64VP;
			}
		}
	}

	if (chip == CHIP_XXX_TRIO64V2_DXGX) {
		u8 cr6f = vga_rcrt(dev->state.vgabase, 0x6f);

		if (! (cr6f & 0x01))
			return CHIP_775_TRIO64V2_DX;
		else
			return CHIP_785_TRIO64V2_GX;
	}

	if (chip == CHIP_XXX_VIRGE_DXGX) {
		u8 cr6f = vga_rcrt(dev->state.vgabase, 0x6f);

		if (! (cr6f & 0x01))
			return CHIP_375_VIRGE_DX;
		else
			return CHIP_385_VIRGE_GX;
	}

	if (chip == CHIP_36X_TRIO3D_1X_2X) {
		switch (vga_rcrt(dev->state.vgabase, 0x2f)) {
		case 0x00:
			return CHIP_360_TRIO3D_1X;
		case 0x01:
			return CHIP_362_TRIO3D_2X;
		case 0x02:
			return CHIP_368_TRIO3D_2X;
		}
	}

	return CHIP_UNKNOWN;
}

/* Sets the pixel clock. Value is in kHz. */
static void s3_set_pixclock(struct s3_device *s3, u32 pixclock)
{
	u16 m, n, r;
	u8 regval;
	int rv;

	rv = svga_compute_pll((s3->chip == CHIP_365_TRIO3D) ? &s3_trio3d_pll : &s3_pll,
			      pixclock, &m, &n, &r, 0);
	if (rv < 0) {
		drm_err(&s3->dev, "cannot set requested pixclock, keeping old value\n");
		return;
	}

	/* Set VGA misc register  */
	regval = vga_r(s3->state.vgabase, VGA_MIS_R);
	vga_w(s3->state.vgabase, VGA_MIS_W, regval | VGA_MIS_ENB_PLL_LOAD);

	/* Set S3 clock registers */
	if (s3->chip == CHIP_357_VIRGE_GX2 ||
	    s3->chip == CHIP_359_VIRGE_GX2P ||
	    s3->chip == CHIP_360_TRIO3D_1X ||
	    s3->chip == CHIP_362_TRIO3D_2X ||
	    s3->chip == CHIP_368_TRIO3D_2X ||
	    s3->chip == CHIP_260_VIRGE_MX) {
		vga_wseq(s3->state.vgabase, 0x12, (n - 2) | ((r & 3) << 6));	/* n and two bits of r */
		vga_wseq(s3->state.vgabase, 0x29, r >> 2); /* remaining highest bit of r */
	} else
		vga_wseq(s3->state.vgabase, 0x12, (n - 2) | (r << 5));
	vga_wseq(s3->state.vgabase, 0x13, m - 2);

	udelay(1000);

	/* Activate clock - write 0, 1, 0 to seq/15 bit 5 */
	regval = vga_rseq(s3->state.vgabase, 0x15); /* | 0x80; */
	vga_wseq(s3->state.vgabase, 0x15, regval & ~(1<<5));
	vga_wseq(s3->state.vgabase, 0x15, regval |  (1<<5));
	vga_wseq(s3->state.vgabase, 0x15, regval & ~(1<<5));
}

/* ------------------------------------------------------------------------- */
/* DDC                                                                       */

#define DDC_REG		0xAA		/* Trio 3D/1X/2X */
#define DDC_MMIO_REG	0xFF20		/* all other chips */
#define DDC_SCL_OUT	(1 << 0)
#define DDC_SDA_OUT	(1 << 1)
#define DDC_SCL_IN	(1 << 2)
#define DDC_SDA_IN	(1 << 3)
#define DDC_DRIVE_EN	(1 << 4)

static bool s3drm_ddc_needs_mmio(int chip)
{
	return !(chip == CHIP_360_TRIO3D_1X  ||
		 chip == CHIP_362_TRIO3D_2X  ||
		 chip == CHIP_368_TRIO3D_2X);
}

static u8 s3drm_ddc_read(struct s3_device *dev)
{
	if (s3drm_ddc_needs_mmio(dev->chip))
		return readb(dev->mmio + DDC_MMIO_REG);
	else
		return vga_rcrt(dev->state.vgabase, DDC_REG);
}

static void s3drm_ddc_write(struct s3_device *dev, u8 val)
{
	if (s3drm_ddc_needs_mmio(dev->chip))
		writeb(val, dev->mmio + DDC_MMIO_REG);
	else
		vga_wcrt(dev->state.vgabase, DDC_REG, val);
}

static void s3drm_ddc_setscl(void *data, int val)
{
	struct s3_device *dev = data;
	unsigned char reg;

	reg = s3drm_ddc_read(dev) | DDC_DRIVE_EN;
	if (val)
		reg |= DDC_SCL_OUT;
	else
		reg &= ~DDC_SCL_OUT;
	s3drm_ddc_write(dev, reg);
}

static void s3drm_ddc_setsda(void *data, int val)
{
	struct s3_device *dev = data;
	unsigned char reg;

	reg = s3drm_ddc_read(dev) | DDC_DRIVE_EN;
	if (val)
		reg |= DDC_SDA_OUT;
	else
		reg &= ~DDC_SDA_OUT;
	s3drm_ddc_write(dev, reg);
}

static int s3drm_ddc_getscl(void *data)
{
	struct s3_device *dev = data;

	return !!(s3drm_ddc_read(dev) & DDC_SCL_IN);
}

static int s3drm_ddc_getsda(void *data)
{
	struct s3_device *dev = data;

	return !!(s3drm_ddc_read(dev) & DDC_SDA_IN);
}

static int s3drm_setup_ddc_bus(struct s3_device *s3)
{
	strcpy(s3->ddc_adapter.name, "s3ddc");
	s3->ddc_adapter.owner		= THIS_MODULE;
	s3->ddc_adapter.algo_data	= &s3->ddc_algo;
	s3->ddc_adapter.dev.parent	= s3->dev.dev;
	s3->ddc_algo.setsda		= s3drm_ddc_setsda;
	s3->ddc_algo.setscl		= s3drm_ddc_setscl;
	s3->ddc_algo.getsda		= s3drm_ddc_getsda;
	s3->ddc_algo.getscl		= s3drm_ddc_getscl;
	s3->ddc_algo.udelay		= 10;
	s3->ddc_algo.timeout		= 20;
	s3->ddc_algo.data		= s3;

	i2c_set_adapdata(&s3->ddc_adapter, s3);

	/*
	 * some Virge cards have external MUX to switch chip I2C bus between
	 * DDC and extension pins - switch it do DDC
	 */
/*	vga_wseq(par->state.vgabase, 0x08, 0x06); - not needed, already unlocked */
	if (s3->chip == CHIP_357_VIRGE_GX2 ||
	    s3->chip == CHIP_359_VIRGE_GX2P ||
	    s3->chip == CHIP_260_VIRGE_MX)
		svga_wseq_mask(s3->state.vgabase, 0x0d, 0x01, 0x03);
	else
		svga_wseq_mask(s3->state.vgabase, 0x0d, 0x00, 0x03);
	/* some Virge need this or the DDC is ignored */
	svga_wcrt_mask(s3->state.vgabase, 0x5c, 0x03, 0x03);

	return i2c_bit_add_bus(&s3->ddc_adapter);
}

/* ------------------------------------------------------------------------- */
/* DRM                                                                       */

static int s3drm_set_par(struct drm_crtc *crtc, struct drm_display_mode *mode)
{
	struct s3_device *s3 = container_of(crtc, struct s3_device, crtc);
	u32 offset_value, screen_size, dbytes;
	u32 bpp = 32; // TODO
	u32 htotal, hsstart, value, multiplex;
	u32 hmul = 1;

	if (bpp != 0) {
		crtc->primary->state->fb->pitches[0] = mode->hdisplay * bpp / 8;
		offset_value = (mode->hdisplay * bpp) / 64;
		screen_size = mode->vdisplay * crtc->primary->state->fb->pitches[0];
	} else {
		offset_value = mode->hdisplay / 16;
		screen_size = (mode->hdisplay * mode->vdisplay) / 64;
	}

	/* Unlock registers */
	vga_wcrt(s3->state.vgabase, 0x38, 0x48);
	vga_wcrt(s3->state.vgabase, 0x39, 0xA5);
	vga_wseq(s3->state.vgabase, 0x08, 0x06);
	svga_wcrt_mask(s3->state.vgabase, 0x11, 0x00, 0x80);

	/* Blank screen and turn off sync */
	svga_wseq_mask(s3->state.vgabase, 0x01, 0x20, 0x20);
	svga_wcrt_mask(s3->state.vgabase, 0x17, 0x00, 0x80);

	/* Set default values */
	svga_set_default_gfx_regs(s3->state.vgabase);
	svga_set_default_atc_regs(s3->state.vgabase);
	svga_set_default_seq_regs(s3->state.vgabase);
	svga_set_default_crt_regs(s3->state.vgabase);
	svga_wcrt_multi(s3->state.vgabase, s3_line_compare_regs, 0xFFFFFFFF);
	svga_wcrt_multi(s3->state.vgabase, s3_start_address_regs, 0);

	/* S3 specific initialization */
	svga_wcrt_mask(s3->state.vgabase, 0x58, 0x10, 0x10); /* enable linear framebuffer */
	svga_wcrt_mask(s3->state.vgabase, 0x31, 0x08, 0x08); /* enable sequencer access to framebuffer above 256 kB */
	svga_wcrt_mask(s3->state.vgabase, 0x33, 0x00, 0x08); /* no DDR */
	svga_wcrt_mask(s3->state.vgabase, 0x43, 0x00, 0x01); /* no DDR */
	svga_wcrt_mask(s3->state.vgabase, 0x5D, 0x00, 0x28); /* Clear strange HSlen bits */

	/* Set the offset register */
	svga_wcrt_multi(s3->state.vgabase, s3_offset_regs, offset_value);

	if (s3->chip != CHIP_357_VIRGE_GX2 &&
	    s3->chip != CHIP_359_VIRGE_GX2P &&
	    s3->chip != CHIP_360_TRIO3D_1X &&
	    s3->chip != CHIP_362_TRIO3D_2X &&
	    s3->chip != CHIP_368_TRIO3D_2X &&
	    s3->chip != CHIP_260_VIRGE_MX) {
		vga_wcrt(s3->state.vgabase, 0x54, 0x18); /* M parameter */
		vga_wcrt(s3->state.vgabase, 0x60, 0xFF); /* N parameter */
		vga_wcrt(s3->state.vgabase, 0x61, 0xFF); /* L parameter */
		vga_wcrt(s3->state.vgabase, 0x62, 0xFF); /* L parameter */
	}

	vga_wcrt(s3->state.vgabase, 0x3A, 0x35);
	svga_wattr(s3->state.vgabase, 0x33, 0x00);

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		svga_wcrt_mask(s3->state.vgabase, 0x09, 0x80, 0x80);
	else
		svga_wcrt_mask(s3->state.vgabase, 0x09, 0x00, 0x80);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		svga_wcrt_mask(s3->state.vgabase, 0x42, 0x20, 0x20);
	else
		svga_wcrt_mask(s3->state.vgabase, 0x42, 0x00, 0x20);

	/* Disable hardware graphics cursor */
	svga_wcrt_mask(s3->state.vgabase, 0x45, 0x00, 0x01);
	/* Disable Streams engine */
	svga_wcrt_mask(s3->state.vgabase, 0x67, 0x00, 0x0C);

	if (s3->chip == CHIP_375_VIRGE_DX) {
		vga_wcrt(s3->state.vgabase, 0x86, 0x80);
		vga_wcrt(s3->state.vgabase, 0x90, 0x00);
	}

	if (s3->chip == CHIP_988_VIRGE_VX) {
		vga_wcrt(s3->state.vgabase, 0x50, 0x00);
		vga_wcrt(s3->state.vgabase, 0x67, 0x50);
		msleep(10); /* screen remains blank sometimes without this */
		vga_wcrt(s3->state.vgabase, 0x63, (mode->hdisplay <= 2) ? 0x90 : 0x09);
		vga_wcrt(s3->state.vgabase, 0x66, 0x90);
	}

	if (s3->chip == CHIP_357_VIRGE_GX2 || s3->chip == CHIP_359_VIRGE_GX2P ||
	    s3->chip == CHIP_360_TRIO3D_1X || s3->chip == CHIP_362_TRIO3D_2X  ||
	    s3->chip == CHIP_368_TRIO3D_2X || s3->chip == CHIP_365_TRIO3D     ||
	    s3->chip == CHIP_375_VIRGE_DX  || s3->chip == CHIP_385_VIRGE_GX   ||
	    s3->chip == CHIP_260_VIRGE_MX) {
		dbytes = mode->hdisplay * ((bpp + 7) / 8);
		vga_wcrt(s3->state.vgabase, 0x91, (dbytes + 7) / 8);
		vga_wcrt(s3->state.vgabase, 0x90, (((dbytes + 7) / 8) >> 8) | 0x80);
		vga_wcrt(s3->state.vgabase, 0x66, 0x81);
	}

	if (s3->chip == CHIP_357_VIRGE_GX2 || s3->chip == CHIP_359_VIRGE_GX2P ||
	    s3->chip == CHIP_360_TRIO3D_1X || s3->chip == CHIP_362_TRIO3D_2X ||
	    s3->chip == CHIP_368_TRIO3D_2X || s3->chip == CHIP_260_VIRGE_MX)
		vga_wcrt(s3->state.vgabase, 0x34, 0x00);
	else /* enable Data Transfer Position Control (DTPC) */
		vga_wcrt(s3->state.vgabase, 0x34, 0x10);

	svga_wcrt_mask(s3->state.vgabase, 0x31, 0x00, 0x40);
	multiplex = 0;
	hmul = 1;

	// TODO
	/* Set mode-specific register values */
	/* For now, force 32 bpp */
	svga_wcrt_mask(s3->state.vgabase, 0x50, 0x30, 0x30);
	svga_wcrt_mask(s3->state.vgabase, 0x67, 0xD0, 0xF0);

	s3_set_pixclock(s3, mode->crtc_clock);

	/* Construct a fake fb_var_screeninfo to pass to svga_set_timings */
	struct fb_var_screeninfo fb_mode;

	fb_mode.xres = mode->hdisplay;
	fb_mode.yres = mode->vdisplay;
	fb_mode.pixclock = KHZ2PICOS(mode->clock);
	fb_mode.left_margin = mode->htotal - mode->hsync_end;
	fb_mode.right_margin = mode->hsync_start - mode->hdisplay;
	fb_mode.upper_margin = mode->vtotal - mode->vsync_end;
	fb_mode.lower_margin = mode->vsync_start - mode->vdisplay;
	fb_mode.hsync_len = mode->hsync_end - mode->hsync_start;
	fb_mode.vsync_len = mode->vsync_end - mode->vsync_start;

	fb_mode.sync = 0;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		fb_mode.sync |= FB_SYNC_HOR_HIGH_ACT;
	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		fb_mode.sync |= FB_SYNC_VERT_HIGH_ACT;
	if (mode->flags & (DRM_MODE_FLAG_CSYNC | DRM_MODE_FLAG_PCSYNC))
		fb_mode.sync |= FB_SYNC_COMP_HIGH_ACT;

	fb_mode.vmode = 0;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		fb_mode.vmode |= FB_VMODE_INTERLACED;
	if (mode->flags & DRM_MODE_FLAG_DBLSCAN)
		fb_mode.vmode |= FB_VMODE_DOUBLE;

	svga_set_timings(s3->state.vgabase, &s3_timing_regs, &fb_mode, hmul, 1,
			 1, 1, hmul, 0);

	/* Set interlaced mode start/end register */
	htotal = ((mode->htotal * hmul) / 8) - 5;
	vga_wcrt(s3->state.vgabase, 0x3C, (htotal + 1) / 2);

	/* Set Data Transfer Position */
	hsstart = (mode->hsync_start * hmul) / 8;
	/* + 2 is needed for Virge/VX, does no harm on other cards */
	value = clamp((htotal + hsstart + 1) / 2 + 2, hsstart + 4, htotal + 1);
	svga_wcrt_multi(s3->state.vgabase, s3_dtpc_regs, value);

	if (mode->type == DRM_MODE_TYPE_DRIVER) {
		/* Enable sync signals */
		svga_wcrt_mask(s3->state.vgabase, 0x17, 0x80, 0x80);
		/* Set horizontal sync polarity */
		if (mode->flags & DRM_MODE_FLAG_NHSYNC)
			svga_wcrt_mask(s3->state.vgabase, 0x11, 0x00, 0x04);
		else
			svga_wcrt_mask(s3->state.vgabase, 0x11, 0x04, 0x04);

		/* Set vertical sync polarity */
		if (mode->flags & DRM_MODE_FLAG_NVSYNC)
			svga_wcrt_mask(s3->state.vgabase, 0x11, 0x00, 0x02);
		else
			svga_wcrt_mask(s3->state.vgabase, 0x11, 0x02, 0x02);

		/* Set interlace */
		if (mode->flags & DRM_MODE_FLAG_INTERLACE)
			svga_wcrt_mask(s3->state.vgabase, 0x42, 0x20, 0x20);
		else
			svga_wcrt_mask(s3->state.vgabase, 0x42, 0x00, 0x20);
	}

	/* Re-enable screen and sync */
	svga_wcrt_mask(s3->state.vgabase, 0x17, 0x80, 0x80);
	svga_wseq_mask(s3->state.vgabase, 0x01, 0x00, 0x20);

	return 0;
}

static int __maybe_unused s3drm_blank(int blank_mode, struct s3_device *s3)
{
	struct drm_device *dev = &s3->dev;

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		drm_dbg(dev, "unblank\n");
		svga_wcrt_mask(s3->state.vgabase, 0x56, 0x00, 0x06);
		svga_wseq_mask(s3->state.vgabase, 0x01, 0x00, 0x20);
		break;
	case FB_BLANK_NORMAL:
		drm_dbg(dev, "blank\n");
		svga_wcrt_mask(s3->state.vgabase, 0x56, 0x00, 0x06);
		svga_wseq_mask(s3->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		drm_dbg(dev, "hsync\n");
		svga_wcrt_mask(s3->state.vgabase, 0x56, 0x02, 0x06);
		svga_wseq_mask(s3->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		drm_dbg(dev, "vsync\n");
		svga_wcrt_mask(s3->state.vgabase, 0x56, 0x04, 0x06);
		svga_wseq_mask(s3->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_POWERDOWN:
		drm_dbg(dev, "sync down\n");
		svga_wcrt_mask(s3->state.vgabase, 0x56, 0x06, 0x06);
		svga_wseq_mask(s3->state.vgabase, 0x01, 0x20, 0x20);
		break;
	}

	return 0;
}

static int s3drm_fb_helper_probe(struct drm_fb_helper *fb_helper, struct drm_fb_helper_surface_size *surf)
{
	struct s3_device *s3;

	s3 = s3_device_of_dev(fb_helper->dev);
	s3->dev.fb_helper->info = drm_fb_helper_alloc_info(fb_helper);
	return 0;
}

static const struct drm_fb_helper_funcs s3drm_fb_helper_funcs = {
	.fb_probe = s3drm_fb_helper_probe,
};

DEFINE_DRM_GEM_FOPS(s3drm_fops);

static const struct drm_driver s3drm_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name			= DRIVER_NAME,
	.desc			= DRIVER_DESC,
	.date			= DRIVER_DATE,
	.major			= DRIVER_MAJOR,
	.minor			= DRIVER_MINOR,
	.patchlevel		= DRIVER_PATCHLEVEL,
	.driver_features	= DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
	.lastclose		= drm_fb_helper_lastclose,
	.fops			= &s3drm_fops,
};

static const uint32_t s3_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const uint64_t s3_primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID,
};


static int s3_primary_plane_helper_atomic_check(struct drm_plane *plane,
						       struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *new_shadow_plane_state =
		to_drm_shadow_plane_state(new_plane_state);
	struct drm_framebuffer *new_fb = new_plane_state->fb;
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;

	int ret;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret)
		return ret;
	else if (!new_plane_state->visible)
		return 0;

	void *buf;

	/* format conversion necessary; reserve buffer */
	buf = drm_format_conv_state_reserve(&new_shadow_plane_state->fmtcnv_state,
					    new_fb->pitches[0], GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	return 0;
}

static void s3_primary_plane_helper_atomic_update(struct drm_plane *plane,
						  struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_device *dev = plane->dev;
	struct s3_device *s3 = s3_device_of_dev(dev);
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int ret, idx;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return;

	if (!drm_dev_enter(dev, &idx))
		goto out_drm_gem_fb_end_cpu_access;

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		struct drm_rect dst_clip = plane_state->dst;
		struct iosys_map dst = IOSYS_MAP_INIT_VADDR_IOMEM(s3->screen_base);

		if (!drm_rect_intersect(&dst_clip, &damage))
			continue;

		iosys_map_incr(&dst, drm_fb_clip_offset(fb->pitches[0], fb->format, &dst_clip));
		drm_fb_blit(&dst, fb->pitches, fb->format->format, shadow_plane_state->data,
			    fb, &damage, &shadow_plane_state->fmtcnv_state);
	}

	drm_dev_exit(idx);
out_drm_gem_fb_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
}

static void s3_primary_plane_helper_atomic_disable(struct drm_plane *plane,
						   struct drm_atomic_state *state)
{
	// TODO
	return;
}

static void s3_crtc_atomic_enable(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	s3drm_set_par(crtc, &crtc->mode);
	return;
}

static const struct drm_plane_helper_funcs s3_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = s3_primary_plane_helper_atomic_check,
	.atomic_update = s3_primary_plane_helper_atomic_update,
	.atomic_disable = s3_primary_plane_helper_atomic_disable,
};

static const struct drm_plane_funcs s3_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static enum drm_mode_status s3_crtc_helper_mode_valid(struct drm_crtc *crtc,
							     const struct drm_display_mode *mode)
{
	struct s3_device *s3 = s3_device_of_dev(crtc->dev);
	// s3drm_check_var();
	return drm_crtc_helper_mode_valid_fixed(crtc, mode, &s3->crtc.mode);
}

/*
 * The CRTC is always enabled. Screen updates are performed by
 * the primary plane's atomic_update function. Disabling clears
 * the screen in the primary plane's atomic_disable function.
 */
static const struct drm_crtc_helper_funcs s3_crtc_helper_funcs = {
	.mode_valid = s3_crtc_helper_mode_valid,
	.atomic_check = drm_crtc_helper_atomic_check,
	.atomic_enable = s3_crtc_atomic_enable,
};

static const struct drm_crtc_funcs s3_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs s3_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int s3_connector_helper_get_modes(struct drm_connector *connector)
{
	struct edid *edid;
	int count;

	if (!connector->ddc)
		goto err_drm_connector_update_edid_property;

	edid = drm_get_edid(connector, connector->ddc);
	if (!edid)
		return 0;

	count = drm_add_edid_modes(connector, edid);
	kfree(edid);

	return count;

err_drm_connector_update_edid_property:
	drm_connector_update_edid_property(connector, NULL);
	return 0;
}

static int drm_connector_helper_detect_from_ddc(struct drm_connector *connector,
					 struct drm_modeset_acquire_ctx *ctx,
					 bool force)
{
	struct i2c_adapter *ddc = connector->ddc;

	if (!ddc)
		return connector_status_unknown;

	if (drm_probe_ddc(ddc))
		return connector_status_connected;

	return connector_status_disconnected;
}

static const struct drm_connector_helper_funcs s3_connector_helper_funcs = {
	.get_modes = s3_connector_helper_get_modes,
	.detect_ctx = drm_connector_helper_detect_from_ddc,
};


static const struct drm_mode_config_funcs s3_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static const struct drm_connector_funcs s3_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* ------------------------------------------------------------------------- */
/* PCI                                                                       */

static int s3_load(struct s3_device *s3, const struct pci_device_id *id)
{
	int ret = 0;
	u8 regval, cr38, cr39;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct pci_bus_region bus_reg;
	struct resource vga_res;
	struct drm_device *dev = &s3->dev;
	struct pci_dev *pdev = to_pci_dev(dev->dev);

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot reserve framebuffer region\n");
		return ret;
	}

	/* Map physical IO memory address into kernel space */
	s3->screen_base = pcim_iomap(pdev, 0, 0);
	if (!s3->screen_base) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "iomap for framebuffer failed\n");
		return ret;
	}

	bus_reg.start = 0;
	bus_reg.end = 64 * 1024;

	vga_res.flags = IORESOURCE_IO;

	pcibios_bus_to_resource(pdev->bus, &vga_res, &bus_reg);

	s3->state.vgabase = (void __iomem *) (unsigned long) vga_res.start;

	/* Unlock regs */
	cr38 = vga_rcrt(s3->state.vgabase, 0x38);
	cr39 = vga_rcrt(s3->state.vgabase, 0x39);
	vga_wseq(s3->state.vgabase, 0x08, 0x06);
	vga_wcrt(s3->state.vgabase, 0x38, 0x48);
	vga_wcrt(s3->state.vgabase, 0x39, 0xA5);

	/* Identify chip type */
	s3->chip = id->driver_data & CHIP_MASK;
	s3->rev = vga_rcrt(s3->state.vgabase, 0x2f);
	if (s3->chip & CHIP_UNDECIDED_FLAG)
		s3->chip = s3_identification(s3);

	/* Find MCLK frequency */
	regval = vga_rseq(s3->state.vgabase, 0x10);
	s3->mclk_freq = ((vga_rseq(s3->state.vgabase, 0x11) + 2) * 14318) / ((regval & 0x1F)  + 2);
	s3->mclk_freq = s3->mclk_freq >> (regval >> 5);

	/* Restore locks */
	vga_wcrt(s3->state.vgabase, 0x38, cr38);
	vga_wcrt(s3->state.vgabase, 0x39, cr39);

	/* Setup DDC via I2C */
	resource_size_t smem_start = pci_resource_start(pdev, 0);
	if (s3drm_ddc_needs_mmio(s3->chip)) {
		s3->mmio = ioremap(smem_start + MMIO_OFFSET, MMIO_SIZE);
		if (s3->mmio)
			svga_wcrt_mask(s3->state.vgabase, 0x53, 0x08, 0x08);	/* enable MMIO */
		else
			dev_err(&pdev->dev, "unable to map MMIO at 0x%zx, disabling DDC",
				(size_t)smem_start + MMIO_OFFSET);
	}

	s3drm_setup_ddc_bus(s3);
	ret = drmm_mode_config_init(dev);
	if (ret)
		return ret;

	dev->mode_config.min_width = 640;
	dev->mode_config.max_width = 1024;
	dev->mode_config.min_height = 480;
	dev->mode_config.max_height = 768;
	dev->mode_config.preferred_depth = 32;
	dev->mode_config.funcs = &s3_mode_config_funcs;

	/* Primary plane */

	primary_plane = &s3->primary_plane;
	ret = drm_universal_plane_init(dev, primary_plane, 0, &s3_primary_plane_funcs,
				       s3_formats, ARRAY_SIZE(s3_formats),
				       s3_primary_plane_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);

	if (ret)
		return ret;
	drm_plane_helper_add(primary_plane, &s3_primary_plane_helper_funcs);

	drm_plane_enable_fb_damage_clips(primary_plane);

	/* CRTC */

	crtc = &s3->crtc;
	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					&s3_crtc_funcs, NULL);

	if (ret)
		return ret;
	drm_crtc_helper_add(crtc, &s3_crtc_helper_funcs);

	/* Encoder */

	encoder = &s3->encoder;
	ret = drm_encoder_init(dev, encoder, &s3_encoder_funcs,
			       DRM_MODE_ENCODER_DAC, NULL);

	if (ret)
		return ret;
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	/* Connector */

	connector = &s3->connector;
	ret = drm_connector_init_with_ddc(dev, connector, &s3_connector_funcs,
					  DRM_MODE_CONNECTOR_VGA, &s3->ddc_adapter);

	if (ret)
		return ret;
	drm_connector_helper_add(connector, &s3_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;

	ret = drm_connector_attach_encoder(connector, encoder);

	if (ret)
		return ret;

	drm_mode_config_reset(dev);
	drm_kms_helper_poll_init(dev);

	return ret;
}

// TODO
static int s3_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct s3_device *s3;
	int ret;

	/* Ignore secondary VGA device because there is no VGA arbitration */
	if (!svga_primary_device(pdev)) {
		dev_info(&(pdev->dev), "ignoring secondary device\n");
		return -ENODEV;
	}

	/* Allocate and fill driver data structure */
	s3 = devm_drm_dev_alloc(&pdev->dev, &s3drm_driver, struct s3_device, dev);
	if (!s3)
		return -ENOMEM;

	/* Prepare PCI device */
	ret = pcim_enable_device(pdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot enable PCI device\n");
		return ret;
	}

	/* Record a reference to the driver data */
	pci_set_drvdata(pdev, s3);

	s3->chip = s3_identification(s3);

	ret = s3_load(s3, id);
	if (ret)
		return ret;

	ret = drm_dev_register(&s3->dev, 0);
	if (ret)
		return ret;

	drm_fbdev_generic_setup(&s3->dev, 32);

	if (s3->chip == CHIP_UNKNOWN)
		drm_info(&s3->dev, "unknown chip, CR2D=%x, CR2E=%x, CRT2F=%x, CRT30=%x\n",
			vga_rcrt(s3->state.vgabase, 0x2d),
			vga_rcrt(s3->state.vgabase, 0x2e),
			vga_rcrt(s3->state.vgabase, 0x2f),
			vga_rcrt(s3->state.vgabase, 0x30));

	return 0;
	// TODO: Error handling.
}

static void s3_pci_remove(struct pci_dev *pdev)
{
	struct s3_device *s3 = pci_get_drvdata(pdev);
	drm_dev_unregister(&s3->dev);
	drm_atomic_helper_shutdown(&s3->dev);
}

static void s3_pci_shutdown(struct pci_dev *pdev)
{
	struct s3_device *s3 = pci_get_drvdata(pdev);
	drm_atomic_helper_shutdown(&s3->dev);
}

static const struct pci_device_id s3_devices[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8810), .driver_data = CHIP_XXX_TRIO },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8811), .driver_data = CHIP_XXX_TRIO },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8814), .driver_data = CHIP_767_TRIO64UVP },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8901), .driver_data = CHIP_XXX_TRIO64V2_DXGX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A13), .driver_data = CHIP_36X_TRIO3D_1X_2X },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8904), .driver_data = CHIP_365_TRIO3D },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8812), .driver_data = CHIP_M65_AURORA64VP },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8902), .driver_data = CHIP_551_PLATO_PX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x5631), .driver_data = CHIP_325_VIRGE },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x883D), .driver_data = CHIP_988_VIRGE_VX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A01), .driver_data = CHIP_XXX_VIRGE_DXGX },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A10), .driver_data = CHIP_357_VIRGE_GX2 },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A11), .driver_data = CHIP_359_VIRGE_GX2P },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8A12), .driver_data = CHIP_359_VIRGE_GX2P },
	{ PCI_DEVICE(PCI_VENDOR_ID_S3, 0x8C01), .driver_data = CHIP_260_VIRGE_MX },
	{ /* End of list */ }
};

static struct pci_driver s3drm_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= s3_devices,
	.probe		= s3_pci_probe,
	.remove		= s3_pci_remove,
	.shutdown	= s3_pci_shutdown,
};

/* ------------------------------------------------------------------------- */

drm_module_pci_driver(s3drm_pci_driver);

MODULE_DEVICE_TABLE(pci, s3_devices);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE(DRIVER_LICENSE);
MODULE_DESCRIPTION(DRIVER_DESC);
