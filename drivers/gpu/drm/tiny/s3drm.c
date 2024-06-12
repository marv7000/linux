/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DRM driver for S3 Trio
 *
 * Copyright 2024 Marvin Friedrich <contact@marvinf.com>
 *
 * This code has been adapted from the fbdev version in
 * linux/drivers/video/fbdev/s3fb.c
 */

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
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fbdev_generic.h>

#include <linux/pci.h>
#include <linux/i2c-algo-bit.h>
#include <linux/console.h>
#include <linux/aperture.h>
#include <linux/init.h>
#include <linux/svga.h>
#include <linux/pm.h>

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
#ifdef CONFIG_TINYDRM_S3_DDC
	u8 __iomem *mmio;
	bool ddc_registered;
	struct i2c_adapter ddc_adapter;
	struct i2c_algo_bit_data ddc_algo;
#endif

	struct drm_device dev;
	struct drm_fb_helper helper;
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

/* ------------------------------------------------------------------------- */
/* DDC                                                                       */

#ifdef CONFIG_TINYDRM_S3_DDC

#define DDC_REG		0xaa		/* Trio 3D/1X/2X */
#define DDC_MMIO_REG	0xff20		/* all other chips */
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

static int s3drm_setup_ddc_bus(struct s3_device *s3dev)
{
	strscpy(s3dev->ddc_adapter.name, s3dev->helper.info->fix.id,
		sizeof(s3dev->ddc_adapter.name));
	s3dev->ddc_adapter.owner		= THIS_MODULE;
	s3dev->ddc_adapter.algo_data	= &s3dev->ddc_algo;
	s3dev->ddc_adapter.dev.parent	= s3dev->helper.info->device;
	s3dev->ddc_algo.setsda		= s3drm_ddc_setsda;
	s3dev->ddc_algo.setscl		= s3drm_ddc_setscl;
	s3dev->ddc_algo.getsda		= s3drm_ddc_getsda;
	s3dev->ddc_algo.getscl		= s3drm_ddc_getscl;
	s3dev->ddc_algo.udelay		= 10;
	s3dev->ddc_algo.timeout		= 20;
	s3dev->ddc_algo.data		= s3dev;

	i2c_set_adapdata(&s3dev->ddc_adapter, s3dev);

	/*
	 * some Virge cards have external MUX to switch chip I2C bus between
	 * DDC and extension pins - switch it do DDC
	 */
/*	vga_wseq(par->state.vgabase, 0x08, 0x06); - not needed, already unlocked */
	if (s3dev->chip == CHIP_357_VIRGE_GX2 ||
	    s3dev->chip == CHIP_359_VIRGE_GX2P ||
	    s3dev->chip == CHIP_260_VIRGE_MX)
		svga_wseq_mask(s3dev->state.vgabase, 0x0d, 0x01, 0x03);
	else
		svga_wseq_mask(s3dev->state.vgabase, 0x0d, 0x00, 0x03);
	/* some Virge need this or the DDC is ignored */
	svga_wcrt_mask(s3dev->state.vgabase, 0x5c, 0x03, 0x03);

	return i2c_bit_add_bus(&s3dev->ddc_adapter);
}
#endif /* CONFIG_DRM_S3_DDC */

/* ------------------------------------------------------------------------- */
/* DRM                                                                       */

/* Set font in S3 fast text mode */
static void s3drm_settile_fast(struct fb_info *info, struct fb_tilemap *map)
{
	const u8 *font = map->data;
	u8 __iomem *fb = (u8 __iomem *) info->screen_base;
	int i, c;

	if ((map->width != 8) || (map->height != 16) ||
	    (map->depth != 1) || (map->length != 256)) {
		drm_err(info, "unsupported font parameters: width %d, height %d, depth %d, length %d\n",
		       map->width, map->height, map->depth, map->length);
		return;
	}

	fb += 2;
	for (i = 0; i < map->height; i++) {
		for (c = 0; c < map->length; c++) {
			fb_writeb(font[c * map->height + i], fb + c * 4);
		}
		fb += 1024;
	}
}

static void s3drm_tilecursor(struct fb_info *info, struct fb_tilecursor *cursor)
{
	struct s3_device *par = info->par;
	svga_tilecursor(par->state.vgabase, info, cursor);
}

static struct fb_tile_ops s3drm_tile_ops = {
	.fb_settile	= svga_settile,
	.fb_tilecopy	= svga_tilecopy,
	.fb_tilefill    = svga_tilefill,
	.fb_tileblit    = svga_tileblit,
	.fb_tilecursor  = s3drm_tilecursor,
	.fb_get_tilemax = svga_get_tilemax,
};

static struct fb_tile_ops s3drm_fast_tile_ops = {
	.fb_settile	= s3drm_settile_fast,
	.fb_tilecopy	= svga_tilecopy,
	.fb_tilefill    = svga_tilefill,
	.fb_tileblit    = svga_tileblit,
	.fb_tilecursor  = s3drm_tilecursor,
	.fb_get_tilemax = svga_get_tilemax,
};

/* Set a colour register */
static int s3drm_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			   u_int transp, struct fb_info *fb)
{
	switch (fb->var.bits_per_pixel) {
	case 0:
	case 4:
		if (regno >= 16)
			return -EINVAL;

		if ((fb->var.bits_per_pixel == 4) && (fb->var.nonstd == 0)) {
			outb(0xF0, VGA_PEL_MSK);
			outb(regno*16, VGA_PEL_IW);
		} else {
			outb(0x0F, VGA_PEL_MSK);
			outb(regno, VGA_PEL_IW);
		}
		outb(red >> 10, VGA_PEL_D);
		outb(green >> 10, VGA_PEL_D);
		outb(blue >> 10, VGA_PEL_D);
		break;
	case 8:
		if (regno >= 256)
			return -EINVAL;

		outb(0xFF, VGA_PEL_MSK);
		outb(regno, VGA_PEL_IW);
		outb(red >> 10, VGA_PEL_D);
		outb(green >> 10, VGA_PEL_D);
		outb(blue >> 10, VGA_PEL_D);
		break;
	case 16:
		if (regno >= 16)
			return 0;

		if (fb->var.green.length == 5)
			((u32*)fb->pseudo_palette)[regno] = ((red & 0xF800) >> 1) |
				((green & 0xF800) >> 6) | ((blue & 0xF800) >> 11);
		else if (fb->var.green.length == 6)
			((u32*)fb->pseudo_palette)[regno] = (red & 0xF800) |
				((green & 0xFC00) >> 5) | ((blue & 0xF800) >> 11);
		else return -EINVAL;
		break;
	case 24:
	case 32:
		if (regno >= 16)
			return 0;

		((u32*)fb->pseudo_palette)[regno] = ((red & 0xFF00) << 8) |
			(green & 0xFF00) | ((blue & 0xFF00) >> 8);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int __maybe_unused s3drm_blank(int blank_mode, struct s3_device *s3dev)
{
	struct drm_device *info = &s3dev->dev;

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		drm_dbg(info, "unblank\n");
		svga_wcrt_mask(s3dev->state.vgabase, 0x56, 0x00, 0x06);
		svga_wseq_mask(s3dev->state.vgabase, 0x01, 0x00, 0x20);
		break;
	case FB_BLANK_NORMAL:
		drm_dbg(info, "blank\n");
		svga_wcrt_mask(s3dev->state.vgabase, 0x56, 0x00, 0x06);
		svga_wseq_mask(s3dev->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		drm_dbg(info, "hsync\n");
		svga_wcrt_mask(s3dev->state.vgabase, 0x56, 0x02, 0x06);
		svga_wseq_mask(s3dev->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		drm_dbg(info, "vsync\n");
		svga_wcrt_mask(s3dev->state.vgabase, 0x56, 0x04, 0x06);
		svga_wseq_mask(s3dev->state.vgabase, 0x01, 0x20, 0x20);
		break;
	case FB_BLANK_POWERDOWN:
		drm_dbg(info, "sync down\n");
		svga_wcrt_mask(s3dev->state.vgabase, 0x56, 0x06, 0x06);
		svga_wseq_mask(s3dev->state.vgabase, 0x01, 0x20, 0x20);
		break;
	}

	return 0;
}

/* Pan the display */
static int s3fb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct s3_device *par = info->par;
	unsigned int offset;

	/* Calculate the offset */
	if (info->var.bits_per_pixel == 0) {
		offset = (var->yoffset / 16) * (info->var.xres_virtual / 2)
		       + (var->xoffset / 2);
		offset = offset >> 2;
	} else {
		offset = (var->yoffset * info->fix.line_length) +
			 (var->xoffset * info->var.bits_per_pixel / 8);
		offset = offset >> 2;
	}

	/* Set the offset */
	svga_wcrt_multi(par->state.vgabase, s3_start_address_regs, offset);

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

/* ------------------------------------------------------------------------- */
/* PCI                                                                       */

// TODO
static int s3_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct pci_bus_region bus_reg;
	struct resource vga_res;
	struct s3_device *s3;
	struct fb_info *info;
	int ret;
	u8 regval, cr38, cr39;

	ret = drm_aperture_remove_conflicting_pci_framebuffers(pdev, &s3drm_driver);
	if (ret)
		return ret;

	/* Allocate and fill driver data structure */
	s3 = devm_drm_dev_alloc(&pdev->dev, &s3drm_driver, struct s3_device, dev);
	if (!s3)
		return -ENOMEM;

	/* Call fb_helper functions to allocate fb_info */
	drm_fb_helper_prepare(&s3->dev, &s3->helper, 32, &s3drm_fb_helper_funcs);
	ret = drm_fb_helper_init(&s3->dev, &s3->helper);
	if (ret) {
		dev_err(&(pdev->dev), "failed to initialize fb helper\n");
		return ret;
	}

	/* Use the fb_info struct from the device */
	info = s3->dev.fb_helper->info;

	/* Ignore secondary VGA device because there is no VGA arbitration */
	if (!svga_primary_device(pdev)) {
		dev_info(&(pdev->dev), "ignoring secondary device\n");
		return -ENODEV;
	}

	/* Prepare PCI device */
	ret = pcim_enable_device(pdev);
	if (ret < 0) {
		dev_err(info->device, "cannot enable PCI device\n");
		return ret;
	}

	ret = pci_request_regions(pdev, DRIVER_NAME);
	if (ret < 0) {
		dev_err(info->device, "cannot reserve framebuffer region\n");
		return ret;
	}

	mutex_init(&s3->open_lock);

	info->flags = FBINFO_PARTIAL_PAN_OK | FBINFO_HWACCEL_YPAN;
	info->fix.smem_start = pci_resource_start(pdev, 0);
	info->fix.smem_len = pci_resource_len(pdev, 0);

	/* Map physical IO memory address into kernel space */
	info->screen_base = pci_iomap_wc(pdev, 0, 0);
	if (!info->screen_base) {
		ret = -ENOMEM;
		dev_err(info->device, "iomap for framebuffer failed\n");
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

	/* Find how much physical memory there is on card */
	/* 0x36 register is accessible even if other registers are locked */
	regval = vga_rcrt(s3->state.vgabase, 0x36);
	if (s3->chip == CHIP_360_TRIO3D_1X ||
	    s3->chip == CHIP_362_TRIO3D_2X ||
	    s3->chip == CHIP_368_TRIO3D_2X ||
	    s3->chip == CHIP_365_TRIO3D) {
		switch ((regval & 0xE0) >> 5) {
		case 0: /* 8MB -- only 4MB usable for display */
		case 1: /* 4MB with 32-bit bus */
		case 2:	/* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 4: /* 2MB on 365 Trio3D */
		case 6: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		}
	} else if (s3->chip == CHIP_357_VIRGE_GX2 ||
		   s3->chip == CHIP_359_VIRGE_GX2P ||
		   s3->chip == CHIP_260_VIRGE_MX) {
		switch ((regval & 0xC0) >> 6) {
		case 1: /* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 3: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		}
	} else if (s3->chip == CHIP_988_VIRGE_VX) {
		switch ((regval & 0x60) >> 5) {
		case 0: /* 2MB */
			info->screen_size = 2 << 20;
			break;
		case 1: /* 4MB */
			info->screen_size = 4 << 20;
			break;
		case 2: /* 6MB */
			info->screen_size = 6 << 20;
			break;
		case 3: /* 8MB */
			info->screen_size = 8 << 20;
			break;
		}
		/* off-screen memory */
		regval = vga_rcrt(s3->state.vgabase, 0x37);
		switch ((regval & 0x60) >> 5) {
		case 1: /* 4MB */
			info->screen_size -= 4 << 20;
			break;
		case 2: /* 2MB */
			info->screen_size -= 2 << 20;
			break;
		}
	} else
		info->screen_size = s3_memsizes[regval >> 5] << 10;

	info->fix.smem_len = info->screen_size;

	/* Find MCLK frequency */
	regval = vga_rseq(s3->state.vgabase, 0x10);
	s3->mclk_freq = ((vga_rseq(s3->state.vgabase, 0x11) + 2) * 14318) / ((regval & 0x1F)  + 2);
	s3->mclk_freq = s3->mclk_freq >> (regval >> 5);

	/* Restore locks */
	vga_wcrt(s3->state.vgabase, 0x38, cr38);
	vga_wcrt(s3->state.vgabase, 0x39, cr39);

	strscpy(info->fix.id, s3_names[s3->chip]);
	info->fix.mmio_start = 0;
	info->fix.mmio_len = 0;
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	info->fix.ypanstep = 0;
	info->fix.accel = FB_ACCEL_NONE;
	info->pseudo_palette = (void*)(s3->pseudo_palette);
	info->var.bits_per_pixel = 8;

#ifdef CONFIG_TINYDRM_S3_DDC
	/* Enable MMIO if needed */
	if (s3drm_ddc_needs_mmio(s3->chip)) {
		s3->mmio = ioremap(info->fix.smem_start + MMIO_OFFSET, MMIO_SIZE);
		if (s3->mmio)
			svga_wcrt_mask(s3->state.vgabase, 0x53, 0x08, 0x08);	/* enable MMIO */
		else
			dev_err(info->device, "unable to map MMIO at 0x%lx, disabling DDC",
				info->fix.smem_start + MMIO_OFFSET);
	}
	if (!s3drm_ddc_needs_mmio(s3->chip) || s3->mmio)
		if (s3drm_setup_ddc_bus(s3) == 0) {
			u8 *edid = fb_ddc_read(&s3->ddc_adapter);
			s3->ddc_registered = true;
			if (edid) {
				fb_edid_to_monspecs(edid, &info->monspecs);
				kfree(edid);
				if (!info->monspecs.modedb)
					dev_err(info->device, "error getting mode database\n");
				else {
					const struct fb_videomode *m;

					fb_videomode_to_modelist(info->monspecs.modedb,
								 info->monspecs.modedb_len,
								 &info->modelist);
					m = fb_find_best_display(&info->monspecs, &info->modelist);
					if (m) {
						fb_videomode_to_var(&info->var, m);
					}
				}
			}
		}
#endif

	drm_fbdev_generic_setup(&s3->dev, 32);

	drm_info(info, "%s on %s, %d MB RAM, %d MHz MCLK\n",
		info->fix.id, pci_name(pdev),
		info->fix.smem_len >> 20, (s3->mclk_freq + 500) / 1000);

	if (s3->chip == CHIP_UNKNOWN)
		drm_info(info, "unknown chip, CR2D=%x, CR2E=%x, CRT2F=%x, CRT30=%x\n",
			vga_rcrt(s3->state.vgabase, 0x2d),
			vga_rcrt(s3->state.vgabase, 0x2e),
			vga_rcrt(s3->state.vgabase, 0x2f),
			vga_rcrt(s3->state.vgabase, 0x30));

	/* Record a reference to the driver data */
	pci_set_drvdata(pdev, s3);

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
