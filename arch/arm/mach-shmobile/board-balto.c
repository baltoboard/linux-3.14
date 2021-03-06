/*
 * BALTO board support
 *
 * Copyright (C) 2014 Arrow
 * Copyright (C) 2014 Pierluigi Passaro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/sh_eth.h>
#include <asm/mach/map.h>
#include "common.h"
#include "irqs.h"
#include "r7s72100.h"
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <linux/spi/rspi.h>
#include <linux/spi/sh_spibsc.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/serial_sci.h>
#include <linux/i2c.h>
#include <linux/i2c-riic.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sh_mmcif.h>
#include <linux/mmc/sh_mobile_sdhi.h>
#include <linux/mfd/tmio.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/platform_data/at24.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_data/sh_adc.h>
#include <linux/usb/r8a66597.h>
#include <linux/platform_data/dma-rza1.h>
#include <linux/input/edt-ft5x06.h>
#include <linux/uio_driver.h>
#include <clocksource/sh_ostm.h>
#include <video/vdc5fb.h>
#include <sound/sh_scux.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/dma-mapping.h>
#include <linux/can/platform/rza1_can.h>
#include <linux/clk.h>
#include <linux/leds.h>
#include <linux/input/vgg804808_ts.h>
#include <linux/platform_data/gpio_backlight.h>

#define EXTCLK 12500000

#define XIP_KERNEL_WITHOUT_EXTERNAL_RAM (defined(CONFIG_XIP_KERNEL) && (CONFIG_PHYS_OFFSET == 0x20000000))

static int usbgs = -1;
static int __init early_usbgs(char *str)
{
	usbgs = 0;
	get_option(&str, &usbgs);
	return 0;
}
early_param("usbgs", early_usbgs);

static int panel = -1;
static int __init early_panel(char *str)
{
	get_option(&str, &panel);
	return 0;
}
early_param("panel", early_panel);


static struct map_desc rza1_io_desc[] __initdata = {
	/* create a 1:1 entity map for 0xe8xxxxxx
	 * used by INTC.
	 */
	{
		.virtual	= 0xe8000000,
		.pfn		= __phys_to_pfn(0xe8000000),
		.length		= SZ_256M,
		.type		= MT_DEVICE_NONSHARED
	},
	/* create a 1:1 entity map for 0xfcfexxxx
	 * used by MSTP, CPG.
	 */
	{
		.virtual	= 0xfcfe0000,
		.pfn		= __phys_to_pfn(0xfcfe0000),
		.length		= SZ_64K,
		.type		= MT_DEVICE_NONSHARED
	},
#ifdef CONFIG_CACHE_L2X0
	/* create a 1:1 entity map for 0x3ffffxxx
	 * used by L2CC (PL310).
	 */
	{
		.virtual	= 0xfffee000,
		.pfn		= __phys_to_pfn(0x3ffff000),
		.length		= SZ_4K,
		.type		= MT_DEVICE_NONSHARED
	},
#endif
};

static void __init rza1_map_io(void)
{
#ifdef CONFIG_DEBUG_LL
	/* Note: Becase we defined a .map_io handler, we must manually set our
	   SCIF3 memory mapping here. see arch/arm/mm/mmu.c */
	debug_ll_io_init();
#endif
	iotable_init(rza1_io_desc, ARRAY_SIZE(rza1_io_desc));
}


/* DMA */
#define CHCFG(reqd_v, loen_v, hien_v, lvl_v, am_v, sds_v, dds_v, tm_v)\
	{								\
		.reqd	=	reqd_v,					\
		.loen	=	loen_v,					\
		.hien	=	hien_v,					\
		.lvl	=	lvl_v,					\
		.am	=	am_v,					\
		.sds	=	sds_v,					\
		.dds	=	dds_v,					\
		.tm	=	tm_v,					\
	}
#define DMARS(rid_v, mid_v)	\
	{								\
		.rid	= rid_v,					\
		.mid	= mid_v,					\
	}

static const struct rza1_dma_slave_config rza1_dma_slaves[] = {
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_TX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI0_RX,
		.addr		= 0xe804e030,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x30),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_TX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_SDHI1_RX,
		.addr		= 0xe804e830,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x31),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_TX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_MMCIF_RX,
		.addr		= 0xe804c834,
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x32),
	},
	{
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SSI0,
		.addr		= 0xe820b018,		/* SSIFTDR_0 */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x1, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_MEM_SRC1,
		.addr		= 0xe820970c,		/* DMATD1_CIM */
		.chcfg		= CHCFG(0x1, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x1, 0x41),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SSI0_MEM,
		.addr		= 0xe820b01c,		/* SSIFRDR_0 */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x2, 0x2, 0x0),
		.dmars		= DMARS(0x2, 0x38),
	}, {
		.slave_id	= RZA1DMA_SLAVE_PCM_SRC0_MEM,
		.addr		= 0xe8209718,		/* DMATU0_CIM */
		.chcfg		= CHCFG(0x0, 0x0, 0x1, 0x1, 0x1, 0x1, 0x1, 0x0),
		.dmars		= DMARS(0x2, 0x40),
	},
};

static const struct rza1_dma_pdata dma_pdata __initconst = {
	.slave		= rza1_dma_slaves,
	.slave_num	= ARRAY_SIZE(rza1_dma_slaves),
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	.channel_num	= 6,	/* Less channels means less RAM (2 for SDHI, 4 for Audio) */
#else
	.channel_num	= 16,	/* 16 MAX channels */
#endif
};

static const struct resource rza1_dma_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8200000, 0x1000),
	DEFINE_RES_MEM(0xfcfe1000, 0x1000),
	DEFINE_RES_NAMED(gic_iid(41), 16, NULL, IORESOURCE_IRQ),
	DEFINE_RES_IRQ(gic_iid(57)),	/* DMAERR */
};

static const struct platform_device_info dma_info  __initconst = {
	.name		= "rza1-dma",
	.id		= -1,
	.res		= rza1_dma_resources,
	.num_res	= ARRAY_SIZE(rza1_dma_resources),
	.data		= &dma_pdata,
	.size_data	= sizeof(dma_pdata),
};

/* Video */
#define	P1CLK			((EXTCLK * 30) / 6)
#define	PIXCLOCK(hz, div)	\
	(u32)(1000000000000 / ((double)(hz) / (double)(div)))

struct pfc_pinmux_assign {
	int port;	/* enum */
	int mode;	/* enum */
	int opts;
};

static struct pfc_pinmux_assign lcd0_common[] = {
	{ P3_0,  ALT1, DIIO_PBDC_DIS},	/* LCD0_CLK */
	{ P3_1,  ALT1, DIIO_PBDC_DIS},	/* LCD0_VSYNC */
	{ P3_2,  ALT1, DIIO_PBDC_DIS},	/* LCD0_HSYNC */
	{ P3_7,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DE */
	{ P3_8,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA0 */
	{ P3_9,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA1 */
	{ P3_10, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA2 */
	{ P3_11, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA3 */
	{ P3_12, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA4 */
	{ P3_13, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA5 */
	{ P3_14, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA6 */
	{ P3_15, ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA7 */
	{ P4_0,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA8 */
	{ P4_1,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA9 */
	{ P4_2,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA10 */
	{ P4_3,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA11 */
	{ P4_4,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA12 */
	{ P4_5,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA13 */
	{ P4_6,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA14 */
	{ P4_7,  ALT1, DIIO_PBDC_DIS},	/* LCD0_DATA15 */
};

static struct pfc_pinmux_assign lvds_common[] = {
	{ P5_0, ALT1, DIR_LVDS},	/* TXCLKOUTP */
	{ P5_1, ALT1, DIR_LVDS},	/* TXCLKOUTM */
	{ P5_2, ALT1, DIR_LVDS},	/* TXOUT2P */
	{ P5_3, ALT1, DIR_LVDS},	/* TXOUT2M */
	{ P5_4, ALT1, DIR_LVDS},	/* TXOUT1P */
	{ P5_5, ALT1, DIR_LVDS},	/* TXOUT1M */
	{ P5_6, ALT1, DIR_LVDS},	/* TXOUT0P */
	{ P5_7, ALT1, DIR_LVDS},	/* TXOUT0M */
};

static void vdc5fb_pinmux(struct pfc_pinmux_assign *pf, size_t num)
{
	size_t n;

	for (n = 0; n < num; pf++, n++)
		r7s72100_pfc_pin_assign(pf->port, pf->mode, pf->opts);
}

#define VDC5_LCD0_BPP 16 /* 16bpp or 32bpp */
#define VDC5_LCD0_FBSIZE (800*480*VDC5_LCD0_BPP/8)
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
#define VDC5_LCD0_FB_ADDR 0	/* allocate at probe */
#else
#define VDC5_LCD0_FB_ADDR ((0x20A00000 - VDC5_LCD0_FBSIZE) & PAGE_MASK)	/* Place at end of internal RAM */
#endif

static const struct resource vdc5fb_lcd0_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	DEFINE_RES_MEM_NAMED(VDC5_LCD0_FB_ADDR, VDC5_LCD0_FBSIZE, "vdc5fb.0: fb"),
	DEFINE_RES_NAMED(75, 23, "vdc5fb.0: irq", IORESOURCE_IRQ),
	DEFINE_RES_MEM_NAMED(0xfcff7a30, 0x0000002d, "lvds: reg"),
};

static int vdc5fb_pinmux_lcd0(struct platform_device *pdev)
{
	vdc5fb_pinmux(lcd0_common, ARRAY_SIZE(lcd0_common));

	return 0;
}

static struct fb_videomode videomode_lcd0 = {
	.name		= "LCD0",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(P1CLK, 2),
	.left_margin	= 80,
	.right_margin	= 80,
	.upper_margin	= 15,
	.lower_margin	= 15,
	.hsync_len	= 96,
	.vsync_len	= 15,
	.sync		= 0,
	.vmode		= 0,
	.flag		= 0,
};

static const struct vdc5fb_pdata vdc5fb_lcd0_pdata = {
	.name			= "LCD0",
	.videomode		= &videomode_lcd0,
	.panel_icksel		= ICKSEL_P1CLK,
	.bpp			= VDC5_LCD0_BPP,
	.panel_width		= 184,	/* mm, unused */
	.panel_height		= 132,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB565,
	.use_lvds		= 0,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_STVA,	/* LCD0_VSYNC */
		[LCD_TCON1]	= TCON_SEL_STH,		/* LCD0_HSYNC */
		[LCD_TCON2]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON3]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON6]	= TCON_SEL_DE,		/* LCD0_EN */
	},
	.pinmux			= vdc5fb_pinmux_lcd0,
	.layers			= {
		/* Graphics 2 - Image Synthesizer */
		/* Full LCD Panel - will be /dev/fb0 */
		[2].xres	= 800,
		[2].yres	= 480,
		[2].x_offset	= 0,
		[2].y_offset	= 0,
		[2].format	= GR_FORMAT(GR_FORMAT_RGB565) | GR_RDSWA(6),
		[2].bpp		= VDC5_LCD0_BPP,
		[2].base	= VDC5_LCD0_FB_ADDR,
		[2].blend	 = 0,
	},
};

static const struct platform_device_info vdc5fb_lcd0_info __initconst = {
	.name		= "vdc5fb",
	.id		= 0,
	.res		= vdc5fb_lcd0_resources,
	.num_res	= ARRAY_SIZE(vdc5fb_lcd0_resources),
	.data		= &vdc5fb_lcd0_pdata,
	.size_data	= sizeof(vdc5fb_lcd0_pdata),
	.dma_mask	= DMA_BIT_MASK(32),	/* only needed if not hardcoding fb */
};

#define VDC5_LVDS_BPP 32 /* 16bpp or 32bpp */
#define VDC5_LVDS_FBSIZE (800*480*VDC5_LVDS_BPP/8)
#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
#define VDC5_LVDS_FB_ADDR 0	/* allocate at probe */
#else
#define VDC5_LVDS_FB_ADDR ((0x20A00000 - VDC5_LVDS_FBSIZE) & PAGE_MASK)	/* Place at end of internal RAM */
#endif

static const struct resource vdc5fb_lvds_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xfcff6000, 0x00002000, "vdc5fb.0: reg"),
	DEFINE_RES_MEM_NAMED(VDC5_LVDS_FB_ADDR, VDC5_LVDS_FBSIZE, "vdc5fb.0: fb"),
	DEFINE_RES_NAMED(75, 23, "vdc5fb.0: irq", IORESOURCE_IRQ),
	DEFINE_RES_MEM_NAMED(0xfcff7a30, 0x0000002d, "lvds: reg"),
};

static int vdc5fb_pinmux_lvds(struct platform_device *pdev)
{
	vdc5fb_pinmux(lvds_common, ARRAY_SIZE(lvds_common));

	return 0;
}

static struct fb_videomode videomode_lvds = {
	.name		= "LVDS",
	.refresh	= 60,
	.xres		= 800,
	.yres		= 480,
	.pixclock	= PIXCLOCK(33264000, 1),
	.left_margin	= 85,
	.right_margin	= 85,
	.upper_margin	= 15,
	.lower_margin	= 15,
	.hsync_len	= 86,
	.vsync_len	= 15,
	.sync		= 0,
	.vmode		= 0,
	.flag		= 0,
};

static const struct vdc5fb_pdata vdc5fb_lvds_pdata = {
	.name			= "LVDS",
	.videomode		= &videomode_lvds,
	.panel_icksel		= OCKSEL_PLL_DIV7,
	.bpp			= VDC5_LVDS_BPP,
	.panel_width		= 184,	/* mm, unused */
	.panel_height		= 132,	/* mm, unused */
	.flm_max		= 1,
	.out_format		= OUT_FORMAT_RGB888,
	.use_lvds		= 1,
	.tcon_sel		= {
		[LCD_TCON0]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON1]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON2]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON3]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON4]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON5]	= TCON_SEL_UNUSED,	/* UNUSED */
		[LCD_TCON6]	= TCON_SEL_UNUSED,	/* UNUSED */
	},
	.pinmux			= vdc5fb_pinmux_lvds,
	.layers			= {
		/* Graphics 2 - Image Synthesizer */
		/* Full LCD Panel - will be /dev/fb0 */
		[2].xres	= 800,
		[2].yres	= 480,
		[2].x_offset	= 0,
		[2].y_offset	= 0,
		[2].format	= GR_FORMAT(GR_FORMAT_ARGB8888) | GR_RDSWA(4),
		[2].bpp		= VDC5_LVDS_BPP,
		[2].base	= VDC5_LVDS_FB_ADDR,
		[2].blend	 = 0,
	},
};

static const struct platform_device_info vdc5fb_lvds_info __initconst = {
	.name		= "vdc5fb",
	.id		= 0,
	.res		= vdc5fb_lvds_resources,
	.num_res	= ARRAY_SIZE(vdc5fb_lvds_resources),
	.data		= &vdc5fb_lvds_pdata,
	.size_data	= sizeof(vdc5fb_lvds_pdata),
	.dma_mask	= DMA_BIT_MASK(32),	/* only needed if not hardcoding fb */
};

static struct gpio_backlight_platform_data gpio_backlight_data = {
	.fbdev = NULL,	/* populated later */
	.gpio = P2_1,
	.def_value = 1,
	.name = "backlight",
};

static struct platform_device gpio_backlight_device = {
	.name = "gpio-backlight",
	.dev = {
		.platform_data = &gpio_backlight_data,
	},
};

/* JCU */
static const struct uio_info jcu_platform_pdata __initconst = {
	.name = "JCU",
	.version = "0",
	.irq = 126, /* Not used */
};

static const struct resource jcu_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe8017000, 0x1000, "jcu:reg"), /* for JCU of RZ */
	DEFINE_RES_MEM_NAMED(0xfcfe0000, 0x2000, "jcu:rstreg clkreg"), /* Use STBCR6 & SWRSTCR2 */
	DEFINE_RES_MEM_NAMED(0x20200000, 0x100000, "jcu:iram"), /* (Non cacheable 1MB) */
};

#if (!XIP_KERNEL_WITHOUT_EXTERNAL_RAM)
static const struct platform_device_info jcu_info __initconst = {
	.name		= "uio_pdrv_genirq",
	.id		= 0,
	.data		= &jcu_platform_pdata,
	.size_data	= sizeof(jcu_platform_pdata),
	.res		= jcu_resources,
	.num_res	= ARRAY_SIZE(jcu_resources),
};
#endif

/* Ether */
static const struct sh_eth_plat_data ether_pdata __initconst = {
	.phy			= 0x00, /* PD60610 */
	.edmac_endian		= EDMAC_LITTLE_ENDIAN,
	.phy_interface		= PHY_INTERFACE_MODE_MII,
	.no_ether_link		= 1
};

static const struct resource ether_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8203000, 0x800),
	DEFINE_RES_MEM(0xe8204800, 0x200),
	DEFINE_RES_IRQ(gic_iid(359)),
};

static const struct platform_device_info ether_info __initconst = {
	.parent		= &platform_bus,
	.name		= "r7s72100-ether",
	.id		= -1,
	.res		= ether_resources,
	.num_res	= ARRAY_SIZE(ether_resources),
	.data		= &ether_pdata,
	.size_data	= sizeof(ether_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* I2C0 */
static const struct resource riic0_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee000, 0x400),
	DEFINE_RES_IRQ(gic_iid(189)),
	DEFINE_RES_NAMED(gic_iid(190),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(191),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(192)),
	DEFINE_RES_IRQ(gic_iid(193)),
	DEFINE_RES_IRQ(gic_iid(194)),
	DEFINE_RES_IRQ(gic_iid(195)),
	DEFINE_RES_IRQ(gic_iid(196)),
};

static const struct riic_platform_data riic0_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic0_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 0,
	.res		= riic0_resources,
	.num_res	= ARRAY_SIZE(riic0_resources),
	.data		= &riic0_pdata,
	.size_data	= sizeof(riic0_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* I2C1 */
static const struct resource riic1_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee400, 0x400),
	DEFINE_RES_IRQ(gic_iid(197)),
	DEFINE_RES_NAMED(gic_iid(198),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(199),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(200)),
	DEFINE_RES_IRQ(gic_iid(201)),
	DEFINE_RES_IRQ(gic_iid(202)),
	DEFINE_RES_IRQ(gic_iid(203)),
	DEFINE_RES_IRQ(gic_iid(204)),
};

static const struct riic_platform_data riic1_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic1_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 1,
	.res		= riic1_resources,
	.num_res	= ARRAY_SIZE(riic1_resources),
	.data		= &riic1_pdata,
	.size_data	= sizeof(riic1_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* I2C2 */
static const struct resource riic2_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfee800, 0x400),
	DEFINE_RES_IRQ(gic_iid(205)),
	DEFINE_RES_NAMED(gic_iid(206),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(207),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(208)),
	DEFINE_RES_IRQ(gic_iid(209)),
	DEFINE_RES_IRQ(gic_iid(210)),
	DEFINE_RES_IRQ(gic_iid(211)),
	DEFINE_RES_IRQ(gic_iid(212)),
};

static const struct riic_platform_data riic2_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic2_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 2,
	.res		= riic2_resources,
	.num_res	= ARRAY_SIZE(riic2_resources),
	.data		= &riic2_pdata,
	.size_data	= sizeof(riic2_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* I2C3 */
static const struct resource riic3_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcfeec00, 0x400),
	DEFINE_RES_IRQ(gic_iid(213)),
	DEFINE_RES_NAMED(gic_iid(214),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_NAMED(gic_iid(215),1,NULL,IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE),
	DEFINE_RES_IRQ(gic_iid(216)),
	DEFINE_RES_IRQ(gic_iid(217)),
	DEFINE_RES_IRQ(gic_iid(218)),
	DEFINE_RES_IRQ(gic_iid(219)),
	DEFINE_RES_IRQ(gic_iid(220)),
};

static const struct riic_platform_data riic3_pdata __initconst = {
	.bus_rate = 100,
};

static const struct platform_device_info riic3_info __initconst = {
	.parent		= &platform_bus,
	.name		= "i2c-riic",
	.id		= 3,
	.res		= riic3_resources,
	.num_res	= ARRAY_SIZE(riic3_resources),
	.data		= &riic3_pdata,
	.size_data	= sizeof(riic3_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

static struct vgg804808_ts_platform_data vgg804808_pdata __initdata = {
	.flags	= VGG804808_TS_SWAP_XY,
	.reset_pin = P5_10,
};

static const struct i2c_board_info i2c0_devices[] __initconst = {
	{
		I2C_BOARD_INFO("vgg804808_ts", 0x38),
		.platform_data = &vgg804808_pdata,
		.irq = 39,	/* IRQ7 */
	},
};

/* Init TOUCH_IRQ7 as input */
static void __init gpio_irq_init(void) {
	/* Set Interrupt Control Register 1 (ICR1) for low edge trigger */
	void __iomem *irc1 = IOMEM(0xfcfef802);
	__raw_writew((__raw_readw(irc1) & ~(0x3 << 14)), irc1);
	r7s72100_pfc_pin_assign(P1_7, ALT4, DIIO_PBDC_DIS);	/* IRQ7 */
}

static const struct i2c_board_info i2c2_devices[] __initconst = {
	{
		I2C_BOARD_INFO("pca9554", 0x20),
	},
};

static const struct gpio_led balto_leds[] = {
/*
	{
		.name			= "BKLED",
		.gpio			= P2_1,
		.default_trigger	= "none",
		.default_state		= LEDS_GPIO_DEFSTATE_ON,
	},
*/
	{
		.name			= "LD1",
		.gpio			= 1023,	/* PCA9538 pin 7 */
		.default_trigger	= "heartbeat",
	},
};

static struct gpio_led_platform_data balto_leds_pdata = {
	.leds		= balto_leds,
	.num_leds	= ARRAY_SIZE(balto_leds),
};

static struct platform_device leds_device = {
	.name	= "leds-gpio",
	.id	= 0,
	.dev	= {
		.platform_data	= &balto_leds_pdata,
	},
};

/* OSTM */
static struct rza1_ostm_pdata ostm_pdata = {
	.clksrc.name = "ostm.0",
	.clksrc.rating = 300,
	.clkevt.name = "ostm.1",
	.clkevt.rating = 300,
};

static const struct resource ostm_resources[] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xfcfec000, 0x030, "ostm.0"),
	[1] = DEFINE_RES_MEM_NAMED(0xfcfec400, 0x030, "ostm.1"),
	[2] = DEFINE_RES_IRQ_NAMED(134, "ostm.0"),
	[3] = DEFINE_RES_IRQ_NAMED(135, "ostm.1"),
};

static const struct platform_device_info ostm_info __initconst = {
	.name		= "ostm",
	.id		= 0,
	.data 		= &ostm_pdata,
	.size_data	= sizeof(ostm_pdata),
	.res		= ostm_resources,
	.num_res	= ARRAY_SIZE(ostm_resources),
};

/* RTC */
static const struct resource rtc_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff1000, 0x2e),
	DEFINE_RES_IRQ(gic_iid(309)),	/* Period IRQ */
	DEFINE_RES_IRQ(gic_iid(310)),	/* Carry IRQ */
	DEFINE_RES_IRQ(gic_iid(308)),	/* Alarm IRQ */
};

static const struct platform_device_info rtc_info __initconst = {
	.parent		= &platform_bus,
	.name		= "sh-rtc",
	.id		= -1,
	.res		= rtc_resources,
	.num_res	= ARRAY_SIZE(rtc_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* SPI Flash */
/* Single Flash only */
static struct mtd_partition spibsc0_flash_partitions[] = {
	{
		.name		= "spibsc0_loader",
		.offset		= 0x00000000,
		.size		= 0x00080000,
		/* .mask_flags	= MTD_WRITEABLE, */
	},
	{
		.name		= "spibsc0_bootenv",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00040000,
	},
	{
		.name		= "spibsc0_kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 0x00400000,
	},
	{
		.name		= "spibsc0_rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition spibsc1_flash_partitions[] = {
	{
		.name		= "spibsc1_data",
		.offset		= 0x00000000,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data spibsc0_flash_pdata = {
	.name	= "m25p80",
	.parts	= spibsc0_flash_partitions,
	.nr_parts = ARRAY_SIZE(spibsc0_flash_partitions),
	.type = "s25fl512s",
};

static struct flash_platform_data spibsc1_flash_pdata = {
	.name	= "m25p80",
	.parts	= spibsc1_flash_partitions,
	.nr_parts = ARRAY_SIZE(spibsc1_flash_partitions),
	.type = "s25fl512s",
};

/* QSPI Flash (Memory Map Mode, read only) */
/* Dual Flash */
static struct mtd_partition qspi_flash_partitions[] __initdata = {
	{
		.name		= "qspi_rootfs",
		.offset		= 0x00800000,
		.size		= 64 * SZ_1M - 0x00800000,
	},
};

static const struct physmap_flash_data qspi_flash_data __initconst = {
	.width		= 4,
	.probe_type	= "map_rom",
	.parts		= qspi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(qspi_flash_partitions),
};

static const struct resource qspi_flash_resources[] __initconst = {
	DEFINE_RES_MEM(0x18000000, SZ_64M),
};

static const struct platform_device_info qspi_flash_info __initconst = {
	.parent		= &platform_bus,
	.name		= "physmap-flash",
	.id		= 0,
	.res		= qspi_flash_resources,
	.num_res	= ARRAY_SIZE(qspi_flash_resources),
	.data		= &qspi_flash_data,
	.size_data	= sizeof(qspi_flash_data),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* PWM Pin (Pin TIOC4A only) */
/* RSKRZA1 does not have TIOC4A attached to anything */
#if 0
static const struct resource pwm_resources[] __initconst = {
	DEFINE_RES_MEM(0xfcff0200, 0x4c),	/* mtu2_3,4 */
	DEFINE_RES_MEM(0xfcff0280, 0x6),	/* mtu2 share regs */
};

static const struct platform_device_info pwm0_info __initconst = {
	.parent		= &platform_bus,
	.name		= "rza1-pwm",
	.id		= 0,
	.res		= pwm_resources,
	.num_res	= ARRAY_SIZE(pwm_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* Backlight */
static struct platform_pwm_backlight_data pwm_backlight_pdata = {
	.max_brightness = 255,
	.dft_brightness = 255,
	.pwm_period_ns = 33333, /* 30kHz */
	.enable_gpio = -1,
};

static struct pwm_lookup pwm_lookup[] = {
	PWM_LOOKUP("rza1-pwm.0", 0, "pwm-backlight.0", NULL),
};

static const struct platform_device_info pwm_backlight_info __initconst = {
	.name		= "pwm-backlight",
	.data		= &pwm_backlight_pdata,
	.size_data	= sizeof(pwm_backlight_pdata),
};
#endif

/* RSPI */
#define RSPI_RESOURCE(idx, baseaddr, irq)				\
static const struct resource rspi##idx##_resources[] __initconst = {	\
	DEFINE_RES_MEM(baseaddr, 0x24),					\
	DEFINE_RES_IRQ_NAMED(irq, "error"),				\
	DEFINE_RES_IRQ_NAMED(irq + 1, "rx"),				\
	DEFINE_RES_IRQ_NAMED(irq + 2, "tx"),				\
}

RSPI_RESOURCE(0, 0xe800c800, gic_iid(270));
RSPI_RESOURCE(1, 0xe800d000, gic_iid(273));
RSPI_RESOURCE(2, 0xe800d800, gic_iid(276));
RSPI_RESOURCE(3, 0xe800e000, gic_iid(279));
RSPI_RESOURCE(4, 0xe800e800, gic_iid(282));

static const struct rspi_plat_data rspi_pdata __initconst = {
	.num_chipselect	= 1,
};

#define r7s72100_register_rspi(idx)					   \
	platform_device_register_resndata(&platform_bus, "rspi-rz", idx,   \
					rspi##idx##_resources,		   \
					ARRAY_SIZE(rspi##idx##_resources), \
					&rspi_pdata, sizeof(rspi_pdata))


static struct spi_board_info balto_spi_devices[] __initdata = {
	{
		/* spidev */
		.modalias		= "spidev",
		.max_speed_hz           = 5000000,
		.bus_num                = 4,
		.chip_select            = 0,
		.mode			= SPI_MODE_3,
	},
	{
		/* SPI Flash0 */
		.modalias = "m25p80",
		.bus_num = 5,
		.chip_select = 0,
		.platform_data = &spibsc0_flash_pdata,
	},
	{
		/* SPI Flash1 */
		.modalias = "m25p80",
		.bus_num = 6,
		.chip_select = 0,
		.platform_data = &spibsc1_flash_pdata,
	},
};

/* spibsc0 */
static const struct sh_spibsc_info spibsc0_pdata = {
	.bus_num	= 5,
};

static const struct resource spibsc0_resources[] __initconst = {
	DEFINE_RES_MEM(0x3fefa000, 0x100),
};

static const struct platform_device_info spibsc0_info __initconst = {
	.name		= "spibsc",
	.id		= 0,
	.data 		= &spibsc0_pdata,
	.size_data	= sizeof(spibsc0_pdata),
	.num_res	= ARRAY_SIZE(spibsc0_resources),
	.res		= spibsc0_resources,
};

/* spibsc1 */
static const struct sh_spibsc_info spibsc1_pdata = {
	.bus_num	= 6,
};

static const struct resource spibsc1_resources[] __initconst = {
	DEFINE_RES_MEM(0x3fefb000, 0x100),
};

static const struct platform_device_info spibsc1_info __initconst = {
	.name		= "spibsc",
	.id		= 1,
	.data 		= &spibsc1_pdata,
	.size_data	= sizeof(spibsc1_pdata),
	.num_res	= ARRAY_SIZE(spibsc1_resources),
	.res		= spibsc1_resources,
};

/* ADC */
static const struct resource adc0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8005800, 0x100),
	DEFINE_RES_MEM(0xfcff0280, 0x6),
	DEFINE_RES_MEM(0xfcff0380, 0x21),
	DEFINE_RES_IRQ(gic_iid(170)),
	DEFINE_RES_IRQ(gic_iid(171)),
	DEFINE_RES_IRQ(gic_iid(146)),
};

static const struct sh_adc_data adc0_pdata __initconst = {
	.num_channels = 8,
	.mtu2_ch = 1,
};

static const struct platform_device_info adc0_info __initconst = {
	.name	= "sh_adc",
	.id	= 0,
	.data		= &adc0_pdata,
	.size_data	= sizeof(adc0_pdata),
	.res		= adc0_resources,
	.num_res	= ARRAY_SIZE(adc0_resources),
	.dma_mask	= DMA_BIT_MASK(32),
};

/* MMCIF */
static const struct resource sh_mmcif_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804c800, 0x100, "MMCIF"),
	DEFINE_RES_IRQ(gic_iid(300)),
	DEFINE_RES_IRQ(gic_iid(301)),
};

static const struct sh_mmcif_plat_data sh_mmcif_pdata __initconst = {
	.sup_pclk	= 0,
	.ccs_unsupported = true,
	.ocr		= MMC_VDD_32_33,
	.caps		= MMC_CAP_4_BIT_DATA |
			  MMC_CAP_8_BIT_DATA |
			  MMC_CAP_NONREMOVABLE,
};

static const struct platform_device_info mmc_info __initconst = {
	.name		= "sh_mmcif",
	.id		= -1,
	.res		= sh_mmcif_resources,
	.num_res	= ARRAY_SIZE(sh_mmcif_resources),
	.data		= &sh_mmcif_pdata,
	.size_data	= sizeof(sh_mmcif_pdata),
};

/* SDHI0 */
static struct sh_mobile_sdhi_info sdhi0_pdata = {
	.dma_slave_tx	= RZA1DMA_SLAVE_SDHI0_TX,
	.dma_slave_rx	= RZA1DMA_SLAVE_SDHI0_RX,
	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.tmio_ocr_mask	= MMC_VDD_32_33,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT | TMIO_MMC_WRPROTECT_DISABLE,
	/* microSD slot has no WP pin => use TMIO_MMC_WRPROTECT_DISABLE */
};

static const struct resource sdhi0_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804e000, 0x100, "SDHI0"),
	DEFINE_RES_IRQ_NAMED(gic_iid(302), SH_MOBILE_SDHI_IRQ_CARD_DETECT),
	DEFINE_RES_IRQ_NAMED(gic_iid(303), SH_MOBILE_SDHI_IRQ_SDCARD),
	DEFINE_RES_IRQ_NAMED(gic_iid(304), SH_MOBILE_SDHI_IRQ_SDIO),
};

static const struct platform_device_info sdhi0_info __initconst = {
	.name		= "sh_mobile_sdhi",
	.id		= 0,
	.res		= sdhi0_resources,
	.num_res	= ARRAY_SIZE(sdhi0_resources),
	.data		= &sdhi0_pdata,
	.size_data	= sizeof(sdhi0_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};

#if 0 /* for reference only */
/* SDHI1 */
static struct sh_mobile_sdhi_info sdhi1_pdata = {
	.dma_slave_tx	= RZA1DMA_SLAVE_SDHI1_TX,
	.dma_slave_rx	= RZA1DMA_SLAVE_SDHI1_RX,
	.tmio_caps	= MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ,
	.tmio_ocr_mask	= MMC_VDD_32_33,
	.tmio_flags	= TMIO_MMC_HAS_IDLE_WAIT,
};

static const struct resource sdhi1_resources[] __initconst = {
	DEFINE_RES_MEM_NAMED(0xe804e800, 0x100, "SDHI1"),
	DEFINE_RES_IRQ_NAMED(gic_iid(305), SH_MOBILE_SDHI_IRQ_CARD_DETECT),
	DEFINE_RES_IRQ_NAMED(gic_iid(306), SH_MOBILE_SDHI_IRQ_SDCARD),
	DEFINE_RES_IRQ_NAMED(gic_iid(307), SH_MOBILE_SDHI_IRQ_SDIO),
};

static const struct platform_device_info sdhi1_info __initconst = {
	.name		= "sh_mobile_sdhi",
	.id		= 1,
	.res		= sdhi1_resources,
	.num_res	= ARRAY_SIZE(sdhi1_resources),
	.data		= &sdhi1_pdata,
	.size_data	= sizeof(sdhi1_pdata),
	.dma_mask	= DMA_BIT_MASK(32),
};
#endif

/* USB Host */
static const struct r8a66597_platdata r8a66597_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_host0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_host0_info __initconst= {
	.name		= "r8a66597_hcd",
	.id		= 0,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host0_resources),
};

static const struct resource r8a66597_usb_host1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_host1_info __initconst = {
	.name		= "r8a66597_hcd",
	.id		= 1,
	.data		= &r8a66597_pdata,
	.size_data	= sizeof(r8a66597_pdata),
	.res		= r8a66597_usb_host1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_host1_resources),
};

/* USB Gadget */
static const struct r8a66597_platdata r8a66597_usb_gadget0_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8010000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(73)),
};

static const struct platform_device_info r8a66597_usb_gadget0_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 0,
	.data		= &r8a66597_usb_gadget0_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget0_pdata),
	.res		= r8a66597_usb_gadget0_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget0_resources),
};

static const struct r8a66597_platdata r8a66597_usb_gadget1_pdata __initconst = {
	.endian = 0,
	.on_chip = 1,
	.xtal = R8A66597_PLATDATA_XTAL_48MHZ,
};

static const struct resource r8a66597_usb_gadget1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe8207000, 0x1a0),
	DEFINE_RES_IRQ(gic_iid(74)),
};

static const struct platform_device_info r8a66597_usb_gadget1_info __initconst = {
	.name		= "r8a66597_udc",
	.id		= 1,
	.data		= &r8a66597_usb_gadget1_pdata,
	.size_data	= sizeof(r8a66597_usb_gadget1_pdata),
	.res		= r8a66597_usb_gadget1_resources,
	.num_res	= ARRAY_SIZE(r8a66597_usb_gadget1_resources),
};

#ifdef CONFIG_CAN_RZA1
static struct resource rz_can0_resources[] = {
	[0] = {
		.start	= 0xe803a000,
		.end	= 0xe803b813,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 255,
		.end	= 255,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {
		.start	= 257,
		.end	= 257,
		.flags	= IORESOURCE_IRQ,
	},
	[3] = {
		.start	= 256,
		.end	= 256,
		.flags	= IORESOURCE_IRQ,
	},
	[4] = {
		.start	= 253,
		.end	= 253,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct rz_can_platform_data rz_can0_data = {
	.channel	= 0,
	.clock_select	= CLKR_CLKC,
};

static struct platform_device_info rz_can_device = {
	.name		= "rz_can",
	.id		= 0,
	.num_res	= ARRAY_SIZE(rz_can0_resources),
	.res		= rz_can0_resources,
	.data		= &rz_can0_data,
	.size_data	= sizeof(rz_can0_data),
};
#endif /* CONFIG_CAN_RZA1 */

/* Audio */
static const struct platform_device_info alsa_soc_info = {
	.name		= "balto_alsa_soc_platform",
	.id		= 0,
};

static const struct resource scux_resources[] __initconst = {
	[0] = DEFINE_RES_MEM_NAMED(0xe8208000, 0x00001778, "scux"),
	[1] = DEFINE_RES_MEM_NAMED(0xe820b000, 0x00002830, "ssif0"),
};

static struct scu_config ssi_ch_value[] = {
	{RP_MEM_SSI0,		SSI0},
	{RP_MEM_SRC1_SSI0,	SSI0},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI0},
	{RC_SSI0_MEM,		SSI0},
	{RC_SSI0_SRC0_MEM,	SSI0},
};

static struct scu_config src_ch_value[] = {
	{RP_MEM_SSI0,		-1},
	{RP_MEM_SRC1_SSI0,	SRC1},
	{RP_MEM_SRC1_DVC1_SSI0,	SRC1},
	{RC_SSI0_MEM,		-1},
	{RC_SSI0_SRC0_MEM,	SRC0},
};

static struct scu_config dvc_ch_value[] = {
	{RP_MEM_SSI0,		-1},
	{RP_MEM_SRC1_SSI0,	-1},
	{RP_MEM_SRC1_DVC1_SSI0,	DVC1},
	{RC_SSI0_MEM,		-1},
	{RC_SSI0_SRC0_MEM,	-1},
};

static struct scu_config audma_slave_value[] = {
	{RP_MEM_SSI0,		RZA1DMA_SLAVE_PCM_MEM_SSI0},
	{RP_MEM_SRC1_SSI0,	RZA1DMA_SLAVE_PCM_MEM_SRC1},
	{RP_MEM_SRC1_DVC1_SSI0,	RZA1DMA_SLAVE_PCM_MEM_SRC1},
	{RC_SSI0_MEM,		RZA1DMA_SLAVE_PCM_SSI0_MEM},
	{RC_SSI0_SRC0_MEM,	RZA1DMA_SLAVE_PCM_SRC0_MEM},
};

static struct scu_config ssi_depend_value[] = {
	{RP_MEM_SSI0,		SSI_INDEPENDANT},
	{RP_MEM_SRC1_SSI0,	SSI_DEPENDANT},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI_DEPENDANT},
	{RC_SSI0_MEM,		SSI_INDEPENDANT},
	{RC_SSI0_SRC0_MEM,	SSI_DEPENDANT},
};

static struct scu_config ssi_mode_value[] = {
	{RP_MEM_SSI0,		SSI_MASTER},
	{RP_MEM_SRC1_SSI0,	SSI_MASTER},
	{RP_MEM_SRC1_DVC1_SSI0,	SSI_MASTER},
	{RC_SSI0_MEM,		SSI_SLAVE},
	{RC_SSI0_SRC0_MEM,	SSI_SLAVE},
};

static struct scu_config src_mode_value[] = {
	{RP_MEM_SSI0,		SRC_CR_ASYNC},
	{RP_MEM_SRC1_SSI0,	SRC_CR_ASYNC},
	{RP_MEM_SRC1_DVC1_SSI0,	SRC_CR_ASYNC},
	{RC_SSI0_MEM,		SRC_CR_ASYNC},
	{RC_SSI0_SRC0_MEM,	SRC_CR_ASYNC},
};

static const struct scu_platform_data scu_pdata __initconst = {
	.ssi_master		= SSI0,
	.ssi_slave		= SSI0,
	.ssi_ch			= ssi_ch_value,
	.ssi_ch_num		= ARRAY_SIZE(ssi_ch_value),
	.src_ch			= src_ch_value,
	.src_ch_num		= ARRAY_SIZE(src_ch_value),
	.dvc_ch			= dvc_ch_value,
	.dvc_ch_num		= ARRAY_SIZE(dvc_ch_value),
	.dma_slave_maxnum	= RZA1DMA_SLAVE_PCM_MAX,
	.audma_slave		= audma_slave_value,
	.audma_slave_num	= ARRAY_SIZE(audma_slave_value),
	.ssi_depend		= ssi_depend_value,
	.ssi_depend_num		= ARRAY_SIZE(ssi_depend_value),
	.ssi_mode		= ssi_mode_value,
	.ssi_mode_num		= ARRAY_SIZE(ssi_mode_value),
	.src_mode		= src_mode_value,
	.src_mode_num		= ARRAY_SIZE(src_mode_value),
};

static const struct platform_device_info scux_info __initconst = {
	.name		= "scux-pcm-audio",
	.id		= 0,
	.data		= &scu_pdata,
	.size_data	= sizeof(scu_pdata),
	.num_res	= ARRAY_SIZE(scux_resources),
	.res		= scux_resources,
};


/* SCIF */
#define R7S72100_SCIF(index, baseaddr, irq)				\
static const struct plat_sci_port scif##index##_platform_data = {	\
	.type		= PORT_SCIF,					\
	.regtype	= SCIx_SH2_SCIF_FIFODATA_REGTYPE,		\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,		\
	.scscr		= SCSCR_RIE | SCSCR_TIE | SCSCR_RE | SCSCR_TE |	\
			  SCSCR_REIE,					\
};									\
									\
static struct resource scif##index##_resources[] = {			\
	DEFINE_RES_MEM(baseaddr, 0x100),				\
	DEFINE_RES_IRQ(irq + 1),					\
	DEFINE_RES_IRQ(irq + 2),					\
	DEFINE_RES_IRQ(irq + 3),					\
	DEFINE_RES_IRQ(irq),						\
}									\

//R7S72100_SCIF(0, 0xe8007000, gic_iid(221));	/* Not used */
//R7S72100_SCIF(1, 0xe8007800, gic_iid(225));	/* Not used */
//R7S72100_SCIF(2, 0xe8008000, gic_iid(229));	/* Not used */
R7S72100_SCIF(3, 0xe8008800, gic_iid(233));
//R7S72100_SCIF(4, 0xe8009000, gic_iid(237));	/* Not used */
//R7S72100_SCIF(5, 0xe8009800, gic_iid(241));	/* Not used */
//R7S72100_SCIF(6, 0xe800a000, gic_iid(245));	/* Not used */
//R7S72100_SCIF(7, 0xe800a800, gic_iid(249));	/* Not used */

#define r7s72100_register_scif(index)					       \
	platform_device_register_resndata(&platform_bus, "sh-sci", index,      \
					  scif##index##_resources,	       \
					  ARRAY_SIZE(scif##index##_resources), \
					  &scif##index##_platform_data,	       \
					  sizeof(scif##index##_platform_data))

static void __init balto_add_standard_devices(void)
{
	struct platform_device *vdc5fb_dev = NULL;
	struct clk *clk;

#ifdef CONFIG_CACHE_L2X0
	/* Early BRESP enable, 16K*8way(defualt) */
	/* NOTES: BRESP can be set for IP version after r2p0 */
	/*        As of linux-3.16, cache-l2x0.c handles this automatically */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,16,0)
	l2x0_init(IOMEM(0xfffee000), 0x40000000, 0xffffffff);	/* Early BRESP enable */
#else
	l2x0_init(IOMEM(0xfffee000), 0x00000000, 0xffffffff);	/* Leave as defaults */
#endif
#endif
	r7s72100_clock_init();
	r7s72100_pinmux_setup();
	r7s72100_add_dt_devices();

	/* set extal clock to 66.25 MHz */
	clk = clk_get(NULL, "extal");
	if (!IS_ERR(clk)) {
		clk_set_rate(clk, EXTCLK);
		clk_put(clk);
	}

	r7s72100_pfc_pin_assign(P1_0, ALT1, DIIO_PBDC_EN);	/* I2C SCL0 */
	r7s72100_pfc_pin_assign(P1_1, ALT1, DIIO_PBDC_EN);	/* I2C SDA0 */
//	r7s72100_pfc_pin_assign(P1_2, ALT1, DIIO_PBDC_EN);	/* I2C SCL1: not used */
//	r7s72100_pfc_pin_assign(P1_3, ALT1, DIIO_PBDC_EN);	/* I2C SDA1: not used */
	r7s72100_pfc_pin_assign(P1_4, ALT1, DIIO_PBDC_EN);	/* I2C SCL2 */
	r7s72100_pfc_pin_assign(P1_5, ALT1, DIIO_PBDC_EN);	/* I2C SDA2 */

	r7s72100_pfc_pin_assign(P1_8, ALT1, DIIO_PBDC_DIS);	/* AN0 */
	r7s72100_pfc_pin_assign(P1_9, ALT1, DIIO_PBDC_DIS);	/* AN1 */
	r7s72100_pfc_pin_assign(P1_10, ALT1, DIIO_PBDC_DIS);	/* AN2 */
	r7s72100_pfc_pin_assign(P1_11, ALT1, DIIO_PBDC_DIS);	/* AN3 */
	r7s72100_pfc_pin_assign(P1_12, ALT1, DIIO_PBDC_DIS);	/* AN4 */
	r7s72100_pfc_pin_assign(P1_13, ALT1, DIIO_PBDC_DIS);	/* AN5 */

	r7s72100_pfc_pin_assign(P4_8, ALT3, DIIO_PBDC_DIS);	/* SD_CD_0 */
	r7s72100_pfc_pin_assign(P4_9, PMODE, PORT_OUT_HIGH);	/* SD_POWER */
	r7s72100_pfc_pin_assign(P4_10, ALT3, DIIO_PBDC_EN);	/* SD_D1_0 */
	r7s72100_pfc_pin_assign(P4_11, ALT3, DIIO_PBDC_EN);	/* SD_D0_0 */
	r7s72100_pfc_pin_assign(P4_12, ALT3, DIIO_PBDC_DIS);	/* SD_CLK_0 */
	r7s72100_pfc_pin_assign(P4_13, ALT3, DIIO_PBDC_EN);	/* SD_CMD_0 */
	r7s72100_pfc_pin_assign(P4_14, ALT3, DIIO_PBDC_EN);	/* SD_D3_0 */
	r7s72100_pfc_pin_assign(P4_15, ALT3, DIIO_PBDC_EN);	/* SD_D2_0 */

	gpio_irq_init();

#ifdef CONFIG_CAN_RZA1
	r7s72100_pfc_pin_assign(P9_0, ALT3, DIIO_PBDC_DIS);	/* CAN CAN0TX */
	r7s72100_pfc_pin_assign(P9_1, ALT3, DIIO_PBDC_DIS);	/* CAN CAN0RX */
	platform_device_register_full(&rz_can_device);
#endif

	i2c_register_board_info(0, i2c0_devices, ARRAY_SIZE(i2c0_devices));
	i2c_register_board_info(2, i2c2_devices, ARRAY_SIZE(i2c2_devices));

	platform_device_register(&leds_device);

#if (!XIP_KERNEL_WITHOUT_EXTERNAL_RAM)
	platform_device_register_full(&jcu_info);
#endif
	platform_device_register_full(&ostm_info);
	platform_device_register_full(&dma_info);
#if 0 /* For refernce only */
	platform_device_register_full(&alsa_soc_info);
	platform_device_register_full(&scux_info);
#endif
	platform_device_register_full(&ether_info);

	platform_device_register_full(&riic0_info);
//	platform_device_register_full(&riic1_info);	/* Not used */
	platform_device_register_full(&riic2_info);
//	platform_device_register_full(&riic3_info);	/* Not used */
	platform_device_register_full(&rtc_info);

#if !defined(CONFIG_XIP_KERNEL) && defined(CONFIG_SPI_SH_SPIBSC)
	/* Need to disable both spibsc channels if using memory mapped QSPI */
	platform_device_register_full(&spibsc0_info);
	platform_device_register_full(&spibsc1_info);
#else
	platform_device_register_full(&qspi_flash_info);
#endif

#if 0 /* For refernce only */
	platform_device_register_full(&nor_flash_info);
	platform_device_register_full(&pwm0_info);
	platform_device_register_full(&pwm_backlight_info);
	pwm_add_table(pwm_lookup, ARRAY_SIZE(pwm_lookup));
#endif
	platform_device_register_full(&adc0_info);
	platform_device_register_full(&sdhi0_info);

	if (panel == 0) {
		vdc5fb_dev = platform_device_register_full(&vdc5fb_lcd0_info);
		printk("%s: registered lcd0 interface\n", __func__);
	} else if (panel == 1) {
		vdc5fb_dev = platform_device_register_full(&vdc5fb_lvds_info);
		printk("%s: registered lvds interface\n", __func__);
	} else {
		printk("%s: no panel registered\n", __func__);
	}

	if (vdc5fb_dev) {
		gpio_backlight_data.fbdev = &(vdc5fb_dev->dev);
		platform_device_register(&gpio_backlight_device);
	}

	if (usbgs == 0) {
		platform_device_register_full(&r8a66597_usb_gadget0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
		printk("%s: registered USB0 as gadget and USB1 as host\n", __func__);
	} else if (usbgs == 1) {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_gadget1_info);
		printk("%s: registered USB0 as host and USB1 as gadget\n", __func__);
	} else {
		platform_device_register_full(&r8a66597_usb_host0_info);
		platform_device_register_full(&r8a66597_usb_host1_info);
		printk("%s: registered USB0 and USB1 as hosts\n", __func__);
	}

	r7s72100_register_rspi(0);
	r7s72100_register_rspi(1);
	r7s72100_register_rspi(2);
	r7s72100_register_rspi(3);
	r7s72100_register_rspi(4);

	/* Register SPI device information */
	spi_register_board_info(balto_spi_devices,
				ARRAY_SIZE(balto_spi_devices));

//	r7s72100_register_scif(0);	/* Not used */
//	r7s72100_register_scif(1);	/* Not used */
//	r7s72100_register_scif(2);	/* Not used */
	r7s72100_register_scif(3);
//	r7s72100_register_scif(4);	/* Not used */
//	r7s72100_register_scif(5);	/* Not used */
//	r7s72100_register_scif(6);	/* Not used */
//	r7s72100_register_scif(7);	/* Not used */

}

static void __init balto_init_late(void)
{
	/* TODO */
}

#define WTCSR 0
#define WTCNT 2
#define WRCSR 4
static void balto_restart(enum reboot_mode mode, const char *cmd)
{
	void *base = ioremap_nocache(0xFCFE0000, 0x10);

	/* Dummy read (must read WRCSR:WOVF at least once before clearing) */
	*(volatile uint8_t *)(base + WRCSR) = *(uint8_t *)(base + WRCSR);

	*(volatile uint16_t *)(base + WRCSR) = 0xA500;	/* Clear WOVF */
	*(volatile uint16_t *)(base + WRCSR) = 0x5A5F;	/* Reset Enable */
	*(volatile uint16_t *)(base + WTCNT) = 0x5A00;	/* Counter to 00 */
	*(volatile uint16_t *)(base + WTCSR) = 0xA578;	/* Start timer */

	while(1); /* Wait for WDT overflow */
}

static void __init balto_init_early(void)
{
	shmobile_init_delay();

#if XIP_KERNEL_WITHOUT_EXTERNAL_RAM
	/* Set the size of our pre-allocated DMA buffer pool because the
	   default is 256KB */
	init_dma_coherent_pool_size(16 * SZ_1K);
#endif
}

static const char * const balto_compat_dt[] __initconst = {
	"arrow,balto",
	NULL,
};

DT_MACHINE_START(BALTO_DT, "balto")
	.init_early	= balto_init_early,
	.init_machine	= balto_add_standard_devices,
	.init_late	= balto_init_late,
	.dt_compat	= balto_compat_dt,
	.map_io		= rza1_map_io,
	.restart	= balto_restart,
MACHINE_END
