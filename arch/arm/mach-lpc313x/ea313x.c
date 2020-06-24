/*  arch/arm/mach-lpc313x/ea313x.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  ea313x board init routines.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/serial_8250.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/sizes.h>

#include <asm/mach/map.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <mach/gpio.h>
#include <mach/i2c.h>
#include <mach/board.h>

/*  PC/104 carrier board constants */
#define PC104base8  EXT_SRAM0_PHYS           //base address for 8-bit I/O
#define PC104base16 (PC104base8+_BIT(15))    //base address for 16-bit I/O

#define PC104FPGAadr (EXT_SRAM1_PHYS+0x8000) //PC104 FPGA control register
#define PC104FPGA   __REG16(PC104FPGAadr)
enum {
  PC104IRQID     = _BIT(0),  //0 selects board id, 1 selects PC/104 IRQs
  PC104ENETRESET = _BIT(1),  //1 resets on board (micrel) ethernet chip
  PC104RESET     = _BIT(2),  //1 resets PC/104 bus
  PC104force16   = _BIT(3),  //force sixteen bit access on PC/104 bus
  PC104force8    = _BIT(4)   //force eight bit access on PC/104 bus
};
#define PC104TEST   __REG16(EXT_SRAM1_PHYS+0xC000) //PC104 FPGA test register

#define XRbase (PC104base8 + 0x400) //base address of EXAR octal UART
//note: not decoded on ESP3G board, so this address used for it as well
#define XR_IRQ  PC104_IRQ6
#define XR_EVT  PC104_EVT6


//PC104 EVT/IRQ mapping -- all are active high
#define PC104_EVT3   EVT_GPIO11
#define PC104_EVT4   EVT_GPIO12
#define PC104_EVT5   EVT_GPIO13
#define PC104_EVT6   EVT_mNAND_RYBN3
#define PC104_EVT7   EVT_GPIO14
#define PC104_EVT10  EVT_GPIO15
#define PC104_EVT11  EVT_GPIO16
#define PC104_EVT14  EVT_GPIO17
#define PC104_EVT15  EVT_GPIO18

#define PC104_IRQ3   GPIO_GPIO11
#define PC104_IRQ4   GPIO_GPIO12
#define PC104_IRQ5   GPIO_GPIO13
#define PC104_IRQ6   GPIO_MNAND_RYBN3
#define PC104_IRQ7   GPIO_GPIO14
#define PC104_IRQ10  GPIO_GPIO15
#define PC104_IRQ11  GPIO_GPIO16
#define PC104_IRQ14  GPIO_GPIO17
#define PC104_IRQ15  GPIO_GPIO18

//This came from leds-pca9532
int lpc313x_USBpower = -1;

static struct gpio_led lpc_led_pins[] __initdata = {
	{
		.name	= "cpu:blue:busy",
		.default_trigger = "cpu0",
		.default_state = LEDS_GPIO_DEFSTATE_ON,
		.gpio		= GPIO_GPIO2  //changed according to discovered baseboard
	},
};
#define lpc313x_LED (lpc_led_pins[0].gpio)

static const struct gpio_led_platform_data lpc_leds __initconst = {
	.num_leds		= ARRAY_SIZE(lpc_led_pins),
	.leds			= lpc_led_pins,
};

// default to reading board identification code from GPIOs 11-19
static int __initdata boardID = -1;

static int __init setBoardID(char *str)
{
	char *end;
	unsigned long override = simple_strtoul(str,&end,0);
    if (end > str)
    	boardID = override;  //valid ID from kernel cmdline overrides GPIOs
	return 1;
}
__setup("board=", setBoardID);


static inline int mbariBoard(int id)
// return 1 for mbariBoard, 0 for non-mbari (Embedded Artists?) origin board
{
  uint8_t low = id;
  return low == 0 || low == 0104;
}

static int16_t pc104sharedIRQ(void)
/*
  return negative value if shared serial IRQ is asserted
  lower 15 bits indicate interrupt number
*/
{
  return GPIO_STATE(IOCONF_EBI_I2STX_0) &
          		1<<(PC104_IRQ6 - BASE_GPIO_EBI_I2STX_0) ?
           IRQ_XR16788_INT | 0x8000 : IRQ_XR16788_INT;
}

static int16_t XRsharedIRQ(void)
/*
  return negative value if shared serial IRQ is asserted
  lower 15 bits indicate interrupt number
*/
{
  return GPIO_STATE(IOCONF_EBI_I2STX_0) &
          		1<<(XR_IRQ - BASE_GPIO_EBI_I2STX_0) ?
           IRQ_XR16788_INT : IRQ_XR16788_INT | 0x8000;
}

static int16_t noSharedIRQ(void)
{
	return -1;
}

int16_t (*shared8250IRQ)(void) = noSharedIRQ;




static struct lpc313x_mci_irq_data irq_data = {
	.irq = IRQ_SDMMC_CD,
};

static int mci_get_cd(u32 slot_id)
{
	return gpio_get_value(GPIO_MI2STX_BCK0);
}

static irqreturn_t ea313x_mci_detect_interrupt(int irq, void *data)
{
	struct lpc313x_mci_irq_data	*pdata = data;

	/* select the opposite level senstivity */
	int level = mci_get_cd(0)?IRQ_TYPE_LEVEL_LOW:IRQ_TYPE_LEVEL_HIGH;

	irq_set_irq_type(pdata->irq, level);

	/* change the polarity of irq trigger */
	return pdata->irq_hdlr(irq, pdata->data);
}


static void requestGPO(int gpio, const char *name, int on)
{
    gpio_request(gpio, name);
    gpio_direction_output(gpio, on);
}

static void exportGPO(int gpio, const char *name, int on)
{
    requestGPO(gpio, name, on);
    gpio_export(gpio, 0);  //do not allow redefinition as an input
}

static void requestGPI(int gpio, const char *name)
{
    gpio_request(gpio, name);
    gpio_direction_input(gpio);
}

static void exportGPI(int gpio, const char *name)
{
    requestGPI(gpio, name);
    gpio_export(gpio, 0);  //do not allow redefinition as an output
}

static void exportBootI(int gpio, const char *name)
{
    requestGPI(gpio, name);
    gpio_export(gpio, 1);  //allow redefinition as an output
}

static int mci_init(u32 slot_id, irq_handler_t irqhdlr, void *data)
{
	/* disable power to the slot */
        exportGPO(GPIO_MI2STX_DATA0, "MMCpower", 1);
        gpio_sysfs_set_active_low(GPIO_MI2STX_DATA0, 1);

	/* set cd pins as GPIO pins */
        exportGPI(GPIO_MI2STX_BCK0, "MMCabsent");

	/* set card detect irq info */
	irq_data.data = data;
	irq_data.irq_hdlr = irqhdlr;
	request_irq(irq_data.irq,
			ea313x_mci_detect_interrupt,
			IRQ_TYPE_LEVEL_LOW,
			"mmc-cd",
			&irq_data);
	/****temporary for PM testing */
	enable_irq_wake(irq_data.irq);

	return irq_data.irq;
}


static void mci_setpower(u32 ignored_slot, u32 volt)
/*
  any non-zero volt switches 3.3V power on
  when power off, ground all mci lines to prevent backflow
*/
{
  (void) ignored_slot;
  if (volt) {
    GPIO_DRV_IP(IOCONF_EBI_MCI, 0xF0000003);
    gpio_set_value(GPIO_MI2STX_DATA0, 0);
  }else{
    gpio_set_value(GPIO_MI2STX_DATA0, 1);
    GPIO_OUT_LOW(IOCONF_EBI_MCI, 0xF0000003);
  }
}

static int mci_get_bus_wd(u32 slot_id)
{
	return 4;
}

static void mci_exit(u32 slot_id)
{
	free_irq(irq_data.irq, &irq_data);
}

static struct resource lpc313x_mci_resources[] = {
	[0] = {
		.start  = IO_SDMMC_PHYS,
		.end	= IO_SDMMC_PHYS + IO_SDMMC_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_MCI,
		.end	= IRQ_MCI,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct lpc313x_mci_board ea313x_mci_platform_data = {
	.num_slots		= 1,
	.detect_delay_ms	= 250,
	.init 			= mci_init,
	.get_cd 		= mci_get_cd,
	.get_bus_wd		= mci_get_bus_wd,
	.setpower 		= mci_setpower,
	.exit			= mci_exit,
};

static u64 mci_dmamask = 0xffffffffUL;
static struct platform_device	lpc313x_mci_device = {
	.name		= "lpc31_mmc",
	.num_resources	= ARRAY_SIZE(lpc313x_mci_resources),
	.dev		= {
		.dma_mask		= &mci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &ea313x_mci_platform_data,
	},
	.resource	= lpc313x_mci_resources,
};

/*
 * DM9000 ethernet device
 */
#if defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
#include <linux/dm9000.h>
static struct resource dm9000_resource[] = {
	[0] = {
		.start	= EXT_SRAM1_PHYS,
		.end	= EXT_SRAM1_PHYS + 0xFF,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= EXT_SRAM1_PHYS + 0x10000,
		.end	= EXT_SRAM1_PHYS + 0x100FF,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_DM9000_ETH_INT,
		.end	= IRQ_DM9000_ETH_INT,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};
/* ARM MPMC contoller as part of low power design doesn't de-assert nCS and nOE for consecutive
reads but just changes address. But DM9000 requires nCS and nOE change between address. So access
other chip select area (nCS0) to force de-assertion of nCS1 and nOE1. Or else wait for long time
such as 80 usecs.
LPC313x has external logic outside of MPMC IP to toggle nOE to split consecutive reads.
The latest Apex bootloader pacth makes use of this feature.
For this to work SYS_MPMC_WTD_DEL0 & SYS_MPMC_WTD_DEL1 should be programmed with MPMC_STWTRD0
& MPMC_STWTRD1 values. The logic only deactivates the nOE for one clock cycle which is
11nsec but DM9000 needs 80nsec between nOEs. So lets add some dummy instructions such as
reading a GPIO register to compensate for extra 70nsec.
*/
# define DM_IO_DELAY()	do { gpio_get_value(GPIO_MNAND_RYBN3);} while(0)

static void dm9000_dumpblk(void __iomem *reg, int count)
{
	int i;
	int tmp;

	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		tmp = readw(reg);
	}
}

static void dm9000_inblk(void __iomem *reg, void *data, int count)
{
	int i;
	u16* pdata = (u16*)data;
	count = (count + 1) >> 1;
	for (i = 0; i < count; i++) {
		DM_IO_DELAY();
		*pdata++ = readw(reg);
	}
}

static struct dm9000_plat_data dm9000_platdata = {
	.flags		= DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM | DM9000_PLATF_SIMPLE_PHY,
	.dumpblk = dm9000_dumpblk,
	.inblk = dm9000_inblk,
};

static struct platform_device dm9000_device = {
	.name		= "dm9000",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm9000_resource),
	.resource	= dm9000_resource,
	.dev		= {
		.platform_data	= &dm9000_platdata,
	}
};
static void __init ea_add_device_dm9000(void)
{
	/*
	 * Configure Chip-Select 2 on SMC for the DM9000.
	 * Note: These timings were calculated for MASTER_CLOCK = 90000000
	 *  according to the DM9000 timings.
	 */
	MPMC_STCONFIG1 = 0x81;
	MPMC_STWTOEN1 = 1;
	/* enable oe toggle between consec reads */
	SYS_MPMC_WTD_DEL1 = _BIT(5) | (MPMC_STWTRD1 = 4);
	MPMC_STWTWEN1 = 1;
	MPMC_STWTWR1 = 1;

	/* Configure Interrupt pin as input, no pull-up */
        requestGPI(GPIO_MNAND_RYBN3, "DM9000IRQ");

	platform_device_register(&dm9000_device);
}
#else
static void __init ea_add_device_dm9000(void) {}
#endif /* CONFIG_DM9000 */



/*
 * KS8851_MLL ethernet device  -- alternative to DM9000
 *  This chip is interfaced on the same CS line as the DM9000
 *  Only one of the two may be populated
 */
#if defined(CONFIG_KS8851_MLL) || defined(CONFIG_KS8851_MLL_MODULE)

#include "linux/ks8851_mll.h"

#define IRQ_KS8851_ETH_INT IRQ_DM9000_ETH_INT

#define KS8851_base  EXT_SRAM1_PHYS

static struct resource ks8851_resource[] = {
	[0] = {
		.start	= KS8851_base,
		.end	= KS8851_base + 5,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= KS8851_base + 6,
		.end	= KS8851_base + 0x3fff,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= IRQ_KS8851_ETH_INT,
		.end	= IRQ_KS8851_ETH_INT,
		.flags	= IORESOURCE_IRQ,
	}
};

static struct ks8851_mll_platform_data ks8851_data = {
	.mac_addr = {0, 0, 0, 0, 0, 0} //default MAC address
};

static struct platform_device ks8851_device = {
	.name		= "ks8851_mll",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(ks8851_resource),
	.resource	= ks8851_resource,
	.dev		= {
		.platform_data	= &ks8851_data,
	}
};

static void __init ea_add_device_ks8851(u32 timing)
/*
 * Configure Chip-Select 2 for the KS8851 w/16-bit parallel bus interface.
 *  and the FPGA control register in case of the PC/104 bus carrier board.
 * Note: These timings were calculated for MASTER_CLOCK = 90000000
 *  according to the KS8851_MLL timings.
 * 57ns strobes for 3G board
 *  timing spec'd in pairs of octal digits (from most to least significant):
 *     WTTURN WTOEN WTRD WTWEN WTWR
 */
{
  MPMC_STCONFIG1 = 0x81;  /* 16-bit transfers */
  MPMC_STWTWR1 = timing & 037;  //4 for 3G board
  MPMC_STWTWEN1 = (timing >>= 6) & 017;
  /* enable oe toggle between consec reads */
  SYS_MPMC_WTD_DEL1 = _BIT(5) | (MPMC_STWTRD1 = (timing>>=6)&037);  //5 for 3G
  MPMC_STWTOEN1 = (timing >>= 6) & 017;
  MPMC_STWTTURN1 = timing >>= 6;

	/* Configure Interrupt pin as input */
        requestGPI(GPIO_GPIO3, "KS8851IRQ");

	platform_device_register(&ks8851_device);
}
#else
static void __init ea_add_device_ks8851(u32 ignored) {}
#endif /* CONFIG_KS8851_MLL */

//for legacy PC/AT style ports
#define ISAport(_base,_irq)   \
	{		      \
		.membase = (void *)io_p2v(PC104base8+_base),  \
		.mapbase = (unsigned long)(PC104base8+_base), \
		.irq		= _irq,		        \
		.uartclk	= 1843200,	        \
		.regshift       = 0,                    \
		.iotype		= UPIO_MEM,		\
		.flags		= UPF_BOOT_AUTOCONF,    \
		.pm = NULL,                             \
	}

#define XRport(offset) \
	{						\
		.membase = (void *)io_p2v(XRbase+offset),  \
		.mapbase = (unsigned long)(XRbase+offset), \
		.irq		= IRQ_XR16788_INT,      \
		.uartclk	= 14745600,		\
		.regshift       = 0,                    \
		.iotype		= UPIO_MEM,		\
		.type           = PORT_XR16788,         \
		.flags		= UPF_BOOT_AUTOCONF, 	\
		.pm = NULL,                             \
	}

static struct plat_serial8250_port exar_data[] = {
#if 0
	XRport(0x00),
	XRport(0x10),
	XRport(0x20),
	XRport(0x30),
	XRport(0x40),
	XRport(0x50),
	XRport(0x60),
	XRport(0x70),
#endif
	{ },
};

static struct platform_device xr16788_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM1,
	.dev			= {
		.platform_data	= exar_data,
	},
};

static struct plat_serial8250_port isa_data[] = {
	ISAport(0x3f8, IRQ_XR16788_INT),
	ISAport(0x2f8, IRQ_XR16788_INT),
	ISAport(0x3e8, IRQ_XR16788_INT),
	ISAport(0x2e8, IRQ_XR16788_INT),
	{ },
};

static struct platform_device isa_device = {
	.name			= "serial8250",
	.id			= PLAT8250_DEV_PLATFORM2,
	.dev			= {
		.platform_data	= isa_data,
	},
};


static void __init ea_add_device_octalUart(u32 timing)
/*
 * Configure Chip-Select 1 for the XR16L788 USART w/8-bit parallel bus interface.
 *  timing spec'd in pairs of octal digits (from most to least significant):
 *     WTTURN WTOEN WTRD WTWEN WTWR
 */
{
  MPMC_STCONFIG0 = 0x80;  /* 8-bit transfers */
  MPMC_STWTWR0 = timing & 037;     //5 for 3G board
  MPMC_STWTWEN0 = (timing >>= 6) & 017;
  /* enable oe toggle between consec reads */
  SYS_MPMC_WTD_DEL0 = _BIT(5) | (MPMC_STWTRD0 = (timing>>=6)&037);  //6 for 3G
  MPMC_STWTOEN0 = (timing >>= 6) & 017;
  MPMC_STWTTURN0 = timing >>= 6;

	platform_device_register(&xr16788_device);
}


#if defined(CONFIG_MTD_NAND_LPC313X) || defined(CONFIG_MTD_NAND_LPC313X_MODULE)
static struct resource lpc313x_nand_resources[] = {
	[0] = {
		.start  = IO_NAND_PHYS,
		.end	= IO_NAND_PHYS + IO_NAND_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start 	= IO_NAND_BUF_PHYS,
		.end 	= IO_NAND_BUF_PHYS + IO_NAND_BUF_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start 	= IRQ_NAND_FLASH,
		.end 	= IRQ_NAND_FLASH,
		.flags	= IORESOURCE_IRQ,
	}
};

#define BLK_SIZE (2048 * 64)
static struct mtd_partition ea313x_nand0_partitions[] = {
	/* The EA3131 board uses the following block scheme:
	128K: Blocks 0   - 0    - LPC31xx info and bad block table
	384K: Blocks 1   - 3    - Apex bootloader
	256K: Blocks 4   - 5    - Apex environment
	4M:   Blocks 6   - 37   - Kernel image
	16M:  Blocks 38  - 165  - Ramdisk image (if used)
	???:  Blocks 166 - end  - Root filesystem/storage */
	{
		.name	= "lpc31nand-std.rootfs",
		.offset	= (BLK_SIZE * 166),
		.size	= MTDPART_SIZ_FULL
	},
	{
		.name	= "lpc31nand-big.rootfs.",
		.offset	= (BLK_SIZE * 38),
		.size	= MTDPART_SIZ_FULL
	},
	{
		.name	= "lpc31nand-max.rootfs",
		.offset	= (BLK_SIZE * 1),
		.size	= MTDPART_SIZ_FULL
	},
	{
		.name	= "lpc31nand-kernel",
		.offset	= (BLK_SIZE * 6),
		.size	= (BLK_SIZE * 32) /* 4MB space */
	},
	{
		.name	= "lpc31nand-ramdisk",
		.offset	= MTDPART_OFS_APPEND,
		.size	= (BLK_SIZE * 128) /* 16MB space */
	},
};

static struct lpc313x_nand_timing ea313x_nanddev_timing = {
	.ns_trsd	= 36,
	.ns_tals	= 36,
	.ns_talh	= 12,
	.ns_tcls	= 36,
	.ns_tclh	= 12,
	.ns_tdrd	= 36,
	.ns_tebidel	= 12,
	.ns_tch		= 12,
	.ns_tcs		= 48,
	.ns_treh	= 24,
	.ns_trp		= 48,
	.ns_trw		= 24,
	.ns_twp		= 36
};

static struct lpc313x_nand_dev_info ea313x_ndev[] = {
	{
		.name		= "nand0",
		.nr_partitions	= ARRAY_SIZE(ea313x_nand0_partitions),
		.partitions	= ea313x_nand0_partitions
	}
};

static struct lpc313x_nand_cfg ea313x_plat_nand = {
	.nr_devices	= ARRAY_SIZE(ea313x_ndev),
	.devices	= ea313x_ndev,
	.timing		= &ea313x_nanddev_timing,
	.support_16bit	= 0,
};

static u64 nand_dmamask = 0xffffffffUL;
static struct platform_device	lpc313x_nand_device = {
	.name		= "lpc31_nand",
	.dev		= {
		.dma_mask		= &nand_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data	= &ea313x_plat_nand,
	},
	.num_resources	= ARRAY_SIZE(lpc313x_nand_resources),
	.resource	= lpc313x_nand_resources,
};
#endif

#if defined(CONFIG_SPI_LPC313X) || defined(CONFIG_SPI_LPC313X_MODULE)
static struct resource lpc313x_spi_resources[] = {
	[0] = {
		.start	= SPI_PHYS,
		.end	= SPI_PHYS + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_SPI,
		.end	= IRQ_SPI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct mtd_partition nor_spi_flash_partitions[] = {
	{
		.name	= "lpc31nor-bootloader",
		.offset	= 0,
		.size	= 0x40000
	},
	{
		.name	= "lpc31nor-bootenv",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 16*1024
	},
	{
		.name	= "lpc31nor-unused",
		.offset	= MTDPART_OFS_APPEND,
		.size	= 48*1024
	},
	{
		.name	= "lpc31nor-kernel",
		.offset	= MTDPART_OFS_APPEND,
		.size	= MTDPART_SIZ_FULL
	},
};

static struct flash_platform_data spi_flash_data = {
	.name		= "lpc31_nor",
	.parts		= nor_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(nor_spi_flash_partitions),
};

static u64 lpc313x_spi_dma_mask = 0xffffffffUL;
static struct platform_device lpc313x_spi_device = {
	.name		= "spi_lpc31",
	.id		= 0,
	.dev		= {
		.dma_mask = &lpc313x_spi_dma_mask,
		.coherent_dma_mask = 0xffffffffUL,
	},
	.num_resources	= ARRAY_SIZE(lpc313x_spi_resources),
	.resource	= lpc313x_spi_resources,
};

/* If both SPIDEV and MTD data flash are enabled with the same chip select, only 1 will work */
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
/* SPIDEV driver registration */
static void spi_set_cs_user(int state)
{
  gpio_direction_output(GPIO_MUART_RTS_N, state);
}

static int __init lpc313x_spidev_register(void)
{
	static struct spi_board_info info __initdata =
	{
		.modalias = "spidev",
		.chip_select = lpc31spiUserDev,
		.max_speed_hz = 1000000,
		.controller_data = spi_set_cs_user,
	};
	return spi_register_board_info(&info, 1);
}
arch_initcall(lpc313x_spidev_register);
#endif

/*  either one Amtel DataFlash *or* Spansion SPI NOR flash may be loaded */
/* MTD Data FLASH driver registration */

#if defined(CONFIG_MTD_DATAFLASH) || defined(CONFIG_MTD_M25P80) || \
	defined(CONFIG_MTD_DATAFLASH_MODULE) || defined(CONFIG_MTD_M25P80_MODULE)
static void spi_set_cs_flash(int state)
{
  gpio_direction_output(GPIO_SPI_CS_OUT0, state);
}
#endif

static int __init lpc313x_spimtd_register(void)
{
	static struct spi_board_info info[] __initdata = {
#if defined(CONFIG_MTD_DATAFLASH) || defined(CONFIG_MTD_DATAFLASH_MODULE)
	  {
		.modalias = "mtd_dataflash",
		.chip_select = lpc31spiAtmelFlash,
		.max_speed_hz = 30000000,
		.controller_data = spi_set_cs_flash,
		.platform_data	= &spi_flash_data,
	  },
#endif
#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
	  {
		.modalias = "m25p80",
		.chip_select = lpc31spiSpansionFlash,
		.max_speed_hz = 45000000,  //max spi speed of lpc31
		.controller_data = spi_set_cs_flash,
		.platform_data	= &spi_flash_data,
	  }
#endif
	};
	return spi_register_board_info(info, ARRAY_SIZE(info));
}
arch_initcall(lpc313x_spimtd_register);
#endif

static struct platform_device *devices[] __initdata = {
	&lpc313x_mci_device,
#if defined(CONFIG_MTD_NAND_LPC313X) || defined(CONFIG_MTD_NAND_LPC313X_MODULE)
	&lpc313x_nand_device,
#endif
#if defined(CONFIG_SPI_LPC313X) || defined(CONFIG_SPI_LPC313X_MODULE)
	&lpc313x_spi_device,
#endif
};

static struct map_desc ea313x_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(EXT_SRAM0_PHYS),
		.pfn		= __phys_to_pfn(EXT_SRAM0_PHYS),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(EXT_SRAM1_PHYS + 0x10000),
		.pfn		= __phys_to_pfn(EXT_SRAM1_PHYS + 0x10000),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_SDMMC_PHYS),
		.pfn		= __phys_to_pfn(IO_SDMMC_PHYS),
		.length		= IO_SDMMC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_USB_PHYS),
		.pfn		= __phys_to_pfn(IO_USB_PHYS),
		.length		= IO_USB_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(PC104FPGAadr),
		.pfn		= __phys_to_pfn(PC104FPGAadr),
		.length		= 2,
		.type		= MT_DEVICE
	},
};

static struct i2c_board_info ea313x_i2c_devices[] __initdata = {
	{
		I2C_BOARD_INFO("pca9532", 0x60),
	},
};

#if defined(CONFIG_MACH_EA3152)
static struct i2c_board_info ea3152_i2c1_devices[] __initdata = {
	{
		I2C_BOARD_INFO("lpc3152-psu", 0x0C),
	},
};
#endif


/*
   Group of gpios used for enabling RS-232 drivers on ESP 3G host
   This group of bits read back as zeros initially on the ESP3G host
   They read back as xxx on the PC104 carrier
   They read back as TODO on the OEM board
*/
#define firstSerialGPIO GPIO_GPIO11
#define numSerialChannels 8
#define allSerialChannels ((1<<numSerialChannels)-1)

#define PeripheralReset  GPIO_I2SRX_BCK0  //resets KS8851 chip on 3G host board

/* defined in irq.c */
int replace_irq(u32 existingIrq, EVENT_T newEvent_pin, EVENT_TYPE_T newType);

static void __init init_irq(void)
{
  if (boardID < 0) //read GPIOs to identify carrier board type
    boardID = (GPIO_STATE(IOCONF_FAST_GPIO)>>firstSerialGPIO)&allSerialChannels;
  if (mbariBoard(boardID)) {
    replace_irq(IRQ_DM9000_ETH_INT, EVT_GPIO3, EVT_ACTIVE_LOW);
    replace_irq(IRQ_EA_VBUS_OVRC, EVT_NAND_NCS_2, EVT_ACTIVE_LOW);
    if (boardID & 0xff) //PC/104 carrier with octal USART on the ESP2G baseboard
      replace_irq(IRQ_XR16788_INT, XR_EVT, EVT_RISING_EDGE);
    else         //ESP 3G with on board octal USART
      replace_irq(IRQ_XR16788_INT, EVT_mNAND_RYBN3, EVT_ACTIVE_LOW);
  }
  lpc313x_init_irq();
}


#if defined(CONFIG_RTC_DRV_DS3234) || defined(CONFIG_RTC_DRV_DS3234_MODULE)
/*
  initialization common to ESP3G and PC104 carrier
*/
static void spi_set_cs_rtc(int state)
{
  gpio_direction_output(GPIO_MUART_CTS_N, state);
}
static struct spi_board_info rtc __initdata = {
	.modalias = "ds3234",
	.chip_select = lpc31spiRTC,
	.max_speed_hz = 2500000,
	.controller_data = spi_set_cs_rtc
};
#endif

static void __init boardInit(const char *signon, u32 timing)
{
  /*  Note that reset generator chip may extend reset by 100ms
      Therefore, it it important to hold off initializing the KS8851 ethernet
      Loading the KS8851 driver from a kernel module ensures this
  */
  printk(signon);
  ea_add_device_ks8851(timing);
  SYS_MUX_UART_SPI = 1;  //SPI CS1 & CS2 lines replace USART CTS & RTS
  requestGPO(GPIO_MUART_RTS_N, "SPIdevCS", 1);
#if defined(CONFIG_RTC_DRV_DS3234) || defined(CONFIG_RTC_DRV_DS3234_MODULE)
  requestGPO(GPIO_MUART_CTS_N, "ds3234CS", 1);
  spi_register_board_info(&rtc, 1);
#endif

  //I2STX_WS0 should be wired to USB_ID
  //early PC104 carrier boards mistakenly connect NAND_RYBN2 to USB_ID
  requestGPO(GPIO_MI2STX_WS0, "USBgadget",
#if defined(CONFIG_USB_GADGET)
    1
#else
    0
#endif
      );

  gpio_sysfs_set_active_low(GPIO_NAND_NCS_2, 1);
  exportGPI(GPIO_NAND_NCS_2, "USBoverload");

  //boot2 is normally pulled high.
  //this keeps USB PWR off during reset (when GPIO19 is not available)
  lpc313x_USBpower = GPIO_GPIO2;
  if (!(boardID & 0x1000)) {  //normal case with USB PWR control on GPIO19
    exportBootI(GPIO_GPIO2, "boot2");
    lpc313x_USBpower = GPIO_GPIO19;
  }
  exportGPO(lpc313x_USBpower, "USB+5V", 1);

  lpc313x_LED = GPIO_GPIO0; //(in case GPIO20 is not available)
  if (!(boardID & 0x2000)) {  //normal case with dedicated output on GPIO20
    exportBootI(GPIO_GPIO0, "boot0");
    lpc313x_LED = GPIO_GPIO20;
  }
}


static void __init ea313x_init(void)
{
  extern unsigned int nr_uarts;  //kludge to avoid extra /dev/ttyS* nodes
  unsigned long resetDone;

  lpc313x_init();

  requestGPO(GPIO_SPI_CS_OUT0, "SPIflashCS", 1);

  platform_add_devices(devices, ARRAY_SIZE(devices));

  /* register i2c busses */
  lpc313x_register_i2c_devices();

#if defined(CONFIG_MACH_EA3152)
  i2c_register_board_info(1, ea3152_i2c1_devices,
	   ARRAY_SIZE(ea3152_i2c1_devices));
#endif
       /* add other devices depending on carrier board type */
  switch (boardID & 0xff) {
    case 0:  /* ESP 3G baseboard */
      requestGPO(PeripheralReset, "PeripheralReset", 0);  /* assert reset */
      gpio_export(PeripheralReset, 1);   /* echo low > gpio58/direction */
      resetDone = jiffies + HZ/100 + 1;  /* deassert at least 10ms from now */
      boardInit("MBARI ESP 3G\n", 0000050004); //fast directly connected strobes
	  shared8250IRQ = XRsharedIRQ;
      ea_add_device_octalUart(0000060005);
      /* Configure UART Interrupt pin as input, no pull-up */
      requestGPI(GPIO_MNAND_RYBN3, "XR16788IRQ");

    /* enable power for each octal UART channels' RS-232 buffer chip */
      exportGPO(GPIO_GPIO11, "ttyS1", 1);
      exportGPO(GPIO_GPIO12, "ttyS2", 1);
      exportGPO(GPIO_GPIO13, "ttyS3", 1);
      exportGPO(GPIO_GPIO14, "ttyS4", 1);
      exportGPO(GPIO_GPIO15, "ttyS5", 1);
      exportGPO(GPIO_GPIO16, "ttyS6", 1);
      exportGPO(GPIO_GPIO17, "ttyS7", 1);
      exportGPO(GPIO_GPIO18, "ttyS8", 1);
      while (jiffies < resetDone) ;          /* deassert reset after 10ms */
      gpio_direction_input(PeripheralReset); /* echo in > gpio58/direction */
      break;

    case 0104:  /* octal 0104 denotes PC/104 carrier */
      PC104FPGA = PC104IRQID | PC104ENETRESET | PC104RESET;
      resetDone = jiffies + HZ/100 + 1;  /* deassert at least 10ms from now */
      boardInit(" PC/104 Carrier\n", 0200060004);  //slower when routed via FPGA
      ea_add_device_octalUart(0202130212);  //much slower for FPGA on PC/104 bus
      if (nr_uarts > 9)  //if there are sufficient I/O ports allocated...
        platform_device_register(&isa_device);  //add legacy ISA ports
      /* Configure UART Interrupt pin as input, no pull-up */
	  shared8250IRQ = pc104sharedIRQ;
      requestGPI(XR_IRQ, "XR16788IRQ");
      while (jiffies < resetDone) ;      /* wait 10ms */
      PC104FPGA = PC104IRQID;            /* deassert enet and PC/104 resets */
      break;

    default:
	  i2c_register_board_info(0, ea313x_i2c_devices,
			  ARRAY_SIZE(ea313x_i2c_devices));
      printk("Embedded Artists LPC31xx (boardID=0x%02x)\n", boardID);
	      /* set the I2SRX_WS0 pin as GPIO_IN for vbus overcurrent flag */
      gpio_sysfs_set_active_low(GPIO_I2SRX_WS0, 1);
      exportGPI(GPIO_I2SRX_WS0, "USBoverload");
      exportBootI(GPIO_GPIO0, "boot0");
      ea_add_device_dm9000();
      nr_uarts = 1;  //avoids having unintialized ports under /dev/ttyS*
  }
  gpio_led_register_device(-1, &lpc_leds);
}


void __init awaitPeripheralReset(void)
/*
  wait up to 1 second for RTC to deassert reset
*/
{
  if (!(boardID & 0xff)) {  //only the 3G host board requires this delay
    unsigned consecutive = 0;
    unsigned long resetDone = jiffies+HZ;
    while (jiffies < resetDone)
      if (gpio_get_value(PeripheralReset)) {
        if (++consecutive >= 5)
          return;  /* reset deasserted for 5 consecutive samples */
      }else
        consecutive = 0;
    printk(KERN_ERR "Peripheral Reset stuck asserted(low)!!\n");
  }
}


void lpc313x_vbus_power(int enable)
{  //FIXME:  This does not handle the Embedded Artists dev board
	if (lpc313x_USBpower >= 0) {
		if (enable) {
			printk (KERN_INFO "USB power ON\n");
			gpio_set_value(lpc313x_USBpower, 1);
		} else {
			printk (KERN_INFO "USB power OFF\n");
			gpio_set_value(lpc313x_USBpower, 0);
		}
	}
}

static void lpc313x_reset(enum reboot_mode mode, const char *cmd)
{
	(void) mode; //unused
	printk("arch_reset: via watchdog\n");

	/* enable WDT clock */
	cgu_clk_en_dis(CGU_SB_WDOG_PCLK_ID, 1);

	/* Disable watchdog */
	WDT_TCR = 0;
	WDT_MCR = WDT_MCR_STOP_MR1 | WDT_MCR_INT_MR1;

	/*  If TC and MR1 are equal a reset is generated. */
	WDT_PR  = 0x00000002;
	WDT_TC  = 0x00000FF0;
	WDT_MR0 = 0x0000F000;
	WDT_MR1 = 0x00001000;
	WDT_EMR = WDT_EMR_CTRL1(0x3);
	/* Enable watchdog timer; assert reset at timer timeout */
	WDT_TCR = WDT_TCR_CNT_EN;
	cpu_reset (0);/* loop forever and wait for reset to happen */

	/*NOTREACHED*/
}


static void __init ea313x_map_io(void)
{
	lpc313x_map_io();
	iotable_init(ea313x_io_desc, ARRAY_SIZE(ea313x_io_desc));
}

extern void __init lpc313x_timer_init(void);
extern void __init cgu_init(void);

#if defined(CONFIG_MACH_EA3152)
MACHINE_START(EA3152, "NXP LPC3152")
#elif defined(CONFIG_MACH_EA313X)
MACHINE_START(EA313X, "NXP LPC31xx")
#else
#error Must select either EA313X or EA3152
#endif
	/* Maintainer: Durgesh Pattamatta, NXP */
	.atag_offset	= 0x100,
	.map_io		= ea313x_map_io,
	.init_irq	= init_irq,
	.init_early = cgu_init,
	.init_time	= lpc313x_timer_init,
	.init_machine	= ea313x_init,
	.restart	= lpc313x_reset,
MACHINE_END

