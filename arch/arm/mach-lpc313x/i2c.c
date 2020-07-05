/*  linux/arch/arm/mach-lpc313x/i2c.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * I2C initialization for LPC313x.
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

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/i2c-pnx.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <mach/i2c.h>
#include <mach/gpio.h>
#include <mach/irqs.h>

#define LPC313x_I2C0_SLV_ADDR            __REG (I2C0_PHYS + 0x014)
#define LPC313x_I2C1_SLV_ADDR            __REG (I2C1_PHYS + 0x014)
#define __initdata
static struct i2c_pnx_algo_data lpc_algo_data0 = {
	.clk = CGU_SB_I2C0_PCLK_ID
};
static struct resource lpcI2C0_resources[] __initdata = {
	{
		.start  = I2C0_PHYS,
		.end	= I2C0_PHYS + I2C_PNX_REGION_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_I2C0,
		.end	= IRQ_I2C0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	}
};
static struct platform_device i2c0_bus __initdata = {
	.name = "pnx-i2c",
	.id = 0,
	.resource = lpcI2C0_resources,
	.num_resources= ARRAY_SIZE(lpcI2C0_resources),
	.dev = {
		.platform_data = &lpc_algo_data0
	}
};

static struct i2c_pnx_algo_data lpc_algo_data1 = {
	.clk = CGU_SB_I2C1_PCLK_ID
};
static struct resource lpcI2C1_resources[] __initdata = {
	{
		.start  = I2C1_PHYS,
		.end	= I2C1_PHYS + I2C_PNX_REGION_SIZE,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= IRQ_I2C1,
		.end	= IRQ_I2C1,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL,
	},
};
static struct platform_device i2c1_bus __initdata = {
	.name = "pnx-i2c",
	.id = 1,
	.resource = lpcI2C1_resources,
	.num_resources= ARRAY_SIZE(lpcI2C1_resources),
	.dev = {
		.platform_data = &lpc_algo_data1
	}
};

static struct platform_device *i2c_busses[] __initdata = {
	&i2c0_bus,
	&i2c1_bus,
};

void __init lpc313x_register_i2c_busses(void)
{
	cgu_clk_disable(CGU_SB_I2C0_PCLK_ID);
	cgu_clk_disable(CGU_SB_I2C1_PCLK_ID);

	/* Enable I2C1 signals */
	GPIO_DRV_IP(IOCONF_I2C1, 0x3);

#if defined (CONFIG_MACH_VAL3153) || defined (CONFIG_MACH_EA313X)
	/* on EA and VAL boards UDA1380 is connected to I2C1
	 * whose slave address is same as LPC313x's default slave
	 * adress causing bus contention errors. So change the
	 * deafult slave address register value of LPC313x here.
	 */
	LPC313x_I2C0_SLV_ADDR = 0x06E;
	LPC313x_I2C1_SLV_ADDR = 0x06E;
#endif

	platform_add_devices(i2c_busses, ARRAY_SIZE(i2c_busses));
}

