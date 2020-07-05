/*
 * Header file for I2C support on PNX010x/4008.
 *
 * Author: Dennis Kovalev <dkovalev@ru.mvista.com>
 *
 * 2004-2006 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __I2C_PNX_H__
#define __I2C_PNX_H__

#include <linux/i2c.h>

#define I2C_PNX_TIMEOUT_DEFAULT		10 /* msec */
#define I2C_PNX_SPEED_KHZ_DEFAULT	100
#define I2C_PNX_REGION_SIZE	0x100

#ifdef CONFIG_HAVE_CLK
#	define PNXclkRef(id) struct clk  *id
#	define PNXclkEnable(id)	clk_enable(id)
#	define PNXclkDisable(id) clk_disable(id)
#	define PNXclkRate(id)	clk_get_rate(id)
#else
#	include <mach/cgu.h>
#	define PNXclkRef(id) CGU_CLOCK_ID_T	id
#	define PNXclkEnable(id)	cgu_clk_enable(id)
#	define PNXclkDisable(id) cgu_clk_disable(id)
#	define PNXclkRate(id)	cgu_get_clk_freq(id)
#endif

struct i2c_pnx_mif {
	int			ret;		/* Return value */
	int			mode;		/* Interface mode */
	struct completion	complete;	/* I/O completion */
	struct timer_list	timer;		/* Timeout */
	u8 *		buf;		/* Data buffer */
	int			len;		/* Length of data buffer */
	int			order;		/* RX Bytes to order via TX */
};

struct i2c_pnx_algo_data {
	void __iomem	*ioaddr;
	int			irq;
	PNXclkRef(clk);
	u32			timeout;
	struct i2c_pnx_mif	mif;
	int			last;
	struct i2c_adapter	adapter;
};

#endif /* __I2C_PNX_H__ */
