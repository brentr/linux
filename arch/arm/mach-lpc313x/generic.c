/*  arch/arm/mach-lpc313x/generic.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Common code for machines with LPC313x and LPC315x SoCs.
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
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/console.h>
#include <linux/serial_8250.h>

#include <asm/errno.h>
#include <mach/hardware.h>

#include <mach/gpio.h>
#include <asm/mach/map.h>

/* local functions */

static void lpc313x_uart_pm(struct uart_port * port, unsigned int state,
			      unsigned int oldstate)
{
	switch (state) {
	case 0:
		/*
		 * Enable the peripheral clock for this serial port.
		 * This is called on uart_open() or a resume event.
		 */
		/* Enable UART base clock */
		cgu_endis_base_freq(CGU_SB_UARTCLK_BASE_ID, 1);

		/* Enable UART IP clock */
		cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);
		cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
		break;
	case 1:
		/* we can wake the system in this state. So leave clocks on */
		printk(KERN_INFO "lpc31_uart_pm: UART can wake\n");
		break;
	case 3:
		/*
		 * Disable the peripheral clock for this serial port.
		 * This is called on uart_close() or a suspend event.
		 */
		/* Disable UART IP clock */
		cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 0);
		cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 0);

		/* Disable UART base clock */
		cgu_endis_base_freq(CGU_SB_UARTCLK_BASE_ID, 0);
		break;
	default:
		printk(KERN_ERR "lpc31_uart_pm: unknown pm %d\n", state);
	}

}

static struct plat_serial8250_port platform_serial_ports[] = {
	{
		.membase = (void *)io_p2v(UART_PHYS),
		.mapbase = (unsigned long)UART_PHYS,
		.irq = IRQ_UART,
		.uartclk = XTAL_CLOCK,
		.regshift = 2,
		.iotype = UPIO_MEM,
		.type	= PORT_NXP16750,
		.flags = UPF_FIXED_TYPE|UPF_FIXED_PORT | UPF_BUGGY_UART|UPF_SKIP_TEST,
		.pm = lpc313x_uart_pm,
	},
	{
		.flags		= 0
	},
};

static struct platform_device serial_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	.dev = {
		.platform_data = &platform_serial_ports,
	},
};

#if defined(CONFIG_HW_RANDOM_LPC313X) || defined(CONFIG_HW_RANDOM_LPC313X_MODULE)

static struct resource rng_resources[] = {
	{
		.start = RNG_PHYS,
		.end   = RNG_PHYS + 4096,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device rng_device = {
	.name = "lpc31-rng",
	.id   = -1,
	.num_resources = ARRAY_SIZE(rng_resources),
	.resource = rng_resources,
};

#endif

static struct platform_device *devices[] __initdata = {
	&serial_device,
#if defined(CONFIG_HW_RANDOM_LPC313X) || defined(CONFIG_HW_RANDOM_LPC313X_MODULE)
	&rng_device,
#endif
};

static struct map_desc lpc313x_io_desc[] __initdata = {
	{
		.virtual	= io_p2v(IO_INTC_PHYS),
		.pfn		= __phys_to_pfn(IO_INTC_PHYS),
		.length		= IO_INTC_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB01_PHYS),
		.pfn		= __phys_to_pfn(IO_APB01_PHYS),
		.length		= IO_APB01_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB2_PHYS),
		.pfn		= __phys_to_pfn(IO_APB2_PHYS),
		.length		= IO_APB2_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB3_PHYS),
		.pfn		= __phys_to_pfn(IO_APB3_PHYS),
		.length		= IO_APB3_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_APB4_PHYS),
		.pfn		= __phys_to_pfn(IO_APB4_PHYS),
		.length		= IO_APB4_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_MPMC_CFG_PHYS),
		.pfn		= __phys_to_pfn(IO_MPMC_CFG_PHYS),
		.length		= IO_MPMC_CFG_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_NAND_BUF_PHYS),
		.pfn		= __phys_to_pfn(IO_NAND_BUF_PHYS),
		.length		= IO_NAND_BUF_SIZE,
		.type		= MT_DEVICE
	},
	{
		.virtual	= io_p2v(IO_ISRAM0_PHYS),
		.pfn		= __phys_to_pfn(IO_ISRAM0_PHYS),
		.length		= IO_ISRAM0_SIZE,
		.type		= MT_DEVICE
	},
};

void __init lpc313x_map_io(void)
{
	iotable_init(lpc313x_io_desc, ARRAY_SIZE(lpc313x_io_desc));
}

#define GPIO_BOOT0 GPIO_GPIO0
#define GPIO_BOOT1 GPIO_GPIO1
#define GPIO_BOOT2 GPIO_GPIO2

static void lpc313x_export_bootsel(void) {
	/* pre-request and export boot mode selector pins */
	gpio_export(GPIO_BOOT0, 1);
	gpio_request(GPIO_BOOT1, "boot1");
	gpio_direction_input(GPIO_BOOT1);
	gpio_export(GPIO_BOOT1, 1);
    //boot2's and 0's direction depends on baseboard type -- init it later..
}


#ifndef CONFIG_SERIAL_8250_CONSOLE
static void __init lpc313x_uart_init(void)
{
	int mul, div;

	/* Switch on the UART clocks */
	cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_IOCONF_PCLK_ID, 1);

	/* check what FDR bootloader is using */
	mul = (UART_FDR_REG >> 4) & 0xF;
	div = UART_FDR_REG & 0xF;
	if (div != 0)  {
		platform_serial_ports[0].uartclk = (XTAL_CLOCK * mul) / (mul + div);
	}
}
#endif

#ifdef CONFIG_DEBUG_FS
extern void __init lpc313x_timer_init_debugfs(void);
extern void __init lpc313x_cgu_init_debugfs(void);
#endif


int __init lpc313x_init(void)
{
	/* Put adc block in low power state.
	 * Once ADC driver is added this should move to driver.
	 */
	SYS_ADC_PD = 1;
	/* Disable ring oscillators used by Random number generators */
	SYS_RNG_OSC_CFG = 0;

	/* Mux I2S signals based on selected channel */
#if defined (CONFIG_SND_I2S_TX0_MASTER)
	/* I2S TX0 WS, DATA */
	GPIO_DRV_IP(IOCONF_EBI_I2STX_0, 0x60);

	/* I2S TX0 BCK */
	GPIO_DRV_IP(IOCONF_EBI_MCI, 0x80);
#endif

#if defined (CONFIG_SND_I2S_TX1_MASTER)
	/* I2S TX1 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2STX_1, 0x7);
#endif

#if defined (CONFIG_SND_I2S_RX0_MASTER) | defined (CONFIG_SND_I2S_RX0_SLAVE)
	/* I2S RX0 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2SRX_0, 0x7);
#endif
#if defined (CONFIG_SND_I2S_RX1_MASTER) | defined (CONFIG_SND_I2S_RX1_SLAVE)
	/* I2S RX1 BCK, WS, DATA */
	GPIO_DRV_IP(IOCONF_I2SRX_1, 0x7);
#endif
	/* AUDIO CODEC CLOCK (256FS) */
	GPIO_DRV_IP(IOCONF_I2STX_1, 0x8);

#ifndef CONFIG_SERIAL_8250_CONSOLE
	lpc313x_uart_init();
#endif

	lpc313x_gpiolib_init();

	lpc313x_export_bootsel();

#ifdef CONFIG_DEBUG_FS
	lpc313x_cgu_init_debugfs();
	lpc313x_timer_init_debugfs();
#endif

	return platform_add_devices(devices, ARRAY_SIZE(devices));
}

#if defined(CONFIG_SERIAL_8250_CONSOLE)
static int __init lpc313x_init_console(void)
{
	static __initdata char serr[] =
		KERN_ERR "Serial port #%u setup failed\n";
	struct uart_port up;
	int mul, div;
       unsigned bootFDR;

	/* Switch on the UART clocks and reset it */
	cgu_clk_en_dis(CGU_SB_UART_APB_CLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_UART_U_CLK_ID, 1);
       bootFDR = UART_FDR_REG;
	cgu_soft_reset_module(UART_SYS_RST_AN_SOFT);
       UART_FDR_REG = bootFDR;  //restore bootloader's FDR

 	/*
	 * Set up serial port #0. Do not use autodetection; the result is
	 * not what we want.
 	 */
	memset(&up, 0, sizeof(up));

	up.membase = (char *) io_p2v(UART_PHYS);
	up.mapbase = (unsigned long)UART_PHYS,
	up.irq = IRQ_UART;
	up.uartclk = XTAL_CLOCK;

	/* use bootloader's FDR */
	mul = (bootFDR >> 4) & 0xF;
	div = bootFDR & 0xF;
	if (div != 0)  {
		up.uartclk = (XTAL_CLOCK * mul) / (mul + div);
	}

	up.regshift = 2;
	up.iotype = UPIO_MEM;
	up.type	= PORT_NXP16750;
	up.flags = UPF_BOOT_AUTOCONF| UPF_FIXED_TYPE | UPF_FIXED_PORT |
    			 UPF_BUGGY_UART|UPF_SKIP_TEST;
	up.line	= 0;
	platform_serial_ports[0].uartclk = up.uartclk;
	if (early_serial_setup(&up))
		printk(serr, up.line);

	return 0;
}
console_initcall(lpc313x_init_console);

#endif /*CONFIG_SERIAL_8250_CONSOLE*/
