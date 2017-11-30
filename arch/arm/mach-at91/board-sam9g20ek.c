/*
 *  Copyright (C) 2005 SAN People
 *  Copyright (C) 2008 Atmel
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
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/at73c213.h>
#include <linux/i2c/at24.h>
#include <linux/clk.h>
#include <linux/dm9000.h>
#include <linux/mtd/physmap.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <mach/at91sam9_smc.h>

#include "sam9_smc.h"
#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 18.432 MHz crystal */
	at91sam9260_initialize(18432000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 0, 0);//debug

	/* USART0 on ttyS1. (Rx, Tx, CTS, RTS, DTR, DSR, DCD, RI) */
/*	at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS \
			   | ATMEL_UART_DTR | ATMEL_UART_DSR | ATMEL_UART_DCD \
			   | ATMEL_UART_RI);
*/
	/* USART1 on ttyS1. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9260_ID_US0, 1, ATMEL_UART_CTS | ATMEL_UART_RTS); //com0
	/* USART2 on ttyS2. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9260_ID_US1, 2, ATMEL_UART_CTS | ATMEL_UART_RTS);//com1
	/* USART3 on ttyS3. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US2, 3, 0);//com2
	/* USART4 on ttyS4. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US3, 4, 0);//com3
	/* USART5 on ttyS5. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US4, 5, 0);//com4
	/* USART6 on ttyS6. (Rx, Tx) */
	at91_register_uart(AT91SAM9260_ID_US5, 6, 0);//com5


	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(0);
}

static void __init ek_init_irq(void)
{
	at91sam9260_init_interrupts(NULL);
}


static struct at91_cf_data at91sam9g20ek_cf_data = {
    .det_pin    = AT91_PIN_PB22,
    .rst_pin    = AT91_PIN_PB23,
    .irq_pin    = AT91_PIN_PB24,
    // .vcc_pin = ... always powered
    .chipselect = 4,
};


/*
 * USB Host port
 */
static struct at91_usbh_data __initdata ek_usbh_data = {
	.ports		= 2,
};

/*
 * USB Device port
 */
static struct at91_udc_data __initdata ek_udc_data = {
	.vbus_pin	= AT91_PIN_PC5,
	.pullup_pin	= 0,		/* pull-up driven by UDC */
};

/*
 * DM9000 ethernet device
 */
#if defined(CONFIG_DM9000)
static struct resource dm9000_resource[] = {
    [0] = {
        .start  = AT91_CHIPSELECT_2,
        .end    = AT91_CHIPSELECT_2 + 3,
        .flags  = IORESOURCE_MEM
    },
    [1] = {
        .start  = AT91_CHIPSELECT_2 + 0x4,
        .end    = AT91_CHIPSELECT_2 + 0x7,
        .flags  = IORESOURCE_MEM
    },
    [2] = {
        .start  = AT91_PIN_PB25,
        .end    = AT91_PIN_PB25,
        .flags  = IORESOURCE_IRQ
    }
};


static struct dm9000_plat_data dm9000_platdata = {
    .flags      = DM9000_PLATF_16BITONLY|DM9000_PLATF_NO_EEPROM,
};

static struct platform_device dm9000_device = {
    .name       = "dm9000",
    .id     = 0,
    .num_resources  = ARRAY_SIZE(dm9000_resource),
    .resource   = dm9000_resource,
    .dev        = {
        .platform_data  = &dm9000_platdata,
    }
};

/*
 * SMC timings for the DM9000.
 * Note: These timings were calculated for MASTER_CLOCK = 100000000 according to the DM9000 timings.
 */

static struct sam9_smc_config __initdata dm9000_smc_config = {
    .ncs_read_setup     = 0,
    .nrd_setup      = 3,
    .ncs_write_setup    = 0,
    .nwe_setup      = 3,

    .ncs_read_pulse     = 11,
    .nrd_pulse      = 6,
    .ncs_write_pulse    = 11,
    .nwe_pulse      = 6,

    .read_cycle     = 22,
    .write_cycle        = 22,

    .mode           = AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_16,
    .tdf_cycles     = 2,
};

static void __init ek_add_device_dm9000(void)
{
    /* Configure chip-select 2 (DM9000) */
    sam9_smc_configure(2, &dm9000_smc_config);

    /* Configure Reset signal as output */
    at91_set_A_periph(AT91_PIN_PC11, 0);

    /* Configure Interrupt pin as input, no pull-up */
    at91_set_gpio_input(AT91_PIN_PB25, 0);

    platform_device_register(&dm9000_device);
}
#else
static void __init ek_add_device_dm9000(void) {}
#endif /* CONFIG_DM9000 */

/*
 * SPI devices.
 */
static struct spi_board_info ek_spi_devices[] = {
#if !defined(CONFIG_MMC_AT91)
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 1,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#if defined(CONFIG_MTD_AT91_DATAFLASH_CARD)
	{	/* DataFlash card */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
#endif
#endif
};

 
/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PA7,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
	{
		.name	= "rfs",
		.offset	= 0,//MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct mtd_partition * __init nand_partitions(int size, int *num_partitions)
{
	*num_partitions = ARRAY_SIZE(ek_nand_partition);
	return ek_nand_partition;
}

/* det_pin is not connected */
static struct atmel_nand_data __initdata ek_nand_data = {
	.ale		= 21,
	.cle		= 22,
	.rdy_pin	= AT91_PIN_PC13,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_ATMEL_BUSWIDTH_16)
	.bus_width_16	= 1,
#else
	.bus_width_16	= 0,
#endif
};

static struct sam9_smc_config __initdata ek_nand_smc_config = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 2,
	.ncs_write_setup	= 0,
	.nwe_setup		= 2,

	.ncs_read_pulse		= 4,
	.nrd_pulse		= 4,
	.ncs_write_pulse	= 4,
	.nwe_pulse		= 4,

	.read_cycle		= 7,
	.write_cycle		= 7,

	.mode			= AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE,
	.tdf_cycles		= 3,
};

static void __init ek_add_device_nand(void)
{
	/* setup bus-width (8 or 16) */
	if (ek_nand_data.bus_width_16)
		ek_nand_smc_config.mode |= AT91_SMC_DBW_16;
	else
		ek_nand_smc_config.mode |= AT91_SMC_DBW_8;

	/* configure chip-select 3 (NAND) */
	sam9_smc_configure(3, &ek_nand_smc_config);

	at91_add_device_nand(&ek_nand_data);
}


/*
 * MCI (SD/MMC)
 * wp_pin and vcc_pin are not connected
 */
static struct at91_mmc_data __initdata ek_mmc_data = {
	.slot_b		= 1,
	.wire4		= 1,
	.det_pin	= AT91_PIN_PC9,
};

/*
 * I2C devices
 */
static struct at24_platform_data at24c16 = {
    .byte_len   = 2 * SZ_1K / 8,
    .page_size  = 256,
    .flags      = 0,
};


static struct i2c_board_info __initdata ek_i2c_devices[] = {
    {
        I2C_BOARD_INFO("24c02", 0x50),
        .platform_data = &at24c16,
    },
    /* more devices can be added using expansion connectors */
};


/*
 * LEDs
 */
static struct gpio_led ek_leds[] = {
#if 0
	{	/* "bottom" led, green, userled1 to be defined */
		.name			= "ds5",
		.gpio			= AT91_PIN_PA6,
		.active_low		= 1,
		.default_trigger	= "none",
	},
#endif
	{	/* "power" led, yellow */
		.name			= "ds1",
		.gpio			= AT91_PIN_PC12,
		.default_trigger	= "heartbeat",
	}
};

#define DK_FLASH_BASE   AT91_CHIPSELECT_0
#define DK_FLASH_SIZE   SZ_4M

static struct physmap_flash_data dk_flash_data = {
    .width      = 2,
};

static struct resource dk_flash_resource = {
    .start      = DK_FLASH_BASE,
    .end        = DK_FLASH_BASE + DK_FLASH_SIZE - 1,
    .flags      = IORESOURCE_MEM,
};

static struct platform_device dk_flash = {
    .name       = "physmap-flash",
    .id     = 0,
    .dev        = {
                .platform_data  = &dk_flash_data,
            },
    .resource   = &dk_flash_resource,
    .num_resources  = 1,
};

static struct sam9_smc_config __initdata dk_flash_smc_config = {
    .ncs_read_setup     = 0,
    .nrd_setup      = 3,
    .ncs_write_setup    = 0,
    .nwe_setup      = 3,

    .ncs_read_pulse     = 11,
    .nrd_pulse      = 6,
    .ncs_write_pulse    = 11,
    .nwe_pulse      = 6,

    .read_cycle     = 22,
    .write_cycle        = 22,

    .mode           = AT91_SMC_READMODE | AT91_SMC_WRITEMODE | AT91_SMC_EXNWMODE_DISABLE | AT91_SMC_BAT_WRITE | AT91_SMC_DBW_16,
    .tdf_cycles     = 2,
};

static void __init ek_add_device_dk_flash(void)
{

    /* configure chip-select 0 (NOR FLASH) */
    sam9_smc_configure(0, &dk_flash_smc_config);

    platform_device_register(&dk_flash);
}


/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
	{
		.gpio		= AT91_PIN_PA30,
		.code		= BTN_3,
		.desc		= "Button 3",
		.active_low	= 1,
		.wakeup		= 1,
	},
	{
		.gpio		= AT91_PIN_PA31,
		.code		= BTN_4,
		.desc		= "Button 4",
		.active_low	= 1,
		.wakeup		= 1,
	}
};

static struct gpio_keys_platform_data ek_button_data = {
	.buttons	= ek_buttons,
	.nbuttons	= ARRAY_SIZE(ek_buttons),
};

static struct platform_device ek_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources	= 0,
	.dev		= {
		.platform_data	= &ek_button_data,
	}
};

static void __init ek_add_device_buttons(void)
{
	at91_set_gpio_input(AT91_PIN_PA30, 1);	/* btn3 */
	at91_set_deglitch(AT91_PIN_PA30, 1);
	at91_set_gpio_input(AT91_PIN_PA31, 1);	/* btn4 */
	at91_set_deglitch(AT91_PIN_PA31, 1);

	platform_device_register(&ek_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif

/*
 * I2C
 */
static struct i2c_board_info __initdata ek_i2c_devices[] = {
	{
		I2C_BOARD_INFO("24c512", 0x50),
		I2C_BOARD_INFO("wm8731", 0x1b),
	},
};


static void __init ek_board_init(void)
{
	/* Serial */
	at91_add_device_serial();
	/* USB Host */
	at91_add_device_usbh(&ek_usbh_data);
	/* USB Device */
	at91_add_device_udc(&ek_udc_data);
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* NAND */
	ek_add_device_nand();
    /* DM9000 ethernet */
    ek_add_device_dm9000();
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* MMC */
	at91_add_device_mmc(0, &ek_mmc_data);
	/* I2C */
//	at91_add_device_i2c(NULL, 0);
    at91_add_device_i2c(ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	/* LEDs */
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	/* Push Buttons */
	ek_add_device_buttons();
	/* PCK0 provides MCLK to the WM8731 */
	at91_set_B_periph(AT91_PIN_PC1, 0);
	/* SSC (for WM8731) */
	at91_add_device_ssc(AT91SAM9260_ID_SSC, ATMEL_SSC_TX);
	/* NOR Flash */
	ek_add_device_dk_flash();
	/* CF */
	at91_add_device_cf(&at91sam9g20ek_cf_data);
}

MACHINE_START(AT91SAM9G20EK, "Atmel AT91SAM9G20-EK")
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
