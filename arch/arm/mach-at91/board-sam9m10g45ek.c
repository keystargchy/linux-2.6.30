/*
 *  Board-specific setup code for the AT91SAM9M10G45 Evaluation Kit family
 *
 *  Covers: * AT91SAM9G45-EKES  board
 *          * AT91SAM9M10G45-EK board
 *
 *  Copyright (C) 2009 Atmel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/fb.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/atmel-mci.h>
#include <linux/i2c/at24.h>


#include <mach/hardware.h>
#include <video/atmel_lcdc.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/at91sam9_smc.h>
#include <mach/at91_shdwc.h>
#include <linux/w1-gpio.h>

#if defined(CONFIG_CAN_MCP2515) || defined(CONFIG_CAN_MCP2515_MODULE)
#include <linux/can/can.h>
#include <linux/can/mcp251x.h>
#endif

#if defined(CONFIG_GPIO_KEYPAD) || defined(CONFIG_GPIO_KEYPAD_MODULE)
#include <mach/keypad.h>
#endif

#include "sam9_smc.h"
#include "generic.h"


static void __init ek_map_io(void)
{
	/* Initialize processor: 12.000 MHz crystal */
	at91sam9g45_initialize(12000000);

	/* DGBU on ttyS0. (Rx & Tx only) */
	at91_register_uart(0, 6, 0);

	/* USART0 on ttyS1. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US0, 0, 0);
	/* USART1 on ttyS2. (Rx, Tx, RTS, CTS) */
	at91_register_uart(AT91SAM9G45_ID_US1, 1, ATMEL_UART_CTS | ATMEL_UART_RTS);
	/* USART2 on ttyS3. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US2, 2, ATMEL_UART_RTS);

	/* USART3 on ttyS4. (Rx, Tx) */
	at91_register_uart(AT91SAM9G45_ID_US3, 3, 0);

	/* set serial console to ttyS0 (ie, DBGU) */
	at91_set_serial_console(6);
}

static void __init ek_init_irq(void)
{
	at91sam9g45_init_interrupts(NULL);
}


/*
 * USB HS Host port (common to OHCI & EHCI)
 */
static struct at91_usbh_data __initdata ek_usbh_hs_data = {
	.ports		= 2,
	.vbus_pin	= {AT91_PIN_PD1, AT91_PIN_PD3},
};


/*
 * USB HS Device port
 */
static struct usba_platform_data __initdata ek_usba_udc_data = {
	.vbus_pin	= AT91_PIN_PC0,
};


/*
 * SPI devices.
 */
#if defined(CONFIG_CAN_MCP2515) || defined(CONFIG_CAN_MCP2515_MODULE)
static struct mcp251x_platform_data mcp251x_data = {
	.f_osc = 16000000UL,
};
#endif
static struct spi_board_info ek_spi_devices[] = {
	{	/* DataFlash chip */
		.modalias	= "mtd_dataflash",
		.chip_select	= 0,
		.max_speed_hz	= 15 * 1000 * 1000,
		.bus_num	= 0,
	},
    {
        .modalias       = "serial_is7x2",
        .chip_select    = 2,
        .max_speed_hz   = 10 * 1000 * 1000,
        .bus_num        = 1,
    },
#if defined(CONFIG_CAN_MCP2515) || defined(CONFIG_CAN_MCP2515_MODULE)
	{
        .modalias = "mcp2515",
        .chip_select = 1,
        .irq = AT91_PIN_PD25,
        .platform_data = &mcp251x_data,
        .max_speed_hz = 5 * 1000 * 1000,
        .bus_num = 0,
        .mode = 0,
    },
#endif
#if defined(CONFIG_SPI_SPIDEV) || defined(CONFIG_SPI_SPIDEV_MODULE)
    {
        .modalias = "spidev",
        .max_speed_hz = 5000000,
        .bus_num = 1,
        .chip_select = 3,
    }
#endif
};


/*
 * MCI (SD/MMC)
 */
static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.bus_width	= 4,
		.detect_pin	= AT91_PIN_PD10,
		.wp_pin		= -1,
	},
};

#if 0 /* only use mci0 channel */
static struct mci_platform_data __initdata mci1_data = {
	.slot[0] = {
		.bus_width	= 8,
		.detect_pin	= AT91_PIN_PB29,
		.wp_pin		= AT91_PIN_PB30,
	},
};
#endif

/*
 * MACB Ethernet device
 */
static struct at91_eth_data __initdata ek_macb_data = {
	.phy_irq_pin	= AT91_PIN_PD5,
	.is_rmii	= 1,
};


/*
 * NAND flash
 */
static struct mtd_partition __initdata ek_nand_partition[] = {
#ifdef BOOT_FROM_NAND
	{
		.name	= "Bootstrap",
		.offset	= 0,
		.size	= 0x20000,
	},
	{
		.name	= "U-Boot",
		.offset	= 0x20000,
		.size	= 0x60000,
	},
	{
		.name	= "Para1",
		.offset	= 0x60000,
		.size	= 0x20000,
	},
	{
		.name	= "Para2",
		.offset	= 0x80000,
		.size	= 0x20000,
	},
	{
		.name	= "Logo",
		.offset	= 0xa0000,
		.size	= 0x100000,
	},
	{
		.name	= "Kernel",
		.offset	= 0x1a0000,
		.size	= 0x200000,
	},
	{
		.name	= "Ramdisk",
		.offset	= 0x3a0000,
		.size	= 0x600000,
	},
	{
		.name	= "Rootfs",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
#else   /* BOOT FROM DATAFLASH */
	{
		.name	= "Kernel",
		.offset	= 0x000000,
		.size	= 0x200000,
	},
	{
		.name	= "Ramdisk",
		.offset	= 0x200000,
		.size	= 0x600000,
	},
    {
        .name   = "Rootfs",
        .offset = 0x800000,
        .size   = 0xE800000,
    },
	{
		.name	= "Share",
		.offset	= MTDPART_OFS_NXTBLK,
		.size	= MTDPART_SIZ_FULL,
	},
#endif
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
	.rdy_pin	= AT91_PIN_PC8,
	.enable_pin	= AT91_PIN_PC14,
	.partition_info	= nand_partitions,
#if defined(CONFIG_MTD_NAND_AT91_BUSWIDTH_16)
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
 * LCD Controller
 */
#if defined(CONFIG_FB_ATMEL) || defined(CONFIG_FB_ATMEL_MODULE)
static struct fb_videomode at91_tft_vga_modes;

static struct fb_videomode __initdata at91_tft_vga_modes_arrary[] = {
	{
		.name           = "TX09D50VM1CCA @ 43",
		.refresh	= 60,
		.xres		= 480,		.yres		= 272,
		.pixclock	= KHZ2PICOS(10000),

		.left_margin	= 1,		.right_margin	= 1,
		.upper_margin	= 40,		.lower_margin	= 1,
		.hsync_len	= 45,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
    {
        .name           = "TX09D50VM1CCA @ 56",
        .refresh    = 60,
        .xres       = 640,      .yres       = 480,
        .pixclock   = KHZ2PICOS(28214),

        .left_margin    = 145,      .right_margin   = 20,
        .upper_margin   = 14,       .lower_margin   = 8,
        .hsync_len  = 5,        .vsync_len  = 1,

        .sync       = 0,
        .vmode      = FB_VMODE_NONINTERLACED,
    },
	{
#if 0
		.name           = "TX09D50VM1CCA @ 70",
		.refresh	= 60,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(34000),

		.left_margin	= 17,		.right_margin	= 11,
		.upper_margin	= 4,		.lower_margin	= 7,
		.hsync_len	= 5,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
#else
		.name           = "TX09D50VM1CCA @ 70",
		.refresh	= 60,
		.xres		= 800,		.yres		= 480,
		.pixclock	= KHZ2PICOS(45000),

		.left_margin	= 80,		.right_margin	= 15,
		.upper_margin	= 80,		.lower_margin	= 7,
		.hsync_len	= 800,		.vsync_len	= 480,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
#endif
	},
	{
		.name           = "TX09D50VM1CCA @ 101",
		.refresh	= 60,
		.xres		= 800,		.yres		= 600,
		.pixclock	= KHZ2PICOS(40000),

		.left_margin	= 17,		.right_margin	= 11,
		.upper_margin	= 4,		.lower_margin	= 7,
		.hsync_len	= 5,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs at91fb_default_monspecs = {
	.manufacturer	= "HIT",
	.monitor        = "TX09D70VM1CCA",

	.modedb		=  &at91_tft_vga_modes,
	.modedb_len	=  1,
	.hfmin		= 15000,
	.hfmax		= 17640,
	.vfmin		= 57,
	.vfmax		= 67,
};

#define AT91SAM9G45_DEFAULT_LCDCON2 	(ATMEL_LCDC_MEMOR_LITTLE \
					| ATMEL_LCDC_DISTYPE_TFT \
					| ATMEL_LCDC_IFWIDTH_16 \
					| ATMEL_LCDC_CLKMOD_ALWAYSACTIVE)

/* Driver datas */
static struct atmel_lcdfb_info __initdata ek_lcdc_data = {
	.lcdcon_is_backlight	= true,
	.default_bpp			= 16,
	.default_dmacon			= ATMEL_LCDC_DMAEN,
	.default_lcdcon2		= AT91SAM9G45_DEFAULT_LCDCON2,
	.default_monspecs		= &at91fb_default_monspecs,
	.guard_time				= 9,
	.lcd_wiring_mode		= ATMEL_LCDC_WIRING_RGB,
};

void __init ek_fb_set_platdata(int default_display)
{
	memcpy(&at91_tft_vga_modes, &at91_tft_vga_modes_arrary[default_display], sizeof(at91_tft_vga_modes));
}

#else
static struct atmel_lcdfb_info __initdata ek_lcdc_data;
void __init ek_fb_set_platdata(int default_display){}
#endif

/*
 * I2C devices
 */
#if 0       /* No eeprom */
static struct at24_platform_data at24c16 = {
    .byte_len   = 2 * SZ_1K / 8,
    .page_size  = 256,
    .flags      = 0,
};
#endif

#if 0
struct sc16is752_platform_data {
    int fifosize;
    int clk;
    int irq;
} sc16is752_data = {
	.fifosize = 64,
	.clk = 18432000,
	.irq = AT91_PIN_PC1,
};
#endif 


static struct i2c_board_info __initdata ek_i2c_devices[] = {
#if 0
    {
        I2C_BOARD_INFO("24c02", 0x50),
        .platform_data = &at24c16,
    },
    {
        I2C_BOARD_INFO("ds3231", 0xd0 >> 1),
    },
#endif
    {
        I2C_BOARD_INFO("rx8025sa", 0x32),
    },
};


/*
 * Touchscreen
 */
static struct at91_tsadcc_data ek_tsadcc_data = {
	.adc_clock		= 1024,
	.pendet_debounce	= 0x0f,
	.ts_sample_hold_time	= 0x0f,
};


/*
 * GPIO Buttons
 */
#if defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
static struct gpio_keys_button ek_buttons[] = {
    {
        .code       = KEY_BACK,
        .gpio       = AT91_PIN_PB25,
        .active_low = 1,
        .desc       = "left_click",
        .wakeup     = 1,
    },
    {
        .code       = BTN_TASK,
        .gpio       = AT91_PIN_PB26,
        .active_low = 1,
        .desc       = "btn_task",
        .wakeup     = 1,
    },
    {
        .code       = KEY_ESC,
        .gpio       = AT91_PIN_PA24,
        .active_low = 1,
        .desc       = "hall_1",
        .wakeup     = 1,
    },
    {
        .code       = KEY_1,
        .gpio       = AT91_PIN_PB30,
        .active_low = 1,
        .desc       = "hall_2",
        .wakeup     = 1,
    },
	{
        .code       = KEY_2,
        .gpio       = AT91_PIN_PA26,
        .active_low = 1,
        .desc       = "hall_3",
        .wakeup     = 1,
    },
    {
        .code       = KEY_3,
        .gpio       = AT91_PIN_PB23,
        .active_low = 1,
        .desc       = "hall_4",
        .wakeup     = 1,
    },

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
	int i;

	for (i = 0; i < ARRAY_SIZE(ek_buttons); i++) {
		at91_set_GPIO_periph(ek_buttons[i].gpio, 1);
		at91_set_deglitch(ek_buttons[i].gpio, 1);
	}

	platform_device_register(&ek_button_device);
}
#else
static void __init ek_add_device_buttons(void) {}
#endif


/*
 * AC97
 * reset_pin is not connected: NRST
 */
static struct ac97c_platform_data ek_ac97_data = {
};


/*
 * LEDs ... these could all be PWM-driven, for variable brightness
 */
static struct gpio_led ek_leds[] = {
	{	
		.name			= "d6",
		.gpio			= AT91_PIN_PD0,
        .active_low     = 1,
		.default_trigger	= "none",
	},
	{
		.name			= "d9",
		.gpio			= AT91_PIN_PA25,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{
		.name			= "d12",
		.gpio			= AT91_PIN_PB21,
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "d13",
		.gpio			= AT91_PIN_PB22,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{
		.name			= "beep",
		.gpio			= AT91_PIN_PB20,
		.active_low		= 1,
		.default_trigger	= "none",
	},
	{	/* LVDS onoff */
		.name           = "lvds_onoff",
		.gpio           = AT91_PIN_PB28,
		.active_low     = 0,
		.default_trigger    = "none",
	},
};


/*
 * PWM Leds
 */
static struct gpio_led ek_pwm_led[] = {
#if defined(CONFIG_LEDS_ATMEL_PWM) || defined(CONFIG_LEDS_ATMEL_PWM_MODULE)
	{	/* "right" led, green, userled1, pwm1 */
		.name			= "backlight_pwm",
		.gpio			= AT91_PWM1,	/* is PWM channel number */
		.active_low		= 1,
		.default_trigger	= "none",
        .freq           = 500,          /* only for pwm led */
	},
    {
        .name           = "pd26",
        .gpio           = AT91_PWM2,
        .active_low     = 0,
        .default_trigger    = "none",
    },
#endif
};

/*
 * GPIO
 */
#if defined(CONFIG_GPIO_SYSFS)
static void at91_add_device_gpio(void)
{
    at91_set_gpio_input(AT91_PIN_PA22, 1);
    at91_set_gpio_input(AT91_PIN_PA24, 1);
    at91_set_gpio_input(AT91_PIN_PA26, 1);
    at91_set_gpio_input(AT91_PIN_PA27, 1);
    at91_set_gpio_input(AT91_PIN_PA28, 1);
    at91_set_gpio_input(AT91_PIN_PA29, 1);
    at91_set_gpio_input(AT91_PIN_PA30, 1);
    at91_set_gpio_input(AT91_PIN_PA31, 1);

    at91_set_gpio_input(AT91_PIN_PB23, 1);
    at91_set_gpio_input(AT91_PIN_PB24, 1);
    at91_set_gpio_input(AT91_PIN_PB29, 1);
    at91_set_gpio_input(AT91_PIN_PB30, 1);

    at91_set_gpio_input(AT91_PIN_PD12, 1);
    at91_set_gpio_input(AT91_PIN_PD13, 1);
    at91_set_gpio_input(AT91_PIN_PD14, 1);
    at91_set_gpio_input(AT91_PIN_PD28, 1);
}
#else
static void at91_add_device_gpio(void) {}
#endif

/*
 * GPIO KEYPAD 6*6
 */
#if defined(CONFIG_GPIO_KEYPAD) || defined(CONFIG_GPIO_KEYPAD_MODULE)
static int row_gpios[] = {
    AT91_PIN_PC16, AT91_PIN_PC17, AT91_PIN_PC18,
    AT91_PIN_PC19, AT91_PIN_PC20, AT91_PIN_PC21,
};
static int col_gpios[] = {
    AT91_PIN_PC22, AT91_PIN_PC23, AT91_PIN_PC24,
    AT91_PIN_PC25, AT91_PIN_PC26, AT91_PIN_PC27,
};

static int palmte_keymap[] = {
    KEY(0, 0, KEY_1),   /* camera Qt::Key_F17 */
    KEY(0, 1, KEY_2),   /* voice memo Qt::Key_F14 */
    KEY(0, 2, KEY_3),   /* voice memo */
    KEY(0, 3, KEY_4),   /* voice memo */
    KEY(0, 4, KEY_5),   /* red button Qt::Key_Hangup */
    KEY(0, 5, KEY_6),

    KEY(1, 0, KEY_7),
    KEY(1, 1, KEY_8),
    KEY(1, 2, KEY_9),
    KEY(1, 3, KEY_0),
    KEY(1, 4, KEY_A),   /* joystick press or Qt::Key_Select */
    KEY(1, 5, KEY_B),

    KEY(2, 0, KEY_C),
    KEY(2, 1, KEY_D),
    KEY(2, 2, KEY_E),
    KEY(2, 3, KEY_F),
    KEY(2, 4, KEY_G),   /* "*" */
    KEY(2, 5, KEY_H),

    KEY(3, 0, KEY_I),
    KEY(3, 1, KEY_J),
    KEY(3, 2, KEY_K),
    KEY(3, 3, KEY_L),
    KEY(3, 4, KEY_M),   /* # F13 Toggle input method Qt::Key_F13 */
    KEY(3, 5, KEY_N),   /* green button Qt::Key_Call */

    KEY(4, 0, KEY_O),   /* left soft Qt::Key_Context1 */
    KEY(4, 1, KEY_P),   /* right soft Qt::Key_Back */
    KEY(4, 2, KEY_Q),   /* shift */
    KEY(4, 3, KEY_R),   /* C (clear) */
    KEY(4, 4, KEY_S),   /* menu Qt::Key_Menu */
    KEY(4, 5, KEY_T),   /* menu Qt::Key_Menu */

    KEY(5, 0, KEY_U),   /* left soft Qt::Key_Context1 */
    KEY(5, 1, KEY_V),   /* right soft Qt::Key_Back */
    KEY(5, 2, KEY_W),   /* shift */
    KEY(5, 3, KEY_X),   /* C (clear) */
    KEY(5, 4, KEY_Y),   /* menu Qt::Key_Menu */
    KEY(5, 5, KEY_Z),   /* menu Qt::Key_Menu */
    0
};

static struct atmel_kp_platform_data palmte_kp_data = {
    .rows   = 6,
    .cols   = 6,
    .keymap = palmte_keymap,
    .keymapsize = ARRAY_SIZE(palmte_keymap),
    .rep    = 1,
    .delay  = 12,
    .col_gpios = col_gpios,
    .row_gpios = row_gpios,
};

static struct platform_device palmte_kp_device = {
    .name       = "atmel-keypad",
    .id     = -1,
    .dev        = {
        .platform_data  = &palmte_kp_data,
    },
};

void __init at91_add_device_keypad(void)
{
    platform_device_register(&palmte_kp_device);
}
#else
void __init at91_add_device_keypad(void) {}
#endif /* CONFIG_GPIO_KEYPAD */



#if defined(CONFIG_W1_SLAVE_THERM)
static struct w1_gpio_platform_data ds18b20_data[] = {
	{
		.pin = AT91_PIN_PA28,
		.is_open_drain = 1,
	},
};

static struct platform_device palmte_ds18b20_device = {
    .name       = "w1-gpio",
    .id     = -1,
    .dev        = {
        .platform_data  = &ds18b20_data,
    },
};

void __init at91_add_device_ds18b20(struct w1_gpio_platform_data *ds, int nr)
{
	int i;

	if (!nr)
		return;

	for (i = 0; i < nr; i++)
		at91_set_gpio_output(ds[i].pin, ds[i].is_open_drain);

	platform_device_register(&palmte_ds18b20_device);
}
#else
void __init at91_add_device_ds18b20(struct w1_gpio_platform_data *ds, int nr) {}
#endif /* CONFIG_W1_SLAVE_THERM */



#if 0
static char tft_type = 's';

static int __init ek_tft_setup(char *str)
{
    tft_type = str[0];
    return 1;
}

__setup("tft=", ek_tft_setup);
#endif

static void __init ek_board_init(void)
{
	int default_display = 0;
#if 0
	switch (tft_type) {
    case 's': /* small or production */
        default_display = 0;
        break;
    case 'm': /* middle */
        default_display = 1;
        break;
    case 'b': /* big */
    default:
        default_display = 2;
        break;
    }
#else
    default_display = setup_panel_config();
#endif
    ek_fb_set_platdata(default_display);

	/* Serial */
	at91_add_device_serial();
	/* USB HS Host */
	at91_add_device_usbh_ohci(&ek_usbh_hs_data);
	at91_add_device_usbh_ehci(&ek_usbh_hs_data);
	/* USB HS Device */
	at91_add_device_usba(&ek_usba_udc_data);
	/* SPI */
	at91_add_device_spi(ek_spi_devices, ARRAY_SIZE(ek_spi_devices));
	/* MMC0 */
	at91_add_device_mci(0, &mci0_data);
	/* MMC1 */
	/* at91_add_device_mci(1, &mci1_data);*/
	/* Ethernet */
	at91_add_device_eth(&ek_macb_data);
	/* NAND */
	ek_add_device_nand();
	/* I2C */
	at91_add_device_i2c(0, ek_i2c_devices, ARRAY_SIZE(ek_i2c_devices));
	/* LCD Controller */
	at91_add_device_lcdc(&ek_lcdc_data);
	/* Touch Screen */
	at91_add_device_tsadcc(&ek_tsadcc_data);
	/* Push Buttons */
	ek_add_device_buttons();
	/* AC97 */
	at91_add_device_ac97(&ek_ac97_data);
	/* LEDs */
	at91_gpio_leds(ek_leds, ARRAY_SIZE(ek_leds));
	at91_pwm_leds(ek_pwm_led, ARRAY_SIZE(ek_pwm_led));

    /* GPIO */
    at91_add_device_gpio();
    /* Extern 6x6 GPIO KEYPAD */
    at91_add_device_keypad();
	/* ds18b20 */
	at91_add_device_ds18b20(ds18b20_data, ARRAY_SIZE(ds18b20_data));
}


#if defined(CONFIG_MACH_AT91SAM9G45EKES)
MACHINE_START(AT91SAM9G45EKES, "Atmel AT91SAM9G45-EKES")
#else
MACHINE_START(AT91SAM9M10EKES, "Atmel AT91SAM9M10-EKES")
#endif
	/* Maintainer: Atmel */
	.phys_io	= AT91_BASE_SYS,
	.io_pg_offst	= (AT91_VA_BASE_SYS >> 18) & 0xfffc,
	.boot_params	= AT91_SDRAM_BASE + 0x100,
	.timer		= &at91sam926x_timer,
	.map_io		= ek_map_io,
	.init_irq	= ek_init_irq,
	.init_machine	= ek_board_init,
MACHINE_END
