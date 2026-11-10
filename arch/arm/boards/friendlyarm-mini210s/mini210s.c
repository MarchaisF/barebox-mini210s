// SPDX-License-Identifier: GPL-2.0-or-later

#include <common.h>
#include <driver.h>
#include <init.h>
#include <linux/sizes.h>
#include <generated/mach-types.h>
#include <gpio.h>
#include <led.h>
#include <io.h>
#include <envfs.h>
#include <platform_data/eth-dm9000.h>
#include <asm/armlinux.h>
#include <mach/iomux.h>
#include <mach/s3c-iomap.h>
#include <mach/s3c-generic.h>

#define S5P_SROM_BW		0xE8000000
#define S5P_SROM_BC1		0xE8000008
#define S5P_MP0_1CON		(S3C_GPIO_BASE + 0x2e0)
#define S5P_MP0_3CON		(S3C_GPIO_BASE + 0x320)
#define S5P_MP0_6CON		(S3C_GPIO_BASE + 0x380)

static struct dm9000_platform_data mini210s_dm9000_data = {
	.srom = 0,
};

static struct gpio_led mini210s_leds[] = {
	{
		.gpio = GPJ20,
		.led = { .name = "led1" },
	}, {
		.gpio = GPJ21,
		.led = { .name = "led2" },
	}, {
		.gpio = GPJ22,
		.led = { .name = "led3" },
	}, {
		.gpio = GPJ23,
		.led = { .name = "led4" },
	},
};

static int mini210s_mem_init(void)
{
	arm_add_mem_device("ram0", S3C_SDRAM_BASE, SZ_512M);
	return 0;
}
mem_initcall(mini210s_mem_init);

static void mini210s_setup_dm9000(void)
{
	uint32_t reg;

	s3c_gpio_mode(PIN(23, 1) | FUNC(2));
	writel(5 << 16, S5P_SROM_BC1);

	reg = readl(S5P_SROM_BW);
	reg &= ~(0xf << 4);
	reg |= 1 << 4;
	writel(reg, S5P_SROM_BW);
}

#ifdef CONFIG_NAND_S5PV210
static void mini210s_setup_nand_pins(void)
{
	u32 value;

	value = readl(S5P_MP0_1CON);
	value = (value & ~(0xf << 8)) | (0x3 << 8);
	writel(value, S5P_MP0_1CON);

	value = readl(S5P_MP0_3CON);
	value = (value & ~0xfffff) | 0x22222;
	writel(value, S5P_MP0_3CON);

	writel(0x22222222, S5P_MP0_6CON);
}
#endif

static int mini210s_console_init(void)
{
	s3c_gpio_mode(GPA00_RXD0 | ENABLE_PU);
	s3c_gpio_mode(GPA01_TXD0);
	s3c_gpio_mode(GPA02_NCTS0 | ENABLE_PU);
	s3c_gpio_mode(GPA03_NRTS0);

	barebox_set_model("FriendlyARM Mini210s");
	barebox_set_hostname("mini210s");

	add_generic_device("s3c_serial", DEVICE_ID_DYNAMIC, NULL,
			   S3C_UART1_BASE, S3C_UART1_SIZE,
			   IORESOURCE_MEM, NULL);
	return 0;
}
console_initcall(mini210s_console_init);

static int mini210s_devices_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mini210s_leds); i++) {
		mini210s_leds[i].active_low = 1;
		gpio_direction_output(mini210s_leds[i].gpio,
				      mini210s_leds[i].active_low);
		led_gpio_register(&mini210s_leds[i]);
	}
	led_set_trigger(LED_TRIGGER_HEARTBEAT, &mini210s_leds[0].led);

	if (IS_ENABLED(CONFIG_DEFAULT_ENVIRONMENT_GENERIC))
		defaultenv_append_directory(defaultenv_friendlyarm_mini210s);

#ifdef CONFIG_NAND_S5PV210
	mini210s_setup_nand_pins();
	add_generic_device("s5pv210-nand", DEVICE_ID_DYNAMIC, NULL,
			   S3C_NAND_BASE, SZ_1M, IORESOURCE_MEM, NULL);
#endif

#ifdef CONFIG_DRIVER_NET_DM9K
	mini210s_setup_dm9000();
	add_dm9000_device(0, S3C_CS1_BASE + 0x1000,
			  S3C_CS1_BASE + 0x1000 + 0x300c,
			  IORESOURCE_MEM_16BIT, &mini210s_dm9000_data);
#endif
	armlinux_set_architecture(MACH_TYPE_MINI210);
	return 0;
}
device_initcall(mini210s_devices_init);