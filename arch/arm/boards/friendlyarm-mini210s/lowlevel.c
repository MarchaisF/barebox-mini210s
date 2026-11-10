// SPDX-License-Identifier: GPL-2.0-only

#include <config.h>
#include <common.h>
#include <init.h>
#include <io.h>
#include <linux/sizes.h>
#include <asm/barebox-arm.h>
#include <asm/barebox-arm-head.h>
#include <mach/s3c-clocks.h>
#include <mach/s3c-iomap.h>
#include <mach/s3c-generic.h>

#define IRAM_CODE_BASE		0xD0020010
#define ARM_HEAD_TEXT_BASE	(IRAM_CODE_BASE + 0x28)
#define ARM_HEAD_IMAGE_SIZE	(IRAM_CODE_BASE + 0x2c)
#define GPJ2CON			0xE0200280
#define GPJ2DAT			0xE0200284
#define GPD0CON			0xE02000A0
#define GPD0DAT			0xE02000A4

#define ADDR_V210_SDMMC_BASE	0xD0037488
#define ADDR_CopySDMMCtoMem	0xD0037F98

static inline void __bare_init mini210s_debug_led(unsigned int led, bool on)
{
	uint32_t value;

	value = readl(GPJ2CON);
	value &= ~(0xf << (4 * led));
	value |= 1 << (4 * led);
	writel(value, GPJ2CON);

	value = readl(GPJ2DAT);
	value &= ~(1 << led);
	value |= (on ? 0 : 1) << led;
	writel(value, GPJ2DAT);
}

static inline void __bare_init mini210s_leds_off(void)
{
	uint32_t value;

	value = readl(GPJ2DAT) | 0x0f;
	writel(value, GPJ2DAT);

	value = readl(GPJ2CON);
	value &= ~0xffff;
	value |= 0x1111;
	writel(value, GPJ2CON);
}

static inline void __bare_init mini210s_buzzer_off(void)
{
	uint32_t value;

	value = readl(GPD0DAT) & ~1;
	writel(value, GPD0DAT);

	value = readl(GPD0CON) & ~0xf;
	value |= 1;
	writel(value, GPD0CON);
}

static int __bare_init s5p_irom_load_mmc(void *dest, uint32_t start_block,
					 uint16_t block_count)
{
	typedef uint32_t (*func_t)(int32_t, uint32_t, uint16_t, uint32_t *, int8_t);
	uint32_t chbase = readl(ADDR_V210_SDMMC_BASE);
	func_t func = (func_t)readl(ADDR_CopySDMMCtoMem);
	int chan = (chbase - 0xEB000000) >> 20;

	if (chan != 0 && chan != 2)
		return 0;

	mini210s_debug_led(0, true);
	chan = func(chan, start_block, block_count, (uint32_t *)dest, 0);
	mini210s_debug_led(0, false);

	return chan ? 1 : 0;
}

static __bare_init __naked void jump_sdram(unsigned long offset)
{
	__asm__ __volatile__("sub lr, lr, %0;"
			     "mov pc, lr;" : : "r"(offset));
}

static __bare_init bool load_stage2(void *dest, size_t size)
{
	return s5p_irom_load_mmc(dest, 1, (size + 511) / 512);
}

void __bare_init barebox_arm_reset_vector(uint32_t r0, uint32_t r1, uint32_t r2)
{
	uint32_t image_base, image_size;

	mini210s_buzzer_off();
	mini210s_leds_off();
	arm_cpu_lowlevel_init();
	mini210s_debug_led(0, false);
	mini210s_debug_led(3, false);

#ifdef CONFIG_S3C_PLL_INIT
	s5p_init_pll();
#endif

	if (get_pc() < IRAM_CODE_BASE)
		goto boot;

	s5p_init_dram_bank_ddr2(S5P_DMC0_BASE, 0x20E00323, 0, 0);
	mini210s_debug_led(2, true);

	image_base = readl(ARM_HEAD_TEXT_BASE);
	image_size = readl(ARM_HEAD_IMAGE_SIZE);

	if (!load_stage2((void *)(image_base - 16), image_size + 16)) {
		mini210s_debug_led(3, true);
		while (1)
			;
	}
	mini210s_debug_led(3, true);
	mini210s_debug_led(0, true);

	jump_sdram(IRAM_CODE_BASE - image_base);

boot:
	barebox_arm_entry(S3C_SDRAM_BASE, SZ_512M, NULL);
}