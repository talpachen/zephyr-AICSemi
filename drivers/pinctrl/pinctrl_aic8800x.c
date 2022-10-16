/*
 * Copyright (c) 2022, Talpa Chen
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pinctrl.h>

#ifndef PMIC_MEM_CHECK
#define PMIC_MEM_CHECK(addr)	((addr) & 0xff000000 == 0x50000000 ? true : false)
#endif

#ifndef PMIC_MEM_READ
static uint32_t PMIC_MEM_READ(uint32_t reg)
{
	return 0;
}
#endif

#ifndef PMIC_MEM_WRITE
static void PMIC_MEM_WRITE(uint32_t reg, uint32_t v)
{
}
#endif

#ifndef PMIC_MEM_MASK_WRITE
static void PMIC_MEM_MASK_WRITE(uint32_t reg, uint32_t v, uint32_t mask)
{
}
#endif

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin)
{
	ARG_UNUSED(reg);

	uint32_t gpcfg;
	uint32_t gpcfg_addr = pin->iomux_base + 4 * pin->pin;

	if (PMIC_MEM_CHECK(gpcfg_addr)) {
		PMIC_MEM_MASK_WRITE(gpcfg_addr, pin->gpcfg, IOMUX_GPIO_CONFIG_SEL_MASK |
													IOMUX_GPIO_CONFIG_PULL_DN_MASK |
													IOMUX_GPIO_CONFIG_PULL_UP_MASK |
													IOMUX_GPIO_CONFIG_PULL_FRC_MASK);
	} else {
		gpcfg = *(uint32_t *)gpcfg_addr & ~(IOMUX_GPIO_CONFIG_SEL_MASK |
											IOMUX_GPIO_CONFIG_PULL_DN_MASK |
											IOMUX_GPIO_CONFIG_PULL_UP_MASK |
											IOMUX_GPIO_CONFIG_PULL_FRC_MASK);
		*(uint32_t *)gpcfg_addr = gpcfg | pin->gpcfg;
	}
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	ARG_UNUSED(reg);
	
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(&pins[i]);
	}

	return 0;
}
