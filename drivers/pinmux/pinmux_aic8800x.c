/*
 * Copyright (c) 2022 Talpa Chen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_aic8800x_pinmux

#include <zephyr/device.h>
#include <zephyr/drivers/pinmux.h>

#define IOMUX_GPIO_CONFIG_SEL_LSB           (0)
#define IOMUX_GPIO_CONFIG_SEL_WIDTH         (4)
#define IOMUX_GPIO_CONFIG_SEL_MASK          (((1 << IOMUX_GPIO_CONFIG_SEL_WIDTH) - 1) << IOMUX_GPIO_CONFIG_SEL_LSB)
#define IOMUX_GPIO_CONFIG_PULL_DN_LSB       (8)
#define IOMUX_GPIO_CONFIG_PULL_DN_WIDTH     (1)
#define IOMUX_GPIO_CONFIG_PULL_DN_MASK      (((1 << IOMUX_GPIO_CONFIG_PULL_DN_WIDTH) - 1) << IOMUX_GPIO_CONFIG_PULL_DN_LSB)
#define IOMUX_GPIO_CONFIG_PULL_UP_LSB       (9)
#define IOMUX_GPIO_CONFIG_PULL_UP_WIDTH     (1)
#define IOMUX_GPIO_CONFIG_PULL_UP_MASK      (((1 << IOMUX_GPIO_CONFIG_PULL_UP_WIDTH) - 1) << IOMUX_GPIO_CONFIG_PULL_UP_LSB)

struct aic8800x_iomux_regs {
    volatile uint32_t GPCFG[16];
    volatile uint32_t AGPCFG[16];
};

struct pinmux_aic8800x_config {
	struct aic8800x_iomux_regs *const iomux_regs;
	bool pmic_area;
};

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

static int pinmux_aic8800x_set(const struct device *dev, uint32_t pin, uint32_t func)
{
	const struct pinmux_aic8800x_config *config = dev->config;
	struct aic8800x_iomux_regs *const iomux_regs = config->iomux_regs;

	if (config->pmic_area) {
		iomux_regs->GPCFG[pin] = (iomux_regs->GPCFG[pin] & ~IOMUX_GPIO_CONFIG_SEL_MASK) |
									((func << IOMUX_GPIO_CONFIG_SEL_LSB) & IOMUX_GPIO_CONFIG_SEL_MASK);
	} else {
		uint_fast32_t temp = PMIC_MEM_READ((uint32_t)(&iomux_regs->GPCFG[pin])) & ~IOMUX_GPIO_CONFIG_SEL_MASK;
		PMIC_MEM_WRITE((uint32_t)(&iomux_regs->GPCFG[pin]), temp | ((func << IOMUX_GPIO_CONFIG_SEL_LSB) & IOMUX_GPIO_CONFIG_SEL_MASK));
	}

	return 0;
}

static int pinmux_aic8800x_get(const struct device *dev, uint32_t pin, uint32_t *func)
{
	const struct pinmux_aic8800x_config *config = dev->config;
	struct aic8800x_iomux_regs *const iomux_regs = config->iomux_regs;

	if (config->pmic_area) {
		*func = (iomux_regs->GPCFG[pin] & IOMUX_GPIO_CONFIG_SEL_MASK) >> IOMUX_GPIO_CONFIG_SEL_LSB;
	} else {
		uint_fast32_t temp = PMIC_MEM_READ((uint32_t)(&iomux_regs->GPCFG[pin]));
		*func = (temp & IOMUX_GPIO_CONFIG_SEL_MASK) >> IOMUX_GPIO_CONFIG_SEL_LSB;
	}

	return 0;
}

static int pinmux_aic8800x_pullup(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_aic8800x_input(const struct device *dev, uint32_t pin, uint8_t func)
{
	return -ENOTSUP;
}

static int pinmux_aic8800x_init(const struct device *dev)
{
	return 0;
}

static const struct pinmux_driver_api pinmux_aic8800x_api = {
	.set = pinmux_aic8800x_set,
	.get = pinmux_aic8800x_get,
	.pullup = pinmux_aic8800x_pullup,
	.input = pinmux_aic8800x_input,
};

#define PINMUX_AIC8800X_INIT(idx)															\
	static const struct pinmux_aic8800x_config pinmux_aic8800x_config##idx = {				\
		.iomux_regs = DT_INST_REG_ADDR(idx),												\
		.pmic_area = (DT_INST_REG_ADDR(idx) & 0xff000000) == 0x50000000 ? true : false,		\
	};																						\
																							\
	DEVICE_DT_INST_DEFINE(idx,																\
				pinmux_aic8800x_init,														\
				NULL,																		\
				NULL,																		\
				&pinmux_aic8800x_config##idx,												\
				PRE_KERNEL_1,																\
				CONFIG_PINMUX_INIT_PRIORITY,												\
				&pinmux_aic8800x_api);

DT_INST_FOREACH_STATUS_OKAY(PINMUX_AIC8800X_INIT)
