/*
 * Copyright (c) 2022 Talpa Chen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
//#include <errno.h>
//#include <drivers/gpio.h>

#include "gpio_utils.h"

#define DT_DRV_COMPAT aicsemi_aic8800x_gpio


#define GET_GPIO_REGS(port)		(((struct gpio_aic8800x_config *)port->config)->gpio_regs)
#define GET_PMIC_ACCESS(port)	(((struct gpio_aic8800x_config *)port->config)->pmic_area)

struct aic8800x_gpio_regs {
    volatile uint32_t VR;           /* 0x000 (R/W) : Val Reg */
    volatile uint32_t MR;           /* 0x004 (R/W) : Msk Reg */
    volatile uint32_t DR;           /* 0x008 (R/W) : Dir Reg */
    volatile uint32_t TELR;         /* 0x00C (R/W) : Trig Edg or Lvl Reg */
    volatile uint32_t TER;          /* 0x010 (R/W) : Trig Edg Reg */
    volatile uint32_t TLR;          /* 0x014 (R/W) : Trig Lvl Reg */
    volatile uint32_t ICR;          /* 0x018 (R/W) : Int Ctrl Reg */
    volatile uint32_t RESERVED0;    /* 0x01C (R)   : Reserved */
    volatile uint32_t ISR;          /* 0x020 (R)   : Int Stat Reg */
    volatile uint32_t IRR;          /* 0x024 (W)   : Int Rm Reg */
    volatile uint32_t TIR;          /* 0x028 (R/W) : Trig In Reg */
    volatile uint32_t FR;           /* 0x02C (R/W) : Fltr Reg */
};

struct gpio_aic8800x_config {
	struct gpio_driver_config common;
	struct aic8800x_gpio_regs *const gpio_regs;
	bool pmic_area;
};

struct gpio_aic8800x_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
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

static int gpio_aic8800x_configure(const struct device *port, gpio_pin_t pin,
				    gpio_flags_t flags)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);
	uint32_t pin_bit = BIT(pin);

	if (!pmic_area) {
		gpio_regs->MR |= pin_bit;

		if ((flags & GPIO_PULL_UP) != 0U) {
			// TODO
		} else if ((flags & GPIO_PULL_DOWN) != 0U) {
			// TODO
		} else {
			// TODO
		}

		if ((flags & GPIO_OUTPUT) != 0U) {
			gpio_regs->DR |= pin_bit;
			if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
				gpio_regs->VR |= pin_bit;
			} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
				gpio_regs->VR &= ~pin_bit;
			}
		}  else if ((flags & GPIO_INPUT) != 0U) {
			gpio_regs->DR &= ~pin_bit;
		} else {
			// TODO
			// analog
		}
	} else {
		PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->MR), pin_bit, pin_bit);

		if ((flags & GPIO_PULL_UP) != 0U) {
			// TODO
		} else if ((flags & GPIO_PULL_DOWN) != 0U) {
			// TODO
		} else {
			// TODO
		}

		if ((flags & GPIO_OUTPUT) != 0U) {
			PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->DR), pin_bit, pin_bit);
			if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0U) {
				PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->VR), pin_bit, pin_bit);
			} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0U) {
				PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->VR), 0, pin_bit);
			}
		}  else if ((flags & GPIO_INPUT) != 0U) {
			PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->DR), 0, pin_bit);
		} else {
			// TODO
			// analog
		}
	}

	
	return 0;
}

static int gpio_aic8800x_port_get_raw(const struct device *port, uint32_t *value)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);

	if (!pmic_area) {
		*value = gpio_regs->VR;
	} else {
		*value = PMIC_MEM_READ((uint32_t)(&gpio_regs->VR));
	}

	return 0;
}

static int gpio_aic8800x_port_set_masked_raw(const struct device *port,
					gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);

	if (!pmic_area) {
		gpio_regs->VR = (gpio_regs->VR & ~mask) | (mask & value);
	} else {
		PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->VR), value, mask);
	}

	return 0;
}

static int gpio_aic8800x_port_set_bits_raw(const struct device *port,
				    gpio_port_pins_t pins)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);

	if (!pmic_area) {
		gpio_regs->VR = gpio_regs->VR | pins;
	} else {
		PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->VR), pins, pins);
	}

	return 0;
}

static int gpio_aic8800x_port_clear_bits_raw(const struct device *port,
					gpio_port_pins_t pins)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);

	if (!pmic_area) {
		gpio_regs->VR = gpio_regs->VR & ~pins;
	} else {
		PMIC_MEM_MASK_WRITE((uint32_t)(&gpio_regs->VR), 0, pins);
	}

	return 0;
}

static int gpio_aic8800x_port_toggle_bits(const struct device *port,
				    gpio_port_pins_t pins)
{
	struct aic8800x_gpio_regs *gpio_regs = GET_GPIO_REGS(port);
	bool pmic_area = GET_PMIC_ACCESS(port);

	if (!pmic_area) {
		gpio_regs->VR = gpio_regs->VR ^ pins;
	} else {
		uint_fast32_t temp = PMIC_MEM_READ((uint32_t)(&gpio_regs->VR)) ^ pins;
		PMIC_MEM_WRITE((uint32_t)(&gpio_regs->VR), temp);
	}

	return 0;
}

static int gpio_aic8800x_pin_interrupt_configure(const struct device *port,
					gpio_pin_t pin,
					enum gpio_int_mode mode,
					enum gpio_int_trig trig)
{
	return -ENOTSUP;
}

static int gpio_aic8800x_manage_callback(const struct device *dev,
				    struct gpio_callback *callback,
				    bool set)
{
	struct gpio_aic8800x_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static void gpio_aic8800x_isr(const struct device *dev)
{
	// TODO
	//struct gpio_aic8800x_data *data = dev->data;
}

static int gpio_aic8800x_init(const struct device *dev)
{
	// GPIOA IRQ Init
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority),
			gpio_aic8800x_isr, DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

static const struct gpio_driver_api gpio_aic8800x_api = {
	.pin_configure = gpio_aic8800x_configure,
	.port_get_raw = gpio_aic8800x_port_get_raw,
	.port_set_masked_raw = gpio_aic8800x_port_set_masked_raw,
	.port_set_bits_raw = gpio_aic8800x_port_set_bits_raw,
	.port_clear_bits_raw = gpio_aic8800x_port_clear_bits_raw,
	.port_toggle_bits = gpio_aic8800x_port_toggle_bits,
	.pin_interrupt_configure = gpio_aic8800x_pin_interrupt_configure,
	.manage_callback = gpio_aic8800x_manage_callback,
	.get_pending_int = NULL,
};

#define GPIO_AIC8800X_INIT(idx)																\
	static const struct gpio_aic8800x_config gpio_aic8800x_config##idx = {					\
		.common = {																			\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(idx),							\
		},																					\
		.gpio_regs = DT_INST_REG_ADDR(idx),													\
		.pmic_area = (DT_INST_REG_ADDR(idx) & 0xff000000) == 0x50000000 ? true : false,		\
	};																						\
																							\
	static struct gpio_aic8800x_data gpio_aic8800x_data##idx;								\
																							\
	DEVICE_DT_INST_DEFINE(idx,																\
				gpio_aic8800x_init,															\
				NULL,																		\
				&gpio_aic8800x_data##idx,													\
				&gpio_aic8800x_config##idx,													\
				PRE_KERNEL_1,																\
				CONFIG_GPIO_INIT_PRIORITY,													\
				&gpio_aic8800x_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_AIC8800X_INIT)
