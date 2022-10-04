/*
 * Copyright (c) 2022, Talpa Chen
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/pinctrl.h>

static void pinctrl_configure_pin(const pinctrl_soc_pin_t *pin, uintptr_t reg)
{
	ARG_UNUSED(reg);

	#if 0
	gpio_init(pin->pin_num);
	gpio_set_function(pin->pin_num, pin->alt_func);
	gpio_set_pulls(pin->pin_num, pin->pullup, pin->pulldown);
	gpio_set_drive_strength(pin->pin_num, pin->drive_strength);
	gpio_set_slew_rate(pin->pin_num, (pin->slew_rate ?
				GPIO_SLEW_RATE_FAST : GPIO_SLEW_RATE_SLOW));
	gpio_set_input_hysteresis_enabled(pin->pin_num, pin->schmitt_enable);
	gpio_set_input_enabled(pin->pin_num, pin->input_enable);
	#endif
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,
			   uintptr_t reg)
{
	for (uint8_t i = 0U; i < pin_cnt; i++) {
		pinctrl_configure_pin(pins++, reg);
	}

	return 0;
}
