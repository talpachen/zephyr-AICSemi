/*
 * Copyright (c) 2022, Talpa Chen
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC_ARM_AICSEMI_AIC8800_PINCTRL_SOC_H_
#define _SOC_ARM_AICSEMI_AIC8800_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IOMUX_GPIO_CONFIG_SEL_LSB           (0)
#define IOMUX_GPIO_CONFIG_SEL_WIDTH         (4)
#define IOMUX_GPIO_CONFIG_SEL_MASK          (((1 << IOMUX_GPIO_CONFIG_SEL_WIDTH) - 1) << IOMUX_GPIO_CONFIG_SEL_LSB)
#define IOMUX_GPIO_CONFIG_PULL_DN_LSB       (8)
#define IOMUX_GPIO_CONFIG_PULL_DN_WIDTH     (1)
#define IOMUX_GPIO_CONFIG_PULL_DN_MASK      (((1 << IOMUX_GPIO_CONFIG_PULL_DN_WIDTH) - 1) << IOMUX_GPIO_CONFIG_PULL_DN_LSB)
#define IOMUX_GPIO_CONFIG_PULL_UP_LSB       (9)
#define IOMUX_GPIO_CONFIG_PULL_UP_WIDTH     (1)
#define IOMUX_GPIO_CONFIG_PULL_UP_MASK      (((1 << IOMUX_GPIO_CONFIG_PULL_UP_WIDTH) - 1) << IOMUX_GPIO_CONFIG_PULL_UP_LSB)
#define IOMUX_GPIO_CONFIG_PULL_FRC_LSB      (16)
#define IOMUX_GPIO_CONFIG_PULL_FRC_WIDTH    (1)
#define IOMUX_GPIO_CONFIG_PULL_FRC_MASK     (((1 << IOMUX_GPIO_CONFIG_PULL_FRC_WIDTH) - 1) << IOMUX_GPIO_CONFIG_PULL_FRC_LSB)


typedef struct pinctrl_soc_pin {
	uint32_t base_addr;
	uint8_t pin;
	union {
		struct {
			uint32_t pinmux : 4;
			uint32_t : 4;
			uint32_t pull_down : 1;
			uint32_t pull_up : 1;
			uint32_t : 6;
			uint32_t pull_frc : 1;
			uint32_t : 15;
		};
		uint32_t gpcfg;
	};
} pinctrl_soc_pin_t;

#define Z_PINCTRL_PINCFG_INIT(node_id)										\
	(IF_ENABLED(DT_PROP(node_id, bias_pull_down), (IOMUX_GPIO_CONFIG_PULL_DN_MASK |))	\
	IF_ENABLED(DT_PROP(node_id, bias_pull_up), (IOMUX_GPIO_CONFIG_PULL_UP_MASK |))	\
	IF_ENABLED(DT_PROP(node_id, drive_push_pull), (IOMUX_GPIO_CONFIG_PULL_FRC_MASK |)))

#define Z_PINCTRL_STATE_PIN_INIT(group, pin_prop, idx)										\
	DT_PROP_BY_IDX(group, pin_prop, idx) | Z_PINCTRL_PINCFG_INIT(group),

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)											\
	{ DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT) }

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_NXP_LPC_11U6X_PINCTRL_SOC_H_ */
