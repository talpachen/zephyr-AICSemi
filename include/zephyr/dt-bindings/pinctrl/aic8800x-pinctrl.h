/*
 * Copyright (c) 2022, Talpa Chen
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_AIC8800X_PINCTRL_COMMON_H_
#define ZEPHYR_AIC8800X_PINCTRL_COMMON_H_

#define AIC8800X_PINMUX(port, pin, mux)		\
	(((((port) - 'A') & 0x7) << 29) |		\					
	(((pin) & 0x1F) << 24) | 				\
	(((mux) & 0xFF) << 16))

#endif	/* ZEPHYR_AIC8800X_PINCTRL_COMMON_H_ */
