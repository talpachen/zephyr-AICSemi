/*
 * Copyright (c) 2022, Talpa Chen
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>

static int aic8800_soc_init(const struct device *dev)
{
	uint32_t key;

	key = irq_lock();

	// TODO
	//SystemInit();
	
	NMI_INIT();

	irq_unlock(key);

	return 0;
}

SYS_INIT(aic8800_soc_init, PRE_KERNEL_1, 0);
