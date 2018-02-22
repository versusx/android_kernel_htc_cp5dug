/*
 * linux/drivers/misc/ap_sync_cbp_oem.c
 *
 * VIA CBP driver for Linux
 *
 * Copyright (C) 2011 VIA TELECOM Corporation, Inc.
 * Author: VIA TELECOM Corporation, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/gpio.h>

static inline int oem_gpio_request(unsigned gpio, const char *label)
{
	return gpio_request(gpio, label);
}

static inline void oem_gpio_free(unsigned gpio)
{
	return gpio_free(gpio);
}

/*config the gpio to be input for irq if the SOC need*/
static inline int oem_gpio_direction_input_for_irq(unsigned gpio)
{
	return gpio_direction_input(gpio);
}

static inline int oem_gpio_direction_output(unsigned gpio, int value)
{
	return gpio_direction_output(gpio, value);
}

/* Get the output level if the gpio is output type;
 * Get the input level if the gpio is input type
 */
static inline int oem_gpio_get_value(unsigned gpio)
{
	return gpio_get_value(gpio);
}

static inline int oem_gpio_to_irq(unsigned gpio)
{
	return gpio_to_irq(gpio);
}
extern int irq_set_irq_type(unsigned int irq, unsigned int type);
extern int irq_set_irq_wake(unsigned int irq, unsigned int type);
/* Set the irq type of the pin.
 * Get the pin level and set the correct edge if the type is both edge and
 * the SOC do not support both edge detection at one time*/
static inline int oem_gpio_set_irq_type(unsigned gpio, unsigned int type)
{
	irq_set_irq_type(oem_gpio_to_irq(gpio), type);
	irq_set_irq_wake(oem_gpio_to_irq(gpio), type);
	return 0;
}
