/*
 * Copyright (c) 2024 Ivan Veloz
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#ifndef _SAM0_I2S_H_
#define _SAM0_I2S_H

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration constants acquired through device tree for device instance */
struct i2s_sam0_cfg {
	const uint32_t n;			// Self's serializer number (0 or 1)
	const struct device *const i2sc;	// Parent i2sc node
	volatile I2s * const regs;		// i2s (controller) register
};
/* Data variables for use by device instance*/
struct i2s_sam0_data {
 	int mydata;
};

/* Constants, extension of the Atmel HAL */
#define I2S_INTENCLR_MASK0	_U_(0x1111)
#define I2S_INTENCLR_MASK1	(I2S_INTENCLR_MASK0 << 1)
#define I2S_INTENCLR_MASKn(n)	(I2S_INTENCLR_MASK0 << (n))


#ifdef __cplusplus
}
#endif
#endif /* _SAM0_I2S_H */
