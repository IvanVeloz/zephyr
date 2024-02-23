/*
 * Copyright (c) 2024 Ivan Veloz
 * Copyright (c) 2017 Piotr Mienkowski
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#ifndef _SAM0_I2SC_H_
#define _SAM0_I2SC_H

#include <zephyr/types.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif


/* i2sc configuration constants acquired through device tree */
struct i2sc_sam0_cfg {
    I2s *regs;
    const struct pinctrl_dev_config *pcfg;
};
/* i2sc variable data structure */
struct i2sc_sam0_data {
    int mydata;
};

/* Public APIs for the Atmel SAM0 I2S controller driver.
 *
 * These are very close copies of the regular I2S API, except that we add a
 * `serializer` argument to specify which of the two serializer units we are
 * addressing on the I2SC peripheral.
 * 
 * The original API is located at <zephyr/drivers/i2s.h> in the include path.
 */

/**
 * @cond INTERNAL_HIDDEN
 *
 * For internal use only, skip these in public documentation.
 */
__subsystem struct i2sc_sam0_driver_api_t {
	int (*configure)(const struct device *dev, enum i2s_dir dir,
			 const struct i2s_config *cfg, uint32_t ser);
	const struct i2s_config *(*config_get)(const struct device *dev,
				  enum i2s_dir dir);
	int (*read)(const struct device *dev, void **mem_block, size_t *size);
	int (*write)(const struct device *dev, void *mem_block, size_t size);
	int (*trigger)(const struct device *dev, enum i2s_dir dir,
		       enum i2s_trigger_cmd cmd);
};
/**
 * @endcond
 */

/**
 * @brief Configure operation of a host I2S controller.
 *
 * The dir parameter specifies if Transmit (TX) or Receive (RX) direction
 * will be configured by data provided via cfg parameter.
 *
 * The function can be called in NOT_READY or READY state only. If executed
 * successfully the function will change the interface state to READY.
 *
 * If the function is called with the parameter cfg->frame_clk_freq set to 0
 * the interface state will be changed to NOT_READY.
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param dir Stream direction: RX, TX, or both, as defined by I2S_DIR_*.
 *            The I2S_DIR_BOTH value may not be supported by some drivers.
 *            For those, the RX and TX streams need to be configured separately.
 * @param cfg Pointer to the structure containing configuration parameters.
 * @param ser Serializer unit to use on the I2S controller (0 or 1).
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid argument.
 * @retval -ENOSYS I2S_DIR_BOTH value is not supported.
 */
__syscall int i2sc_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *cfg, uint32_t ser);

static inline int z_impl_i2sc_configure(const struct device *dev, 
    enum i2s_dir dir, const struct i2s_config *cfg, uint32_t ser)
{
    const struct i2sc_sam0_driver_api_t *api =
        (const struct i2sc_sam0_driver_api_t *)dev->api;
    return api->configure(dev, dir, cfg, ser);
}

#ifdef __cplusplus
}
#endif
#endif /* _SAM0_I2S_H */

#include <syscalls/i2s_sam0_i2sc.h>
