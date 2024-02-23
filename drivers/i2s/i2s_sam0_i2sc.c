/*
 * Copyright (c) 2024 Ivan Veloz
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * The SAMD21 I2S peripheral has two I2S independently configurable serializers
 * sharing the same memory registers. They both support RX or TX modes and can 
 * be configured in any combination. This is an unusual architecture and it 
 * doesn´t fit the Zephyr API directly. Because of that, this initial 
 * implementation only supports configuring the I2S peripheral as 1 RX, 1 TX or 
 * 1 RX + 1 TX.
 * 
 * The SAMD21 also supports 1 TX + 1 TX and 1 RX + 1 RX, but this would need an
 * intermediate abstraction layer or an API extension.
 * 
 * If done as an abstraction layer, the abstraction has to present the one 
 * physical I2S Controller as two virtual I2S devices on the device tree.
 * Each virtual device can then be configured as either 1 TX or 1 RX. 
 * 
 * The abstraction could also allow a single virtual device to be configured as 
 * 1 RX + 1 TX, providing backwards compatibility. The second virtual device 
 * would be disabled.
 * 
 * I could make that driver in the future if time allows. But, if someone else 
 * wants to take this task before I start, please let me know.
 */


#include <errno.h>
#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include "i2s_sam0_i2sc.h"

#define DT_DRV_COMPAT atmel_sam0_i2sc

LOG_MODULE_REGISTER(i2sc_sam0, CONFIG_I2S_LOG_LEVEL);


static int i2sc_sam0_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg, uint32_t ser)
{
	LOG_DBG("i2sc_sam0_configure");
	return -EINVAL;
}

static int i2sc_sam0_read(const struct device *dev, void **mem_block, 
			 size_t *size)
{
	LOG_DBG("i2sc_sam0_read");
	return -EIO;
}

static int i2sc_sam0_write(const struct device *dev, void *mem_block, 
			  size_t size)
{
	LOG_DBG("i2sc_sam0_write");
	return -EIO;
}

static int i2sc_sam0_trigger(const struct device *dev, enum i2s_dir dir, 
			    enum i2s_trigger_cmd cmd)
{
	LOG_DBG("i2sc_sam0_trigger");
	return -EIO;
}

static int i2sc_sam0_initialize(const struct device *dev)
{
	const struct i2sc_sam0_cfg *cfg = dev->config;
	LOG_DBG("REV_I2S = 0x%08x", REV_I2S);
	return 0;
}

static const struct i2sc_sam0_driver_api_t i2sc_sam0_driver_api = {
	.configure = i2sc_sam0_configure,
	.read = i2sc_sam0_read,
	.write = i2sc_sam0_write,
	.trigger = i2sc_sam0_trigger,
	//.config_get = ,
	//.buf_read = ,
	//.buf_write = ,
};

#define I2SC_SAM0_INIT(inst)    			        	\
        static const struct i2sc_sam0_cfg i2sc_sam0_config_##inst = {	\
                .regs = (I2s *)DT_INST_REG_ADDR(inst),			\
        };								\
        static struct i2sc_sam0_data i2sc_sam0_data_##inst = {		\
                .mydata = 123,						\
        };								\
                                                                        \
        DEVICE_DT_INST_DEFINE(inst, 					\
                &i2sc_sam0_initialize, 					\
                NULL,							\
                &i2sc_sam0_data_##inst, 				\
                &i2sc_sam0_config_##inst,				\
                POST_KERNEL,						\
                CONFIG_I2S_INIT_PRIORITY,				\
                &i2sc_sam0_driver_api);


DT_INST_FOREACH_STATUS_OKAY(I2SC_SAM0_INIT)
