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

#define DT_DRV_COMPAT atmel_sam0_i2s

#include <errno.h>
#include <soc.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include "i2s_sam0_i2sc.h"
#include "i2s_sam0.h"

LOG_MODULE_REGISTER(i2s_sam0, CONFIG_I2S_LOG_LEVEL);

static int i2s_sam0_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	LOG_DBG("i2s_sam0_configure");
	const struct i2s_sam0_cfg *cfg = dev->config;
	return i2sc_configure(cfg->i2sc, dir, i2s_cfg, cfg->serializer);
}

static int i2s_sam0_read(const struct device *dev, void **mem_block, 
			 size_t *size)
{
	LOG_DBG("i2s_sam0_read");
	return -EIO;
}

static int i2s_sam0_write(const struct device *dev, void *mem_block, 
			  size_t size)
{
	LOG_DBG("i2s_sam0_write");
	return -EIO;
}

static int i2s_sam0_trigger(const struct device *dev, enum i2s_dir dir, 
			    enum i2s_trigger_cmd cmd)
{
	LOG_DBG("i2s_sam0_trigger");
	return -EIO;
}

static int i2s_sam0_initialize(const struct device *dev)
{
	const struct i2s_sam0_cfg *cfg = dev->config;
	if(cfg->serializer > I2S_SER_NUM - 1) {
		LOG_ERR("I2S serializer address 0x%08x out of range.",
			cfg->serializer);
		/*
		 * Only two serializers are included in the I2S controller,
		 * and the address range is from 0 to 1. So make sure the 
		 * device tree for your SoC looks something like this:
		 * 
		 * i2sc: i2sc@0xabcd1234 {
		 *	reg = <0xabcd1234 0x38>
		 *	i2s0: i2s@0 {
		 *		reg = <0>
		 *	};
		 *	i2s1: i2s@0 {
		 *		reg = <1>
		 *	};
		 * }
		 */
		return -EINVAL;
	}
	LOG_DBG("REV_I2S = 0x%08x", REV_I2S);

	int ret;
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	LOG_DBG("pinctrl_apply_state returned %i",ret);
	return ret;
}

static const struct i2s_driver_api i2s_sam0_driver_api = {
	.configure = i2s_sam0_configure,
	.read = i2s_sam0_read,
	.write = i2s_sam0_write,
	.trigger = i2s_sam0_trigger,
	//.config_get = ,
	//.buf_read = ,
	//.buf_write = ,
};

#define I2S_SAM0_INIT(inst) 						\
        PINCTRL_DT_INST_DEFINE(inst);					\
	static const struct i2s_sam0_cfg i2s_sam0_config_##inst = {	\
		/* Initialize ROM values as needed */			\
		/* Get properties from the device tree using DT_inst_ */\
		.serializer = DT_REG_ADDR(DT_INST(inst,DT_DRV_COMPAT)),	\
		.i2sc = DEVICE_DT_GET(DT_INST_PARENT(inst)),		\
                .pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		\
	};								\
	static struct i2s_sam0_data i2s_sam0_data_##inst = {		\
		/* Initialize RAM values as needed */			\
		/* Get properties from the device tree using DT_inst_ */\
		.mydata = 123,						\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, 					\
		&i2s_sam0_initialize, 					\
		NULL,							\
		&i2s_sam0_data_##inst, 					\
		&i2s_sam0_config_##inst,				\
		POST_KERNEL,						\
		CONFIG_I2S_INIT_PRIORITY,				\
		&i2s_sam0_driver_api);


DT_INST_FOREACH_STATUS_OKAY(I2S_SAM0_INIT)

