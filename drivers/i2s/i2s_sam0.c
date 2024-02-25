/*
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>
 * Copyright (c) 2024 Ivan Veloz
 *
 * SPDX-License-Identifier: Apache-2.0
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

static void wait_synchronization(I2s *regs)
{
#if defined(I2S_SYNCBUSY_MASK)
	/* SYNCBUSY is a register */
	while ((regs->SYNCBUSY.reg & I2S_SYNCBUSY_MASK) != 0) {
	}
#else
#error Unsupported device
#endif
}

static int i2s_sam0_configure(const struct device *dev, enum i2s_dir dir,
			      const struct i2s_config *i2s_cfg)
{
	const struct i2s_sam0_cfg *const cfg = dev->config;
	struct i2s_sam0_data *const data = dev->data;

	i2sc_configure(cfg->i2sc, dir, i2s_cfg, dev);

	/*
        uint8_t word_size;
        uint8_t channels;
        i2s_fmt_t format;
        i2s_opt_t options;
        uint32_t frame_clk_freq;
        struct k_mem_slab *mem_slab;
        size_t block_size;
        int32_t timeout;
	*/
	/*
	i2s_cfg.word_size = 16U;
	i2s_cfg.channels = 2U;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.frame_clk_freq = 44100;
	i2s_cfg.block_size = BLOCK_SIZE;
	i2s_cfg.timeout = 2000;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER
			| I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.mem_slab = &tx_0_mem_slab;
	*/
	return 0;

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
	const struct i2s_sam0_cfg *const cfg = dev->config;
	struct i2s_sam0_data *const data = dev->data;
	I2s *const i2s = cfg->regs;
	int ret;

	return 0;
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



#define I2S_SAM0_INIT_INST(inst) 					\
        PINCTRL_DT_INST_DEFINE(inst);					\
	static const struct i2s_sam0_cfg i2s_sam0_config_##inst = {	\
		/* Initialize ROM values as needed */			\
		/* Get properties from the device tree using DT_inst_ */\
		.n = DT_REG_ADDR(DT_DRV_INST(inst)),		\
		.i2sc = DEVICE_DT_GET(DT_INST_PARENT(inst)),		\
		.regs = (I2s *)DT_REG_ADDR(DT_INST_PARENT(inst)),	\
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


DT_INST_FOREACH_STATUS_OKAY(I2S_SAM0_INIT_INST)

