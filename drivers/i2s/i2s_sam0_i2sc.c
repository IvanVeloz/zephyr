/*
 * Copyright (c) 2019 Derek Hageman <hageman@inthat.cloud>
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
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#include "i2s_sam0.h"
#include "i2s_sam0_i2sc.h"

#define DT_DRV_COMPAT atmel_sam0_i2sc

LOG_MODULE_REGISTER(i2sc_sam0, CONFIG_I2S_LOG_LEVEL);

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

static void refresh_bindings(const struct device *dev)
{
	const struct i2sc_sam0_cfg *const cfg = dev->config;
	struct i2sc_sam0_data *const data = dev->data;
	for(int i = 0; i < cfg->i2s_n; i++) {
		struct device * child = (struct device *) cfg->i2s_dev[i];
		LOG_DBG("Child %1x: %s %p",i,child->name,child);
		i2s_read(child,NULL,NULL);
	}
}

static int i2sc_sam0_configure(const struct device *dev, enum i2s_dir dir,
			       const struct i2s_config *i2s_cfg, 
			       const struct device *child)
{
	const struct i2sc_sam0_cfg *const cfg = dev->config;
	struct i2sc_sam0_data *const data = dev->data;
	volatile I2s * const i2s = cfg->regs;
	int ret;
	refresh_bindings(dev);


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
	const struct i2sc_sam0_cfg *const cfg = dev->config;
	struct i2sc_sam0_data *const data = dev->data;
	I2s *const i2s = cfg->regs;
	int ret;

	LOG_DBG("API REV_I2S = 0x%08x", REV_I2S);
	LOG_DBG("Detected %1x child(ren).", cfg->i2s_n);
	for(int i = 0; i < cfg->i2s_n; i++) {
		struct device * child = (struct device *) cfg->i2s_dev[i];
		LOG_DBG("Child %1x: %s %p",i,child->name,child);
	}


#ifdef MCLK
#error Unsupported device /* TODO: buy hardware with MCLK and test*/
#else
	/* Enable the GCLK */
	/* TODO, or place this on the children, or call the children to do it */
	/* Be wary of the order of operations. */

	/* Enable I2S clock in PM */
	PM->APBCMASK.reg |= cfg->pm_apbcmask;
#endif
	/* Disable all I2S serializers' interrupts */
	i2s->INTENCLR.reg = I2S_INTENCLR_MASK;
	//LOG_DBG("i2s->INTENCLR.reg = 0x%04x",I2S_INTENCLR_MASK);

	/* Apply the children pin states */
	/* TODO loop over children */
	/*
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_DBG("pinctrl_apply_state returned %i",ret);
		return ret;
	}*/

	/* Soft reset the logic (DS40001882H section 29.5.3 paragraph 4)*/
	/* TODO */

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


/* Gets an array initializer with all of the children nodes that have status 
 * "okay". Since this parent initializes before the children, the array will
 * be full of NULL pointers. If this ever changes, it will have valid pointers.
 * We obtain valid pointers at runtime in i2sc_sam_configure().
 * Example: const struct device *i2s[] = I2SC_SAM0_CHILDREN_ARRAY(inst);
 * expands to: const struct device *i2s[] = { devicepointer1, devicepointer2 };
 */
#define I2SC_SAM0_CHILDREN_ARRAY(inst)					\
	{ DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst, DEVICE_DT_GET, (,)) }

#define I2SC_SAM0_CHILDREN_DEVICE_DT_NAME(inst)				\
	{ DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst, DEVICE_DT_NAME, (,)) }

/* This is just a hack to construct 1+1+1+... when called by a FOREACH() macro.
 * Necessary because the macros expect this custommacro(node_id) format.
 */
#define I2SC_SAM0_ONE(node_id)	1

/* Gets the number of children nodes that have status "okay"
 * Example: uint32_t i2sn = I2SC_SAM0_CHILDREN_ARRAY_SIZE(inst);
 * expands to: uint32_t i2sn = 1+1;
 */
#define I2SC_SAM0_CHILDREN_ARRAY_SIZE(inst)				\
	DT_INST_FOREACH_CHILD_STATUS_OKAY_SEP(inst,I2SC_SAM0_ONE, (+))

#define GET_I2S_CHILDREN(inst)						\
static char 

/* 
 *
 */
#ifdef MCLK
#error Unsupported device /* TODO: buy hardware with MCLK and test*/
#else /* !MCLK */
#define I2SC_SAM0_CONFIG(inst)						\
static const struct i2sc_sam0_cfg i2sc_sam0_config_##inst = {		\
	/* Initialize ROM values as needed */				\
	/* Get properties from the device tree using DT_inst_ */	\
	.regs = (I2s *)DT_INST_REG_ADDR(inst),				\
	.pm_apbcmask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, pm, bit)),	\
	.i2s_n = I2SC_SAM0_CHILDREN_ARRAY_SIZE(inst),			\
	.i2s_dev = (const struct device *[])I2SC_SAM0_CHILDREN_ARRAY(inst),\
}
#endif

#define I2SC_SAM0_INIT_INST(inst)    			        	\
	I2SC_SAM0_CONFIG(inst);						\
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


DT_INST_FOREACH_STATUS_OKAY(I2SC_SAM0_INIT_INST)
