/*
 * Copyright (c) 2024 Ivan Veloz
 *
 * SPDX-License-Identifier: Apache-2.0
 */



#ifndef _SAM0_I2S_H_
#define _SAM0_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration constants acquired through device tree */
struct i2s_sam0_cfg {
    const uint32_t serializer;          // Self's serializer address (0 to 1)
    const struct device *i2sc;          // Parent i2sc node
    const struct pinctrl_dev_config *pcfg;  // Pin configuration structure

};

struct i2s_sam0_data {
    int mydata;
};

#ifdef __cplusplus
}
#endif
#endif /* _SAM0_I2S_H */
