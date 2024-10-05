/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __SENSOR_MELEXIS_MLX90394_H__
#define __SENSOR_MELEXIS_MLX90394_H__

#include "sensor.h"
#include "mlx90394.h"

#define mlx90394_CTRL_NOP                       0x10
#define mlx90394_CTRL_RESET                     0x11

int rt_hw_mlx90394_init(const char *name, struct rt_sensor_config *cfg);

#endif
