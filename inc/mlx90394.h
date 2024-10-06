/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#ifndef __MLX90394_H__
#define __MLX90394_H__

#include <rtthread.h>

#define MLX90394RLD_AAA_000 0
#define MLX90394RLD_AAA_001 1

#define MLX90394    MLX90394RLD_AAA_001

#if MLX90394 == MLX90394RLD_AAA_000
#define MLX90394_I2C_ADDRESS                    (0x10)        // default for mlx90394
#define MAGNETO10_MAG_FLUX_RESOLUTION   0.15    //uT/LSB
#elif MLX90394 == MLX90394RLD_AAA_001
#define MLX90394_I2C_ADDRESS                    (0x60)        // default for mlx90394
#define MAGNETO10_MAG_FLUX_RESOLUTION   1.5     //uT/LSB
#endif

#define MAGNETO10_TEMPERATURE_RES       50.0

#define MLX90394_ADDR_STAT1           0x0
#define MLX90394_ADDR_STAT2           0x7
#define MLX90394_ADDR_TEMPERATURE     0x8

#define MLX90394_ADDR_CID             0xA
#define MLX90394_ADDR_DID             0xB

#define MLX90394_ADDR_CTRL1           0x0E
#define MLX90394_ADDR_CTRL2           0x0F
#define MLX90394_ADDR_RST             0x11
#define MLX90394_ADDR_CTRL3           0x14
#define MLX90394_ADDR_CTRL4           0x15

#define MLX90394_ADDR_WOC_THRESHOLD_X 0x58
#define MLX90394_ADDR_WOC_THRESHOLD_Y 0x5A
#define MLX90394_ADDR_WOC_THRESHOLD_Z 0x5C

//ADDR = 0x00
union mlx90394_stat1
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t drdy     : 1;    //BIT0
        rt_uint8_t stat1_1  : 1;
        rt_uint8_t stat1_2  : 1;
        rt_uint8_t rt       : 1;
        rt_uint8_t interrupt: 1;
        rt_uint8_t stat1_5  : 1;
        rt_uint8_t stat1_6  : 1;
        rt_uint8_t stat1_7  : 1;
    };
};

//ADDR = 0x07
union mlx90394_stat2
{
    rt_uint8_t byte_val;

    struct
    {
        rt_uint8_t hovf_x   : 1;    //BIT0
        rt_uint8_t hovf_y   : 1;
        rt_uint8_t hovf_z   : 1;
        rt_uint8_t dor      : 1;
        rt_uint8_t stat2_4  : 1;
        rt_uint8_t stat2_5  : 1;
        rt_uint8_t stat2_6  : 1;
        rt_uint8_t stat2_7  : 1;
    };
};

typedef union
{
  struct
  {
    uint8_t mode      : 4;
    uint8_t x_en      : 1;
    uint8_t y_en      : 1;
    uint8_t z_en      : 1;
    uint8_t swoc      : 1;
  };
  uint8_t byte_val;
} mlx90394_ctrl1_t;

typedef union
{
  struct
  {
    uint8_t woc_mode      : 2;
    uint8_t intrepb       : 1;
    uint8_t intb_scl_b    : 1;
    uint8_t intdur        : 2;
    uint8_t config        : 2;
  };
  uint8_t byte_val;
} mlx90394_ctrl2_t;

typedef union
{
  struct
  {
    uint8_t dig_filt_temp    : 3;
    uint8_t dig_filt_hall_xy : 3;
    uint8_t osr_temp         : 1;
    uint8_t osr_hall         : 1;
  };
  uint8_t byte_val;
} mlx90394_ctrl3_t;

typedef union
{
  struct
  {
    uint8_t dig_filt_hall_z : 3;
    uint8_t drdy_en         : 1;
    uint8_t dnc1            : 1;
    uint8_t t_en            : 1;
    uint8_t dnc2            : 1;
    uint8_t dnc3            : 1;
  };
  uint8_t byte_val;
} mlx90394_ctrl4_t;

/* 3-axis data structure */
struct mlx90394_xyz
{
    rt_int16_t x;
    rt_int16_t y;
    rt_int16_t z;
};

/* 3-axis data structure */
struct mlx90394_xyz_flux
{
    float x;
    float y;
    float z;
};

enum mlx90394_mode
{
    POWER_DOWN_MODE                     = 0x0,
    SINGLE_MEASUREMENT_MODE             = 0x1,
    CONTINUOUS_MEASUREMENT_MODE_5HZ     = 0x2,
    CONTINUOUS_MEASUREMENT_MODE_10HZ    = 0x3,
    CONTINUOUS_MEASUREMENT_MODE_15HZ    = 0x4,
    CONTINUOUS_MEASUREMENT_MODE_50HZ    = 0x5,
    CONTINUOUS_MEASUREMENT_MODE_100HZ   = 0x6,
    SELF_TEST                           = 0x7,
//    POWER_DOWN_MODE                     = 0x8,
//    SINGLE_MEASUREMENT_MODE             = 0x9,
    CONTINUOUS_MEASUREMENT_MODE_200HZ   = 0xA,
    CONTINUOUS_MEASUREMENT_MODE_500HZ   = 0xB,
    CONTINUOUS_MEASUREMENT_MODE_700HZ   = 0xC,
    CONTINUOUS_MEASUREMENT_MODE_1100HZ  = 0xD,
    CONTINUOUS_MEASUREMENT_MODE_1400HZ  = 0xE,
//    POWER_DOWN_MODE                     = 0xF
};

enum cmd
{
    CMD_NOP               = 0x00,
    CMD_EXIT              = 0x80,
    CMD_START_BURST       = 0x10,
    CMD_WAKE_ON_CHANGE    = 0x20,
    CMD_START_MEASUREMENT = 0x30,
    CMD_READ_MEASUREMENT  = 0x40,
    CMD_READ_REGISTER     = 0x50,
    CMD_WRITE_REGISTER    = 0x60,
    CMD_MEMORY_RECALL     = 0xd0,
    CMD_MEMORY_STORE      = 0xe0,
    CMD_RESET             = 0xf0
};

enum axis_flag
{
    Z_FLAG = 0x8,
    Y_FLAG = 0x4,
    X_FLAG = 0x2,
    T_FLAG = 0x1
};

/** HALLCONF settings for CONF1 register. */
typedef enum mlx90394_hallconf
{
    mlx90394_HALLCONF_0 = (0x0),
    mlx90394_HALLCONF_C = (0xC),
} mlx90394_hallconf_t;

/** Gain settings for CONF1 register. */
typedef enum mlx90394_gain
{
    mlx90394_GAIN_5X = (0x00),
    mlx90394_GAIN_4X,
    mlx90394_GAIN_3X,
    mlx90394_GAIN_2_5X,
    mlx90394_GAIN_2X,
    mlx90394_GAIN_1_67X,
    mlx90394_GAIN_1_33X,
    mlx90394_GAIN_1X
} mlx90394_gain_t;

/** Resolution settings for CONF3 register. */
typedef enum mlx90394_resolution
{
    mlx90394_RES_16,
    mlx90394_RES_17,
    mlx90394_RES_18,
    mlx90394_RES_19,
} mlx90394_resolution_t;

/** Digital filter settings for CONF3 register. */
typedef enum mlx90394_filter
{
    mlx90394_FILTER_0,
    mlx90394_FILTER_1,
    mlx90394_FILTER_2,
    mlx90394_FILTER_3,
    mlx90394_FILTER_4,
    mlx90394_FILTER_5,
    mlx90394_FILTER_6,
    mlx90394_FILTER_7,
} mlx90394_filter_t;

/** Oversampling settings for CONF3 register. */
typedef enum mlx90394_oversampling
{
    mlx90394_OSR_0,
    mlx90394_OSR_1,
    mlx90394_OSR_2,
    mlx90394_OSR_3,
} mlx90394_oversampling_t;

/* mlx90394 config structure */
struct mlx90394_config
{
    rt_uint16_t accel_range;
    rt_uint16_t gyro_range;
};

/* mlx90394 device structure */
struct mlx90394_device
{
    rt_device_t bus;
    rt_uint8_t id;
    rt_uint8_t i2c_addr;
    struct mlx90394_config config;
};

/**
 * This function initialize the mlx90394 device.
 *
 * @param dev_name the name of transfer device
 * @param param the i2c device address for i2c communication, RT_NULL for spi
 *
 * @return the pointer of device driver structure, RT_NULL reprensents  initialization failed.
 */
struct mlx90394_device *mlx90394_init(const char *dev_name, rt_uint8_t param);

/**
 * This function releases memory
 *
 * @param dev the pointer of device driver structure
 */
void mlx90394_deinit(struct mlx90394_device *dev);

rt_err_t mlx90394_nop(struct mlx90394_device *dev);
rt_err_t mlx90394_exit(struct mlx90394_device *dev);
rt_err_t mlx90394_reset(struct mlx90394_device *dev);

rt_err_t mlx90394_get_gain_sel(struct mlx90394_device *dev, mlx90394_gain_t *gain);
rt_err_t mlx90394_get_resolution(struct mlx90394_device *dev, mlx90394_resolution_t *res_x, mlx90394_resolution_t *res_y, mlx90394_resolution_t *res_z);

#endif
