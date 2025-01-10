/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-02-14     lgnq         the first version
 */

#include "sensor_melexis_mlx90394.h"
#include <stdlib.h>

#define DBG_TAG "sensor.melexis.mlx90394"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define mlx_dev ((struct mlx90394_device *)sensor->parent.user_data)

static struct mlx90394_device *_mlx90394_init(struct rt_sensor_intf *intf)
{
    rt_uint8_t i2c_addr = (rt_uint32_t)(intf->user_data) & 0xff;

    return mlx90394_init(intf->dev_name, i2c_addr);
}

static rt_err_t _mlx90394_set_range(rt_sensor_t sensor, rt_int32_t range)
{
//    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
//    {
//        rt_uint8_t range_ctr;
//
//        if (range < 2000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_2G;
//        else if (range < 4000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_4G;
//        else if (range < 8000)
//            range_ctr = MPU6XXX_ACCEL_RANGE_8G;
//        else
//            range_ctr = MPU6XXX_ACCEL_RANGE_16G;
//
//        LOG_D("acce set range %d", range_ctr);
//
//        return mlx90394_set_param(mpu_dev, MPU6XXX_ACCEL_RANGE, range_ctr);
//    }
//    else if (sensor->info.type == RT_SENSOR_CLASS_GYRO)
//    {
//        rt_uint8_t range_ctr;
//
//        if (range < 250000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_250DPS;
//        else if (range < 500000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_500DPS;
//        else if (range < 1000000UL)
//            range_ctr = MPU6XXX_GYRO_RANGE_1000DPS;
//        else
//            range_ctr = MPU6XXX_GYRO_RANGE_2000DPS;
//
//        LOG_D("gyro set range %d", range);
//
//        return mlx90394_set_param(mpu_dev, MPU6XXX_GYRO_RANGE, range_ctr);
//    }
    return RT_EOK;
}

rt_err_t mlx90394_get_info(rt_sensor_t sensor)
{
    rt_err_t res = RT_EOK;

    rt_uint8_t cid;
    rt_uint8_t did;
    mlx90394_ctrl1_t ctrl1;

    struct mlx90394_device *dev = ((struct mlx90394_device *)sensor->parent.user_data);

    if (dev == RT_NULL)
    {
        rt_kprintf("Please probe mlx90394 first!\n");
        return -RT_ERROR;
    }

    res  = mlx90394_get_cid(dev, &cid);
    res += mlx90394_get_did(dev, &did);
    res += mlx90394_get_ctrl1(dev, &ctrl1);

    rt_kprintf("cid:%x\n", cid);
    rt_kprintf("did:%x\n", did);
    rt_kprintf("xonoff:%x\n", ctrl1.x_en);
    rt_kprintf("yonoff:%x\n", ctrl1.y_en);
    rt_kprintf("zonoff:%x\n", ctrl1.z_en);

    return res;
}

static rt_size_t _mlx90394_polling_get_data(rt_sensor_t sensor, struct rt_sensor_data *data)
{
    if (sensor->info.type == RT_SENSOR_CLASS_MAG)
    {
        struct mlx90394_xyz_flux xyz;

        if (mlx90394_single_measurement((struct mlx90394_device *)sensor->parent.user_data, &xyz) != RT_EOK)
        {
            rt_kprintf("mlx90394_single_measurement error\r\n");

            return 0;
        }

        data->type = RT_SENSOR_CLASS_MAG;
        data->data.mag.x = xyz.x;
        data->data.mag.y = xyz.y;
        data->data.mag.z = xyz.z;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_size_t mlx90394_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    RT_ASSERT(buf);

    if (sensor->config.mode == RT_SENSOR_MODE_POLLING)
    {
        return _mlx90394_polling_get_data(sensor, buf);
    }
    else
        return 0;
}

static rt_err_t mlx90394_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = mlx_dev->id;
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _mlx90394_set_range(sensor, (rt_int32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = -RT_EINVAL;
        break;
    case RT_SENSOR_CTRL_SET_MODE:
        result = mlx90394_set_mode((struct mlx90394_device *)sensor->parent.user_data, (rt_uint32_t)args & 0xff);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        break;
    case RT_SENSOR_CTRL_SELF_TEST:
        break;
    case RT_SENSOR_CTRL_USER_CMD_RESET:
        result = mlx90394_reset((struct mlx90394_device *)sensor->parent.user_data);
        break;
    case RT_SENSOR_CTRL_USER_CMD_INFO:
        result = mlx90394_get_info(sensor);
        break;
    default:
        return -RT_ERROR;
    }
    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    mlx90394_fetch_data,
    mlx90394_control
};

int rt_hw_mlx90394_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    struct mlx90394_device *mlx_dev_temp;
    rt_sensor_t sensor_mps = RT_NULL;

    mlx_dev_temp = _mlx90394_init(&cfg->intf);
    if (mlx_dev_temp == RT_NULL)
    {
        LOG_E("_mlx90394 init err!");
        goto __exit;
    }

    /* MPS sensor register */
    {
        sensor_mps = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_mps == RT_NULL)
            goto __exit;

        sensor_mps->info.type       = RT_SENSOR_CLASS_MAG;
        sensor_mps->info.vendor     = RT_SENSOR_VENDOR_MELEXIS;
        sensor_mps->info.model      = "mlx90394";
        sensor_mps->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_mps->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_mps->info.range_max  = 16000;
        sensor_mps->info.range_min  = 2000;
        sensor_mps->info.period_min = 5;

        rt_memcpy(&sensor_mps->config, cfg, sizeof(struct rt_sensor_config));
        sensor_mps->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_mps, name, RT_DEVICE_FLAG_RDWR, mlx_dev_temp);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            goto __exit;
        }
    }

    LOG_I("sensor init success");
    return RT_EOK;

__exit:
    if (mlx_dev_temp)
        mlx90394_deinit(mlx_dev_temp);

    return -RT_ERROR;
}

int rt_hw_mlx90394_port(void)
{
    struct rt_sensor_config cfg;

    cfg.intf.dev_name  = "i2c2";
    cfg.intf.user_data = (void *)MLX90394_I2C_ADDRESS;
//    cfg.irq_pin.pin = RT_PIN_NONE;

    rt_hw_mlx90394_init("mps", &cfg);

    return 0;
}
INIT_ENV_EXPORT(rt_hw_mlx90394_port);

static void read_mps_entry(void *parameter)
{
    rt_device_t dev = RT_NULL;
    struct rt_sensor_data sensor_data;
    rt_size_t res;

    dev = rt_device_find(parameter);
    if (dev == RT_NULL)
    {
        LOG_E("Can't find device:%s\n", parameter);
        return;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            LOG_E("device is already opened!\n");
        }
        else
        {
            LOG_E("open device failed!\n");
            return;
        }
    }

//    rt_device_control(dev, RT_SENSOR_CTRL_SET_ODR, (void *)100);

    while (1)
    {
        res = rt_device_read(dev, 0, &sensor_data, 1);
        if (res != 1)
        {
            LOG_E("read data failed!size is %d\n", res);
            rt_device_close(dev);
            return;
        }
        else
        {
            rt_kprintf("data:%d,%d,%d\n", sensor_data.data.mag.x, sensor_data.data.mag.y, sensor_data.data.mag.z);
        }

        rt_thread_mdelay(10);
    }
}

rt_err_t mlx90394_measurement_onoff(int argc, char **argv)
{
    rt_thread_t mlx90394_thread;

    if (!strcmp(argv[1], "on"))
    {
        mlx90394_thread = rt_thread_create("mlx90394", read_mps_entry, "mag_mps", 1024, RT_THREAD_PRIORITY_MAX / 2, 20);
        if (mlx90394_thread != RT_NULL)
        {
            rt_thread_startup(mlx90394_thread);

            return 0;
        }
    }
    else if (!strcmp(argv[1], "off"))
    {
        mlx90394_thread = rt_thread_find("mlx90394");

        if (mlx90394_thread != RT_NULL)
        {
            rt_thread_delete(mlx90394_thread);

            return 0;
        }
    }

    return -1;
}

rt_err_t mlx90394_ops_ctrl(int argc, char **argv)
{
    rt_size_t res = RT_EOK;
    rt_device_t dev = RT_NULL;

    rt_uint16_t p = atoi(argv[2]);

    dev = rt_device_find("mag_mps");
    if (dev == RT_NULL)
    {
        LOG_E("Can't find device:%s\n");
        return -RT_ERROR;
    }

    res = rt_device_open(dev, RT_DEVICE_FLAG_RDWR);
    if (res != RT_EOK)
    {
        if (res == -RT_EBUSY)
        {
            LOG_E("device is already opened!\n");
        }
        else
        {
            LOG_E("open device failed!\n");
            return -RT_ERROR;
        }
    }

    if (rt_device_control(dev, atoi(argv[1]), &p))
    {
        LOG_E("device control set failed, 0x%x 0x%x!\n", atoi(argv[1]), atoi(argv[2]));
        return -RT_ERROR;
    }

    return res;
}

#ifdef FINSH_USING_MSH
    MSH_CMD_EXPORT(mlx90394_measurement_onoff, mlx90394 sensor function);
//    MSH_CMD_EXPORT(mlx90394_ctrl_set_sample_freq, mlx90394 sensor function);
    MSH_CMD_EXPORT(mlx90394_ops_ctrl, mlx90394 sensor function);
#endif

