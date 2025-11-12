#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tsl2591.h>
#include <zephyr/logging/log.h>

#include "als_drv.h"

#define LOG_LEVEL CONFIG_ALS_DRV_LOG_LEVEL
LOG_MODULE_REGISTER(als_drv);

struct sensor_value lux, full_spectrum, infrared;

const struct device *const dev = DEVICE_DT_GET_ONE(ams_tsl2591);

/**
 * @brief Initialisation of Ambient Light Sensor Driver 
 *
 * @note TSL2591 is used for ambient light sensing.
 *
 * @param NIL.
 *
 * @return 0 on success, negative error code on failure
 */
int16_t als_init(void)
{
    int16_t ret = 0;

    if (!device_is_ready(dev)) {
        LOG_ERR("Device %s is not ready", dev->name);
        return -ENODEV;
    }

    LOG_INF("Device %s initialized successfully", dev->name);

    /* Configure the sensor with high gain for low light conditions */
    struct sensor_value gain_val = {
        .val1 = TSL2591_SENSOR_GAIN_MED,
        .val2 = 0
    };
    
    ret = sensor_attr_set(dev, SENSOR_CHAN_LIGHT,
                            SENSOR_ATTR_GAIN_MODE, &gain_val);
    if (ret < 0) {
        LOG_ERR("E%d: Failed to set gain", ret);
        return -ENODEV;
    }
    
    return ret;
}

/**
 * @brief Reads data from the Ambient Light Sensor 
 *
 * @note NIL
 *
 * @param als Pointer to the ambient light data structure to fill with sensor readings.
 *
 * @return 0 on success, negative error code on failure
 */
int16_t als_get_data(als_data_t *als)
{
    int16_t ret = 0;

    /* Fetch sensor data */
    ret = sensor_sample_fetch(dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch sensor data: %d", ret);
        return ret;
    }

    /* Get calculated lux value */
    ret = sensor_channel_get(dev, SENSOR_CHAN_LIGHT, &lux);
    if (ret < 0) {
        LOG_ERR("Failed to get lux data: %d", ret);
    } else {
        LOG_DBG("Light: %d.%01d lux, ", lux.val1, lux.val2);
        als->lux = (uint16_t)(lux.val1);
    }

    /* Get full spectrum value */
    ret = sensor_channel_get(dev, SENSOR_CHAN_ALL, &full_spectrum);
    if (ret < 0) {
        LOG_ERR("Failed to get full spectrum data: %d", ret);
    } else {
        LOG_DBG("Full spectrum: %d, ", full_spectrum.val1);
        als->full_spectrum = (uint16_t)full_spectrum.val1;
    }

    /* Get infrared value */
    ret = sensor_channel_get(dev, SENSOR_CHAN_IR, &infrared);
    if (ret < 0) {
        LOG_ERR("Failed to get infrared data: %d", ret);
    } else {
        LOG_DBG("Infrared: %d\n", infrared.val1);
        als->infrared = (uint16_t)infrared.val1;
    }

    LOG_INF("Lux: %d, Full Spectrum: %d, Infrared: %d",
          als->lux, als->full_spectrum, als->infrared);

    return ret;
}