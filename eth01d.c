// SPDX-License-Identifier: GPL-2.0-or-later
/* Econarae ETH-01D humidity and temperature sensor driver
 *
 * Copyright (C) 2019 Gaurav Kalra <gvkalra@gmail.com>
 *
 * heavily based on the hih6130 driver
 * Copyright (C) 2012 Iain Paton <ipaton0@gmail.com>
 *
 * Data sheets available (2019-06-12) at
 * https://www.icbanq.com/data/ICBShop/board/09.EcoNarae_ETH-01D.pdf
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

/**
 * struct eth01d - ETH-01D device specific data
 * @client: pointer to I2C client device
 * @lock: mutex to protect measurement values
 * @valid: only false before first measurement is taken
 * @last_update: time of last update (jiffies)
 * @temperature: cached temperature measurement value
 * @humidity: cached humidity measurement value
 * @write_length: length for I2C measurement request
 */
struct eth01d {
    struct i2c_client *client;
    struct mutex lock;
    bool valid;
    unsigned long last_update;
    int temperature;
    int humidity;
    size_t write_length;
};

/**
 * eth01d_temp_ticks_to_millicelsius() - convert raw temperature ticks to
 * milli celsius
 * @ticks: temperature ticks value received from sensor
 */
static inline int eth01d_temp_ticks_to_millicelsius(int ticks)
{
    ticks = ticks >> 2;
    /*
     * Formula T = ( ticks / ( 2^14 - 1 ) ) * 165 - 40
     */
    return (DIV_ROUND_CLOSEST(ticks * 1650, 16383) - 400) * 100;
}

/**
 * eth01d_rh_ticks_to_per_cent_mille() - convert raw humidity ticks to
 * one-thousandths of a percent relative humidity
 * @ticks: humidity ticks value received from sensor
 */
static inline int eth01d_rh_ticks_to_per_cent_mille(int ticks)
{
    ticks &= ~0xC000; /* clear don't care bits */
    /*
     * Formula RH = ( ticks / ( 2^14 - 1 ) ) * 100
     */
    return DIV_ROUND_CLOSEST(ticks * 1000, 16383) * 100;
}

/**
 * eth01d_update_measurements() - get updated measurements from device
 * @dev: device
 *
 * Returns 0 on success, else negative errno.
 */
static int eth01d_update_measurements(struct device *dev)
{
    struct eth01d *eth01d = dev_get_drvdata(dev);
    struct i2c_client *client = eth01d->client;
    int ret = 0;
    int t;
    unsigned char tmp[4];
    struct i2c_msg msgs[1] = {
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = 4,
            .buf = tmp,
        }
    };

    mutex_lock(&eth01d->lock);

    /*
     * While the measurement can be completed in ~35ms the sensor takes
     * much longer to react to a change in external conditions. How quickly
     * it reacts depends on airflow and other factors outwith our control.
     * The datasheet specifies maximum 'Response time' for humidity at 6s
     * and temperature at 20s under specified conditions.
     * We therefore choose to only read the sensor at most once per 8 seconds.
     * This trades off pointless activity polling the sensor much faster
     * than it can react against better response times in conditions more
     * favourable than specified in the datasheet.
     */
    if (time_after(jiffies, eth01d->last_update + (8 * HZ)) || !eth01d->valid) {

        /*
         * Write to slave address to request a measurement.
         * According with the datasheet it should be with no data, but
         * for systems with I2C bus drivers that do not allow zero
         * length packets we write one dummy byte to allow sensor
         * measurements on them.
         */
        tmp[0] = 0;
        ret = i2c_master_send(client, tmp, eth01d->write_length);
        if (ret < 0)
            goto out;

        /* measurement cycle time is ~35msec */
        msleep(40);

        ret = i2c_transfer(client->adapter, msgs, 1);
        if (ret < 0)
            goto out;

        t = (tmp[0] << 8) + tmp[1];
        eth01d->humidity = eth01d_rh_ticks_to_per_cent_mille(t);

        t = (tmp[2] << 8) + tmp[3];
        eth01d->temperature = eth01d_temp_ticks_to_millicelsius(t);

        eth01d->last_update = jiffies;
        eth01d->valid = true;
    }
out:
    mutex_unlock(&eth01d->lock);

    return ret >= 0 ? 0 : ret;
}

/**
 * eth01d_temperature_show() - show temperature measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t eth01d_temperature_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    struct eth01d *eth01d = dev_get_drvdata(dev);
    int ret;

    ret = eth01d_update_measurements(dev);
    if (ret < 0)
        return ret;
    return sprintf(buf, "%d\n", eth01d->temperature);
}

/**
 * eth01d_humidity_show() - show humidity measurement value in sysfs
 * @dev: device
 * @attr: device attribute
 * @buf: sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t eth01d_humidity_show(struct device *dev,
                     struct device_attribute *attr, char *buf)
{
    struct eth01d *eth01d = dev_get_drvdata(dev);
    int ret;

    ret = eth01d_update_measurements(dev);
    if (ret < 0)
        return ret;
    return sprintf(buf, "%d\n", eth01d->humidity);
}

/* sysfs attributes */
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, eth01d_temperature_show, NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, eth01d_humidity_show, NULL, 0);

static struct attribute *eth01d_attrs[] = {
    &sensor_dev_attr_temp1_input.dev_attr.attr,
    &sensor_dev_attr_humidity1_input.dev_attr.attr,
    NULL
};

ATTRIBUTE_GROUPS(eth01d);

static int eth01d_probe(struct i2c_client *client,
                   const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct eth01d *eth01d;
    struct device *hwmon_dev;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "adapter does not support true I2C\n");
        return -ENODEV;
    }

    eth01d = devm_kzalloc(dev, sizeof(*eth01d), GFP_KERNEL);
    if (!eth01d)
        return -ENOMEM;

    eth01d->client = client;
    mutex_init(&eth01d->lock);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_QUICK))
        eth01d->write_length = 1;

    hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
                               eth01d,
                               eth01d_groups);
    return PTR_ERR_OR_ZERO(hwmon_dev);
}

/* Device ID table */
static const struct i2c_device_id eth01d_id[] = {
    { "eth01d", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, eth01d_id);

static const struct of_device_id __maybe_unused eth01d_of_match[] = {
    { .compatible = "econarae,eth01d", },
    { }
};
MODULE_DEVICE_TABLE(of, eth01d_of_match);

static struct i2c_driver eth01d_driver = {
    .driver = {
        .name = "eth01d",
        .of_match_table = of_match_ptr(eth01d_of_match),
    },
    .probe       = eth01d_probe,
    .id_table    = eth01d_id,
};

module_i2c_driver(eth01d_driver);

MODULE_AUTHOR("Gaurav Kalra <gvkalra@gmail.com>");
MODULE_DESCRIPTION("Econarae ETH-01D humidity and temperature sensor driver");
MODULE_LICENSE("GPL");
