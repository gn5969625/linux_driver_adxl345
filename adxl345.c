#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include "adxl345.h"
#define DRV_VERSION "V1.0"
#define ADXL345_DRIVER_NAME "adxl345"
#define ADXL345_SCALE_AVAIL "0.038358 0.038339 0.038330 0.038325"
// * IIO driver for adxl345; 7-bit I2C address: 0x53.
/*
 * The accelerometer has two measurement ranges:
 *
 * -2g - +2g (10-bit, signed)
 * -4g - +4g (11-bit, signed)
 * -8g - +8g (12-bit, signed)
 * -16g - +16g (13-bit, signed)
 *
 * scale1 = (2 + 2) * 9.81 / (2^10 - 1)     = 0.038358
 * scale1 = (4 + 4) * 9.81 / (2^11 - 1)     = 0.038339
 * scale2 = (8 + 8) * 9.81 / (2^12 - 1)     = 0.038330
 * scale3 = (16 + 16) * 9.81 / (2^13 - 1)   = 0.038325
 */

static const int adxl345_scale_table [][2] = {
	{0,38358},{0,38339},{0,38330},{0,38325}
};

static const struct {
        int val;
        int val2;
} adxl345_samp_freq_table[] = {
        {3200, 0}, {1800, 0}, {800, 0}, {400, 0}, {200, 0}, {100, 0}, {50, 0}, {25, 0},
        {12, 500000}, {6, 250000}
};
static IIO_CONST_ATTR(in_accel_scale_available, ADXL345_SCALE_AVAIL);
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("6.25 12.5 25 50 100 200 400 800 1600 3200");
static struct attribute *adxl345_attributes[] = {
        &iio_const_attr_in_accel_scale_available.dev_attr.attr,
        &iio_const_attr_sampling_frequency_available.dev_attr.attr,
        NULL,
};
static const struct attribute_group adxl345_attribute_group = {
        .attrs = adxl345_attributes
};

struct adxl345_data {
	struct i2c_client *client;
	struct mutex lock;
};
//just for debug test
static struct i2c_client *adxl345_client;
static struct mutex test_lock;
//

#define ADXL345_ACCEL_CHANNEL(index, reg, axis) {                       \
        .type = IIO_ACCEL,                                              \
        .address = reg,                                                 \
        .modified = 1,                                                  \
        .channel2 = IIO_MOD_##axis,                                     \
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                   \
        .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |          \
                                    BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
        .scan_index = index,                                            \
        .scan_type = {                                                  \
                .sign = 's',                                            \
                .realbits = 13,                                         \
                .storagebits = 16,                                      \
		.shift = 3,                                             \
                .endianness = IIO_CPU,                                  \
        },                                                              \
}
//just for debug
static ssize_t adxl345_show_raw(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char raw_data;
	unsigned short datax,datay,dataz;
	mutex_lock(&test_lock);
	datax = i2c_smbus_read_word_data(to_i2c_client(dev),ADXL345_REG_DATAX0); //data address x :16 bit 
	datay = i2c_smbus_read_word_data(to_i2c_client(dev),ADXL345_REG_DATAY0); //data address y :16 bit
	dataz = i2c_smbus_read_word_data(to_i2c_client(dev),ADXL345_REG_DATAZ0); //data address z :16 bit
	mutex_unlock(&test_lock);
	return sprintf(buf,"datax = %x,datay = %x,dataz = %x\n", datax, datay, dataz);
}

static DEVICE_ATTR(adxl345_raw, 0644 , adxl345_show_raw, NULL);
static struct attribute *adxl345_attrs[] = {
    &dev_attr_adxl345_raw.attr,
    NULL
};
static struct attribute_group adxl345_attr_group = {
    .name = "adxl345_sysfs",
    .attrs = adxl345_attrs,
};

static int adxl345_read_accel(struct adxl345_data *data,u8 address) {
	int ret;
	struct i2c_client *client = data->client;
	ret = i2c_smbus_read_word_data(client, address);
	if(ret < 0)
		dev_err(&client->dev, "register read failed\n");
	return ret;
}
static int adxl345_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask) {
	struct adxl345_data *data = iio_priv(indio_dev);
	int ret;
	switch(mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);
		ret = adxl345_read_accel(data,chan->address);
		*val = ret;
		mutex_unlock(&data->lock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		//temporary hardcode set,because our init
		*val = adxl345_scale_table[3][0];
		*val2 = adxl345_scale_table[3][1];	
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		//temporary hardcode set,device default ,ref datasheet page 25/40 for Register 0x2C
		*val = adxl345_samp_freq_table[5].val;
		*val2 = adxl345_samp_freq_table[5].val2;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}
static int adxl345_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask) {
	struct adxl345_data *data = iio_priv(indio_dev);
	int i,ret,index;
	index = -1;

	switch(mask) {
	case IIO_CHAN_INFO_SCALE:
		for(i = 0;i < ARRAY_SIZE(adxl345_scale_table);i++) {
			if (val == adxl345_scale_table[i][0] && val2 == adxl345_scale_table[i][1]) {
				index++;
				break;
			}

		}

		if(index < -1)
			return -EINVAL;

		mutex_lock(&data->lock);
		//temporary no operation
		ret = 1;
		mutex_unlock(&data->lock);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		for(i = 0;i < ARRAY_SIZE(adxl345_samp_freq_table);i++) {
                        if (val == adxl345_samp_freq_table[i].val && val2 == adxl345_samp_freq_table[i].val2) {
                                index++;
                                break;
                        }

                }

		if(index < -1)
                	return -EINVAL;
		mutex_lock(&data->lock);
		//temporary no operation
		ret = 1;
                mutex_unlock(&data->lock);
		return ret;

	}
	return -EINVAL;
}

static const struct iio_chan_spec adxl345_channels[] = {
	ADXL345_ACCEL_CHANNEL(0, ADXL345_REG_DATAX0, X),
	ADXL345_ACCEL_CHANNEL(1, ADXL345_REG_DATAY0, Y),
	ADXL345_ACCEL_CHANNEL(2, ADXL345_REG_DATAZ0, Z),
};
static const struct iio_info adxl345_info = {
	.read_raw      = adxl345_read_raw,
	.write_raw     = adxl345_write_raw,
	.driver_module = THIS_MODULE,
	.attrs         = &adxl345_attribute_group,
};


static int adxl345_dev_init(void)
{
	unsigned char res;
	printk("%s called\n", __func__);
	res = i2c_smbus_write_byte_data(adxl345_client, ADXL345_REG_DATA_FORMAT, 0x0B); //+-16g,13 bit mode
	res = i2c_smbus_write_byte_data(adxl345_client, ADXL345_REG_POWER_CTL, 0x08); //start measurement
	res = i2c_smbus_write_byte_data(adxl345_client, ADXL345_REG_INT_ENABLE, 0x80); //enable data_ready interrupt
	res = i2c_smbus_read_byte_data(adxl345_client, ADXL345_REG_DEVID); //adxl345 device id
	if( res == ADXL345_DEVICE_ID) {
		printk("init adxl345 is ok and device id is %x\n",res);
	} 
	return 0;
}

static int adxl345_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//struct proc_dir_entry *file;
	int ret;
	struct iio_dev *indio_dev;
	struct adxl345_data *data;
	indio_dev = iio_device_alloc(sizeof(*data));
	data = iio_priv(indio_dev);
	data->client = i2c;
	i2c_set_clientdata(i2c, indio_dev);
	mutex_init(&data->lock);
	indio_dev->dev.parent = &i2c->dev;
	indio_dev->info = &adxl345_info;
	indio_dev->name = ADXL345_DRIVER_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxl345_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl345_channels);

	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	//just for debug
	mutex_init(&test_lock);
	adxl345_client = i2c;
	//
	adxl345_dev_init();
	printk("adxl345 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &adxl345_attr_group);
	ret = iio_device_register(indio_dev);
	return 0;
}

static int adxl345_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &adxl345_attr_group);
	iio_device_unregister(i2c_get_clientdata(i2c));
	iio_device_free(i2c_get_clientdata(i2c));
	return 0;
}

static const struct i2c_device_id adxl345_id[] = {  
    { "adxl345", 0},
    {}
};
MODULE_DEVICE_TABLE(i2c, adxl345_id);

static struct of_device_id adxl345_of_match[] = {
        { .compatible = "analogd,adxl345"},
        { }
};
MODULE_DEVICE_TABLE(of, adxl345_of_match);

struct i2c_driver adxl345_driver = {
    .driver = {
        .name           = ADXL345_DRIVER_NAME,
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(adxl345_of_match),
    },
    .probe      = adxl345_probe,
    .remove     = adxl345_remove,
    .id_table   = adxl345_id,
};

module_i2c_driver(adxl345_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-adxl345 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
