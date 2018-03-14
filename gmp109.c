/* ST LSM6DS3H Accelerometer sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include <cust_baro.h>
#include <hwmsensor.h>
#include <barometer.h>
#include "gmp109.h"
//#include "gmp109_API.h"


/*---------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_GMP109_LOWPASS	/*apply low pass filter on output */
#define C_MAX_FIR_LENGTH (32)

/*----------------------------------------------------------------------------*/
#define GMP109_AXIS_X          0
#define GMP109_AXIS_Y          1
#define GMP109_AXIS_Z          2
#define GMP109_AXES_NUM        3
#define GMP109_DATA_LEN       6
#define GMP109_DEV_NAME        "GMP109_BAROMETER"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id gmp109_i2c_id[] = { {GMP109_DEV_NAME, 0}, {} };

static int gmp109_init_flag = -1;

static DECLARE_WAIT_QUEUE_HEAD(open_wq);

/*----------------------------------------------------------------------------*/
enum enum_ADX_TRC {
	ADX_TRC_FILTER = 0x01,
	ADX_TRC_RAWDATA = 0x02,
	ADX_TRC_IOCTL = 0x04,
	ADX_TRC_CALI = 0X08,
	ADX_TRC_INFO = 0X10,
};
#define ADX_TRC enum enum_ADX_TRC
/*----------------------------------------------------------------------------*/
enum ACCEL_TRC {
	ACCEL_TRC_FILTER = 0x01,
	ACCEL_TRC_RAWDATA = 0x02,
	ACCEL_TRC_IOCTL = 0x04,
	ACCEL_TRC_CALI = 0X08,
	ACCEL_TRC_INFO = 0X10,
	ACCEL_TRC_DATA = 0X20,
};
#define ACCEL_TRC enum enum_ACCEL_TRC
/*----------------------------------------------------------------------------*/
struct scale_factor {
	u8 whole;
	u8 fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
	struct scale_factor scalefactor;
	int sensitivity;
};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct data_filter {
	s16 raw[C_MAX_FIR_LENGTH][GMP109_AXES_NUM];
	int sum[GMP109_AXES_NUM];
	int num;
	int idx;
};

struct baro_hw baro_cust;
static struct baro_hw *hw = &baro_cust;
/* For  driver get cust info */
struct baro_hw *get_cust_baro(void)
{
	return &baro_cust;
}

/*----------------------------------------------------------------------------*/
struct gmp109_i2c_data {
	struct i2c_client *client;
	struct baro_hw *hw;
	struct hwmsen_convert cvt;
	atomic_t layout;
	/*misc */
	struct work_struct eint_work;
	atomic_t trace;
	atomic_t suspend;
	atomic_t selftest;
	atomic_t filter;
	s16 cali_sw[GMP109_AXES_NUM + 1];

	/*data */	
	s32 data[GMP109_AXES_NUM + 1];
	int sensitivity;
	u8 sample_rate;

#if defined(CONFIG_GMP109_LOWPASS)
	atomic_t firlen;
	atomic_t fir_en;
	struct data_filter fir;
#endif
};
/*----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------*/
static int gmp109_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int gmp109_i2c_remove(struct i2c_client *client);
static int GMP109_init_client(struct i2c_client *client, bool enable);
static int GMP109_SetMode(struct i2c_client *client, u8 mode);

static int GMP109_ReadBaroRawData(struct i2c_client *client, s32 data[GMP109_AXES_NUM]);
static int gmp109_suspend(struct device *dev);
static int gmp109_resume(struct device *dev);
//static int GMP109_SetSampleRate(struct i2c_client *client, u8 sample_rate);

#if 0
static int GMP109_Enable_Func(struct i2c_client *client, GMP109_ACC_GYRO_FUNC_EN_t newValue);
static int GMP109_Int_Ctrl(struct i2c_client *client, GMP109_ACC_GYRO_INT_ACTIVE_t int_act,
			     GMP109_ACC_GYRO_INT_LATCH_CTL_t int_latch);
#endif

static int gmp109_local_init(void);
static int gmp109_local_uninit(void);


static DEFINE_MUTEX(gmp109_init_mutex);
static DEFINE_MUTEX(gmp109_factory_mutex);


#ifdef CONFIG_OF
static const struct of_device_id baro_of_match[] = {
	{.compatible = "mediatek,barometer"},
	{},
};
#endif

#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops gmp109_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(gmp109_suspend, gmp109_resume)
};
#endif

static struct i2c_driver gmp109_i2c_driver = {
	.probe = gmp109_i2c_probe,
	.remove = gmp109_i2c_remove,

	.id_table = gmp109_i2c_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = GMP109_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		   .pm = &gmp109_pm_ops,
#endif
#ifdef CONFIG_OF
		   .of_match_table = baro_of_match,	/*need add in dtsi first*/
#endif
		   },
};

static struct baro_init_info gmp109_init_info = {
	.name = GMP109_DEV_NAME,
	.init = gmp109_local_init,
	.uninit = gmp109_local_uninit,
};

/*----------------------------------------------------------------------------*/
struct i2c_client *gmp109_acc_i2c_client;

static struct gmp109_i2c_data *obj_i2c_data;
static bool sensor_power;
static bool enable_status;


/*----------------------------------------------------------------------------*/

#define GSE_TAG                  "[GMP109]"

//#define GSE_FUN(f)               pr_debug(GSE_TAG"%s\n", __func__)
#define GSE_FUN(f)               pr_err(GSE_TAG"%s\n", __func__)
#define GSE_PR_ERR(fmt, args...)    pr_err(GSE_TAG "%s %d : " fmt, __func__, __LINE__, ##args)
//#define GSE_LOG(fmt, args...)    pr_debug(GSE_TAG "%s %d : " fmt, __func__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    pr_err(GSE_TAG "%s %d : " fmt, __func__, __LINE__, ##args)


static int mpu_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	int err = 0;
	u8 beg = addr;
	struct i2c_msg msgs[2] = { {0}, {0} };

	mutex_lock(&gmp109_init_mutex);

	if (!client) {
		mutex_unlock(&gmp109_init_mutex);
		return -EINVAL;
	} else if (len > C_I2C_FIFO_SIZE) {
		mutex_unlock(&gmp109_init_mutex);
		GSE_PR_ERR("length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data;

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != 2) {
		GSE_PR_ERR("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	} else
		err = 0;
	mutex_unlock(&gmp109_init_mutex);
	return err;

}

static int mpu_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{				/*because address also occupies one byte, the maximum length for write is 7 bytes */
	int err = 0;
	int idx = 0;
	int num = 0;
	char buf[C_I2C_FIFO_SIZE];

	mutex_lock(&gmp109_init_mutex);
	if (!client) {
		mutex_unlock(&gmp109_init_mutex);
		return -EINVAL;
	} else if (len >= C_I2C_FIFO_SIZE) {
		mutex_unlock(&gmp109_init_mutex);
		GSE_PR_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	num = 0;
	buf[num++] = addr;
	for (idx = 0; idx < len; idx++)
		buf[num++] = data[idx];

	err = i2c_master_send(client, buf, num);
	if (err < 0) {
		mutex_unlock(&gmp109_init_mutex);
		GSE_PR_ERR("i2c send command error!!\n");
		return -EFAULT;
	}
	mutex_unlock(&gmp109_init_mutex);
	return err;
}

int GMP109_hwmsen_read_block(u8 addr, u8 *buf, u8 len)
{
	if (gmp109_acc_i2c_client == NULL) {
		GSE_PR_ERR("GMP109_hwmsen_read_block null ptr!!\n");
		return GMP109_ERR_I2C;
	}
	return mpu_i2c_read_block(gmp109_acc_i2c_client, addr, buf, len);
}
EXPORT_SYMBOL(GMP109_hwmsen_read_block);

int GMP109_hwmsen_write_block(u8 addr, u8 *buf, u8 len)
{
	if (gmp109_acc_i2c_client == NULL) {
		GSE_PR_ERR("GMP109_hwmsen_write_block null ptr!!\n");
		return GMP109_ERR_I2C;
	}
	return mpu_i2c_write_block(gmp109_acc_i2c_client, addr, buf, len);
}
EXPORT_SYMBOL(GMP109_hwmsen_write_block);

/*static int GMP109_set_bank(struct i2c_client *client, u8 bank)
{
	int res = 0;
	u8 databuf[2];

	databuf[0] = bank;
	res = mpu_i2c_write_block(client, GMP109_WHO_AM_I, databuf, 0x1);
	if (res < 0) {
		GSE_PR_ERR("GMP109_set_bank fail at %x", bank);
		return GMP109_ERR_I2C;
	}

	return GMP109_SUCCESS;
}
*/
/*----------------------------------------------------------------------------*/

#if 1
static void GMP109_dumpReg(struct i2c_client *client)
{
	int i = 0;
	u8 addr = 0x10;
	u8 regdata = 0;

	for (i = 0; i < 25; i++) {
		/*dump all*/
		mpu_i2c_read_block(client, addr, &regdata, 0x01);
		GSE_LOG("Reg addr=%x regdata=%x\n", addr, regdata);
		addr++;
	}
}
#endif
static void GMP109_power(struct baro_hw *hw, unsigned int on)
{
	static unsigned int power_on;

	GSE_LOG("power %s\n", on ? "on" : "off");
#if 0
	if (power_on == on)	/*power status not change*/
		GSE_LOG("ignore power control: %d\n", on);
	else if (on) {		/* power on*/
		if (!hwPowerOn(hw->power_id, hw->power_vol, "LSM6DS3H"))
			GSE_PR_ERR("power on fails!!\n");
	} else {			/* power off*/
		if (!hwPowerDown(hw->power_id, "LSM6DS3H"))
			GSE_PR_ERR("power off fail!!\n");
	}
#endif
	power_on = on;
}

/*----------------------------------------------------------------------------*/

static int GMP109_write_rel_calibration(struct gmp109_i2c_data *obj, int dat[GMP109_AXES_NUM])
{
	obj->cali_sw[GMP109_AXIS_X] =
	    obj->cvt.sign[GMP109_AXIS_X] * dat[obj->cvt.map[GMP109_AXIS_X]];
	obj->cali_sw[GMP109_AXIS_Y] =
	    obj->cvt.sign[GMP109_AXIS_Y] * dat[obj->cvt.map[GMP109_AXIS_Y]];
	obj->cali_sw[GMP109_AXIS_Z] =
	    obj->cvt.sign[GMP109_AXIS_Z] * dat[obj->cvt.map[GMP109_AXIS_Z]];
#if DEBUG
	if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {
		GSE_LOG("test  (%5d, %5d, %5d) ->(%5d, %5d, %5d)->(%5d, %5d, %5d))\n",
			obj->cvt.sign[GMP109_AXIS_X], obj->cvt.sign[GMP109_AXIS_Y],
			obj->cvt.sign[GMP109_AXIS_Z], dat[GMP109_AXIS_X], dat[GMP109_AXIS_Y],
			dat[GMP109_AXIS_Z], obj->cvt.map[GMP109_AXIS_X],
			obj->cvt.map[GMP109_AXIS_Y], obj->cvt.map[GMP109_AXIS_Z]);
		GSE_LOG("write calibration data  (%5d, %5d, %5d)\n", obj->cali_sw[GMP109_AXIS_X],
			obj->cali_sw[GMP109_AXIS_Y], obj->cali_sw[GMP109_AXIS_Z]);
	}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int GMP109_ResetCalibration(struct i2c_client *client)
{
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int GMP109_ReadCalibration(struct i2c_client *client, int dat[GMP109_AXES_NUM])
{
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);

	dat[obj->cvt.map[GMP109_AXIS_X]] =
	    obj->cvt.sign[GMP109_AXIS_X] * obj->cali_sw[GMP109_AXIS_X];
	dat[obj->cvt.map[GMP109_AXIS_Y]] =
	    obj->cvt.sign[GMP109_AXIS_Y] * obj->cali_sw[GMP109_AXIS_Y];
	dat[obj->cvt.map[GMP109_AXIS_Z]] =
	    obj->cvt.sign[GMP109_AXIS_Z] * obj->cali_sw[GMP109_AXIS_Z];

#if DEBUG
	if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {
		GSE_LOG("Read calibration data  (%5d, %5d, %5d)\n",
			dat[GMP109_AXIS_X], dat[GMP109_AXIS_Y], dat[GMP109_AXIS_Z]);
	}
#endif

	return 0;
}

/*----------------------------------------------------------------------------*/

static int GMP109_WriteCalibration(struct i2c_client *client, int dat[GMP109_AXES_NUM])
{
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);
	int cali[GMP109_AXES_NUM];

	GSE_FUN();
	if (!obj || !dat) {
		GSE_PR_ERR("null ptr!!\n");
		return -EINVAL;
	}
		cali[obj->cvt.map[GMP109_AXIS_X]] =
		    obj->cvt.sign[GMP109_AXIS_X] * obj->cali_sw[GMP109_AXIS_X];
		cali[obj->cvt.map[GMP109_AXIS_Y]] =
		    obj->cvt.sign[GMP109_AXIS_Y] * obj->cali_sw[GMP109_AXIS_Y];
		cali[obj->cvt.map[GMP109_AXIS_Z]] =
		    obj->cvt.sign[GMP109_AXIS_Z] * obj->cali_sw[GMP109_AXIS_Z];
		cali[GMP109_AXIS_X] += dat[GMP109_AXIS_X];
		cali[GMP109_AXIS_Y] += dat[GMP109_AXIS_Y];
		cali[GMP109_AXIS_Z] += dat[GMP109_AXIS_Z];
#if DEBUG
		if (atomic_read(&obj->trace) & ACCEL_TRC_CALI) {
			GSE_LOG("write accel calibration data  (%5d, %5d, %5d)-->(%5d, %5d, %5d)\n",
				dat[GMP109_AXIS_X], dat[GMP109_AXIS_Y], dat[GMP109_AXIS_Z],
				cali[GMP109_AXIS_X], cali[GMP109_AXIS_Y],
				cali[GMP109_AXIS_Z]);
		}
#endif
		return GMP109_write_rel_calibration(obj, cali);

}

/*----------------------------------------------------------------------------*/

static int GMP109_SetRegister(struct i2c_client *client, u8 reg,u8 data)
{
	u8 databuf[2] = { data,0 };
	int res;
	res = mpu_i2c_write_block(client, reg, databuf, 0x1);
	if (res < 0) 
	{
		GSE_PR_ERR("read GMP109 register err!\n");
		return GMP109_ERR_I2C;
	}
	res = mpu_i2c_read_block(client, reg, databuf, 0x1);
	if (res < 0)
	{
		GSE_PR_ERR("read GMP109 register err!\n");
		return GMP109_ERR_I2C;
	}	
	GSE_LOG("Set Reg(0x%02x) to 0x%02x, result=0x%02x  \n",reg,data, databuf[0]);
	
    return GMP109_SUCCESS;
}



static int GMP109_SetMode(struct i2c_client *client,u8 mode)//force,contineous mode
{
	u8 databuf[2] = { mode,0 };
	int res;
	res = mpu_i2c_read_block(client, GMP109_CTRL1, databuf, 0x1);
	if (res < 0)
	{
		GSE_PR_ERR("read GMP109 CTRL1 register err!\n");
		return GMP109_ERR_I2C;
	}	
	databuf[0]|=(0x03&mode);
	res = mpu_i2c_write_block(client, GMP109_CTRL1, databuf, 0x1);
	if (res < 0)
	{
		GSE_PR_ERR("write GMP109 CTRL1 register err!\n");
		return GMP109_ERR_I2C;
	}
	res = mpu_i2c_read_block(client, GMP109_CTRL1, databuf, 0x1);
	if (res < 0)
	{
		GSE_PR_ERR("read GMP109 CTRL1 register err!\n");
		return GMP109_ERR_I2C;
	}	
	GSE_LOG("GMP109_CTRL1 :databuf[0] = 0x%02x \n", databuf[0]);
	
 	return GMP109_SUCCESS;
}



#if 0 //Steve 20180313
static int GMP109_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);	/*obj_i2c_data;*/

	if (enable == sensor_power) {
		GSE_LOG("Sensor power status is newest!\n");
		return GMP109_SUCCESS;
	}

	if (mpu_i2c_read_block(client, GMP109_CTRL1_XL, databuf, 0x01)) {
		GSE_PR_ERR("read lsm6ds3h power ctl register err!\n");
		return GMP109_ERR_I2C;
	}
	GSE_LOG("GMP109_CTRL1_XL:databuf[0] =  %x!\n", databuf[0]);


	if (true == enable) {
		databuf[0] &= ~GMP109_ACC_ODR_MASK;	/*clear lsm6ds3h  ODR bits*/
		databuf[0] |= obj->sample_rate;	/*GMP109_ACC_ODR_104HZ; //default set 100HZ for LSM6DS3H acc*/
	} else {
		/* do nothing*/
		databuf[0] &= ~GMP109_ACC_ODR_MASK;	/*clear lsm6ds3h acc ODR bits*/
		databuf[0] |= GMP109_ACC_ODR_POWER_DOWN;
	}
	/*databuf[1] = databuf[0];*/
	/*databuf[0] = GMP109_CTRL1_XL;*/
	res = mpu_i2c_write_block(client, GMP109_CTRL1_XL, databuf, 0x1);
	if (res < 0) {
		GSE_LOG("LSM6DS3H set power mode: ODR 100hz failed!\n");
		return GMP109_ERR_I2C;
	}
	GSE_LOG("set LSM6DS3H  power mode:ODR 100HZ ok %d!\n", enable);

	sensor_power = enable;

	return GMP109_SUCCESS;
}


/*----------------------------------------------------------------------------*/
static int GMP109_SetFullScale(struct i2c_client *client, u8 acc_fs)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);

	GSE_FUN();

	if (mpu_i2c_read_block(client, GMP109_CTRL1_XL, databuf, 0x01)) {
		GSE_PR_ERR("read GMP109_CTRL1_XL err!\n");
		return GMP109_ERR_I2C;
	}
	GSE_LOG("read  GMP109_CTRL1_XL register: 0x%x\n", databuf[0]);

	databuf[0] &= ~GMP109_ACC_RANGE_MASK;	/*clear*/
	databuf[0] |= acc_fs;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = GMP109_CTRL1_XL;*/

	res = mpu_i2c_write_block(client, GMP109_CTRL1_XL, databuf, 0x1);
	if (res < 0) {
		GSE_PR_ERR("write full scale register err!\n");
		return GMP109_ERR_I2C;
	}
	switch (acc_fs) {
	case GMP109_ACC_RANGE_2g:
		obj->sensitivity = GMP109_ACC_SENSITIVITY_2G;
		break;
	case GMP109_ACC_RANGE_4g:
		obj->sensitivity = GMP109_ACC_SENSITIVITY_4G;
		break;
	case GMP109_ACC_RANGE_8g:
		obj->sensitivity = GMP109_ACC_SENSITIVITY_8G;
		break;
	case GMP109_ACC_RANGE_16g:
		obj->sensitivity = GMP109_ACC_SENSITIVITY_16G;
		break;
	default:
		obj->sensitivity = GMP109_ACC_SENSITIVITY_2G;
		break;
	}

	if (mpu_i2c_read_block(client, GMP109_CTRL9_XL, databuf, 0x01)) {
		GSE_PR_ERR("read GMP109_CTRL9_XL err!\n");
		return GMP109_ERR_I2C;
	}
	GSE_LOG("read  GMP109_CTRL9_XL register: 0x%x\n", databuf[0]);

	databuf[0] &= ~GMP109_ACC_ENABLE_AXIS_MASK;	/*clear*/
	databuf[0] |=
	    GMP109_ACC_ENABLE_AXIS_X | GMP109_ACC_ENABLE_AXIS_Y | GMP109_ACC_ENABLE_AXIS_Z;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = GMP109_CTRL9_XL;*/

	res = mpu_i2c_write_block(client, GMP109_CTRL9_XL, databuf, 0x1);
	if (res < 0) {
		GSE_PR_ERR("write full scale register err!\n");
		return GMP109_ERR_I2C;
	}

	return GMP109_SUCCESS;
}

/*----------------------------------------------------------------------------*/
/* set the acc sample rate*/
static int GMP109_SetSampleRate(struct i2c_client *client, u8 sample_rate)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();
	/*set Sample Rate will enable power and should changed power status*/
	res = GMP109_SetPowerMode(client, true);
	if (res != GMP109_SUCCESS)
		return res;

	if (mpu_i2c_read_block(client, GMP109_CTRL1_XL, databuf, 0x01)) {
		GSE_PR_ERR("read acc data format register err!\n");
		return GMP109_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~GMP109_ACC_ODR_MASK;	/*clear*/
	databuf[0] |= sample_rate;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = GMP109_CTRL1_XL;*/

	res = mpu_i2c_write_block(client, GMP109_CTRL1_XL, databuf, 0x1);
	if (res < 0) {
		GSE_PR_ERR("write sample rate register err!\n");
		return GMP109_ERR_I2C;
	}

	return GMP109_SUCCESS;
}
#endif
#if 0
static int GMP109_Int_Ctrl(struct i2c_client *client, GMP109_ACC_GYRO_INT_ACTIVE_t int_act,
			     GMP109_ACC_GYRO_INT_LATCH_CTL_t int_latch)
{
	u8 databuf[2] = { 0 };
	int res = 0;
	u8 op_reg = 0;

	GSE_FUN();

	/*config latch int or no latch*/
	op_reg = GMP109_TAP_CFG;
	if (mpu_i2c_read_block(client, op_reg, databuf, 0x01)) {
		GSE_PR_ERR("%s read data format register err!\n", __func__);
		return GMP109_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~GMP109_ACC_GYRO_INT_LATCH_CTL_MASK;	/*clear*/
	databuf[0] |= int_latch;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = op_reg;*/
	res = mpu_i2c_write_block(client, op_reg, databuf, 0x01);
	if (res < 0) {
		GSE_PR_ERR("write enable tilt func register err!\n");
		return GMP109_ERR_I2C;
	}
	/* config high or low active*/
	op_reg = GMP109_CTRL3_C;
	if (mpu_i2c_read_block(client, op_reg, databuf, 0x01)) {
		GSE_PR_ERR("%s read data format register err!\n", __func__);
		return GMP109_ERR_I2C;
	}
	GSE_LOG("read  acc data format register: 0x%x\n", databuf[0]);

	databuf[0] &= ~GMP109_ACC_GYRO_INT_ACTIVE_MASK;	/*clear*/
	databuf[0] |= int_act;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = op_reg;*/
	res = mpu_i2c_write_block(client, op_reg, databuf, 0x1);
	if (res < 0) {
		GSE_PR_ERR("write enable tilt func register err!\n");
		return GMP109_ERR_I2C;
	}

	return GMP109_SUCCESS;
}

static int GMP109_Enable_Func(struct i2c_client *client, GMP109_ACC_GYRO_FUNC_EN_t newValue)
{
	u8 databuf[2] = { 0 };
	int res = 0;

	GSE_FUN();

	if (mpu_i2c_read_block(client, GMP109_CTRL10_C, databuf, 0x01)) {
		GSE_PR_ERR("%s read GMP109_CTRL10_C register err!\n", __func__);
		return GMP109_ERR_I2C;
	}
	GSE_LOG("%s read acc data format register: 0x%x\n", __func__, databuf[0]);
	databuf[0] &= ~GMP109_ACC_GYRO_FUNC_EN_MASK;	/*clear*/
	databuf[0] |= newValue;

	/*databuf[1] = databuf[0];*/
	/*databuf[0] = GMP109_CTRL10_C;*/
	res = mpu_i2c_write_block(client, GMP109_CTRL10_C, databuf, 0x01);
	if (res < 0) {
		GSE_PR_ERR("%s write GMP109_CTRL10_C register err!\n", __func__);
		return GMP109_ERR_I2C;
	}

	return GMP109_SUCCESS;
}
#endif

s16 GMP109_acc_TransfromResolution(s16 rawData, int sensitivity)
{
	s64 tempValue;
	s64 tempPlusValue = 0;

	tempValue = (s64) (rawData) * sensitivity * GRAVITY_EARTH_1000;
	if (tempValue < 0) {
		tempPlusValue = tempValue * -1;
		do_div(tempPlusValue, (1000 * 1000));
		tempValue = tempPlusValue * -1;
	} else {
		do_div(tempValue, (1000 * 1000));
	}

	return (s16) tempValue;
}

/*----------------------------------------------------------------------------*/
static int GMP109_ReadBaroData(struct i2c_client *client, char *buf, int bufsize)
{
	struct gmp109_i2c_data *obj;
	u8 databuf[20];
	int acc[GMP109_AXES_NUM];
	int res = 0;

	memset(databuf, 0, sizeof(u8) * 20);

	if (buf == NULL)
		return -1;
	if (client == NULL) {
		*buf = 0;
		return -2;
	}
	obj = (struct gmp109_i2c_data *)i2c_get_clientdata(client);

	res = GMP109_ReadBaroRawData(client, obj->data);
	if (res < 0) 
	{
		GSE_PR_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		sprintf(buf, "%x %x %x", obj->data[0], obj->data[1],obj->data[2]);
		GSE_LOG("GMP109_ReadBaroRawData result %s\n",buf);

		if (atomic_read(&obj->trace) & ADX_TRC_IOCTL) {	/*atomic_read(&obj->trace) & ADX_TRC_IOCTL*/
			/*GSE_LOG("gsensor data: %s!\n", buf);*/
			GSE_LOG("raw data:obj->data:%04x %04x %04x\n", obj->data[GMP109_AXIS_X],
				obj->data[GMP109_AXIS_Y], obj->data[GMP109_AXIS_Z]);
			GSE_LOG("acc:%04x %04x %04x\n", acc[GMP109_AXIS_X], acc[GMP109_AXIS_Y],
				acc[GMP109_AXIS_Z]);

			/*GMP109_dumpReg(client);*/
		}
	}
	return 0;
}

static int GMP109_ReadBaroRawData(struct i2c_client *client, s32 data[GMP109_AXES_NUM])
{
	int err = 0;
	char databuf[6] = { 0 };

	if (client == NULL)
		err = -EINVAL;
	else {
		
		GMP109_SetMode(client, GMP109_Force_mode);
		mdelay(10);
#if 0
		do{
			GSE_LOG("Set Force_mode\n");
			err = mpu_i2c_read_block(client, GMP109_STATUS, databuf, 0x1);
			if (err < 0)
			{
				GSE_PR_ERR("read GMP109 CTRL1 register err!\n");
				return GMP109_ERR_I2C;
			}	
			mdelay(1);
			GSE_LOG("DRDY=0x%02x)\n",databuf[0]);
		}while(!(databuf[0]&GMP109_DRDY_VALUE));
#endif		
		if (hwmsen_read_block(client, GMP109_TEMPH, databuf, 6)) 
		{
			GSE_PR_ERR("GMP109 read baro data  error\n");
			return -2;
		}
		else
		{
			data[0] =//pressure
			    (s32) ((databuf[3] << 16) |(databuf[4]<<8)| databuf[5] );
			data[1] =//temperature
			    (s32) ((databuf[0] << 16) |(databuf[1]<<8)| databuf[2] );
			data[2] =0;
			
			data[0]<<=8;data[0]>>=8;
			data[1]<<=8;data[1]>>=8;
			
			GSE_LOG("raw data:Temp=(%02x,%02x,%02x,0x%04x),Pressure=(%02x,%02x,%02x,0x%04x)\n",
				databuf[0],databuf[1],databuf[2],data[1],databuf[3],databuf[4],databuf[5],data[0]);
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int GMP109_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];

	memset(databuf, 0, sizeof(u8) * 10);

	if ((buf == NULL) || (bufsize <= 30))
		return -1;

	if (client == NULL) {
		*buf = 0;
		return -2;
	}

	sprintf(buf, "GMP109 02 Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = gmp109_acc_i2c_client;
	char strbuf[GMP109_BUFSIZE];

	if (client == NULL) {
		GSE_PR_ERR("i2c client is null!!\n");
		return 0;
	}
	GSE_FUN();

	GMP109_ReadChipInfo(client, strbuf, GMP109_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = gmp109_acc_i2c_client;
	char strbuf[GMP109_BUFSIZE];
	int x, y, z;

	if (client == NULL) {
		GSE_PR_ERR("i2c client is null!!\n");
		return 0;
	}

	GMP109_ReadBaroData(client, strbuf, GMP109_BUFSIZE);
	if (sscanf(strbuf, "%x %x %x", &x, &y, &z) != 3)
		GSE_LOG("get data format error\n");
	return snprintf(buf, PAGE_SIZE, "%d, %d, %d\n", x, y, z);
}

static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = gmp109_acc_i2c_client;
	s32 data[GMP109_AXES_NUM] = { 0 };

	if (client == NULL) {
		GSE_PR_ERR("i2c client is null!!\n");
		return 0;
	}

	GMP109_ReadBaroRawData(client, data);
	return snprintf(buf, PAGE_SIZE, "%x,%x,%x\n", data[0], data[1], data[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct gmp109_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct gmp109_i2c_data *obj = obj_i2c_data;
	int trace;

	if (obj == NULL) {
		GSE_PR_ERR("i2c_data obj is null!!\n");
		return count;
	}

	if (kstrtos32(buf, 10, &trace) == 0)
		atomic_set(&obj->trace, trace);
	else
		GSE_PR_ERR("invalid content: '%s', length = %zu\n", buf, count);

	return count;
}

static ssize_t show_chipinit_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct gmp109_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct gmp109_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_PR_ERR("i2c_data obj is null!!\n");
		return count;
	}

	GMP109_init_client(obj->client, true);
	GMP109_dumpReg(obj->client);

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	struct gmp109_i2c_data *obj = obj_i2c_data;

	if (obj == NULL) {
		GSE_PR_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	if (obj->hw) {
		len +=
		    snprintf(buf + len, PAGE_SIZE - len,
			     "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n",
			     obj->hw->i2c_num, obj->hw->direction, obj->sensitivity,
			     obj->hw->power_id, obj->hw->power_vol);
		GMP109_dumpReg(obj->client);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "CUST: NULL\n");
	}
	return len;
}

static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct gmp109_i2c_data *data = obj_i2c_data;

	if (data == NULL) {
		GSE_PR_ERR("gmp109_i2c_data is null!!\n");
		return -1;
	}

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		       data->hw->direction, atomic_read(&data->layout), data->cvt.sign[0],
		       data->cvt.sign[1], data->cvt.sign[2], data->cvt.map[0], data->cvt.map[1],
		       data->cvt.map[2]);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int layout = 0;
	struct gmp109_i2c_data *data = obj_i2c_data;

	if (data == NULL) {
		GSE_PR_ERR("gmp109_i2c_data is null!!\n");
		return count;
	}

	if (kstrtos32(buf, 10, &layout) == 0) {
		atomic_set(&data->layout, layout);
		if (!hwmsen_get_convert(layout, &data->cvt)) {
			GSE_PR_ERR("HWMSEN_GET_CONVERT function error!\r\n");
		} else if (!hwmsen_get_convert(data->hw->direction, &data->cvt)) {
			GSE_PR_ERR("invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		} else {
			GSE_PR_ERR("invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		GSE_PR_ERR("invalid format = '%s'\n", buf);
	}

	return count;
}

/*----------------------------------------------------------------------------*/

static DRIVER_ATTR(chipinfo, S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensorrawdata, S_IRUGO, show_sensorrawdata_value, NULL);
static DRIVER_ATTR(sensordata, S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(trace, S_IWUSR | S_IRUGO, show_trace_value, store_trace_value);
static DRIVER_ATTR(chipinit, S_IWUSR | S_IRUGO, show_chipinit_value, store_chipinit_value);
static DRIVER_ATTR(status, S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(layout, S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *GMP109_attr_list[] = {
	&driver_attr_chipinfo,	/*chip information */
	&driver_attr_sensordata,	/*dump sensor data */
	&driver_attr_sensorrawdata,	/*dump sensor raw data */
	&driver_attr_trace,	/*trace log */
	&driver_attr_status,
	&driver_attr_chipinit,
	&driver_attr_layout,
};

/*----------------------------------------------------------------------------*/
static int gmp109_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)ARRAY_SIZE(GMP109_attr_list);

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, GMP109_attr_list[idx]);
		if (err != 0) {
			GSE_PR_ERR("driver_create_file (%s) = %d\n",
				GMP109_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static int gmp109_delete_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(ARRAY_SIZE(GMP109_attr_list));

	if (driver == NULL)
		return -EINVAL;

	for (idx = 0; idx < num; idx++)
		driver_remove_file(driver, GMP109_attr_list[idx]);
	return err;
}


/*----------------------------------------------------------------------------*/
static int GMP109_init_client(struct i2c_client *client, bool enable)
{
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);
	GSE_FUN();
	GSE_LOG(" gmp109 addr 0x%02x!\n", client->addr);
#if 0
	res = GMP109_CheckDeviceID(client);
	if (res != GMP109_SUCCESS)
		return res;

	res = GMP109_Set_RegInc(client, true);
	if (res != GMP109_SUCCESS)
		return res;

	res = GMP109_SetFullScale(client, GMP109_ACC_RANGE_2g);	/*we have only this choice*/
	if (res != GMP109_SUCCESS)
		return res;
	res = GMP109_SetSampleRate(client, obj->sample_rate);
	if (res != GMP109_SUCCESS)
		return res;

	res = GMP109_SetPowerMode(client, enable);
	if (res != GMP109_SUCCESS)
		return res;
#endif
  
	GSE_LOG("GMP109_init_client OK!\n");
	/*acc setting*/

#ifdef CONFIG_GMP109_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));
#endif

	return GMP109_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int gmp109_open_report_data(int open)
{
	/*should queuq work to report event if  is_report_input_direct=true*/
	return 0;
}

/* if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL*/
static int gmp109_enable_nodata(int en)
{
	int value = en;
	int err = 0;
	struct gmp109_i2c_data *priv = obj_i2c_data;

	if (priv == NULL) {
		GSE_PR_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}

	if (value == 1) {
		enable_status = true;
	} else {
		enable_status = false;
		priv->sample_rate = 0x20;	/*default rate*/
	}
	GSE_LOG("enable value=%d, sensor_power =%d\n", value, sensor_power);

	if (((value == 0) && (sensor_power == false)) || ((value == 1) && (sensor_power == true)))
		GSE_LOG("Gsensor device have updated!\n");
	

	GSE_LOG("%s OK!\n", __func__);
	return err;
}

static int gmp109_set_delay(u64 ns)
{
	int value = 0;
//	int err = 0;
//	u8 sample_delay = 0;
	struct gmp109_i2c_data *priv = obj_i2c_data;

	value = (int)ns / 1000000LL;

	if (priv == NULL) {
		GSE_PR_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	priv->sample_rate = 66;

	if (value >= 50)
		atomic_set(&priv->filter, 0);
	else {
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[GMP109_AXIS_X] = 0;
		priv->fir.sum[GMP109_AXIS_Y] = 0;
		priv->fir.sum[GMP109_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	}
	GSE_LOG("%s delay=%d ms\n", __func__, value);

	return 0;
}

static int gmp109_batch(int flag, int64_t samplingPeriodNs, int64_t maxBatchReportLatencyNs)
{
	int value = 0;

	value = (int)samplingPeriodNs / 1000 / 1000;

	GSE_LOG("gmp109 barometer set delay = (%d) ok.\n", value);
	return gmp109_set_delay(samplingPeriodNs);
}

static int gmp109_flush(void)
{
	return baro_flush_report();
}

static int gmp109_get_data(int *x, int *status)
{
	char buff[GMP109_BUFSIZE];
	int y,z;
	struct gmp109_i2c_data *priv = obj_i2c_data;
	if (priv == NULL) {
		GSE_PR_ERR("obj_i2c_data is NULL!\n");
		return -1;
	}
	if (atomic_read(&priv->trace) & ACCEL_TRC_DATA)
		GSE_LOG("%s (%d),\n", __func__, __LINE__);

	memset(buff, 0, sizeof(buff));
	GMP109_ReadBaroData(priv->client, buff, GMP109_BUFSIZE);
	GSE_LOG("buff=%s size of int=%d\n",buff,sizeof(int));

	if (sscanf(buff, "%x %x %x", x, &y, &z) != 3)
		GSE_LOG("get data format error\n");
//	else
	*status = SENSOR_STATUS_ACCURACY_HIGH;
	GSE_LOG("Pressure=%d,status=%d\n",*x,*status);

	return 0;
}

#if 0				/*ioctl related */
/******************************************************************************
 * Function Configuration
******************************************************************************/
static int gmp109_open(struct inode *inode, struct file *file)
{
	file->private_data = gmp109_acc_i2c_client;

	if (file->private_data == NULL) {
		GSE_PR_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int gmp109_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long gmp109_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client *)file->private_data;
	struct gmp109_i2c_data *obj = (struct gmp109_i2c_data *)i2c_get_clientdata(client);
	char strbuf[GMP109_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];

	/*GSE_FUN(f);*/
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));


	if (err) {
		GSE_PR_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch (cmd) {
	case GSENSOR_IOCTL_INIT:
		break;

	case GSENSOR_IOCTL_READ_CHIPINFO:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		GMP109_ReadChipInfo(client, strbuf, GMP109_BUFSIZE);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_SENSORDATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		GMP109_ReadBaroData(client, strbuf, GMP109_BUFSIZE);

		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_READ_GAIN:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		break;

	case GSENSOR_IOCTL_READ_OFFSET:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}

		GMP109_ReadBaroRawData(client, (s16 *) strbuf);
		if (copy_to_user(data, strbuf, strlen(strbuf) + 1)) {
			err = -EFAULT;
			break;
		}
		break;

	case GSENSOR_IOCTL_SET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		if (copy_from_user(&sensor_data, data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		if (atomic_read(&obj->suspend)) {
			GSE_PR_ERR("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		} else {
#if 0
			cali[GMP109_AXIS_X] =
			    (s64) (sensor_data.x) * 1000 * 1000 / (obj->sensitivity * GRAVITY_EARTH_1000);	/*NTC*/
			cali[GMP109_AXIS_Y] =
			    (s64) (sensor_data.y) * 1000 * 1000 / (obj->sensitivity *
								   GRAVITY_EARTH_1000);
			cali[GMP109_AXIS_Z] =
			    (s64) (sensor_data.z) * 1000 * 1000 / (obj->sensitivity *
								   GRAVITY_EARTH_1000);
#else
			cali[GMP109_AXIS_X] = (s64) (sensor_data.x);
			cali[GMP109_AXIS_Y] = (s64) (sensor_data.y);
			cali[GMP109_AXIS_Z] = (s64) (sensor_data.z);
#endif
			err = GMP109_WriteCalibration(client, cali);
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		err = GMP109_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		data = (void __user *)arg;
		if (data == NULL) {
			err = -EINVAL;
			break;
		}
		err = GMP109_ReadCalibration(client, cali);
		if (err < 0)
			break;
#if 0
		sensor_data.x =
		    (s64) (cali[GMP109_AXIS_X]) * obj->sensitivity * GRAVITY_EARTH_1000 / (1000 * 1000);	/*NTC*/
		sensor_data.y =
		    (s64) (cali[GMP109_AXIS_Y]) * obj->sensitivity * GRAVITY_EARTH_1000 / (1000 *
											     1000);
		sensor_data.z =
		    (s64) (cali[GMP109_AXIS_Z]) * obj->sensitivity * GRAVITY_EARTH_1000 / (1000 *
											     1000);
#else
		sensor_data.x = (s64) (cali[GMP109_AXIS_X]);
		sensor_data.y = (s64) (cali[GMP109_AXIS_Y]);
		sensor_data.z = (s64) (cali[GMP109_AXIS_Z]);
#endif
		if (copy_to_user(data, &sensor_data, sizeof(sensor_data))) {
			err = -EFAULT;
			break;
		}
		break;

	default:
		GSE_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long gmp109_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long err = 0;

	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;

	switch (cmd) {
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA,
					       (unsigned long)arg32);
		if (err) {
			GSE_PR_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_SET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_PR_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_GET_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		if (err) {
			GSE_PR_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
		if (arg32 == NULL) {
			err = -EINVAL;
			break;
		}

		err =
		    file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		if (err) {
			GSE_PR_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
			return err;
		}
		break;

	default:
		GSE_PR_ERR("unknown IOCTL: 0x%08x\n", cmd);
		err = -ENOIOCTLCMD;
		break;

	}

	return err;
}
#endif

/*----------------------------------------------------------------------------*/
static const struct file_operations gmp109_acc_fops = {
	.owner = THIS_MODULE,
	.open = gmp109_open,
	.release = gmp109_release,
	.unlocked_ioctl = gmp109_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gmp109_compat_ioctl,
#endif
};

/*----------------------------------------------------------------------------*/
static struct miscdevice gmp109_acc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &gmp109_acc_fops,
};
#endif				/*ioctl related */
/*----------------------------------------------------------------------------*/

static int gmp109_factory_enable_sensor(bool enabledisable, int64_t sample_periods_ms)
{
	int err;
#if 0
	err = bmi160_acc_enable_nodata(enabledisable == true ? 1 : 0);
	if (err) {
		GSE_PR_ERR("%s enable sensor failed!\n", __func__);
		return -1;
	}
	err = bmi160_acc_batch(0, sample_periods_ms * 1000000, 0);
	if (err) {
		GSE_PR_ERR("%s enable set batch failed!\n", __func__);
		return -1;
	}
#endif
	err = GMP109_init_client(gmp109_acc_i2c_client, 0);
	if (err) {
		GSE_PR_ERR("initialize client fail!!\n");
		return -1;
	}
	return 0;
}

static int gmp109_factory_get_data(int32_t *data)//(int32_t data[3], int *status)
{
	int err = 0;
	int status;
#if 0
	return bmi160_acc_get_data(&data[0], &data[1], &data[2], status);
#endif
	mutex_lock(&gmp109_factory_mutex);
	if (sensor_power == false) {
		//err = GMP109_SetPowerMode(gmp109_acc_i2c_client, true);
		if (err)
			GSE_PR_ERR("Power on lsm6ds3h error %d!\n", err);

		msleep(50);
	}
	mutex_unlock(&gmp109_factory_mutex);

	return gmp109_get_data(data, &status);
}

static int gmp109_factory_get_raw_data(int32_t *data)
{
	s32 strbuf[3];

	GMP109_ReadBaroRawData(gmp109_acc_i2c_client, strbuf);
	data[0] = strbuf[0];
	data[1] = strbuf[1];
	data[2] = strbuf[2];

	return 0;
}

static int gmp109_factory_enable_calibration(void)
{
	return 0;
}

static int gmp109_factory_clear_cali(void)
{
	int err = 0;

	/* err = BMI160_ACC_ResetCalibration(obj_data); */
	err = GMP109_ResetCalibration(gmp109_acc_i2c_client);
	if (err) {
		GSE_PR_ERR("gmp109_ResetCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int gmp109_factory_set_cali(int32_t data)
{
	int err = 0;
	int cali[3] = { 0 };
	cali[GMP109_AXIS_X] = data;
	cali[GMP109_AXIS_Y] = data;
	cali[GMP109_AXIS_Z] = data;
	err = GMP109_WriteCalibration(gmp109_acc_i2c_client, cali);
	if (err) {
		GSE_PR_ERR("GMP109_WriteCalibration failed!\n");
		return -1;
	}
	return 0;
}

static int gmp109_factory_get_cali(int32_t *data)
{
	int err = 0;
	int cali[3] = { 0 };
#if 0
	err = BMI160_ACC_ReadCalibration(obj_data, cali);
	if (err) {
		GSE_PR_ERR("bmi160_ReadCalibration failed!\n");
		return -1;
	}
	data[0] = cali[BMI160_ACC_AXIS_X]
	    * GRAVITY_EARTH_1000 / obj_data->reso->sensitivity;
	data[1] = cali[BMI160_ACC_AXIS_Y]
	    * GRAVITY_EARTH_1000 / obj_data->reso->sensitivity;
	data[2] = cali[BMI160_ACC_AXIS_Z]
	    * GRAVITY_EARTH_1000 / obj_data->reso->sensitivity;
#endif
	err = GMP109_ReadCalibration(gmp109_acc_i2c_client, cali);
	if (err) {
		GSE_PR_ERR("GMP109_ReadCalibration failed!\n");
		return -1;
	}
	*data = cali[GMP109_AXIS_X];
	//data[0] = cali[GMP109_AXIS_Y];
	//data[2] = cali[GMP109_AXIS_Z];
	return 0;
}

static int gmp109_factory_do_self_test(void)
{
	return 0;
}

static struct baro_factory_fops gmp109_factory_fops = {
	.enable_sensor = gmp109_factory_enable_sensor,
	.get_data = gmp109_factory_get_data,
	.get_raw_data = gmp109_factory_get_raw_data,
	.enable_calibration = gmp109_factory_enable_calibration,
	.clear_cali = gmp109_factory_clear_cali,
	.set_cali = gmp109_factory_set_cali,
	.get_cali = gmp109_factory_get_cali,
	.do_self_test = gmp109_factory_do_self_test,	
};

static struct baro_factory_public gmp109_factory_device = {
	.gain = 1,
	.sensitivity = 1,
	.fops = &gmp109_factory_fops,
};

/*----------------------------------------------------------------------------*/

#ifdef CONFIG_PM_SLEEP
static int gmp109_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	if (obj == NULL) {
		GSE_PR_ERR("null pointer!!\n");
		return -EINVAL;
	}
	atomic_set(&obj->suspend, 1);
	
	GMP109_SetMode(obj->client, GMP109_Standby_mode);

	sensor_power = false;

	GMP109_power(obj->hw, 0);

	return err;
}

/*----------------------------------------------------------------------------*/
static int gmp109_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gmp109_i2c_data *obj = i2c_get_clientdata(client);
	GSE_FUN();

	if (obj == NULL) {
		GSE_PR_ERR("null pointer!!\n");
		return -1;
	}
	GMP109_SetRegister(obj->client,GMP109_RESET,GMP109_RESET_COMMAND);

	GMP109_power(obj->hw, 1);

	atomic_set(&obj->suspend, 0);

	return 0;
}
#endif
/*
static int gmp109_enable(int en)
{
	int value = 0;
	int err = 0;
	struct gmp109_i2c_data *f_obj = obj_i2c_data;

	if (f_obj == NULL)
		return -1;

	value = en;
	//factory_mode = 1;
	if (value == 1) {
		f_obj->enable = true;
		
	} else {
		f_obj->enable = false;		
	}
	if (f_obj->flush) {
		if (value == 1) {
			MAGN_LOG("will call gmc306_flush in gmc306_enable\n");
			barohub_flush();
		} else
			f_obj->flush = false;
	}
	wake_up(&open_wq);
	return err;
}
*/
/*----------------------------------------------------------------------------*/
static int gmp109_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client = NULL;
	struct gmp109_i2c_data *obj = NULL;
	struct baro_control_path ctl_path = { 0 };
	struct baro_data_path data_path = { 0 };
	int err = 0;

	GSE_FUN();
	err = get_baro_dts_func(client->dev.of_node, hw);
	if (err) {
		GSE_PR_ERR("get dts info fail\n");
		err = -EFAULT;
		goto exit;
	}
	GMP109_power(hw, 1);

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj) {
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(struct gmp109_i2c_data));

	obj->hw = hw;
	obj->sample_rate = 0x40;	/*104HZ??*/

	/*atomic_set(&obj->layout, obj->hw->direction);
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if (err) {
		GSE_PR_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}*/

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client, obj);

	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	init_waitqueue_head(&open_wq);

	gmp109_acc_i2c_client = new_client;
	err = GMP109_init_client(new_client, false);	/*??*/
	if (err)
		goto exit_init_failed;


	/* err = misc_register(&gmp109_acc_device); */		   
	err = baro_factory_device_register(&gmp109_factory_device);
	if (err) {
		GSE_PR_ERR("acc_factory register failed!\n");
		goto exit_misc_device_register_failed;
	}

	/*mutex_lock(&gmp109_init_mutex);*/

	ctl_path.is_use_common_factory = true;
	err = gmp109_create_attr(&(gmp109_init_info.platform_diver_addr->driver));
	if (err < 0)
		goto exit_create_attr_failed;


	ctl_path.open_report_data = gmp109_open_report_data;
	//ctl_path.enable = gmp109_enable;
	ctl_path.enable_nodata = gmp109_enable_nodata;
	ctl_path.set_delay = gmp109_set_delay;
	
	ctl_path.batch = gmp109_batch;
	ctl_path.flush = gmp109_flush;
	ctl_path.is_report_input_direct = false;
	ctl_path.is_support_batch = obj->hw->is_batch_supported;

	err = baro_register_control_path(&ctl_path);
	if (err) {
		GSE_PR_ERR("register acc control path err\n");
		goto exit_init_failed;
	}

	data_path.get_data = gmp109_get_data;
	data_path.vender_div = 1600;
	err = baro_register_data_path(&data_path);
	if (err) {
		GSE_PR_ERR("register acc data path err= %d\n", err);
		goto exit_init_failed;
	}
/*mutex_unlock(&gmp109_init_mutex);*/

	gmp109_init_flag = 0;
	GSE_LOG("%s: OK\n", __func__);
	return 0;

exit_create_attr_failed:
	/*misc_deregister(&gmp109_acc_device); */
exit_misc_device_register_failed:
exit_init_failed:
//exit_kfree:
	kfree(obj);
exit:
	gmp109_init_flag = -1;
	GSE_PR_ERR("%s: err = %d\n", __func__, err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int gmp109_i2c_remove(struct i2c_client *client)
{
	int err = 0;

	GMP109_power(hw, 0);

	/*gmp109_init_flag = -1;*/
	err = gmp109_delete_attr(&(gmp109_init_info.platform_diver_addr->driver));
	if (err)
		GSE_PR_ERR("gmp109_i2c_remove fail: %d\n", err);


	/*misc_deregister(&gmp109_acc_device); */
	baro_factory_device_deregister(&gmp109_factory_device);

	gmp109_acc_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/

static int gmp109_local_uninit(void)
{
	/*GSE_FUN();*/
	GMP109_power(hw, 0);
	i2c_del_driver(&gmp109_i2c_driver);
	return 0;
}

static int gmp109_local_init(void)
{
	/*GSE_FUN();*/

#if 0				/* sensors-1.0 */
	GMP109_power(hw, 1);	/*??*/
#endif

	if (i2c_add_driver(&gmp109_i2c_driver)) {
		GSE_PR_ERR("add driver error\n");
		return -1;
	}
	if (gmp109_init_flag == -1) {
		GSE_PR_ERR("%s init failed!\n", __func__);
		return -1;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init gmp109_init(void)
{
	baro_driver_add(&gmp109_init_info);

	return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit gmp109_exit(void)
{
	GSE_FUN();
}

/*----------------------------------------------------------------------------*/
module_init(gmp109_init);
module_exit(gmp109_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GMP109 BAROMETER");
MODULE_AUTHOR("Yue.Wu@mediatek.com");
