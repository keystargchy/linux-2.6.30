/*
 *  linux/drivers/i2c/chips/ds1337.c 
 * 
 *  Copyright (C) 2005 James Chapman <jchapman@katalix.com> 
 * 
 *      based on linux/drivers/acorn/char/pcf8583.c 
 *  Copyright (C) 2000 Russell King 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation. 
 * 
 * Driver for Dallas Semiconductor DS1337 and DS1339 real time clock chip 
 * 
 *   History: 
 *      2007/8/11 Port by WenBinWu for PNX8550,add interface for user application; 
 *      2007/9/12 add source comment by liuym for ds3231 chipset     
 *      2008/4/24 add read temperature funtion for ds3231 chipset    
 */  
 
#include <linux/module.h>  
#include <linux/kernel.h>   
#include <linux/init.h>  
#include <linux/slab.h>  
#include <linux/i2c.h>  
#include <linux/string.h>  
#include <linux/rtc.h>          /* get the user-level API */  
#include <linux/bcd.h>  
#include <linux/list.h>  
#include <linux/fs.h>  
#include <linux/miscdevice.h>  
#include <asm/uaccess.h>  
#include <linux/ioctl.h>  
#include <linux/major.h>  
#include <linux/errno.h>  
#include <linux/device.h>
   
/* Device registers */  
#define DS3231_REG_SECOND          0x00  
#define DS3231_REG_MINUTE          0x01  
#define DS3231_REG_HOUR            0x02    /*x [12/24] 2 1 | 8 4 2 1 */  
#define DS3231_REG_DAY             0x03  
#define DS3231_REG_DATE            0x04  
#define DS3231_REG_MONTH           0x05  
#define ds3231_REG_YEAR        0x06     /* 8  4 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL1_SECOND  0x07    /* A1M1 4 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL1_MIN     0x08    /* A1M2 4 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL1_HR      0x09    /* A1M3 [12/24] 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL1_DAY     0x0a    /* A1M4 [DY/DT] 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL2_MIN     0x0b    /* A2M2 4 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL2_HR      0x0c    /* A2M3 [12/24] 2 1 | 8 4 2 1 */  
#define ds3231_REG_AL2_DAY     0x0d    /* A2M4 [DY/DT] 2 1 | 8 4 2 1 */  
#define DS3231_REG_CONTROL         0x0e    /*ESOC x x RS2 RS1 INTCN A2IE A1IE*/  
#define DS3231_REG_STATUS          0x0f    /*OSF X X X X X A1F A2F*/  
 
#define DS3231_REG_TEMP_HI     0x11  
#define DS3231_REG_TEMP_LO 0x12  
 
/* Control reg */  
#define ds3231_CTRL_A1IE       (1<<0)  
#define ds3231_CTRL_A2IE       (1<<1)  
#define ds3231_CTRL_INTCN      (1<<2)  
#define ds3231_CTRL_RS         (1<<3)  
#define ds3231_CTRL_EOSC       (1<<7)  
 
/* Status reg */  
#define ds3231_STATUS_A1F      (1<<0)  
#define ds3231_STATUS_A2F      (1<<1)  
#define ds3231_STATUS_OSF      (1<<7)  
 
/* CLKOUT frequencies , RS*/  
#define ds3231_FD_32768HZ      (0x3)  
#define ds3231_FD_1024HZ       (0x2)  
#define ds3231_FD_32           (0x1)  
#define ds3231_FD_1HZ          (0x0)  
 
#define DEVNAME   "rtc"  
   


static struct i2c_client *ds3231_client;
  
/* 
 * Internal variables 
 */  

static inline int ds3231_read(struct i2c_client *client, u8 reg, u8 *value)  
{  
	s32 tmp = i2c_smbus_read_byte_data(client, reg);  
	if (tmp < 0)  
		return -EIO;  
  	
	*value = tmp;  
	return 0;  
}  

static inline int ds3231_write (struct i2c_client *client, u8 reg, u8 value)  
{
	return i2c_smbus_write_byte_data(client, reg, value);  
}

static int ds3231_get_datetime(struct i2c_client *client, struct rtc_time *dt)  
{  
	u8 buf[7];  
	u8 val;  
 
	int err = 0;  
	u8 i = 0;  
        
	if (!dt) {  
		dev_dbg(&client->dev, "%s: EINVAL: dt=NULL\n", __FUNCTION__);  
		return -EINVAL;  
	}  
     

	dev_dbg(&client->dev, "%s: RTC not running!\n", __FUNCTION__);  
     
	for (i=0; i<7; i++) {  
		err = ds3231_read(client, i, &buf[i]);  
		if (err != 0)  
			break;  
	}  
        
	dev_dbg(&client->dev, "%s: [%d] %02x %02x %02x %02x %02x %02x %02x\n",  
				__FUNCTION__, err, buf[0], buf[1], buf[2], buf[3],  
                buf[4], buf[5], buf[6]);  
        
	if (err == 0) {  
		dt->tm_sec = bcd2bin(buf[0]);  
		dt->tm_min = bcd2bin(buf[1]);  
		val = buf[2] & 0x3f;
		dt->tm_hour = bcd2bin(val);  
		dt->tm_wday = bcd2bin(buf[3]) - 1;
		dt->tm_mday = bcd2bin(buf[4]);  
		val = buf[5] & 0x7f;  
		dt->tm_mon = bcd2bin(val) - 1; 
		dt->tm_year = bcd2bin(buf[6]);  
                
	if (buf[5] & 0x80)  
		dt->tm_year += 100;  
               
		dev_dbg(&client->dev, "%s: secs=%d, mins=%d, "  
                "hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",  
				__FUNCTION__, dt->tm_sec, dt->tm_min,  
				dt->tm_hour, dt->tm_mday,  
				dt->tm_mon, dt->tm_year, dt->tm_wday);  
		return 0;  
	}  
	dev_err(&client->dev, "error reading data! %d\n", err);  
	return -EIO;  
}  
  
static int ds3231_set_datetime(struct i2c_client *client, struct rtc_time *dt)  
{  
	u8 buf[7];  
	u8 val;  
	u8 i = 0;  
  
	if (!dt) {  
		dev_dbg(&client->dev, "%s: EINVAL: dt=NULL\n", __FUNCTION__);  
		return -EINVAL;  
	}  
  
	dev_dbg(&client->dev, "%s: secs=%d, mins=%d, hours=%d, "  
				"mday=%d, mon=%d, year=%d, wday=%d\n", __FUNCTION__,  
				dt->tm_sec, dt->tm_min, dt->tm_hour,  
				dt->tm_mday, dt->tm_mon, dt->tm_year, dt->tm_wday);  
	buf[0] = bin2bcd(dt->tm_sec);  
	buf[1] = bin2bcd(dt->tm_min);  
	buf[2] = bin2bcd(dt->tm_hour);  
	buf[3] = bin2bcd(dt->tm_wday + 1);  
	buf[4] = bin2bcd(dt->tm_mday);  
	buf[5] = bin2bcd(dt->tm_mon + 1);  
	val = dt->tm_year;  
	if (val >= 100) {  
		val -= 100;  
		buf[5] |= (1 << 7);  
	}  
        
	buf[6] = bin2bcd(val);  
      
	for (i=0; i<7; i++)  {  
		ds3231_write(client, i, buf[i]);  
	}  
      
	return 0;  
}  
 
static int ds3231_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)  
{
	struct rtc_time rtc_tm;  
   
	switch (cmd) {  
	case RTC_RD_TIME:  
		memset(&rtc_tm, 0, sizeof (struct rtc_time));  
		if (ds3231_get_datetime(ds3231_client, &rtc_tm))  
			return -EFAULT;  
		if (copy_to_user((struct rtc_time*)arg, &rtc_tm, sizeof(struct rtc_time)))  
			return -EFAULT;  
		break;  
	case RTC_SET_TIME:  
		if (copy_from_user(&rtc_tm, (struct rtc_time*)arg, sizeof(struct rtc_time)))  
			return -EFAULT;  
		if (ds3231_set_datetime(ds3231_client, &rtc_tm))  
			return -EFAULT;  
		break;  
	break;  
	case RTC_ALM_READ:  
	case RTC_ALM_SET:  
	break;  
	default:  
		return -EINVAL;  
	}  
   
	return 0;  
}

/*
 *	file operations for device file register though ds3231_MiscDev,
 *	which is defined below
 */
static struct file_operations ds3231_fops    =   {  
   	.owner          =   THIS_MODULE,  
   	.read           =   NULL,  
   	.open           =   NULL,  
   	.release        =   NULL,  
   	.ioctl          =   ds3231_ioctl,  
};
 
/*
 *	structure used in dynamically register and unregister device , 
 *	search for "misc_register" and "misc_deregister" 
 */  
static struct miscdevice ds3231_MiscDev = {
	.minor =   MISC_DYNAMIC_MINOR,  
    .name  =   DEVNAME,  
    .fops  =   &ds3231_fops  
};

static void ds3231_init_client(struct i2c_client *client)  
{  
	u8 status = 0, control = 0;
	u8 val = 0;  
   
	ds3231_read(client, DS3231_REG_STATUS, &status);  
	ds3231_read(client, DS3231_REG_CONTROL,&control);  
          
	if ((status & 0x80) || (control & 0x80)) {  
		u8 i = 0;  
	
		dev_dbg(&client->dev, "%s: RTC not running!\n", __FUNCTION__);  
            
		for (i=14; i<16; i++)   
			ds3231_write(client, i, 0);  
	} else {  
		dev_dbg(&client->dev, "RTC running!\n");  
	
		ds3231_read(client, DS3231_REG_HOUR, &val);  
		if ((val & (1 << 6)))  
			ds3231_write(client, DS3231_REG_HOUR, val & 0x3f);  
	}  
} 

static int ds3231_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct i2c_adapter  *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
        return -EIO;

	ds3231_client = client;
	
	/* Register a misc device called "ds3231". */  
	ret = misc_register(&ds3231_MiscDev);
	if (ret) 
		goto exit_misc_register;
    
	ds3231_init_client(client);
		
	return ret;  

exit_misc_register:
	return ret;
  
}

static int __devexit ds3231_remove(struct i2c_client *client)
{
	int ret;
	
	ret = misc_deregister(&ds3231_MiscDev);
	if (ret)
		dev_info(&client->dev, 
					"DS3231: could not misc_deregister the device\n");  
	return ret;  
}

static const struct i2c_device_id ds3231_id[] = {
    { "ds3231", 0 },
    { }
};


static struct i2c_driver ds3231_driver = {
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "rtc-ds3231",
    },
    .probe  = ds3231_probe,
    .remove = __devexit_p(ds3231_remove),
    .id_table = ds3231_id,
};

  
static int __init ds3231_init(void)  
{  
	return i2c_add_driver(&ds3231_driver);  
}  
  
static void __exit ds3231_exit(void)  
{  
	i2c_del_driver(&ds3231_driver);  
}  
  
  
MODULE_AUTHOR("James Chapman <jchapman@katalix.com>");  
MODULE_DESCRIPTION("DS3231 RTC driver");  
MODULE_LICENSE("GPL");  
  
  
module_init(ds3231_init);  
module_exit(ds3231_exit);  
  
