/**
 * 9G45 Gerneral Purpose Backup Register
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <linux/fs.h>

#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/at91sam9g45.h>

#define DEVICE_NAME "gpbr"

enum GPBR_IOCTL_CMD {
    GPBR_SET_CHANNEL,
};

static int ch = 0;  /* channel number */
static unsigned int *gpbr_base;

static int gpbr_open (struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t gpbr_read (struct file *filp, char *buffer, size_t count, loff_t *ppos)
{
    int ret = 0;
    unsigned int data;

    data = at91_sys_read (AT91_GPBR + 4 * ch);

    ret = copy_to_user (buffer, &data, sizeof(int));
    if (ret < 0) {
        printk (KERN_ERR "%s: copy_to_user fail\n", DEVICE_NAME);
        return -EPERM;
    }

    return sizeof(int);
}

static ssize_t gpbr_write (struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
    int ret = 0;
    unsigned int data;

    ret = copy_from_user ((char *)&data, (char *)buffer, sizeof(int));
    if (ret < 0) {
        printk (KERN_ERR "%s: copy_from_user fail\n", DEVICE_NAME);
        return -EPERM;
    }

    at91_sys_write (AT91_GPBR + 4 * ch, data);

    return sizeof(int);
}

static long gpbr_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
    switch (cmd) {

        case GPBR_SET_CHANNEL:
            ch = arg & 3;
        break;
        default:
            printk (KERN_ERR "%s, ioctl cmd not support\n", DEVICE_NAME);
        break;
    }
    return 0;
}

static int gpbr_release (struct inode *inode, struct file *filp)
{
    return 0;
}

static const struct file_operations gpbr_fops = {
    .owner  = THIS_MODULE,
    .open   = gpbr_open,
    .read   = gpbr_read,
    .write  = gpbr_write,
    .unlocked_ioctl = gpbr_ioctl,
    .release    = gpbr_release,
};

static struct miscdevice gpbr_miscdev = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = DEVICE_NAME,
    .fops   = &gpbr_fops,
};

static int __init atmel_gpbr_init (void)
{
    int ret = 0;

    ret = misc_register (&gpbr_miscdev);
    if (ret) {
        printk (KERN_ERR "cannot register miscdev on minor=%d (%d)\n", MISC_DYNAMIC_MINOR, ret);
        goto out;
    }

    printk (KERN_INFO "atmel %s initialized\n", DEVICE_NAME);

    return 0;
out1:
    misc_deregister(&gpbr_miscdev);
out:
    return ret;
}

static void __exit atmel_gpbr_exit (void)
{
    misc_deregister(&gpbr_miscdev);
}

module_init (atmel_gpbr_init);
module_exit (atmel_gpbr_exit);
