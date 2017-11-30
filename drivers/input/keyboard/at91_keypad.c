/*
* Copyright (C) 2003 Nokia Corporation
 * Written by Timo Ter盲s <ext-timo.teras@nokia.com>
 *
*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
//#include <asm/arch/gpio.h>
//#include <asm/arch/keypad.h>
#include <mach/gpio.h>
#include <mach/keypad.h>
#include <mach/hardware.h>

#include <asm/irq.h>
//#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>

static int kp_enable = 1;
//static int kp_cur_group = -1;

struct atmel_kp {
    struct input_dev *input;
    struct timer_list timer;
    int irq;
    unsigned int rows;
    unsigned int cols;
    unsigned long delay;
};

static void atmel_kp_tasklet(unsigned long data);


DECLARE_TASKLET_DISABLED(kp_tasklet, atmel_kp_tasklet, 0);

static unsigned char keypad_state[6];
static DEFINE_MUTEX(kp_enable_mutex);

static int *keymap;
static unsigned int *row_gpios;
static unsigned int *col_gpios;


static void set_col_gpio_val(struct atmel_kp *kp, u8 value)
{
    int col;
    for (col = 0; col < kp->cols; col++) {
        if (value & (1 << col))
            at91_set_gpio_output(col_gpios[col], 1);
        else
            at91_set_gpio_output(col_gpios[col], 0);
    }
}

static u8 get_row_gpio_val(struct atmel_kp *kp)
{
    int row;
    u8 value = 0;

    for (row = 0; row < kp->rows; row++) {
        if (at91_get_gpio_value(row_gpios[row]))
            value |= (1 << row);
    }
    return value;
}

static void atmel_kp_scan_keypad(struct atmel_kp *kp, unsigned char *state)
{
    int col = 0;
    
    /* read the keypad status */
    for (col = 0; col < kp->cols; col++) {
        set_col_gpio_val(kp, ~(1 << col));
        udelay(20);
        state[col] = (~(get_row_gpio_val(kp))) & 0x3f;
    }
    set_col_gpio_val(kp, 0);
}

static inline int atmel_kp_find_key(int col, int row)
{
    int i, key;

    key = KEY(col, row, 0);
    for (i = 0; keymap[i] != 0; i++)
        if ((keymap[i] & 0xff000000) == key)
            return keymap[i] & 0x00ffffff;
    return -1;
}

static void atmel_kp_timer(unsigned long data)
{
    tasklet_schedule(&kp_tasklet);
}

static void atmel_kp_tasklet(unsigned long data)
{
    struct atmel_kp *atmel_kp_data = (struct atmel_kp *) data;  
    unsigned char new_state[6], changed, key_down = 0;
    int /*i, */col, row;
    int spurious = 0;
    
    /* check for any changes */
    atmel_kp_scan_keypad(atmel_kp_data, new_state);     
    
    /* check for changes and print those */
    for (col = 0; col < atmel_kp_data->cols; col++) {
        changed = new_state[col] ^ keypad_state[col];
#if 0
        key_down |= new_state[col]; 
#else
        key_down = new_state[col] & ((1<<atmel_kp_data->rows)-1);
#endif
        
        if (changed == 0)
            continue;
        for (row = 0; row < atmel_kp_data->rows; row++) {
            int key;
            if (!(changed & (1 << row)))
                continue;
            key = atmel_kp_find_key(col, row);
            if (key < 0) {
                printk(KERN_WARNING
                      "atmel-keypad: Spurious key event %d-%d\n",
                       col, row);
                /* We scan again after a couple of seconds */
                spurious = 1;
                continue;
            }
            input_report_key(atmel_kp_data->input, key & ~GROUP_MASK, new_state[col] & (1 << row)); 
        }
    }

    memcpy(keypad_state, new_state, sizeof(keypad_state));

    if (key_down) {
        int delay = HZ / 20;
        /* some key is pressed - keep irq disabled and use timer
         * to poll the keypad */
        if (spurious)
            delay = 2 * HZ;
        mod_timer(&atmel_kp_data->timer, jiffies + delay);
    } else {
        /* enable interrupts */
#if 0
        for (i = 0; i < atmel_kp_data->rows; i++)
            enable_irq(row_gpios[i]);
#endif
    }
}


static irqreturn_t atmel_kp_interrupt(int irq, void *dev_id)
{
#if 0
    int i;
    struct atmel_kp *kp = dev_id;
    
    for (i = 0; i < kp->rows; i++) {
        disable_irq(row_gpios[i]);
    }
#endif
    tasklet_schedule(&kp_tasklet);

    return IRQ_HANDLED;
}

static ssize_t atmel_kp_enable_show(struct device *dev,
                   struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%u\n", kp_enable);
}

static ssize_t atmel_kp_enable_store(struct device *dev, struct device_attribute *attr,
                    const char *buf, size_t count)
{
    int state;

#if 0
    struct atmel_kp * kp = dev_get_drvdata(dev);
#endif
    if (sscanf(buf, "%u", &state) != 1)
        return -EINVAL;

    if ((state != 1) && (state != 0))
        return -EINVAL;

    mutex_lock(&kp_enable_mutex);
    if (state != kp_enable) {
#if 0
        if (state)
            enable_irq(kp->irq);
        else
            disable_irq(kp->irq);
#endif
        kp_enable = state;
    }
    mutex_unlock(&kp_enable_mutex);

    return strnlen(buf, count);
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, atmel_kp_enable_show, atmel_kp_enable_store);


#ifdef CONFIG_PM
static int atmel_kp_suspend(struct platform_device *dev, pm_message_t state)
{
    /* Nothing yet */

    return 0;
}

static int atmel_kp_resume(struct platform_device *dev)
{
    /* Nothing yet */

    return 0;
}
#else
#define atmel_kp_suspend    NULL
#define atmel_kp_resume NULL
#endif

static int __init atmel_kp_probe(struct platform_device *pdev)
{
    struct atmel_kp *kp;
    struct input_dev *input_dev;
    struct atmel_kp_platform_data *pdata =  pdev->dev.platform_data;
    int i, irq_idx,  col_idx, row_idx,  ret;

    if (!pdata->rows || !pdata->cols || !pdata->keymap) {
        printk(KERN_ERR "No rows, cols or keymap from pdata\n");
        return -EINVAL;
    }

    kp = kzalloc(sizeof(struct atmel_kp), GFP_KERNEL);
    input_dev = input_allocate_device();
    if (!kp || !input_dev) {
        kfree(kp);
        input_free_device(input_dev);
        return -ENOMEM;
    }

    platform_set_drvdata(pdev, kp);

    kp->input = input_dev;

    /* Disable the interrupt for the MPUIO keyboard */
    keymap = pdata->keymap;

    if (pdata->rep)
        __set_bit(EV_REP, input_dev->evbit);

    if (pdata->delay)
        kp->delay = pdata->delay;

    if (pdata->row_gpios && pdata->col_gpios) {
        row_gpios = pdata->row_gpios;
        col_gpios = pdata->col_gpios;
    }

    kp->rows = pdata->rows;
    kp->cols = pdata->cols;

    /* Cols: outputs */
    for (col_idx = 0; col_idx < kp->cols; col_idx++)
        at91_set_gpio_output(col_gpios[col_idx], 0);
    
        /* Rows: inputs */
    for (row_idx = 0; row_idx < kp->rows; row_idx++) {
        at91_set_gpio_input(row_gpios[row_idx], 1);
        at91_set_deglitch(row_gpios[row_idx], 1);
    }

    setup_timer(&kp->timer, atmel_kp_timer, (unsigned long)kp);

    /* get the irq and init timer*/
    tasklet_enable(&kp_tasklet);
    kp_tasklet.data = (unsigned long) kp;

    ret = device_create_file(&pdev->dev, &dev_attr_enable);
    if (ret < 0)
        goto err1;

    /* setup input device */
    __set_bit(EV_KEY, input_dev->evbit);
    for (i = 0; keymap[i] != 0; i++)
        __set_bit(keymap[i] & KEY_MAX, input_dev->keybit);
    input_dev->name = "atmel-keypad";
    input_dev->phys = "atmel-keypad/input0";
    input_dev->dev.parent = &pdev->dev;

    input_dev->id.bustype = BUS_HOST;
    input_dev->id.vendor = 0x0001;
    input_dev->id.product = 0x0001;
    input_dev->id.version = 0x0100;

    ret = input_register_device(kp->input);
    if (ret < 0) {
        printk(KERN_ERR "Unable to register atmle-keypad input device\n");
        goto err2;
    }

    /* scan current status and enable interrupt */
    atmel_kp_scan_keypad(kp, keypad_state);
    
    for (irq_idx = 0; irq_idx < kp->rows; irq_idx++) {
        int irq = gpio_to_irq(row_gpios[irq_idx]);
        if (irq < 0) {
            pr_err("atmel-keypad: Unable to get irq number for GPIO %d, error %d\n", row_gpios[irq_idx], irq);
            goto err3;
        }
#if 0
        if (request_irq(row_gpios[irq_idx], atmel_kp_interrupt, IRQF_TRIGGER_LOW,
#else
        if (request_irq(irq, atmel_kp_interrupt, IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
            "atmel-keypad", kp) < 0)
        goto err3;
    }

    return 0;
err3:
    input_unregister_device(kp->input);
    input_dev = NULL;
err2:
    device_remove_file(&pdev->dev, &dev_attr_enable);
err1:
    kfree(kp);
    input_free_device(input_dev);

    return -EINVAL;
}

static int atmel_kp_remove(struct platform_device *pdev)
{
    int i;
    struct atmel_kp *kp = platform_get_drvdata(pdev);

    device_remove_file(&pdev->dev, &dev_attr_enable);

    /* disable keypad interrupt handling */
    tasklet_disable(&kp_tasklet);
    
    del_timer_sync(&kp->timer);
    tasklet_kill(&kp_tasklet);

    /* unregister everything */
    input_unregister_device(kp->input);

    for (i = 0; i < kp->rows; i++) 
        free_irq(row_gpios[i], kp);

    kfree(kp);

    return 0;
}

static struct platform_driver atmel_kp_driver = {
    .probe      = atmel_kp_probe,
    .remove     = atmel_kp_remove,
    .suspend    = atmel_kp_suspend,
    .resume     = atmel_kp_resume,
    .driver     = {
        .name   = "atmel-keypad",
    },
};


static int __devinit atmel_kp_init(void)
{
    printk(KERN_INFO "Atmel Keypad Driver\n");
    return platform_driver_register(&atmel_kp_driver);
}

static void __exit atmel_kp_exit(void)
{
    platform_driver_unregister(&atmel_kp_driver);
}

module_init(atmel_kp_init);
module_exit(atmel_kp_exit);

MODULE_DESCRIPTION("Atmel Keypad Driver");
MODULE_LICENSE("GPL");
