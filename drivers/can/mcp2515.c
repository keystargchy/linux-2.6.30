/*
 * Microchip MCP2515 CAN controller driver.
 *
 * Copyright (C) 2007 Embedall Technology Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/can/can.h>
#include <linux/delay.h>
#include <linux/can/mcp251x.h>
#include <linux/semaphore.h>


/* SPI interface instruction set */
#define INSTRUCTION_WRITE       0x02
#define INSTRUCTION_READ        0x03
#define INSTRUCTION_BIT_MODIFY  0x05
#define INSTRUCTION_LOAD_TXB(n) (0x40 + 2 * (n))
#define INSTRUCTION_READ_RXB(n) (0x90 + 4 * (n))
#define INSTRUCTION_RESET       0xc0
#define INSTRUCTION_RTS(n)	(0x80 + (1<<n))
#define INSTRUCTION_RX_STATE    0xb0
#  define RX_STATE_RTR		 0x04
#  define RX_STATE_IDE		 0x08
#define INSTRUCTION_CAN_STATE   0xa0
#  define CAN_STATE_TX2IF	 0x01
#  define CAN_STATE_TX2REQ 	 0x02
#  define CAN_STATE_TX1IF	 0x04
#  define CAN_STATE_TX1REQ       0x08
#  define CAN_STATE_TX0IF        0x10
#  define CAN_STATE_TX0REQ 	 0x20
#  define CAN_STATE_RX1IF	 0x40
#  define CAN_STATE_RX0IF	 0x80

/* MPC251x registers */
#define CANSTAT           0x0e
#define CANCTRL           0x0f
#  define CANCTRL_REQOP_MASK        0xe0
#  define CANCTRL_REQOP_CONF        0x80
#  define CANCTRL_REQOP_LISTEN_ONLY 0x60
#  define CANCTRL_REQOP_LOOPBACK    0x40
#  define CANCTRL_REQOP_SLEEP       0x20
#  define CANCTRL_REQOP_NORMAL      0x00
#  define CANCTRL_OSM               0x08
#  define CANCTRL_ABAT              0x10
#define TEC           0x1c
#define REC           0x1d
#define CNF1          0x2a
#define CNF2          0x29
#  define CNF2_BTLMODE  0x80
#define CNF3          0x28
#  define CNF3_SOF      0x08
#  define CNF3_WAKFIL   0x04
#  define CNF3_PHSEG2_MASK 0x07
#define CANINTE       0x2b
#  define CANINTE_MERRE 0x80
#  define CANINTE_WAKIE 0x40
#  define CANINTE_ERRIE 0x20
#  define CANINTE_TX2IE 0x10
#  define CANINTE_TX1IE 0x08
#  define CANINTE_TX0IE 0x04
#  define CANINTE_RX1IE 0x02
#  define CANINTE_RX0IE 0x01
#define CANINTF       0x2c
#  define CANINTF_MERRF 0x80
#  define CANINTF_WAKIF 0x40
#  define CANINTF_ERRIF 0x20
#  define CANINTF_TX2IF 0x10
#  define CANINTF_TX1IF 0x08
#  define CANINTF_TX0IF 0x04
#  define CANINTF_RX1IF 0x02
#  define CANINTF_RX0IF 0x01
#define EFLG          0x2d
#  define EFLG_RX1OVR   0x80
#  define EFLG_RX0OVR   0x40
#define TXBCTRL(n)  ((n * 0x10) + 0x30)
#  define TXBCTRL_TXREQ  0x08
#  define TXBCTRL_TXPRI(n) (n)
#  define TXBCTRL_TXERR	   (1 << 4)
#  define TXBCTRL_MLOA     (1 << 5)
#  define TXBCTRL_ABTF     (1 << 6)
#define RXBCTRL(n)  ((n * 0x10) + 0x60)
#  define RXBCTRL_MASK   0x60
#  define RXBCTRL_RXRTR  0x08
#  define RXBCTRL_BULK	(1 << 2)
#  define RXBCTRL_RXM_MACH_ALL	(0 << 6)
#  define RXBCTRL_RXM_MACH_STD	(1 << 6)
#  define RXBCTRL_RXM_MACH_EXT	(2 << 6)
#  define RXBCTRL_TXM_MACH_OFF	(3 << 6)
#  define RXBCTRL_FILHIT_MASK    0x07
#define RXM_BASE(n)   (0x20 + (n *  4))
#define RXF_BASE(n)   ((n>2)?(0x10 + (n-3)*4):(0x00 + (n*4)))
	
	

#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0

#define BTLMODE_CNF3    0x80

#define SEG1            0x00
#define SEG2            0x01
#define SEG3            0x02
#define SEG4            0x03
#define SEG5            0x04
#define SEG6            0x05
#define SEG7            0x06
#define SEG8            0x07

#define BRP1            0x00
#define BRP2            0x01
#define BRP3            0x02
#define BRP4            0x03
#define BRP5            0x04
#define BRP6            0x05
#define BRP7            0x06
#define BRP8            0x07

//#define CAN_FOSC_12MHZ
#define CAN_FOSC_16MHZ

#if defined(CAN_FOSC_12MHZ)
#define CAN_CNF1_10K		(SJW3 | 0x1d)
#define CAN_CNF2_10K		(BTLMODE_CNF3|(SEG6<<3)|SEG7)
#define CAN_CNF3_10K		(SEG6)

#define CAN_CNF1_20K		(SJW2 | 0x0E)
#define CAN_CNF2_20K		(BTLMODE_CNF3|(SEG6<<3)|SEG7)
#define CAN_CNF3_20K		(SEG6)

#define CAN_CNF1_50K		(SJW1 | 0x05)
#define CAN_CNF2_50K		(BTLMODE_CNF3|(SEG6<<3)|SEG7)
#define CAN_CNF3_50K		(SEG6)

#define CAN_CNF1_100K		(SJW1 | BRP3)
#define CAN_CNF2_100K		(BTLMODE_CNF3|(SEG6<<3)|SEG7)
#define CAN_CNF3_100K		(SEG6)

#define CAN_CNF1_125K		(SJW1 | BRP3)
#define CAN_CNF2_125K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_125K		(SEG4)

#define CAN_CNF1_250K		(SJW1 | BRP2)
#define CAN_CNF2_250K		(BTLMODE_CNF3|(SEG3<<3)|SEG5)
#define CAN_CNF3_250K		(SEG3)

#define CAN_CNF1_500K		(SJW1 | BRP1)
#define CAN_CNF2_500K		(BTLMODE_CNF3|(SEG3<<3)|SEG5)
#define CAN_CNF3_500K		(SEG3)

#define CAN_CNF1_750K		(SJW1 | BRP1)
#define CAN_CNF2_750K		(BTLMODE_CNF3|(SEG3<<2)|SEG1)
#define CAN_CNF3_750K		(SEG3)

#define CAN_CNF1_1000K  	(SJW1 | BRP1)
#define CAN_CNF2_1000K  	(BTLMODE_CNF3|(SEG2<<3)|SEG1)
#define CAN_CNF3_1000K		(SEG2)
#elif defined(CAN_FOSC_16MHZ)
#define CAN_CNF1_10K		(SJW1 | 0x31)
#define CAN_CNF2_10K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_10K		(SEG4)

#define CAN_CNF1_20K		(SJW1 | 0x18)
#define CAN_CNF2_20K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_20K		(SEG4)

#define CAN_CNF1_50K		(SJW1 | 0x09)
#define CAN_CNF2_50K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_50K		(SEG4)

#define CAN_CNF1_100K		(SJW1 | BRP5)
#define CAN_CNF2_100K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_100K		(SEG4)

#define CAN_CNF1_125K		(SJW1 | BRP4)
#define CAN_CNF2_125K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_125K		(SEG4)

#define CAN_CNF1_250K		(SJW1 | BRP2)
#define CAN_CNF2_250K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_250K		(SEG4)

#define CAN_CNF1_500K		(SJW1 | BRP1)
#define CAN_CNF2_500K		(BTLMODE_CNF3|(SEG4<<3)|SEG7)
#define CAN_CNF3_500K		(SEG4)

#define CAN_CNF1_750K		(SJW1 | BRP1)
#define CAN_CNF2_750K		(BTLMODE_CNF3|(SEG3<<3)|SEG4)
#define CAN_CNF3_750K		(SEG3)

#define CAN_CNF1_800K		(SJW1 | BRP1)
#define CAN_CNF2_800K		(BTLMODE_CNF3|(SEG3<<3)|SEG3)
#define CAN_CNF3_800K		(SEG3)

#define CAN_CNF1_1000K  	(SJW1 | BRP1)
#define CAN_CNF2_1000K  	(BTLMODE_CNF3|(SEG2<<3)|SEG3)
#define CAN_CNF3_1000K		(SEG2)
#endif	/* end define CAN_FOSC_12MHZ */

/* Buffer size required for the largest SPI transfer (i.e., reading a frame).
 */
#define SPI_TRANSFER_BUF_LEN (2*(6 + CAN_FRAME_MAX_DATA_LEN))

/* Buffer size required for ring buffer of receive and send */
#define MCP251X_BUF_LEN 8



#define CAN_DEV_MAX 8

#define DRIVER_NAME "mcp2515"

static dev_t can_devt;

static int can_minor = 0;

static struct class *can_class;


struct mcp251x {
	struct cdev cdev;
	//struct class_device *class_dev;
	struct device *class_dev;
	struct semaphore lock;	 /* semaphore for spi bus share. */
	struct semaphore rxblock;	 /* semaphore for ring buffer of receive. */
	struct semaphore txblock;	 /* semaphore for ring buffer of send. */
	
	uint8_t *spi_transfer_buf;	 /* temp buffer for spi bus transfer. */

	struct can_frame rxb[MCP251X_BUF_LEN]; /* ring buffer for receive. */
	struct can_frame txb[MCP251X_BUF_LEN]; /* ring buffer for send. */
	
	int txbin;			 /* pos of in for ring buffer of send. */
	int txbout;			 /* pos of out for ring buffer of send. */
	int rxbin;			 /* pos of in for ring buffer of receive. */
	int rxbout;			 /* pos of out for ring buffer of receive. */
	
	int bit_rate;		 /* save bit rate of current set. */
	int count;			 /* count of the device opened. */
    
	wait_queue_head_t wq;	 /* queue for read process. */
    
	struct work_struct irq_work; /* bottom half of interrupt task. */
    
	struct spi_device *spi;	 /* save the point of struce spi_device. */
	struct can_filter filter;	 /* save the filter data of current set. */
};

/* ........................................................................ */

#if 0
static void mcp251x_start_tx(struct spi_device *spi, uint8_t buf_idx)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	uint8_t *tx_buf;
	int ret;
	
	tx_buf = chip->spi_transfer_buf;
	
	down(&chip->lock);
	
	tx_buf[0] = INSTRUCTION_RTS(buf_idx);
	ret = spi_write(spi, chip->spi_transfer_buf, 1);
	if (ret < 0)
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);

	up(&chip->lock);
}
#endif

static uint8_t mcp251x_read_state(struct spi_device *spi, uint8_t cmd)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	uint8_t *tx_buf, *rx_buf;
	uint8_t val;
	int ret;

	tx_buf = chip->spi_transfer_buf;
	rx_buf = chip->spi_transfer_buf + 8;

	down(&chip->lock);
	
	tx_buf[0] = cmd;
	ret = spi_write_then_read(spi, tx_buf, 1, rx_buf, 1);
	if (ret < 0) {
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);
		val = 0;
	} else
		val = rx_buf[0];

	up(&chip->lock);

	return val;
}


static uint8_t mcp251x_read_reg(struct spi_device *spi, uint8_t reg)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	uint8_t *tx_buf, *rx_buf;
	uint8_t val;
	int ret;

	tx_buf = chip->spi_transfer_buf;
	rx_buf = chip->spi_transfer_buf + 8;

	down(&chip->lock);
	
	tx_buf[0] = INSTRUCTION_READ;
	tx_buf[1] = reg;
	ret = spi_write_then_read(spi, tx_buf, 2, rx_buf, 1);
	if (ret < 0) {
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);
		val = 0;
	} else
		val = rx_buf[0];

	up(&chip->lock);

	return val;
}


static void mcp251x_write_reg(struct spi_device *spi, uint8_t reg, uint8_t val)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	int ret;

	down(&chip->lock);

	chip->spi_transfer_buf[0] = INSTRUCTION_WRITE;
	chip->spi_transfer_buf[1] = reg;
	chip->spi_transfer_buf[2] = val;

	ret = spi_write(spi, chip->spi_transfer_buf, 3);
	if (ret < 0)
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);

	up(&chip->lock);
}


static void mcp251x_write_bits(struct spi_device *spi, uint8_t reg, uint8_t mask, uint8_t val)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	int ret;

	down(&chip->lock);

	chip->spi_transfer_buf[0] = INSTRUCTION_BIT_MODIFY;
	chip->spi_transfer_buf[1] = reg;
	chip->spi_transfer_buf[2] = mask;
	chip->spi_transfer_buf[3] = val;

	ret = spi_write(spi, chip->spi_transfer_buf, 4);
	if (ret < 0)
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);

	up(&chip->lock);
}

static void mcp251x_hw_reset(struct spi_device *spi)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	int ret;

	down(&chip->lock);

	chip->spi_transfer_buf[0] = INSTRUCTION_RESET;

	ret = spi_write(spi, chip->spi_transfer_buf, 1);
	if (ret < 0)
		dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);
	
	up(&chip->lock);
}


static void __devinit mcp251x_hw_init(struct spi_device *spi)
{
	mcp251x_hw_reset(spi);	
	mcp251x_write_reg(spi,0x0c,0x0c);
}

static void mcp251x_hw_sleep(struct spi_device *spi)
{
#if 0   /* sleep disable */
	mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_SLEEP);
#endif
}

static void mcp251x_hw_wakeup(struct spi_device *spi)
{
	/* Can only wake up by generating a wake-up interrupt. */
	mcp251x_write_bits(spi, CANINTE, CANINTE_WAKIE, CANINTE_WAKIE);
	mcp251x_write_bits(spi, CANINTF, CANINTF_WAKIF, CANINTF_WAKIF);
	mdelay(10);
}


static int mcp251x_set_bit_rate(struct spi_device *spi, int bit_rate)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	unsigned char canctrl,val;
#if 0
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	int tqs; /* tbit/TQ */
	int brp;
	int ps1, ps2, propseg, sjw;

	/* Determine the BRP value that gives the requested bit rate. */
	for(brp = 0; brp < 8; brp++) {
		tqs = pdata->f_osc / (2 * (brp + 1)) / bit_rate;
		if (tqs >= 5 && tqs <= 25
		    && (pdata->f_osc / (2 * (brp + 1)) / tqs) == bit_rate)
			break;
	}
	if (brp >= 8)
		return -EINVAL;

	/* The CAN bus bit time (tbit) is determined by:
	 *   tbit = (SyncSeg + PropSeg + PS1 + PS2) * TQ
	 * with:
	 *     SyncSeg = 1
	 *     sample point (between PS1 and PS2) must be at 60%-70% of the bit time
	 *     PropSeg + PS1 >= PS2
	 *     PropSeg + PS1 >= Tdelay
	 *     PS2 > SJW
	 *     1 <= PropSeg <= 8, 1 <= PS1 <=8, 2 <= PS2 <= 8
	 * SJW = 1 is sufficient in most cases.
	 * Tdelay is usually 1 or 2 TQ.
	 */

	propseg = ps1 = ps2 = (tqs - 1) / 3;
	if (tqs - (1 + propseg + ps1 + ps2) == 2)
		ps1++;
	if (tqs - (1 + propseg + ps1 + ps2) == 1)
		ps2++;
	sjw = 1;
	
	printk("bit rate: BRP = %d, Tbit = %d TQ, PropSeg = %d, PS1 = %d, PS2 = %d, SJW = %d\n",
		brp, tqs, propseg, ps1, ps2, sjw);

	dev_dbg(&spi->dev,"bit rate: BRP = %d, Tbit = %d TQ, PropSeg = %d, PS1 = %d, PS2 = %d, SJW = %d\n",
		brp, tqs, propseg, ps1, ps2, sjw);

	mcp251x_write_reg(spi, CNF1, ((sjw-1) << 6) | brp);
	mcp251x_write_reg(spi, CNF2, CNF2_BTLMODE | ((ps1-1) << 3) | (propseg-1));
	mcp251x_write_bits(spi, CNF3, CNF3_PHSEG2_MASK, (ps2-1));
	chip->bit_rate = pdata->f_osc / (2 * (brp + 1)) / tqs;
#endif
#if 1
	canctrl = mcp251x_read_reg(spi, CANCTRL);
	do {
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_CONF);
		val = mcp251x_read_reg(spi, CANCTRL);
	}while((val&0xe0) != CANCTRL_REQOP_CONF);
			
	switch(bit_rate) {
		case 10000:
			/* save the state and put it to config mode. */
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_10K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_10K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_10K);
		break;
		
		case 20000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_20K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_20K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_20K);
		break;
		
		case 50000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_50K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_50K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_50K);
		break;
		
		case 100000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_100K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_100K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_100K);
		break;
		
		case 125000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_125K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_125K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_125K);
		break;
		
		case 250000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_250K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_250K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_250K);
		break;

		case 500000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_500K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_500K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_500K);
		break;
		
		case 750000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_750K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_750K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_750K);
		break;

		case 800000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_800K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_800K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_800K);
		break;

		case 1000000:
			mcp251x_write_reg(spi, CNF1, CAN_CNF1_1000K);
			mcp251x_write_reg(spi, CNF2, CAN_CNF2_1000K);
			mcp251x_write_reg(spi, CNF3, CAN_CNF3_1000K);
		break;

		default:
			printk("<kernel>: baud rate %d not support\n", bit_rate);
		break;
	}	
	mcp251x_write_reg(spi, CANCTRL, canctrl);
	chip->bit_rate = bit_rate;
#endif
	
	return 0;
}

static int mcp251x_get_bit_rate(struct spi_device *spi)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);

	return chip->bit_rate;
}

static int mcp251x_set_filter(struct spi_device *spi, struct can_filter *filter)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	
	uint8_t canctrl;
	uint8_t local_buf;
	int i;
	
	canctrl = mcp251x_read_reg(spi, CANCTRL);
	
	mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_CONF);

	for (i=0; i<CAN_FILTER_REG_NUM; i++) {
		if (filter->fid[i].active == 0) {
			local_buf = 0;
			mcp251x_write_reg(spi, RXF_BASE(i)+0, local_buf);
			mcp251x_write_reg(spi, RXF_BASE(i)+1, local_buf);
			mcp251x_write_reg(spi, RXF_BASE(i)+2, local_buf);
			mcp251x_write_reg(spi, RXF_BASE(i)+3, local_buf);
			continue;
		}
		local_buf = filter->fid[i].id >> 3;
		mcp251x_write_reg(spi, RXF_BASE(i)+0, local_buf);
		local_buf = (filter->fid[i].id << 5) | (filter->fid[i].ide << 3) | (filter->fid[i].eid >> 16);
		mcp251x_write_reg(spi, RXF_BASE(i)+1, local_buf);	
		local_buf = filter->fid[i].eid >> 8;
		mcp251x_write_reg(spi, RXF_BASE(i)+2, local_buf);
		local_buf = filter->fid[i].eid;
		mcp251x_write_reg(spi, RXF_BASE(i)+3, local_buf);
	}
	for (i=0; i<2; i++) {
		local_buf = filter->sidmask >> 3;
		mcp251x_write_reg(spi, RXM_BASE(i)+0, local_buf);
		local_buf = (filter->sidmask << 5) | (filter->eidmask >> 16);
		mcp251x_write_reg(spi, RXM_BASE(i)+1, local_buf);
		local_buf = filter->eidmask >> 8;
		mcp251x_write_reg(spi, RXM_BASE(i)+2, local_buf);
		local_buf = filter->eidmask;
		mcp251x_write_reg(spi, RXM_BASE(i)+3, local_buf);
	}
	
	mcp251x_write_reg(spi, CANCTRL, canctrl);
	
	mcp251x_write_bits(spi, RXBCTRL(0), RXBCTRL_MASK, filter->mode << 6);
	mcp251x_write_bits(spi, RXBCTRL(1), RXBCTRL_MASK, filter->mode << 6);
	
	memcpy(&chip->filter, filter, sizeof(struct can_filter));
	
	return 0;
}

static int mcp251x_get_filter(struct spi_device *spi, struct can_filter *filter)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);

	memcpy(filter, &chip->filter, sizeof(struct can_filter));
		
	return 0;	
}

/* If MCP251X ready, copy data from ring buffer to MCP251X send buffer and set
 * TXBCTRL_TXREQ.
 */
static int mcp251x_hw_tx(struct spi_device *spi, int tx_buf_idx)
{

	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	uint8_t buf[13] ;
//	unsigned char val;
	struct can_frame *frame;
//	int ret;
	int i;
	

	dev_dbg(&spi->dev,"%s()\n", __FUNCTION__);

	if (chip->txbout != chip->txbin) {

		if (down_interruptible(&chip->txblock))
			return -ERESTARTSYS;
		
		frame = &chip->txb[chip->txbout];
			
	//	down(&chip->lock);
			
		if (frame->header.dlc > CAN_FRAME_MAX_DATA_LEN)
			frame->header.dlc = CAN_FRAME_MAX_DATA_LEN;
		if (frame->header.ide == 0)
			frame->header.eid = 0;
	/*		
		buf[0] = INSTRUCTION_LOAD_TXB(tx_buf_idx);
		buf[1] = frame->header.id >> 3;
		buf[2] = (frame->header.id << 5) | (frame->header.ide << 3)
			| (frame->header.eid >> 16);
		buf[3]  = frame->header.eid >> 8;
		buf[4]  = frame->header.eid;
		buf[5] = (frame->header.rtr << 6) | frame->header.dlc;
		memcpy(buf + 6, frame->data, frame->header.dlc);
	
		for(i=0;i<frame->header.dlc+6;i++) {
			printk("buf[%d] : %d\n",i,buf[i]);		
		}
		ret = spi_write(spi, buf, 6 + CAN_FRAME_MAX_DATA_LEN);
		if (ret < 0) 
			dev_dbg(&spi->dev,"%s: failed: ret = %d\n", __FUNCTION__, ret);
	*/
		
		buf[0] = frame->header.id >> 3;
		buf[1] = (frame->header.id << 5) | (frame->header.ide << 3)	| (frame->header.eid >> 16);
		buf[2]  = frame->header.eid >> 8;
		buf[3]  = frame->header.eid;
		buf[4] = (frame->header.rtr << 6) | frame->header.dlc;
		memcpy(buf + 5, frame->data, frame->header.dlc);
		/*		
		for(i=0;i<frame->header.dlc+5;i++) {
			printk("buf[%d] : %x\n",i,buf[i]);		
		}
		*/
		if (tx_buf_idx == 0) {
			for(i=0;i<(frame->header.dlc+5);i++) {
				mcp251x_write_reg(spi,(0x31+i),buf[i]);
			}
		} else if(tx_buf_idx ==1) {
			for(i=0;i<(frame->header.dlc+5);i++)  {
				mcp251x_write_reg(spi,(0x41+i),buf[i]);
			}
		} else {
			for(i=0;i<(frame->header.dlc+5);i++)  {
				mcp251x_write_reg(spi,(0x51+i),buf[i]);
			}
		}

	//	up(&chip->lock);
		
		/* update pos of ring buffer */
		chip->txbout++;
		if (chip->txbout >= MCP251X_BUF_LEN)
			chip->txbout = 0;

		up(&chip->txblock);
		mcp251x_write_reg(spi, TXBCTRL(tx_buf_idx), TXBCTRL_TXREQ);
	}
	return 0;
}

/* Receive data from internat buffer of MCP251X and save it to ring buffer.
 */
static int mcp251x_hw_rx(struct spi_device *spi, int buf_idx)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	uint8_t rx_buf[13];
//	int ret;
	struct can_frame *frame;
	int i;
	
	if (down_interruptible(&chip->rxblock))
		return -ERESTARTSYS;
		
	frame = &chip->rxb[chip->rxbin];

//	down(&chip->lock);
	/*
	buf[0] = INSTRUCTION_READ_RXB(buf_idx);
	rx_buf = buf + (6 + CAN_FRAME_MAX_DATA_LEN);
	ret = spi_write_then_read(spi, buf, 1, rx_buf, 13);
	if (ret < 0)
		dev_dbg(&spi->dev, "%s: failed: ret = %d\n", __FUNCTION__, ret);
	*/
	if (buf_idx == 0) 
		for(i=0;i<(5 + CAN_FRAME_MAX_DATA_LEN);i++) {
			rx_buf[i] = mcp251x_read_reg(spi,0x61+i);	
		}
	else
		for(i=0;i<(5 + CAN_FRAME_MAX_DATA_LEN);i++) {
			rx_buf[i] = mcp251x_read_reg(spi,0x71+i);	
		}

	frame->header.id = (rx_buf[0] << 3) | (rx_buf[1] >> 5);
	frame->header.ide = (rx_buf[1] >> 3) & 0x1;
	frame->header.srr = (rx_buf[1] >> 4) & 0x1;
	if (frame->header.ide == 1)
		frame->header.eid = (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
	else
		frame->header.eid = 0;
	frame->header.rtr = (rx_buf[4] >> 6) & 0x1;
	frame->header.rb1 = (rx_buf[4] >> 5) & 0x1;
	frame->header.rb0 = (rx_buf[4] >> 4) & 0x1;
	frame->header.dlc = rx_buf[4] & 0x0f;
	memcpy(frame->data, rx_buf + 5, CAN_FRAME_MAX_DATA_LEN);
	
//	up(&chip->lock);

	/* update pos of ring buffer */
	chip->rxbin++;
	if (chip->rxbin >= MCP251X_BUF_LEN)
		chip->rxbin = 0;
			
	up(&chip->rxblock);

	return 0;
}

/* ........................................................................ */

/* bottom half task for interrupt */
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
static void mcp251x_irq_handler(struct work_struct *work)
{
	struct mcp251x *chip = container_of(work, struct mcp251x, irq_work);
	struct spi_device *spi = chip->spi;
#else
static void mcp251x_irq_handler(void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
#endif
	uint8_t intf, rxs;

	for(;;) {
		
		intf = mcp251x_read_reg(spi, CANINTF);
		if (intf == 0x00)
			break;

		dev_dbg(&spi->dev,"interrupt:%s%s%s%s%s%s%s%s\n",
			(intf & CANINTF_MERRF) ? " MERR":"",
			(intf & CANINTF_WAKIF) ? " WAK":"",
			(intf & CANINTF_ERRIF) ? " ERR":"",
			(intf & CANINTF_TX2IF) ? " TX2":"",
			(intf & CANINTF_TX1IF) ? " TX1":"",
			(intf & CANINTF_TX0IF) ? " TX0":"",
			(intf & CANINTF_RX1IF) ? " RX1":"",
			(intf & CANINTF_RX0IF) ? " RX0":"");
			
		rxs = mcp251x_read_state(spi, INSTRUCTION_RX_STATE); 
		dev_dbg(&spi->dev, 
			"rx_state:%s%s\n",
			(rxs & RX_STATE_IDE) ? " IDE":"",
			(rxs & RX_STATE_RTR) ? " RTR":"");
			
		if (intf & CANINTF_MERRF) {
#if 0
			uint8_t txbnctrl;
			/* if there are no pending Tx buffers, restart queue */
			txbnctrl = mcp251x_read_reg(spi, TXBCTRL(0));
			if (!(txbnctrl & TXBCTRL_TXREQ))
				netif_wake_queue(&chip->can->ndev);
#endif
		}
		if (intf & CANINTF_ERRIF) {
			uint8_t eflg = mcp251x_read_reg(spi, EFLG);
	    
			dev_dbg(&spi->dev, "EFLG = 0x%02x\n", eflg);
	    
			if (eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) {
#if 0
				if (eflg & EFLG_RX0OVR)
					chip->stats.rx_over_errors++;
				if (eflg & EFLG_RX1OVR)
					chip->stats.rx_over_errors++;
#endif
				mcp251x_write_reg(spi, EFLG, 0x00);
			}
		}

		if (intf & CANINTF_TX0IF) /* If ready to send, copy data to send buffer. */
			mcp251x_hw_tx(spi, 0);
		if (intf & CANINTF_TX1IF)
			mcp251x_hw_tx(spi, 1);
		if (intf & CANINTF_TX2IF)
			mcp251x_hw_tx(spi, 2);
		if (intf & CANINTF_RX0IF) /* If received data, copy data to ring buffer. */
			mcp251x_hw_rx(spi, 0);
		if (intf & CANINTF_RX1IF)
			mcp251x_hw_rx(spi, 1);
	
		mcp251x_write_bits(spi, CANINTF, intf, 0x00);
	
		/* If ring buffer of receive is not empty, wake up the read queue. */
		if (chip->rxbin != chip->rxbout)
			wake_up_interruptible(&chip->wq);
	}
}


static irqreturn_t mcp251x_irq(int irq, void *dev_id)
{
	struct spi_device *spi = dev_id;
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);

	/* Can't do anything in interrupt context so fire of the interrupt
	 * handling workqueue. */
	schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}

/* ........................................................................ */

static int mcp251x_open(struct inode *inode, struct file *file)
{
	struct mcp251x *chip = container_of(inode->i_cdev, struct mcp251x, cdev);
	struct spi_device *spi = chip->spi;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	
	file->private_data = chip;

	if (!chip->count) {
		
		if (pdata->transceiver_enable)
			pdata->transceiver_enable(1);

		mcp251x_hw_reset(spi);
		mcp251x_hw_wakeup(spi);

		/* Enable interrupts */
		mcp251x_write_reg(spi, CANINTE,
				  CANINTE_ERRIE | CANINTE_TX2IE
				  | CANINTE_TX1IE | CANINTE_TX0IE
				  | CANINTE_RX1IE | CANINTE_RX0IE);

		/* put device into normal mode */
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL);
		mcp251x_write_reg(spi, RXBCTRL(0), RXBCTRL_BULK);
	}
	
	chip->count++;
	
	return 0;
}

static int mcp251x_release(struct inode *inode, struct file *file)
{
	
	struct mcp251x *chip = container_of(inode->i_cdev, struct mcp251x, cdev);
	struct spi_device *spi = chip->spi;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;

	chip->count--;
	
	if (chip->count)
		return 0;

	/* disable and clear pending interrupts */
	mcp251x_write_reg(spi, CANINTE, 0x00);
	mcp251x_write_reg(spi, CANINTF, 0x00);

	/* go to sleep */
	mcp251x_hw_sleep(spi);

	if (pdata->transceiver_enable)
		pdata->transceiver_enable(0);
		

	return 0;
}

static int mcp251x_write(struct file *file, const char __user *buf, size_t count, loff_t *ofs)
{
	
	struct mcp251x *chip = file->private_data;
	struct spi_device *spi = chip->spi;
	struct can_frame *frame;
	int ret;
	uint8_t txreq;

	if (count < sizeof(struct can_frame))
		return -EINVAL;

	if (down_interruptible(&chip->txblock))
		return -ERESTARTSYS;
	
	frame = &chip->txb[chip->txbin];
	ret = copy_from_user(frame, buf, sizeof(struct can_frame));
	
	chip->txbin++;
	if (chip->txbin >= MCP251X_BUF_LEN)
		chip->txbin = 0;
	
	up(&chip->txblock);
	
	txreq = mcp251x_read_state(spi, INSTRUCTION_CAN_STATE);
	if (!(txreq & CAN_STATE_TX0REQ)) 
		mcp251x_hw_tx(spi, 0);		
	if (!(txreq & CAN_STATE_TX1REQ)) 
		mcp251x_hw_tx(spi, 1);
	if (!(txreq & CAN_STATE_TX2REQ))
		mcp251x_hw_tx(spi, 2);
	
	return count;
}

static ssize_t mcp251x_read(struct file *file, char __user *buf, size_t count, loff_t *ofs)
{
	
	struct mcp251x *chip = file->private_data;
	struct can_frame *frame;
	
	if (count != sizeof(struct can_frame))
		return -EINVAL;
		
	if (down_interruptible(&chip->rxblock))
		return -ERESTARTSYS;
	
	while (chip->rxbin == chip->rxbout) {
		up(&chip->rxblock);
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(chip->wq, (chip->rxbin != chip->rxbout)))
			return -ERESTARTSYS;
		if (down_interruptible(&chip->rxblock))
			return -ERESTARTSYS;
	}
	
	frame = &chip->rxb[chip->rxbout];
	if (copy_to_user(buf, frame, sizeof(struct can_frame))) {
		up(&chip->rxblock);
		return -EFAULT;
	}
	chip->rxbout++;
	if(chip->rxbout >= MCP251X_BUF_LEN)
		chip->rxbout = 0;
		
	up(&chip->rxblock);
		
	return count;
    
#if 0
retry:
	if (chip->rxbin != chip->rxbout) {
		
		down(&chip->rxblock);
		
		frame = &chip->rxb[chip->rxbout];
		if (copy_to_user(buf, frame, sizeof(struct can_frame))) {
			up(&chip->rxblock);
			return -EFAULT;
		}
		chip->rxbout++;
		if(chip->rxbout >= MCP251X_BUF_LEN)
			chip->rxbout = 0;
		
		up(&chip->rxblock);
		
		return count;
	}
	else {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		interruptible_sleep_on(&chip->wq);
		if (signal_pending(current))
			return -ERESTARTSYS;
			
		goto retry;
	}
#endif
}

static int mcp251x_ioctl(struct inode *inode, struct file *file, 
			 unsigned int cmd, unsigned long arg)
{

	struct mcp251x *chip = container_of(inode->i_cdev, struct mcp251x, cdev);
	struct spi_device *spi = chip->spi;
	int ret = 0;

	switch(cmd) {
	
	case CAN_IOCTRESET:		/* reset devcie */
		mcp251x_hw_reset(spi);
		break;
	case CAN_IOCTWAKEUP:	/* wake up device */
		mcp251x_hw_wakeup(spi);
		break;
	case CAN_IOCSRATE:		/* set bit rate */
		ret = mcp251x_set_bit_rate(spi, (int)arg);
		mdelay(10);
		break;
	case CAN_IOCGRATE:		/* get bit rate */
		*((int *)arg) = mcp251x_get_bit_rate(spi);
		break;
	case CAN_IOCSFILTER:	/* set filter */
		ret = mcp251x_set_filter(spi, (struct can_filter *)arg);
		break;
	case CAN_IOCGFILTER:	/* get filter */
		ret = mcp251x_get_filter(spi, (struct can_filter *)arg);
		break;
	case CAN_IOCTNORMALMODE:	/* turn to normal mode */
		//mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_NORMAL);
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_NORMAL);
		break;
	case CAN_IOCTLOOPBACKMODE:	/* turn to loopback mode */
		//mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_LOOPBACK);
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_LOOPBACK);
		break;
	case CAN_IOCTLISTENONLYMODE: /* turn to listen only mode */
		//mcp251x_write_bits(spi, CANCTRL, CANCTRL_REQOP_MASK, CANCTRL_REQOP_LISTEN_ONLY);
		mcp251x_write_reg(spi, CANCTRL, CANCTRL_REQOP_LISTEN_ONLY);
		break;
	case CAN_IOCTSLEEPMODE:	/* turn to sleep mode */
		mcp251x_hw_sleep(spi);
		break;
	default:
		return -ENOTTY;
	}
	
	return ret;
}

static const struct file_operations mcp251x_fops = {
	.owner = THIS_MODULE,
	.read = mcp251x_read,
	.write = mcp251x_write,
	.ioctl = mcp251x_ioctl,
	.open = mcp251x_open,
	.release = mcp251x_release,
};

/* ........................................................................ */

static int __devexit mcp251x_remove(struct spi_device *spi)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);

	dev_dbg(&spi->dev, "%s: stop\n",  __FUNCTION__);

	device_unregister(chip->class_dev);

	cdev_del(&chip->cdev);
	free_irq(spi->irq, spi);
	kfree(chip->spi_transfer_buf);

	return 0;
}

static int __devinit mcp251x_probe(struct spi_device *spi)
{
	struct mcp251x *chip;
	int ret = 0;

	dev_dbg(&spi->dev,"%s: start\n",  __FUNCTION__);

	chip = kmalloc(sizeof(struct mcp251x), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	dev_set_drvdata(&spi->dev, chip);
	
	chip->txbin = chip->txbout = 0;
	chip->rxbin = chip->rxbout = 0;
	chip->count = 0;
	chip->spi = spi;
	init_MUTEX(&chip->lock);
	init_MUTEX(&chip->txblock);
	init_MUTEX(&chip->rxblock);
	init_waitqueue_head(&chip->wq);
    
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20))
	INIT_WORK(&chip->irq_work, mcp251x_irq_handler);
#else
	INIT_WORK(&chip->irq_work, mcp251x_irq_handler, spi);
#endif
    
	chip->spi_transfer_buf = kmalloc(SPI_TRANSFER_BUF_LEN, GFP_KERNEL);
	if (!chip->spi_transfer_buf) {
		ret = -ENOMEM;
		goto error_buf;
	}
	
	ret = request_irq(spi->irq, mcp251x_irq,IRQF_SAMPLE_RANDOM, DRIVER_NAME, spi);// SA_SAMPLE_RANDOM,
	if (ret < 0) {
		dev_err(&spi->dev,"request irq %d failed (ret = %d)\n", spi->irq, ret);
		goto error_irq;
	}
	
	cdev_init(&chip->cdev, &mcp251x_fops);
	chip->cdev.owner = THIS_MODULE;
	ret = cdev_add(&chip->cdev, MKDEV(MAJOR(can_devt), can_minor), 1);
	if (ret < 0) {
		dev_err(&spi->dev, "register char device failed (ret = %d)\n", ret);
		goto error_register;
	}

	chip->class_dev = device_create(can_class, NULL,
					      MKDEV(MAJOR(can_devt), can_minor),
					      &spi->dev, "can%d", can_minor);
	if (IS_ERR(chip->class_dev)) {
		dev_err(&spi->dev, "cannot create CAN class device\n");
		ret = PTR_ERR(chip->class_dev);
		goto error_class_reg;
	}
	
	dev_info(&spi->dev, "device register at dev(%d:%d)\n",
		 MAJOR(can_devt), can_minor);
	
	mcp251x_hw_init(spi);
	mcp251x_set_bit_rate(spi, 125000); /* A reasonable default */
	mcp251x_hw_sleep(spi);

	can_minor++;
	
	return 0;
	
error_class_reg:
	cdev_del(&chip->cdev);
error_register:
	free_irq(spi->irq, spi);
error_irq:
	kfree(chip->spi_transfer_buf);
error_buf:
	kfree(chip);
error_alloc:
	return ret;
}

#ifdef CONFIG_PM
static int mcp251x_suspend(struct spi_device *spi, pm_message_t mesg)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;

	if (chip->count)
		return 0;

	mcp251x_hw_sleep(spi);
	if (pdata->transceiver_enable)
		pdata->transceiver_enable(0);

	return 0;
}

static int mcp251x_resume(struct spi_device *spi)
{
	struct mcp251x *chip = dev_get_drvdata(&spi->dev);
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;

	if (!chip->count)
		return 0;
		
	if (pdata->transceiver_enable)
		pdata->transceiver_enable(1);
	mcp251x_hw_wakeup(spi);

	return 0;
}
#endif

static struct spi_driver mcp251x_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= mcp251x_probe,
	.remove	= __devexit_p(mcp251x_remove),
#ifdef CONFIG_PM
	.suspend	= mcp251x_suspend,
	.resume	= mcp251x_resume,
#endif
};

static int __init mcp251x_init(void)
{
	int ret;
	
	can_class = class_create(THIS_MODULE, "can");
	if (IS_ERR(can_class))
		return PTR_ERR(can_class);
	ret = alloc_chrdev_region(&can_devt, 0, CAN_DEV_MAX, DRIVER_NAME);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to allocate char dev region\n",
		       __FILE__);
		class_destroy(can_class);
		return ret;
	}
	return spi_register_driver(&mcp251x_driver);
}
module_init(mcp251x_init);

static void __exit mcp251x_exit(void)
{
	class_destroy(can_class);
	unregister_chrdev_region(can_devt, CAN_DEV_MAX);
	spi_unregister_driver(&mcp251x_driver);
}
module_exit(mcp251x_exit);


MODULE_DESCRIPTION("MCP251x CAN controller driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Renzhou.Meng");
