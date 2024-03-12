/* Goodix's GF316M/GF318M/GF3118M/GF518M/GF5118M/GF516M/GF816M/GF3208/GF5206
 *  fingerprint sensor linux driver for factory test
 *
 * 2010 - 2015 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <net/sock.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

/* MTK header */
#include "mtk_spi.h"
#include "mtk_spi_hal.h"
#include "mtk_gpio.h"
#include "mach/gpio_const.h"

#include "gf_spi_tee.h"

#define SPI_CLK_TOTAL_TIME  107

extern u8 g_debug_level;

int gf_ioctl_spi_init_cfg_cmd(struct mt_chip_conf *mcc, unsigned long arg)
{
    int retval = 0;

    return retval;
}

/* gf_spi_setup_conf_ree, configure spi speed and transfer mode in REE mode
  *
  * speed: 1, 4, 6, 8 unit:MHz
  * mode: DMA mode or FIFO mode
  */
void gf_spi_setup_conf_factory(struct gf_device *gf_dev, u32 speed, enum spi_transfer_mode mode)
{
	struct mt_chip_conf *mcc = &gf_dev->spi_mcc;

	switch (speed) {
	case 1:
		/* set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
		break;
	case 4:
		/* set to 4MHz clock */
		mcc->high_time = 15;
		mcc->low_time = 15;
		break;
	case 6:
		/* set to 6MHz clock */
		mcc->high_time = 10;
		mcc->low_time = 10;
		break;
	case 8:
		/* set to 8MHz clock */
		mcc->high_time = 8;
		mcc->low_time = 8;
		break;
	default:
		/* default set to 1MHz clock */
		mcc->high_time = 50;
		mcc->low_time = 50;
	}

	if ((mode == DMA_TRANSFER) || (mode == FIFO_TRANSFER)) {
		mcc->com_mod = mode;
	} else {
		/* default set to FIFO mode */
		mcc->com_mod = FIFO_TRANSFER;
	}

	if (spi_setup(gf_dev->spi))
		gf_debug(ERR_LOG, "%s, failed to setup spi conf\n", __func__);

}

static int gf_spi_transfer_raw_ree(struct gf_device *gf_dev, u8 *tx_buf, u8 *rx_buf, u32 len)
{
    struct spi_message msg;
    struct spi_transfer xfer;

    spi_message_init(&msg);
    memset(&xfer, 0, sizeof(struct spi_transfer));

    xfer.tx_buf = tx_buf;
    xfer.rx_buf = rx_buf;
    xfer.len = len;
    spi_message_add_tail(&xfer, &msg);
    spi_sync(gf_dev->spi, &msg);

    return 0;
}

int gf_ioctl_transfer_raw_cmd(struct gf_device *gf_dev, unsigned long arg, unsigned int bufsiz)
{
	struct gf_ioc_transfer_raw ioc_xraw;
	int retval = 0;

	do {
		u8 *tx_buf;
		u8 *rx_buf;
		uint32_t len;

		if (copy_from_user(&ioc_xraw, (struct gf_ioc_transfer_raw *)arg, sizeof(struct gf_ioc_transfer_raw))) {
			gf_debug(ERR_LOG, "%s: Failed to copy gf_ioc_transfer_raw from user to kernel\n", __func__);
			retval = -EFAULT;
			break;
		}

		if ((ioc_xraw.len > bufsiz) || (ioc_xraw.len == 0)) {
			gf_debug(ERR_LOG, "%s: request transfer length larger than maximum buffer\n", __func__);
			retval = -EINVAL;
			break;
		}

		if (ioc_xraw.read_buf == NULL || ioc_xraw.write_buf == NULL) {
			gf_debug(ERR_LOG, "%s: read buf and write buf can not equal to NULL simultaneously.\n", __func__);
			retval = -EINVAL;
			break;
		}

		/* change speed and set transfer mode */
#if 0
		if (ioc_xraw.len > 32) {
			gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time, ioc_xraw.low_time, DMA_TRANSFER);
		} else {
			gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time, ioc_xraw.low_time, FIFO_TRANSFER);
		}
#else
		if (ioc_xraw.len > 32) {
			gf_spi_setup_conf_factory(gf_dev, 8, DMA_TRANSFER);
		} else {
			gf_spi_setup_conf_factory(gf_dev, ioc_xraw.high_time, FIFO_TRANSFER);
		}
#endif

		len = ioc_xraw.len;
		if (len % 1024 != 0 && len > 1024) {
			len = ((ioc_xraw.len / 1024) + 1) * 1024;
		}

		tx_buf = kzalloc(len, GFP_KERNEL);
		if (NULL == tx_buf) {
			gf_debug(ERR_LOG, "%s: failed to allocate raw tx buffer\n", __func__);
			retval = -EMSGSIZE;
			break;
		}

		rx_buf = kzalloc(len, GFP_KERNEL);
		if (NULL == rx_buf) {
			kfree(tx_buf);
			gf_debug(ERR_LOG, "%s: failed to allocate raw rx buffer\n", __func__);
			retval = -EMSGSIZE;
			break;
		}

		if (copy_from_user(tx_buf, ioc_xraw.write_buf, ioc_xraw.len)) {
			kfree(tx_buf);
			kfree(rx_buf);
			gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer from user to kernel\n");
			retval = -EFAULT;
			break;
		}

		gf_spi_transfer_raw_ree(gf_dev, tx_buf, rx_buf, len);

		if (copy_to_user(ioc_xraw.read_buf, rx_buf, ioc_xraw.len)) {
			gf_debug(ERR_LOG, "Failed to copy gf_ioc_transfer_raw from kernel to user\n");
			retval = -EFAULT;
		}

		kfree(tx_buf);
		kfree(rx_buf);
	} while (0);

	return retval;
}

