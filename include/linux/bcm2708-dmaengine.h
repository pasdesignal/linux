/*
 * BCM2708 DMA Engine support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __LINUX_BCM2708_DMA_H
#define __LINUX_BCM2708_DMA_H

#include <linux/dmaengine.h>

struct bcm2708_dma_slave_config {
	struct dma_slave_config cfg;
	unsigned int sync_dreq;
};

#endif /* __LINUX_BCM2708_DMA_H */
