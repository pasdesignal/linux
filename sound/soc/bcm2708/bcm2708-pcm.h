/*
 * ALSA PCM interface for Broadcom BCM2708 SoC
 *
 * Author:      Florian Meier, <koalo@koalo.de>
 *              Copyright 2013
 *
 * based on
 * 	ALSA PCM interface for the OMAP SoC
 * 	Copyright (C) 2008 Nokia Corporation
 * 	Contact: Jarkko Nikula <jarkko.nikula@bitmer.com>
 *     		 Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef __BCM2708_PCM_H__
#define __BCM2708_PCM_H__

struct snd_pcm_substream;

struct bcm2708_pcm_dma_data {
	char		*name;		/* stream identifier */
	int		dma_req;	/* DMA request line */
	unsigned long	port_addr;	/* transmit/receive register */
	void (*set_threshold)(struct snd_pcm_substream *substream);
	int		data_type;	/* 8, 16, 32 (bits) or 0 to let bcm2708-pcm
					 * to decide the sDMA data type */
	int		packet_size;	/* packet size only in PACKET mode */
};

#endif
