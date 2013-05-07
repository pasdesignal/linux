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
 * TODO This should be transferred to the generic DMA engine driver
 * 	as soon as the official BCM2708 kernel migrates to 3.10. 
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bcm2708-dmaengine.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

#include "bcm2708-pcm.h"

static const struct snd_pcm_hardware bcm2708_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME |
				  SNDRV_PCM_INFO_JOINT_DUPLEX |
				  SNDRV_PCM_INFO_NO_PERIOD_WAKEUP,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 32,
	.period_bytes_max	= 64 * 1024,
	.periods_min		= 2,
	.periods_max		= 255,
	.buffer_bytes_max	= 128 * 1024,
};


/* this may get called several times by oss emulation */
static int bcm2708_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct bcm2708_pcm_dma_data *dma_data;
	struct bcm2708_dma_slave_config config;
	struct dma_chan *chan;
	int err = 0;

	/* get DMA data (e.g. FIFO address and DREQ) */
	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	/* return - there seems to be no use for bufferless
	 * transfer with this SoC */
	if (!dma_data)
		return 0;

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	chan = snd_dmaengine_pcm_get_chan(substream);
	if (!chan)
		return -EINVAL;

	/* fills in addr_width and direction */
	err = snd_hwparams_to_dma_slave_config(substream, params, &config.cfg);
	if (err)
		return err;
	
	/* Override the *_dma addr_width if requested by the DAI driver */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		config.cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	else
		config.cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	config.cfg.src_addr = dma_data->port_addr;
	config.cfg.dst_addr = dma_data->port_addr;
	config.cfg.src_maxburst = dma_data->packet_size;
	config.cfg.dst_maxburst = dma_data->packet_size;
	config.sync_dreq = dma_data->dma_req;

	return dmaengine_slave_config(chan, &config.cfg);
}

static int bcm2708_pcm_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

static int bcm2708_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct bcm2708_pcm_dma_data *dma_data;
	int ret = 0;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	if (ret == 0)
		ret = snd_dmaengine_pcm_trigger(substream, cmd);

	return ret;
}

static snd_pcm_uframes_t bcm2708_pcm_pointer(struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

static int bcm2708_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct bcm2708_pcm_dma_data *dma_data;
	int ret;

	snd_soc_set_runtime_hwparams(substream, &bcm2708_pcm_hardware);

	/* Ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	dma_data = snd_soc_dai_get_dma_data(rtd->cpu_dai, substream);
	return snd_dmaengine_pcm_open(substream, NULL, NULL);
}

static int bcm2708_pcm_close(struct snd_pcm_substream *substream)
{
	snd_dmaengine_pcm_close(substream);
	return 0;
}

static int bcm2708_pcm_mmap(struct snd_pcm_substream *substream,
	struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

static struct snd_pcm_ops bcm2708_pcm_ops = {
	.open		= bcm2708_pcm_open,
	.close		= bcm2708_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= bcm2708_pcm_hw_params,
	.hw_free	= bcm2708_pcm_hw_free,
	.trigger	= bcm2708_pcm_trigger,
	.pointer	= bcm2708_pcm_pointer,
	.mmap		= bcm2708_pcm_mmap,
};

static u64 bcm2708_pcm_dmamask = DMA_BIT_MASK(64);

static int bcm2708_pcm_preallocate_dma_buffer(struct snd_pcm *pcm,
	int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = bcm2708_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_writecombine(pcm->card->dev, size,
					   &buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;

	buf->bytes = size;
	return 0;
}

static void bcm2708_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_writecombine(pcm->card->dev, buf->bytes,
				      buf->area, buf->addr);
		buf->area = NULL;
	}
}

static int bcm2708_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_card *card = rtd->card->snd_card;
	struct snd_pcm *pcm = rtd->pcm;
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &bcm2708_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	if (pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream) {
		ret = bcm2708_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream) {
		ret = bcm2708_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

out:
	/* free preallocated buffers in case of error */
	if (ret)
		bcm2708_pcm_free_dma_buffers(pcm);

	return ret;
}

static struct snd_soc_platform_driver bcm2708_soc_platform = {
	.ops		= &bcm2708_pcm_ops,
	.pcm_new	= bcm2708_pcm_new,
	.pcm_free	= bcm2708_pcm_free_dma_buffers,
};

static int bcm2708_pcm_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev,
			&bcm2708_soc_platform);
}

static int bcm2708_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver bcm2708_pcm_driver = {
	.driver = {
			.name = "bcm2708-pcm-audio",
			.owner = THIS_MODULE,
	},

	.probe = bcm2708_pcm_probe,
	.remove = bcm2708_pcm_remove,
};

module_platform_driver(bcm2708_pcm_driver);

MODULE_AUTHOR("Florian Meier");
MODULE_DESCRIPTION("BCM2708 PCM DMA module");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bcm2708-pcm-audio");

