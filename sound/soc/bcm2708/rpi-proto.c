/*
 * ASoC driver for PROTO AudioCODEC (with a WM8731)
 * connected to a Raspberry Pi
 *
 * Author:      Florian Meier, <koalo@koalo.de>
 *	      Copyright 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "../codecs/wm8731.h"

static const unsigned int wm8731_rates_11289600[] = {
	8000, 32000, 48000, 96000,
};

static struct snd_pcm_hw_constraint_list wm8731_constraints_11289600 = {
	.list = wm8731_rates_11289600,
	.count = ARRAY_SIZE(wm8731_rates_11289600),
};

static int snd_rpi_proto_startup(struct snd_pcm_substream *substream)
{
	/* Setup constraints, because there is a 11.2896 MHz XTAL on the board */
	snd_pcm_hw_constraint_list(substream->runtime, 0,
				SNDRV_PCM_HW_PARAM_RATE,
				&wm8731_constraints_11289600);
	return 0;
}

static int snd_rpi_proto_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int sysclk = 11289600; /* This is fixed on this board */

	/* Set proto sysclk */
	int ret = snd_soc_dai_set_sysclk(codec_dai, WM8731_SYSCLK_XTAL,
		sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(substream->pcm->dev,
				"Failed to set WM8731 SYSCLK: %d\n", ret);
		return ret;
	}

	return 0;
}

/* machine stream operations */
static struct snd_soc_ops snd_rpi_proto_ops = {
	.startup = snd_rpi_proto_startup,
	.hw_params = snd_rpi_proto_hw_params,
};

static struct snd_soc_dai_link snd_rpi_proto_dai[] = {
{
	.name		= "WM8731",
	.stream_name	= "WM8731 HiFi",
	.cpu_dai_name	= "bcm2708-i2s.0",
	.codec_dai_name	= "wm8731-hifi",
	.platform_name	= "bcm2708-pcm-audio.0",
	.codec_name	= "wm8731.1-001a",
	.dai_fmt	= SND_SOC_DAIFMT_I2S
				| SND_SOC_DAIFMT_NB_NF
				| SND_SOC_DAIFMT_CBM_CFM,
	.ops		= &snd_rpi_proto_ops,
},
};

/* audio machine driver */
static struct snd_soc_card snd_rpi_proto = {
	.name		= "snd_rpi_proto",
	.dai_link	= snd_rpi_proto_dai,
	.num_links	= ARRAY_SIZE(snd_rpi_proto_dai),
};

static int snd_rpi_proto_probe(struct platform_device *pdev)
{
	int ret = 0;

	snd_rpi_proto.dev = &pdev->dev;
	ret = snd_soc_register_card(&snd_rpi_proto);
	if (ret) {
		dev_err(&pdev->dev,
				"snd_soc_register_card() failed: %d\n", ret);
	}

	return ret;
}


static int snd_rpi_proto_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_rpi_proto);
}

static struct platform_driver snd_rpi_proto_driver = {
	.driver = {
		.name   = "snd-rpi-proto",
		.owner  = THIS_MODULE,
	},
	.probe	  = snd_rpi_proto_probe,
	.remove	 = snd_rpi_proto_remove,
};

module_platform_driver(snd_rpi_proto_driver);

MODULE_AUTHOR("Florian Meier");
MODULE_DESCRIPTION("ASoC Driver for Raspberry Pi connected to PROTO board (WM8731)");
MODULE_LICENSE("GPL");
