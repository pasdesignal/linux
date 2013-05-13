/*
 * Driver for the TDA1541A codec
 * Copyright 2013 Florian Meier <koalo@koalo.de>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/soc.h>

static struct snd_soc_dai_driver tda1541a_dai = {
	.name = "tda1541a-hifi",
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
};

static struct snd_soc_codec_driver soc_codec_dev_tda1541a;

static int tda1541a_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_tda1541a,
			&tda1541a_dai, 1);
}

static int tda1541a_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver tda1541a_codec_driver = {
	.probe 		= tda1541a_probe,
	.remove 	= tda1541a_remove,
	.driver		= {
		.name	= "tda1541a-codec",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(tda1541a_codec_driver);

MODULE_AUTHOR("Florian Meier <koalo@koalo.de>");
MODULE_DESCRIPTION("ASoC TDA1541A codec driver");
MODULE_LICENSE("GPL");

