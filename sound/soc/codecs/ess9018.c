/*
 * Driver for the ESS9018 codec
 * It is a dummy driver without any controls.
 *
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

static struct snd_soc_dai_driver ess9018_dai = {
	.name = "ess9018-hifi",
	.playback = {
		.channels_min = 1,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE
	},
};

static struct snd_soc_codec_driver soc_codec_dev_ess9018;

static int ess9018_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_dev_ess9018,
			&ess9018_dai, 1);
}

static int ess9018_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static struct platform_driver ess9018_codec_driver = {
	.probe 		= ess9018_probe,
	.remove 	= ess9018_remove,
	.driver		= {
		.name	= "ess9018-codec",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(ess9018_codec_driver);

MODULE_AUTHOR("Florian Meier <koalo@koalo.de>");
MODULE_DESCRIPTION("ASoC ESS9018 codec driver");
MODULE_LICENSE("GPL");

