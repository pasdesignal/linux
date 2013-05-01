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

static struct platform_driver tda1541a_driver = {
	.driver = {
		.name = "tda1541a-codec",
		.owner = THIS_MODULE,
	},
	.probe = tda1541a_probe,
	.remove = tda1541a_remove,
};

/*
 * Register driver and device
 *
 * TODO This is not the best solution, but it is ok for now.
 * 	Later this should be handled by the device tree.
 */

static struct platform_device *pdev;

static const struct platform_device_info tda1541a_dev_info = {
	.name = "tda1541a-codec",
	.id = -1,
};

static int tda1541a_init(void)
{
	int rc = platform_driver_register(&tda1541a_driver);

	if (rc == 0) {
		pdev = platform_device_register_full(&tda1541a_dev_info);
		if (IS_ERR(pdev)) {
			platform_driver_unregister(&tda1541a_driver);
			rc = PTR_ERR(pdev);
		}
	}
	return rc;
}
subsys_initcall(tda1541a_init);

static void __exit tda1541a_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&tda1541a_driver);
}
module_exit(tda1541a_exit);

MODULE_AUTHOR("Florian Meier <koalo@koalo.de>");
MODULE_DESCRIPTION("ASoC TDA1541A codec driver");
MODULE_LICENSE("GPL");

