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

static struct platform_driver ess9018_driver = {
	.driver = {
		.name = "ess9018-codec",
		.owner = THIS_MODULE,
	},
	.probe = ess9018_probe,
	.remove = ess9018_remove,
};

/*
 * Register driver and device
 *
 * TODO This is not the best solution, but it is ok for now.
 * 	Later this should be handled by the device tree.
 */

static struct platform_device *pdev;

static const struct platform_device_info ess9018_dev_info = {
	.name = "ess9018-codec",
	.id = -1,
};

static int ess9018_init(void)
{
	int rc = platform_driver_register(&ess9018_driver);

	if (rc == 0) {
		pdev = platform_device_register_full(&ess9018_dev_info);
		if (IS_ERR(pdev)) {
			platform_driver_unregister(&ess9018_driver);
			rc = PTR_ERR(pdev);
		}
	}
	return rc;
}
subsys_initcall(ess9018_init);

static void __exit ess9018_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&ess9018_driver);
}
module_exit(ess9018_exit);

MODULE_AUTHOR("Florian Meier <koalo@koalo.de>");
MODULE_DESCRIPTION("ASoC ESS9018 codec driver");
MODULE_LICENSE("GPL");

