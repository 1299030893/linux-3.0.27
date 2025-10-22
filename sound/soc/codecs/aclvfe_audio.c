/*
 * ALSA SoC ACLVFE dummy driver
 *
 *  This driver is used by devices which have an on-chip ACLVFE controllers
 *  where no codec is needed.  This file provides stub codec that can be used
 *  in these configurations.
 *  Based on OMAP4 ACLVFE audio codec driver
 *
 * Author:      Deepu Raj <deepu.raj@ti.com>
 * Author:      Vaibhav Bedia <vaibhav.bedia@ti.com>
 * Copyright:   (C) 2011  Texas Instruments, India
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

#define ACLVFE_RATES	SNDRV_PCM_RATE_8000_96000
#define ACLVFE_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static int aclvfe_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai);

static int aclvfe_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir);

static int aclvfe_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt);

static struct snd_soc_codec_driver soc_codec_aclvfe;

static struct snd_soc_dai_ops aclvfe_dai_ops = {
	.hw_params	= aclvfe_hw_params,
//	.digital_mute	= aclvfe_mute,
	.set_sysclk	= aclvfe_set_dai_sysclk,
	.set_fmt	= aclvfe_set_dai_fmt,
};
/* Dummy dai driver for ACLVFE */
static struct snd_soc_dai_driver aclvfe_dai = {
	.name = "ACLVFE-DAI-CODEC",
	.capture = {
			.stream_name = "Capture",
			.channels_min = 2,
			.channels_max = 2,
			.rates = ACLVFE_RATES,
			.formats = ACLVFE_FORMATS,},
	.ops = &aclvfe_dai_ops,
};

static int aclvfe_hw_params(struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct snd_soc_dai *dai)
{
	return 0;
}

static int aclvfe_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int aclvfe_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	return 0;
}
static int aclvfe_codec_probe(struct platform_device *pdev)
{
	int ret;

	ret = snd_soc_register_codec(&pdev->dev, &soc_codec_aclvfe,
					&aclvfe_dai, 1);
	if (ret < 0)
		printk(KERN_INFO " ACLVFE Codec Register Failed\n");

	return ret;
}

static int aclvfe_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver aclvfe_codec_driver = {
	.probe		= aclvfe_codec_probe,
	.remove		= aclvfe_codec_remove,
	.driver		= {
		.name	= "aclvfe-dummy-codec",
		.owner	= THIS_MODULE,
	},
};

static int __init aclvfe_modinit(void)
{
	return platform_driver_register(&aclvfe_codec_driver);
}

static void __exit aclvfe_exit(void)
{
	platform_driver_unregister(&aclvfe_codec_driver);
}

module_init(aclvfe_modinit);
module_exit(aclvfe_exit);

MODULE_AUTHOR("Vaibhav Bedia <vaibhav.bedia@ti.com>");
MODULE_DESCRIPTION(" ACLVFE Dummy codec Interface");
MODULE_LICENSE("GPL");
