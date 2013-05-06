/*
 * ALSA SoC I2S Audio Layer for Broadcom BCM2708 SoC
 *
 * Author:      Florian Meier, <koalo@koalo.de>
 *              Copyright 2013
 *
 * based on 
 *	Raspberry Pi PCM I2S ALSA Driver
 *	Copyright (c) by Phil Poole 2013
 *
 * 	ALSA SoC I2S (McBSP) Audio Layer for TI DAVINCI processor
 *      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * 	Copyright (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * 	OMAP ALSA SoC DAI driver using McBSP port
 * 	Copyright (C) 2008 Nokia Corporation
 * 	Contact: Jarkko Nikula <jarkko.nikula@bitmer.com>
 * 		 Peter Ujfalusi <peter.ujfalusi@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include "bcm2708-pcm.h"

/* Clock registers */
#define BCM2708_CLK_PCMCTL_REG  0x00
#define BCM2708_CLK_PCMDIV_REG  0x04

/* Clock register settings */
#define BCM2708_CLK_PASSWD	(0x5a000000)
#define BCM2708_CLK_MASH(v)	((v) << 9)
#define BCM2708_CLK_FLIP	(1 << 8)
#define BCM2708_CLK_BUSY	(1 << 7)
#define BCM2708_CLK_KILL	(1 << 5)
#define BCM2708_CLK_ENAB	(1 << 4)
#define BCM2708_CLK_SRC(v)	(v)

#define BCM2708_CLK_DIVI(v)	((v) << 12)
#define BCM2708_CLK_DIVF(v)	(v)

enum {
	BCM2708_CLK_MASH_0 = 0,
	BCM2708_CLK_MASH_1,
	BCM2708_CLK_MASH_2,
	BCM2708_CLK_MASH_3,
};

enum {
	BCM2708_CLK_SRC_GND = 0,
	BCM2708_CLK_SRC_OSC,
	BCM2708_CLK_SRC_DBG0,
	BCM2708_CLK_SRC_DBG1,
	BCM2708_CLK_SRC_PLLA,
	BCM2708_CLK_SRC_PLLC,
	BCM2708_CLK_SRC_PLLD,
	BCM2708_CLK_SRC_HDMI,
};

/* Most clocks are not useable (freq = 0) */
static const unsigned int bcm2708_clk_freq[BCM2708_CLK_SRC_HDMI+1] = {
	[BCM2708_CLK_SRC_GND] 		= 0,
	[BCM2708_CLK_SRC_OSC]		= 19200000,
	[BCM2708_CLK_SRC_DBG0]		= 0,
	[BCM2708_CLK_SRC_DBG1]		= 0,
	[BCM2708_CLK_SRC_PLLA]		= 0,
	[BCM2708_CLK_SRC_PLLC]		= 0,
	[BCM2708_CLK_SRC_PLLD]		= 500000000,
	[BCM2708_CLK_SRC_HDMI]		= 0,
};


/* I2S registers */
#define BCM2708_I2S_CS_A_REG    	0x00
#define BCM2708_I2S_FIFO_A_REG  	0x04
#define BCM2708_I2S_MODE_A_REG  	0x08
#define BCM2708_I2S_RXC_A_REG   	0x0c
#define BCM2708_I2S_TXC_A_REG   	0x10
#define BCM2708_I2S_DREQ_A_REG  	0x14
#define BCM2708_I2S_INTEN_A_REG 	0x18
#define BCM2708_I2S_INTSTC_A_REG	0x1c
#define BCM2708_I2S_GRAY_REG    	0x20

/* I2S register settings */
#define BCM2708_I2S_STBY		(1 << 25)
#define BCM2708_I2S_SYNC		(1 << 24)
#define BCM2708_I2S_RXSEX		(1 << 23)
#define BCM2708_I2S_RXF			(1 << 22)
#define BCM2708_I2S_TXE			(1 << 21)
#define BCM2708_I2S_RXD			(1 << 20)
#define BCM2708_I2S_TXD			(1 << 19)
#define BCM2708_I2S_RXR			(1 << 18)
#define BCM2708_I2S_TXW			(1 << 17)
#define BCM2708_I2S_CS_RXERR		(1 << 16)
#define BCM2708_I2S_CS_TXERR		(1 << 15)
#define BCM2708_I2S_RXSYNC		(1 << 14)
#define BCM2708_I2S_TXSYNC		(1 << 13)
#define BCM2708_I2S_DMAEN		(1 << 9)
#define BCM2708_I2S_RXTHR(v)		((v) << 7)
#define BCM2708_I2S_TXTHR(v)		((v) << 5)
#define BCM2708_I2S_RXCLR		(1 << 4)
#define BCM2708_I2S_TXCLR		(1 << 3)
#define BCM2708_I2S_TXON		(1 << 2)
#define BCM2708_I2S_RXON		(1 << 1)
#define BCM2708_I2S_EN			(1)

#define BCM2708_I2S_CLKDIS		(1 << 28)
#define BCM2708_I2S_PDMN		(1 << 27)
#define BCM2708_I2S_PDME		(1 << 26)
#define BCM2708_I2S_FRXP		(1 << 25)
#define BCM2708_I2S_FTXP		(1 << 24)
#define BCM2708_I2S_CLKM		(1 << 23)
#define BCM2708_I2S_CLKI		(1 << 22)
#define BCM2708_I2S_FSM			(1 << 21)
#define BCM2708_I2S_FSI			(1 << 20)
#define BCM2708_I2S_FLEN(v)		((v) << 10)
#define BCM2708_I2S_FSLEN(v)		(v)

#define BCM2708_I2S_CHWEX 		(1 << 15)
#define BCM2708_I2S_CHEN 		(1 << 14)
#define BCM2708_I2S_CHPOS(v)		((v) << 4)
#define BCM2708_I2S_CHWID(v)		(v)
#define BCM2708_I2S_CH1(v) 		((v) << 16)
#define BCM2708_I2S_CH2(v) 		(v)

#define BCM2708_I2S_TX_PANIC(v)		((v) << 24)
#define BCM2708_I2S_RX_PANIC(v)		((v) << 16)
#define BCM2708_I2S_TX(v)		((v) << 8)
#define BCM2708_I2S_RX(v)		(v)

#define BCM2708_I2S_INT_RXERR		(1 << 3)
#define BCM2708_I2S_INT_TXERR		(1 << 2)
#define BCM2708_I2S_INT_RXR		(1 << 1)
#define BCM2708_I2S_INT_TXW		(1 << 0)

/* I2S DMA interface */
#define BCM2708_I2S_FIFO_PHYSICAL_ADDR	0x7E203004
#define BCM2708_DMA_DREQ_PCM_TX 	2
#define BCM2708_DMA_DREQ_PCM_RX 	3

/* General device struct */
struct bcm2708_i2s_dev {
	struct device *dev;
	struct bcm2708_pcm_dma_data	dma_params[2];
	void __iomem			*i2s_base;
	void __iomem			*clk_base;
	unsigned int fmt;
};

static inline void bcm2708_i2s_write_reg(struct bcm2708_i2s_dev *dev,
					   int reg, u32 val)
{
	dev_dbg(dev->dev, "I2S write to register %p = %x\n",dev->clk_base + reg,val);
	__raw_writel(val, dev->i2s_base + reg);
}

static inline u32 bcm2708_i2s_read_reg(struct bcm2708_i2s_dev *dev, int reg)
{
	return __raw_readl(dev->i2s_base + reg);
}

static inline void bcm2708_i2s_set_bits(struct bcm2708_i2s_dev *dev,
					   int reg, u32 val)
{
	u32 oldval = __raw_readl(dev->i2s_base + reg);
	bcm2708_i2s_write_reg(dev, reg, oldval | val);
}

static inline void bcm2708_i2s_clear_bits(struct bcm2708_i2s_dev *dev,
					   int reg, u32 val)
{
	u32 oldval = __raw_readl(dev->i2s_base + reg);
	bcm2708_i2s_write_reg(dev, reg, oldval & ~val);
}

static inline void bcm2708_clk_write_reg(struct bcm2708_i2s_dev *dev,
					   int reg, u32 val)
{
	dev_dbg(dev->dev, "PCM clk write to register %p = %x\n",dev->clk_base + reg,val);
	__raw_writel(val, dev->clk_base + reg);
}

static inline u32 bcm2708_clk_read_reg(struct bcm2708_i2s_dev *dev, int reg)
{
	return __raw_readl(dev->clk_base + reg);
}

static void bcm2708_i2s_stop_clock(struct bcm2708_i2s_dev *dev)
{
	int timeout = 1000;

        /* stop clock */
        unsigned int clkreg = bcm2708_clk_read_reg(dev, BCM2708_CLK_PCMCTL_REG);
        bcm2708_clk_write_reg(dev, BCM2708_CLK_PCMCTL_REG, ~(BCM2708_CLK_ENAB)
			& (BCM2708_CLK_PASSWD | clkreg) );

	/* wait for the BUSY flag going down */
	while((bcm2708_clk_read_reg(dev, BCM2708_CLK_PCMCTL_REG) & BCM2708_CLK_BUSY) 
			&& timeout > 0)
	{ 
		timeout--;
	}

	if(timeout <= 0)
	{
		/* KILL the clock */
		dev_err(dev->dev, "I2S clock didn't stop. Kill the clock!\n");
		bcm2708_clk_write_reg(dev, BCM2708_CLK_PCMCTL_REG, BCM2708_CLK_KILL 
			| BCM2708_CLK_PASSWD | clkreg );
	}
}

static void bcm2708_i2s_clear_fifo(struct bcm2708_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	int timeout = 1000000;

	/* clear the FIFO - requires at least 2 PCM clock cycles to take effect*/
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) 
	{
        	bcm2708_i2s_set_bits(dev, BCM2708_I2S_CS_A_REG, BCM2708_I2S_RXCLR);
	}
	else
	{
        	bcm2708_i2s_set_bits(dev, BCM2708_I2S_CS_A_REG, BCM2708_I2S_TXCLR);
	}
	
	/* Wait for 2 PCM clock cycles */

	/*
	 * Set the SYNC flag - after 2 PCM clock cycles it will be read as high 
	 * FIXME: This does not seem to work!
	 */
	bcm2708_i2s_set_bits(dev, BCM2708_I2S_CS_A_REG, BCM2708_I2S_SYNC);

	/* Wait for the SYNC flag going up */
	while(!(bcm2708_clk_read_reg(dev, BCM2708_I2S_CS_A_REG) & BCM2708_I2S_SYNC)
			&& timeout > 0)
	{ 
		timeout--;
	}

	if(timeout <= 0)
	{
		dev_err(dev->dev, "I2S SYNC error!\n");
	}
}

static void bcm2708_i2s_start(struct bcm2708_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	/* 
	 * Start the clock if in master mode.
	 * If it is already running nothing happens.
	 * TODO Needs testing: Is no clock in slave mode ok?
	 */
	unsigned int master = dev->fmt & SND_SOC_DAIFMT_MASTER_MASK;
	if(master == SND_SOC_DAIFMT_CBS_CFS || master == SND_SOC_DAIFMT_CBS_CFM)
	{
		unsigned int clkreg = bcm2708_clk_read_reg(dev, BCM2708_CLK_PCMCTL_REG);
		bcm2708_clk_write_reg(dev, BCM2708_CLK_PCMCTL_REG, BCM2708_CLK_PASSWD
				| clkreg
				| BCM2708_CLK_ENAB);
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) 
	{
        	bcm2708_i2s_set_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_RXON);
	}
	else
	{
        	bcm2708_i2s_set_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_TXON);
	}
}

static void bcm2708_i2s_stop(struct bcm2708_i2s_dev *dev,
		struct snd_pcm_substream *substream)
{
	if(substream->stream == SNDRV_PCM_STREAM_CAPTURE)
	{
        	bcm2708_i2s_clear_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_RXON);
	}
	else
	{
        	bcm2708_i2s_clear_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_TXON);
	}

	/* Check if both streams are deactivated and not SND_SOC_DAIFMT_CONT */
	if(((bcm2708_i2s_read_reg(dev,BCM2708_I2S_CS_A_REG) 
				& (BCM2708_I2S_TXON | BCM2708_I2S_RXON)) == 0)
				&& !(dev->fmt & SND_SOC_DAIFMT_CONT))
	{
		bcm2708_i2s_stop_clock(dev);
	}
}

static int bcm2708_i2s_set_dai_fmt(struct snd_soc_dai *dai,
				      unsigned int fmt)
{
        struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	dev->fmt = fmt;
	return 0;
}

static int bcm2708_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
        struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	unsigned int sampling_rate = params_rate(params);
	unsigned int data_length, data_delay, half_frame;
	unsigned int ch1pos, ch2pos, mode, format;
	unsigned int mash = BCM2708_CLK_MASH_1;
	unsigned int divi, divf, target_frequency;
	int clk_src = -1;
	unsigned int master = dev->fmt & SND_SOC_DAIFMT_MASTER_MASK;
	unsigned int bit_master = (master == SND_SOC_DAIFMT_CBS_CFS || master == SND_SOC_DAIFMT_CBS_CFM);
	unsigned int frame_master = (master == SND_SOC_DAIFMT_CBS_CFS || master == SND_SOC_DAIFMT_CBM_CFS);

	/* should this still be running stop it */
	bcm2708_i2s_stop_clock(dev);

	/* enable PCM block */
        bcm2708_i2s_set_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_EN);

	/* disable STBY - requires at least 4 PCM clock cycles to take effect */
        bcm2708_i2s_set_bits(dev, BCM2708_I2S_CS_A_REG, BCM2708_I2S_STBY);

	/*
	 * Adjust the data length according to the format.
	 * We prefill the half frame length with an integer 
	 * divider of 2400 as explained at the clock settings.
	 * Maybe it is overwritten there, if the Integer mode 
	 * does not apply.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		data_length = 16;
		half_frame = 20;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		data_length = 32;
		half_frame = 40;
		break;
	default:
		return -EINVAL;
	}

	/* 
	 * Clock Settings
	 * 
	 * The target frequency of the bit clock is
	 * 	sampling rate * frame length
	 * 
	 * Integer mode:
	 * Sampling rates that are multiples of 8000 kHz
	 * can be driven by the oscillator of 19.2 MHz
	 * with an integer divider as long as the frame length 
	 * is an integer divider of 19200000/8000=2400 as set up above. 
	 * This is no longer possible if the sampling rate
	 * is too high (e.g. 192 kHz), because the oscillator is too slow.
	 *
	 * MASH mode:
	 * For all other sampling rates, it is not possible to
	 * have an integer divider. Approximate the clock
	 * with the MASH module that induces a slight frequency
	 * variance. To minimize that it is best to have the fastest
	 * clock here. That is PLLD with 500 MHz.
	 */
	target_frequency = sampling_rate*half_frame*2;
	clk_src = BCM2708_CLK_SRC_OSC;
	mash = BCM2708_CLK_MASH_0;

	if(bcm2708_clk_freq[clk_src] % target_frequency == 0 && bit_master && frame_master) {
		divi = bcm2708_clk_freq[clk_src]/target_frequency;
		divf = 0;
	}
	else {
		uint64_t dividend;

		half_frame = data_length*2; /* overwrite half frame length, because the above trick is not needed */
		target_frequency = sampling_rate*half_frame*2;

		clk_src = BCM2708_CLK_SRC_PLLD;
		mash = BCM2708_CLK_MASH_1;
		
		dividend = bcm2708_clk_freq[clk_src];
		dividend *= 1024;
		do_div(dividend, target_frequency);
		divi = dividend / 1024;
		divf = dividend % 1024;
	}

        /* Set clock divider */
        bcm2708_clk_write_reg(dev, BCM2708_CLK_PCMDIV_REG, BCM2708_CLK_PASSWD 
			| BCM2708_CLK_DIVI(divi)
			| BCM2708_CLK_DIVF(divf));

        /* Setup clock, but don't start it yet */
        bcm2708_clk_write_reg(dev, BCM2708_CLK_PCMCTL_REG, BCM2708_CLK_PASSWD
			| BCM2708_CLK_MASH(mash)
			//| BCM2708_CLK_ENAB
			| BCM2708_CLK_SRC(clk_src) );


	/* Setup the frame format */
	format = BCM2708_I2S_CHEN;

	if(data_length > 24) {
		format |= BCM2708_I2S_CHWEX;
	}
	format |= BCM2708_I2S_CHWID((data_length-8)&0xf);

	switch (dev->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		data_delay = 1;
		break;
	default: 
		// TODO others are possible but are not implemented at the moment
                dev_err(dev->dev, "%s:bad format\n", __func__);
		return -EINVAL;
	}

	ch1pos = data_delay;
	ch2pos = half_frame+data_delay;

	switch(params_channels(params)) {
	case 2:
		format = BCM2708_I2S_CH1(format) | BCM2708_I2S_CH2(format);
		format |= BCM2708_I2S_CH1(BCM2708_I2S_CHPOS(ch1pos));
		format |= BCM2708_I2S_CH2(BCM2708_I2S_CHPOS(ch2pos));
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
		bcm2708_i2s_write_reg(dev, BCM2708_I2S_RXC_A_REG, format);
	}
	else {
		bcm2708_i2s_write_reg(dev, BCM2708_I2S_TXC_A_REG, format);
	}

 
	/* Setup the I2S mode */
	mode = 0;

	if(data_length <= 16) {
		mode = bcm2708_i2s_read_reg(dev, BCM2708_I2S_MODE_A_REG);

		/* Use frame packed mode (2 channels per 32 bit word) */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE) {
			/* preserve setting for other direction */
			mode &= BCM2708_I2S_FTXP;

			/* set for this direction */
			mode |= BCM2708_I2S_FRXP;
		}
		else {
			/* preserve setting for other direction */
			mode &= BCM2708_I2S_FRXP;

			/* set for this direction */
			mode |= BCM2708_I2S_FTXP;
		}
	}

	mode |= BCM2708_I2S_FLEN(half_frame*2-1);
	mode |= BCM2708_I2S_FSLEN(half_frame); 

	/* master or slave? */
	switch (dev->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* cpu is master */
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		/* 
		 * codec is bit clock master
	         * cpu is frame master
		 */
		mode |= BCM2708_I2S_CLKM;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		/* 
		 * codec is frame master
	         * cpu is bit clock master
		 */
		mode |= BCM2708_I2S_FSM;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		/* codec is master */
		mode |= BCM2708_I2S_CLKM;
		mode |= BCM2708_I2S_FSM;
		break;
	default:
                dev_err(dev->dev, "%s:bad master\n", __func__);
		return -EINVAL;
	}

	/* 
	 * Invert clocks? 
	 * 
	 * The BCM approach seems to be inverted to the classical I2S approach.
	 */
	switch (dev->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/* none - therefore, both for BCM */
		mode |= BCM2708_I2S_CLKI;
		mode |= BCM2708_I2S_FSI;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		/* both - therefore, none for BCM*/
		break;
	case SND_SOC_DAIFMT_NB_IF:
		/* invert only frame sync - therefore, invert only bit clock for BCM */
		mode |= BCM2708_I2S_CLKI;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* invert only bit clock - therefore, invert only frame sync for BCM*/
		mode |= BCM2708_I2S_FSI;
		break;
	default:
		return -EINVAL;
	}

        bcm2708_i2s_write_reg(dev, BCM2708_I2S_MODE_A_REG, mode);


	/* Setup the DMA parameters */
        bcm2708_i2s_set_bits(dev, BCM2708_I2S_CS_A_REG, BCM2708_I2S_RXTHR(1) 
			| BCM2708_I2S_TXTHR(1)
			| BCM2708_I2S_DMAEN);

        bcm2708_i2s_write_reg(dev,BCM2708_I2S_DREQ_A_REG, 
			  BCM2708_I2S_TX_PANIC(0x10)
			| BCM2708_I2S_RX_PANIC(0x30)
			| BCM2708_I2S_TX(0x30)
			| BCM2708_I2S_RX(0x20));

	return 0;
}


static int bcm2708_i2s_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	bcm2708_i2s_clear_fifo(dev, substream);
	return 0;
}

static int bcm2708_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		bcm2708_i2s_start(dev, substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		bcm2708_i2s_stop(dev, substream);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bcm2708_i2s_startup(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *dai)
{
	struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);
	snd_soc_dai_set_dma_data(dai, substream, &dev->dma_params[substream->stream]);
	return 0;
}

static void bcm2708_i2s_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct bcm2708_i2s_dev *dev = snd_soc_dai_get_drvdata(dai);

	bcm2708_i2s_stop(dev, substream);

	/* If both streams are stopped, disable module and clock */
	if((bcm2708_i2s_read_reg(dev,BCM2708_I2S_CS_A_REG) 
				& (BCM2708_I2S_TXON | BCM2708_I2S_RXON)) == 0)
	{
		/* Disable the module */
        	bcm2708_i2s_clear_bits(dev,BCM2708_I2S_CS_A_REG, BCM2708_I2S_EN);

		/* 
	         * Stopping clock is necessary, because stop does 
		 * not stop the clock when SND_SOC_DAIFMT_CONT 
		 */
		bcm2708_i2s_stop_clock(dev);
	}
}

static const struct snd_soc_dai_ops bcm2708_i2s_dai_ops = {
	.startup	= bcm2708_i2s_startup,
	.shutdown	= bcm2708_i2s_shutdown,
	.prepare	= bcm2708_i2s_prepare,
	.trigger	= bcm2708_i2s_trigger,
	.hw_params	= bcm2708_i2s_hw_params,
	.set_fmt	= bcm2708_i2s_set_dai_fmt,
};

static struct snd_soc_dai_driver bcm2708_i2s_dai = {
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = 	SNDRV_PCM_RATE_8000_192000,
		.formats = 	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE
		},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = 	SNDRV_PCM_RATE_8000_192000,
		.formats = 	SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE
		},
	.ops = &bcm2708_i2s_dai_ops,
};

static void bcm2708_i2s_setup_gpio(void)
{
	/* TODO Handle this in the device tree! */
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

        {
unsigned int *gpio;
	int pin;
	gpio = ioremap(GPIO_BASE, SZ_16K);

	/* SPI is on GPIO 7..11 */
	for (pin = 28; pin <= 31; pin++) {
		INP_GPIO(pin);		/* set mode to GPIO input first */
		SET_GPIO_ALT(pin, 2);	/* set mode to ALT 0 */
	}
        } 

#undef INP_GPIO
#undef SET_GPIO_ALT
}

static int bcm2708_i2s_probe(struct platform_device *pdev)
{
	struct bcm2708_i2s_dev *dev;
        int i;
	void __iomem *base[2];

        /* request both ioareas */
        for(i = 0; i <= 1; i++)
        {
                struct resource *mem, *ioarea;
                mem = platform_get_resource(pdev, IORESOURCE_MEM, i);
                if (!mem) 
                {
                       dev_err(&pdev->dev, "I2S probe: Memory resource could not be found.\n");
                       return -ENODEV;
                }

                ioarea = devm_request_mem_region(&pdev->dev, mem->start,
                                           resource_size(mem),
                                           pdev->name);
                if (!ioarea) 
                {
                        dev_err(&pdev->dev, "I2S probe: Memory region already claimed.\n");
                        return -EBUSY;
                }

                base[i] = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
                if (!base[i]) {
                        dev_err(&pdev->dev, "I2S probe: ioremap failed.\n");
                        return -ENOMEM;
                }
        }

        dev = devm_kzalloc(&pdev->dev, sizeof(struct bcm2708_i2s_dev),
                           GFP_KERNEL);
        if (!dev)
	{
        	dev_err(&pdev->dev, "I2S probe: kzalloc failed.\n");
                return -ENOMEM;
	}
          
        dev->i2s_base = base[0];
        dev->clk_base = base[1];

	bcm2708_i2s_setup_gpio();

	/* Set the appropriate DMA parameters */
	dev->dma_params[SNDRV_PCM_STREAM_PLAYBACK].port_addr =
	    (dma_addr_t)BCM2708_I2S_FIFO_PHYSICAL_ADDR;

	dev->dma_params[SNDRV_PCM_STREAM_CAPTURE].port_addr =
	    (dma_addr_t)BCM2708_I2S_FIFO_PHYSICAL_ADDR;

	dev->dma_params[SNDRV_PCM_STREAM_PLAYBACK].dma_req = BCM2708_DMA_DREQ_PCM_TX;
	dev->dma_params[SNDRV_PCM_STREAM_CAPTURE].dma_req = BCM2708_DMA_DREQ_PCM_RX;

	/* Store the pdev */
	dev->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, dev);

	return snd_soc_register_dai(&pdev->dev, &bcm2708_i2s_dai);
}

static int bcm2708_i2s_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver bcm2708_i2s_driver = {
	.probe		= bcm2708_i2s_probe,
	.remove		= bcm2708_i2s_remove,
	.driver		= {
		.name	= "bcm2708-i2s",
		.owner	= THIS_MODULE,
	},
};

module_platform_driver(bcm2708_i2s_driver);

MODULE_AUTHOR("Florian Meier");
MODULE_DESCRIPTION("BCM2708 I2S Interface");
MODULE_LICENSE("GPL");
