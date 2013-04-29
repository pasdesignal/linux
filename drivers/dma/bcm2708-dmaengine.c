/*
 * BCM2708 DMA engine support
 *
 * This driver only supports cyclic DMA transfers 
 * as needed for the I2S module.
 *
 * Author:      Florian Meier, <koalo@koalo.de>
 *              Copyright 2013
 *
 * based on 
 * 	OMAP DMAengine support by Russell King
 *
 *	Raspberry Pi PCM I2S ALSA Driver
 *	Copyright (c) by Phil Poole 2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/bcm2708-dmaengine.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/irq.h>

#include "virt-dma.h"

#include <mach/dma.h>
#include <mach/irqs.h>

struct bcm2708_dmadev {
	struct dma_device ddev;
	spinlock_t lock;
	struct tasklet_struct task;
	struct list_head pending;
};

struct bcm2708_chan {
	struct virt_dma_chan vc;
	struct list_head node;

	struct bcm2708_dma_slave_config	cfg;
	unsigned dma_sig;
	bool cyclic;

	int dma_ch;
	struct bcm2708_desc *desc;
	unsigned sgidx;

        void __iomem *dma_chan_base;
        int dma_irq_number;
        struct irqaction dma_irq_handler;
};

struct bcm2708_sg {
	dma_addr_t addr;
	uint32_t en;		/* number of elements (24-bit) */
	uint32_t fn;		/* number of frames (16-bit) */
};

struct bcm2708_desc {
	struct virt_dma_desc vd;
	enum dma_transfer_direction dir;
	dma_addr_t dev_addr;

	uint8_t es;
	unsigned int sync_type;
	unsigned int sync_dreq;

	unsigned int control_block_size;
  	struct bcm2708_dma_cb *control_block_base;
  	dma_addr_t control_block_base_phys;

	unsigned sglen;
	struct bcm2708_sg sg[0];
};

#define BCM2708_DMA_DATA_TYPE_S8 1
#define BCM2708_DMA_DATA_TYPE_S16 2
#define BCM2708_DMA_DATA_TYPE_S32 4
#define BCM2708_DMA_DATA_TYPE_S128 16

static const unsigned es_bytes[] = {
	[BCM2708_DMA_DATA_TYPE_S8] = 1,
	[BCM2708_DMA_DATA_TYPE_S16] = 2,
	[BCM2708_DMA_DATA_TYPE_S32] = 4,
        [BCM2708_DMA_DATA_TYPE_S128] = 16
};

static inline struct bcm2708_dmadev *to_bcm2708_dma_dev(struct dma_device *d)
{
	return container_of(d, struct bcm2708_dmadev, ddev);
}

static inline struct bcm2708_chan *to_bcm2708_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct bcm2708_chan, vc.chan);
}

static inline struct bcm2708_desc *to_bcm2708_dma_desc(struct dma_async_tx_descriptor *t)
{
	return container_of(t, struct bcm2708_desc, vd.tx);
}

static void bcm2708_dma_desc_free(struct virt_dma_desc *vd)
{
	struct bcm2708_desc *desc = container_of(vd, struct bcm2708_desc, vd);
        dma_free_coherent(desc->vd.tx.chan->device->dev, 
			desc->control_block_size, 
			desc->control_block_base, 
			desc->control_block_base_phys);
	kfree(desc);
}

static void bcm2708_dma_start_sg(struct bcm2708_chan *c, struct bcm2708_desc *d,
		unsigned idx)
{
	struct bcm2708_sg *sg = d->sg + idx;
	int frame;
	int frames = sg->fn;

	/*
	 * Iterate over all frames and create a control block
	 * for each frame and link them together.
	 */
	for(frame = 0; frame < frames; frame++)
	{
		struct bcm2708_dma_cb *control_block = &d->control_block_base[frame];

		/* Setup adresses */
		if (d->dir == DMA_DEV_TO_MEM)
		{
			control_block->info = BCM2708_DMA_D_INC;
			control_block->src = d->dev_addr;
			control_block->dst = sg->addr+frame*sg->en;
		}
		else
		{
			control_block->info = BCM2708_DMA_S_INC;
			control_block->src = sg->addr+frame*sg->en;
			control_block->dst = d->dev_addr;
		}

		/* Enable interrupt */
		control_block->info |= BCM2708_DMA_INT_EN;

		/* Setup synchronization */
		if(d->sync_type != 0)
			control_block->info |= d->sync_type;

		/* Setup DREQ channel */
		if(d->sync_dreq != 0)
			control_block->info |= BCM2708_DMA_PER_MAP(d->sync_dreq);

		/* Length of a frame */
		control_block->length=sg->en; 
		
		/* 
		 * Next block is the next frame.
		 * This DMA engine driver currently only supports cyclic DMA.
		 * Therefore, wrap around at number of frames.
		 */
		control_block->next = d->control_block_base_phys +
			sizeof(struct bcm2708_dma_cb)*((frame+1)%(frames));

		/* The following fields are not used here */
		control_block->stride=0;
		control_block->pad[0]=0;
		control_block->pad[1]=0;
	}

	/* Start the DMA transfer */
	bcm_dma_start(c->dma_chan_base, d->control_block_base_phys);
}

static void bcm2708_dma_start_desc(struct bcm2708_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vc);
	struct bcm2708_desc *d;

	if (!vd) {
		c->desc = NULL;
		return;
	}

	list_del(&vd->node);

	c->desc = d = to_bcm2708_dma_desc(&vd->tx);
	c->sgidx = 0;

	bcm2708_dma_start_sg(c, d, 0);
}

static irqreturn_t bcm2708_dma_callback(int irq, void *data)
{
	struct bcm2708_chan *c = data;
	struct bcm2708_desc *d;
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);

    	/* acknowledge interrupt */
	writel(BCM2708_DMA_INT, c->dma_chan_base + BCM2708_DMA_CS);

	d = c->desc;

	if (d) {
		if (!c->cyclic) {
			if (++c->sgidx < d->sglen) {
				bcm2708_dma_start_sg(c, d, c->sgidx);
			} else {
				bcm2708_dma_start_desc(c);
				vchan_cookie_complete(&d->vd);
			}
		} else {
			vchan_cyclic_callback(&d->vd);
		}
	}

	/* keep the DMA engine running */
	dsb(); /* ARM synchronization barrier */
	writel(BCM2708_DMA_ACTIVE, c->dma_chan_base + BCM2708_DMA_CS);

	spin_unlock_irqrestore(&c->vc.lock, flags);

	return IRQ_HANDLED;
}

/*
 * This callback schedules all pending channels.  We could be more
 * clever here by postponing allocation of the real DMA channels to
 * this point, and freeing them when our virtual channel becomes idle.
 *
 * We would then need to deal with 'all channels in-use'
 */
static void bcm2708_dma_sched(unsigned long data)
{
	struct bcm2708_dmadev *d = (struct bcm2708_dmadev *)data;
	LIST_HEAD(head);

	spin_lock_irq(&d->lock);
	list_splice_tail_init(&d->pending, &head);
	spin_unlock_irq(&d->lock);

	while (!list_empty(&head)) {
		struct bcm2708_chan *c = list_first_entry(&head,
			struct bcm2708_chan, node);

		spin_lock_irq(&c->vc.lock);
		list_del_init(&c->node);
		bcm2708_dma_start_desc(c);
		spin_unlock_irq(&c->vc.lock);
	}
}

static int bcm2708_dma_alloc_chan_resources(struct dma_chan *chan)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);
        int ret;

	dev_info(c->vc.chan.device->dev, "allocating channel for %u\n", c->dma_sig);

        ret = bcm_dma_chan_alloc(BCM_DMA_FEATURE_FAST,
				 &c->dma_chan_base,
				 &c->dma_irq_number);
        if (ret < 0) 
                return ret;

        c->dma_ch = ret;


        c->dma_irq_handler.name = "DMA engine IRQ handler";
        c->dma_irq_handler.flags = 0;
	c->dma_irq_handler.handler = bcm2708_dma_callback;

	ret = request_any_context_irq(c->dma_irq_number, bcm2708_dma_callback, 0, "DMA IRQ", c);
    	if (ret < 0)
		return ret;

        return 0;
}

static void bcm2708_dma_free_chan_resources(struct dma_chan *chan)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);

	vchan_free_chan_resources(&c->vc);
        bcm_dma_chan_free(c->dma_ch);
	free_irq(c->dma_irq_number, c);

	dev_info(c->vc.chan.device->dev, "freeing channel for %u\n", c->dma_sig);
}

static size_t bcm2708_dma_sg_size(struct bcm2708_sg *sg)
{
	return sg->en * sg->fn;
}

static size_t bcm2708_dma_desc_size(struct bcm2708_desc *d)
{
	unsigned i;
	size_t size;

	for (size = i = 0; i < d->sglen; i++)
		size += bcm2708_dma_sg_size(&d->sg[i]);

	return size * es_bytes[d->es];
}

static size_t bcm2708_dma_desc_size_pos(struct bcm2708_desc *d, dma_addr_t addr)
{
	unsigned i;
	size_t size;

	for (size = i = 0; i < d->sglen; i++) {
		size_t this_size = bcm2708_dma_sg_size(&d->sg[i]); 

		if (size)
			size += this_size;
		else if (addr >= d->sg[i].addr &&
			 addr < d->sg[i].addr + this_size)
			size += d->sg[i].addr + this_size - addr;
	}
	return size;
}


/*
 * Returns current physical source address for the given DMA channel.
 * If the channel is running the caller must disable interrupts prior calling
 * this function and process the returned value before re-enabling interrupt to
 * prevent races with the interrupt handler.
 */
dma_addr_t bcm2708_get_dma_src_pos(struct bcm2708_chan *c)
{
	return readl(c->dma_chan_base + BCM2708_DMA_SOURCE_AD);
}

/*
 * Returns current physical destination address for the given DMA channel.
 * If the channel is running the caller must disable interrupts prior calling
 * this function and process the returned value before re-enabling interrupt to
 * prevent races with the interrupt handler.
 */
dma_addr_t bcm2708_get_dma_dst_pos(struct bcm2708_chan *c)
{
	return readl(c->dma_chan_base + BCM2708_DMA_DEST_AD);
}

static enum dma_status bcm2708_dma_tx_status(struct dma_chan *chan,
	dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_SUCCESS || !txstate)
		return ret;

	spin_lock_irqsave(&c->vc.lock, flags);
	vd = vchan_find_desc(&c->vc, cookie);
	if (vd) {
		txstate->residue = bcm2708_dma_desc_size(to_bcm2708_dma_desc(&vd->tx));
	} else if (c->desc && c->desc->vd.tx.cookie == cookie) {
		struct bcm2708_desc *d = c->desc;
		dma_addr_t pos;

		if (d->dir == DMA_MEM_TO_DEV)
			pos = bcm2708_get_dma_src_pos(c);
		else if (d->dir == DMA_DEV_TO_MEM)
			pos = bcm2708_get_dma_dst_pos(c);
		else
			pos = 0;

		txstate->residue = bcm2708_dma_desc_size_pos(d, pos);
	} else {
		txstate->residue = 0;
	}

	spin_unlock_irqrestore(&c->vc.lock, flags);

	return ret;
}

static void bcm2708_dma_issue_pending(struct dma_chan *chan)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vc.lock, flags);
	if (vchan_issue_pending(&c->vc) && !c->desc) {
		struct bcm2708_dmadev *d = to_bcm2708_dma_dev(chan->device);
		spin_lock(&d->lock);
		if (list_empty(&c->node))
			list_add_tail(&c->node, &d->pending);
		spin_unlock(&d->lock);
		tasklet_schedule(&d->task);
	}
	spin_unlock_irqrestore(&c->vc.lock, flags);
}


struct dma_async_tx_descriptor *bcm2708_dma_prep_dma_cyclic(
	struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
	size_t period_len, enum dma_transfer_direction direction,
	unsigned long flags, void *context)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct bcm2708_desc *d;
	dma_addr_t dev_addr;
	unsigned int es, sync_type, sync_dreq;

	/* Grab configuration */
	if (direction == DMA_DEV_TO_MEM) {
		dev_addr = c->cfg.cfg.src_addr;
		dev_width = c->cfg.cfg.src_addr_width;
		sync_type = BCM2708_DMA_S_DREQ;
		sync_dreq = c->cfg.sync_dreq;
	} else if (direction == DMA_MEM_TO_DEV) {
		dev_addr = c->cfg.cfg.dst_addr;
		dev_width = c->cfg.cfg.dst_addr_width;
                sync_type = BCM2708_DMA_D_DREQ;		  
		sync_dreq = c->cfg.sync_dreq;
	} else {
		dev_err(chan->device->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	/* Bus width translates to the element size (ES) */
	switch (dev_width) {
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		es = BCM2708_DMA_DATA_TYPE_S32;
		break;
	default:
		return NULL;
	}

	/* Now allocate and setup the descriptor. */
	d = kzalloc(sizeof(*d) + sizeof(d->sg[0]), GFP_ATOMIC);
	if (!d)
		return NULL;

	d->dir = direction;
	d->dev_addr = dev_addr;
	d->es = es;
	d->sync_type = sync_type;
	d->sync_dreq = sync_dreq;
	d->sg[0].addr = buf_addr;
	d->sg[0].en = period_len;
	d->sg[0].fn = buf_len / period_len;
	d->sglen = 1;

	/* Allocate memory for control blocks */
	d->control_block_size = d->sg[0].fn*sizeof(struct bcm2708_dma_cb);
        d->control_block_base = dma_alloc_coherent(chan->device->dev, 
			d->control_block_size, &d->control_block_base_phys, GFP_KERNEL);
			  
	if(!d->control_block_base)
	{
		dev_err(chan->device->dev, "%s: Memory allocation error\n", __func__);
		return NULL;
	}

        memset(d->control_block_base, 0, d->control_block_size);

	if (!c->cyclic) {
		c->cyclic = true;
		/* nothing else is implemented */
	}

        return vchan_tx_prep(&c->vc, &d->vd, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
}

static int bcm2708_dma_slave_config(struct bcm2708_chan *c, struct dma_slave_config *cfg)
{
	if ((cfg->direction == DMA_DEV_TO_MEM && cfg->src_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES) ||
	    (cfg->direction == DMA_MEM_TO_DEV && cfg->dst_addr_width != DMA_SLAVE_BUSWIDTH_4_BYTES) )
	{
		return -EINVAL;
	}

	memcpy(&c->cfg, cfg, sizeof(c->cfg));

	return 0;
}

static int bcm2708_dma_terminate_all(struct bcm2708_chan *c)
{
	struct bcm2708_dmadev *d = to_bcm2708_dma_dev(c->vc.chan.device);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&c->vc.lock, flags);

	/* Prevent this channel being scheduled */
	spin_lock(&d->lock);
	list_del_init(&c->node);
	spin_unlock(&d->lock);

	/*
	 * Stop DMA activity: we assume the callback will not be called
	 * after bcm_dma_abort() returns (even if it does, it will see
	 * c->desc is NULL and exit.)
	 */
	if (c->desc) {
		c->desc = NULL;
		bcm_dma_abort(c->dma_chan_base);

		/* Wait for stopping */
  		while (readl(c->dma_chan_base + BCM2708_DMA_CS) & BCM2708_DMA_ACTIVE);
	}

	vchan_get_all_descriptors(&c->vc, &head);
	spin_unlock_irqrestore(&c->vc.lock, flags);
	vchan_dma_desc_free_list(&c->vc, &head);

	return 0;
}

static int bcm2708_dma_pause(struct bcm2708_chan *c)
{
	/* FIXME: not supported by platform private API */
	return -EINVAL;
}

static int bcm2708_dma_resume(struct bcm2708_chan *c)
{
	/* FIXME: not supported by platform private API */
	return -EINVAL;
}

static int bcm2708_dma_control(struct dma_chan *chan, enum dma_ctrl_cmd cmd,
	unsigned long arg)
{
	struct bcm2708_chan *c = to_bcm2708_dma_chan(chan);
	int ret;

	switch (cmd) {
	case DMA_SLAVE_CONFIG:
		ret = bcm2708_dma_slave_config(c, (struct dma_slave_config *)arg);
		break;

	case DMA_TERMINATE_ALL:
		ret = bcm2708_dma_terminate_all(c);
		break;

	case DMA_PAUSE:
		ret = bcm2708_dma_pause(c);
		break;

	case DMA_RESUME:
		ret = bcm2708_dma_resume(c);
		break;

	default:
		ret = -ENXIO;
		break;
	}

	return ret;
}

static int bcm2708_dma_chan_init(struct bcm2708_dmadev *od, int dma_sig)
{
	struct bcm2708_chan *c;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->dma_sig = dma_sig;
	c->vc.desc_free = bcm2708_dma_desc_free;
	vchan_init(&c->vc, &od->ddev);
	INIT_LIST_HEAD(&c->node);

	od->ddev.chancnt++;

	return 0;
}

static void bcm2708_dma_free(struct bcm2708_dmadev *od)
{
	tasklet_kill(&od->task);
	while (!list_empty(&od->ddev.channels)) {
		struct bcm2708_chan *c = list_first_entry(&od->ddev.channels,
			struct bcm2708_chan, vc.chan.device_node);

		list_del(&c->vc.chan.device_node);
		tasklet_kill(&c->vc.task);
		kfree(c);
	}
	kfree(od);
}

static int bcm2708_dma_probe(struct platform_device *pdev)
{
	struct bcm2708_dmadev *od;
	int rc, i;

	od = kzalloc(sizeof(*od), GFP_KERNEL);
	if (!od)
		return -ENOMEM;

	dma_cap_set(DMA_SLAVE, od->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, od->ddev.cap_mask);
	od->ddev.device_alloc_chan_resources = bcm2708_dma_alloc_chan_resources;
	od->ddev.device_free_chan_resources = bcm2708_dma_free_chan_resources;
	od->ddev.device_tx_status = bcm2708_dma_tx_status;
	od->ddev.device_issue_pending = bcm2708_dma_issue_pending;
	od->ddev.device_prep_dma_cyclic = bcm2708_dma_prep_dma_cyclic;
	od->ddev.device_control = bcm2708_dma_control;
	od->ddev.dev = &pdev->dev;
	INIT_LIST_HEAD(&od->ddev.channels);
	INIT_LIST_HEAD(&od->pending);
	spin_lock_init(&od->lock);

	tasklet_init(&od->task, bcm2708_dma_sched, (unsigned long)od);

	for (i = 0; i < 127; i++) {
		rc = bcm2708_dma_chan_init(od, i);
		if (rc) {
			bcm2708_dma_free(od);
			return rc;
		}
	}

	rc = dma_async_device_register(&od->ddev);
	if (rc) {
		pr_warn("BCM2708-DMA: failed to register slave DMA engine device: %d\n",
			rc);
		bcm2708_dma_free(od);
	} else {
		platform_set_drvdata(pdev, od);
	}

	dev_info(&pdev->dev, "BCM2708 DMA engine driver\n");

	return rc;
}

static int bcm2708_dma_remove(struct platform_device *pdev)
{
	struct bcm2708_dmadev *od = platform_get_drvdata(pdev);

	dma_async_device_unregister(&od->ddev);
	bcm2708_dma_free(od);

	return 0;
}

static struct platform_driver bcm2708_dma_driver = {
	.probe	= bcm2708_dma_probe,
	.remove	= bcm2708_dma_remove,
	.driver = {
		.name = "bcm2708-dmaengine",
		.owner = THIS_MODULE,
	},
};

static struct platform_device *pdev;

static const struct platform_device_info bcm2708_dma_dev_info = {
	.name = "bcm2708-dmaengine",
	.id = -1,
	.dma_mask = DMA_BIT_MASK(32),
};

static int bcm2708_dma_init(void)
{
	int rc = platform_driver_register(&bcm2708_dma_driver);

	if (rc == 0) {
		pdev = platform_device_register_full(&bcm2708_dma_dev_info);
		if (IS_ERR(pdev)) {
			platform_driver_unregister(&bcm2708_dma_driver);
			rc = PTR_ERR(pdev);
		}
	}
	return rc;
}
subsys_initcall(bcm2708_dma_init);

static void __exit bcm2708_dma_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&bcm2708_dma_driver);
}
module_exit(bcm2708_dma_exit);

MODULE_AUTHOR("Florian Meier");
MODULE_DESCRIPTION("BCM2708 DMA engine driver");
MODULE_LICENSE("GPL");

