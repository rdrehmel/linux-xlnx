#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/crc32.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/phylink.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/iopoll.h>
#include <linux/phy/phy.h>
#include <linux/pm_runtime.h>
#include <linux/ptp_classify.h>
#include <linux/reset.h>
#include <linux/firmware/xlnx-zynqmp.h>
#include <linux/crc32.h>
#include <linux/inetdevice.h>
#include <trace/events/irq.h>
#include "macb.h"
#include "macb_prism.h"

static int
prism_macb_init_queue(
	struct platform_device *pdev,
	struct macb_queue *queue,
	int q
)
{
	struct prism_sp *sp_rx;
	struct prism_sp *sp_tx;

	queue->SP_TER = PRISM_SP_TER;
	queue->SP_ISR = PRISM_SP_ISR;
	queue->SP_IER = PRISM_SP_IER;
	queue->SP_IDR = PRISM_SP_IDR;
	queue->SP_IMR = PRISM_SP_IMR;
	queue->SP_QP_LSB = PRISM_SP_QP_LSB;
	queue->SP_QP_MSB = PRISM_SP_QP_MSB;

	/*
	 * Handle the Prism SP.
	 */
	sp_rx = NULL;
	if (get_prism_sp(pdev, "prism-sp-rx", q, &sp_rx) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}
	queue->sp_rx = sp_rx;
	printk("Prism: %s: queue->sp_rx is %px\n", __func__, sp_rx);

	sp_tx = NULL;
	if (get_prism_sp(pdev, "prism-sp-tx", q, &sp_tx) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	}
	queue->sp_tx = sp_tx;
	printk("Prism: %s: queue->sp_tx is %px\n", __func__, sp_tx);

	if (sp_rx == NULL) {
		printk("Prism: %s: Come on, how are we supposed to use split IRQs when there is no SP RX?\n", __func__);
	}
	else {
		sp_rx->irq_handler = gem_prism_sp_desc_rx_hardirq_queue;
		sp_rx->irq_data = queue;
	}

	if (sp_tx == NULL) {
		printk("Prism: %s: Come on, how are we supposed to use split IRQs when there is no SP TX?\n", __func__);
	}
	else {
		sp_tx->irq_handler = gem_prism_sp_desc_tx_hardirq_queue;
		sp_tx->irq_data = queue;
	}

	return 0;
}

static int
prism_sp_count_queues(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int m, n;

	if ((m = of_property_count_u32_elems(np, "prism-sp-rx")) < 0) {
		printk("Prism: %s: Cannot find property prism-sp-rx, errval=%d\n", __func__, m);
		return 0;
	}
	if ((n = of_property_count_u32_elems(np, "prism-sp-tx")) < 0) {
		printk("Prism: %s: Cannot find property prism-sp-tx, errval=%d\n", __func__, n);
		return 0;
	}
	if (m != n) {
		printk("Prism: %s: Number of prism-sp-rx (%d) and prism-sp-tx (%d) doesn not match.\n",
			__func__, m, n);
		return 0;
	}
	printk("Prism: %s: This SP has %d queues\n", __func__, m);

	return n;
}

static int
get_prism_sp(struct platform_device *pdev, const char *s, int n, struct prism_sp **spp)
{
	struct device_node *np = pdev->dev.of_node;
	phandle rproc_phandle;

	if (of_property_read_u32_index(np, s, n, &rproc_phandle) == 0) {
		struct rproc *rproc;
		struct prism_sp *sp;

		rproc = rproc_get_by_phandle(rproc_phandle);
		if (rproc == NULL) {
			printk("Prism: %s: Could not find remote proc specified by property '%s', deferring probe.\n", __func__, s);
			return -EPROBE_DEFER;
		}
		// XXX Trust.
		// This is probably ok for research purposes but should be done
		// The Right Way(tm) for productive use.
		sp = (struct prism_sp *)rproc->priv;
		sp->booted = 0;
		*spp = sp;
		return 0;
	}
	printk("Prism: %s: No property with name '%s' found.\n", __func__, s);
	return -ENODEV;
}

static void gem_assign_no(struct macb *bp, struct resource *regs)
{
	switch (regs->start) {
	case 0xff0b0000:
		bp->no = 0;
		break;
	case 0xff0c0000:
		bp->no = 1;
		break;
	case 0xff0d0000:
		bp->no = 2;
		break;
	case 0xff0e0000:
		bp->no = 3;
		break;
	default:
		printk("Prism: %s: Couldn't assign a unique number for the GEM at %px\n", __func__, (void *)regs->start);
		break;
	}
}

const char *
prism_construct_fwfilename(const char *dir)
{
	char *p;
	int len;

	len = strlen("prism-sp-") +
		strlen(dir) +
		strlen("-firmware.elf");
	// Terminating NUL.
	len += 1;
	p = kmalloc(len, GFP_KERNEL);
	snprintf(p, len, "prism-sp-%s-firmware.elf", dir);

	return p;
}

static inline void
__gem_prism_sp_desc_rx_hardirq_queue(struct macb_queue *queue)
{

	if (prism_sp_gem_queue_read_rx_irq(queue)) {
		prism_sp_gem_queue_disable_rx_irq(queue);
		prism_sp_gem_queue_clear_rx_irq(queue);

		if (napi_schedule_prep(&queue->napi_rx)) {
			netdev_vdbg(queue->bp->dev, "scheduling RX softirq\n");
			__napi_schedule(&queue->napi_rx);
		}
	}
}

static irqreturn_t
gem_prism_sp_desc_rx_hardirq_queue(int irq, void *irq_data)
{
	struct macb_queue *queue = (struct macb_queue *)irq_data;

	__gem_prism_sp_desc_rx_hardirq_queue(queue);

	return IRQ_HANDLED;
}

static inline void
__gem_prism_sp_desc_tx_hardirq_queue(struct macb_queue *queue)
{

	if (prism_sp_gem_queue_read_tx_irq(queue)) {
		prism_sp_gem_queue_disable_tx_irq(queue);
		prism_sp_gem_queue_clear_tx_irq(queue);

		if (napi_schedule_prep(&queue->napi_tx)) {
			netdev_vdbg(queue->bp->dev, "scheduling TX softirq\n");
			__napi_schedule(&queue->napi_tx);
		}
	}
}

static irqreturn_t
gem_prism_sp_desc_tx_hardirq_queue(int irq, void *irq_data)
{
	struct macb_queue *queue = (struct macb_queue *)irq_data;

	__gem_prism_sp_desc_tx_hardirq_queue(queue);

	return IRQ_HANDLED;
}

/*
 * The NAPI softirq function for RX.
 */
static int macb_rx_poll_split_irq(struct napi_struct *napi, int budget)
{
	struct macb_queue *queue = container_of(napi, struct macb_queue, napi_rx);
	int work_done;

	work_done = gem_rx(queue, napi, budget);

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		prism_sp_gem_queue_enable_rx_irq(queue);

		if (macb_rx_pending(queue)) {
			prism_sp_gem_queue_disable_rx_irq(queue);
			prism_sp_gem_queue_clear_rx_irq(queue);
			napi_schedule(napi);
		}
	}

	/* TODO: Handle errors */

	return work_done;
}

/*
 * The NAPI softirq function for TX.
 */
static int macb_tx_poll_split_irq(struct napi_struct *napi, int budget)
{
	struct macb_queue *queue = container_of(napi, struct macb_queue, napi_tx);
	int work_done;

	work_done = macb_tx_complete(queue, budget);

	if (work_done < budget && napi_complete_done(napi, work_done)) {
		prism_sp_gem_queue_enable_tx_irq(queue);

		if (macb_tx_complete_pending(queue)) {
			prism_sp_gem_queue_disable_tx_irq(queue);
			prism_sp_gem_queue_clear_tx_irq(queue);
			napi_schedule(napi);
		}
	}

	return work_done;
}
