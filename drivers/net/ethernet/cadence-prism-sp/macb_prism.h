#ifndef _MACB_PRISM_H_
#define _MACB_PRISM_H_

static int macb_rx_poll_dummy(struct napi_struct *napi, int budget);
static int macb_tx_poll_dummy(struct napi_struct *napi, int budget);
static irqreturn_t gem_prism_sp_desc_rx_hardirq_dummy(int irq, void *irq_data);
static irqreturn_t gem_prism_sp_desc_tx_hardirq_dummy(int irq, void *irq_data);
static irqreturn_t gem_prism_sp_desc_rx_hardirq_queue(int irq, void *irq_data);
static irqreturn_t gem_prism_sp_desc_tx_hardirq_queue(int irq, void *irq_data);

static int macb_rx_poll_split_irq(struct napi_struct *napi, int budget);
static int macb_tx_poll_split_irq(struct napi_struct *napi, int budget);
static int get_prism_sp(struct platform_device *, const char *, int, struct prism_sp **);

static inline bool gem_uses_prism_sp(struct macb *bp)
{
	return bp->uses_prism_sp;
}

static inline void prism_sp_gem_queue_enable_irqs(struct macb_queue *queue)
{
	prism_sp_rx_queue_writel(queue, SP_IER, (1 << PRISM_SP_RX_IXR_DONE_BITN) | (1 << PRISM_SP_RX_IXR_NODESC_BITN));
	prism_sp_tx_queue_writel(queue, SP_IER, (1 << PRISM_SP_TX_IXR_DONE_BITN));
}

/*
 * TX
 */
static inline void prism_sp_gem_queue_clear_tx_irq(struct macb_queue *queue)
{
	prism_sp_tx_queue_writel(queue, SP_ISR, 1 << PRISM_SP_TX_IXR_DONE_BITN);
}

static inline void prism_sp_gem_queue_enable_tx_irq(struct macb_queue *queue)
{
	prism_sp_tx_queue_writel(queue, SP_IER, 1 << PRISM_SP_TX_IXR_DONE_BITN);
}

static inline void prism_sp_gem_queue_disable_tx_irq(struct macb_queue *queue)
{
	prism_sp_tx_queue_writel(queue, SP_IDR, 1 << PRISM_SP_TX_IXR_DONE_BITN);
}

static inline int prism_sp_gem_queue_read_tx_irq(struct macb_queue *queue)
{
	return (prism_sp_tx_queue_readl(queue, SP_ISR) & (1 << PRISM_SP_TX_IXR_DONE_BITN)) != 0;
}

/*
 * RX
 */
static inline void prism_sp_gem_queue_clear_rx_irq(struct macb_queue *queue)
{
	prism_sp_rx_queue_writel(queue, SP_ISR, 1 << PRISM_SP_RX_IXR_DONE_BITN);
}

static inline void prism_sp_gem_queue_enable_rx_irq(struct macb_queue *queue)
{
	prism_sp_rx_queue_writel(queue, SP_IER, 1 << PRISM_SP_RX_IXR_DONE_BITN);
}

static inline void prism_sp_gem_queue_disable_rx_irq(struct macb_queue *queue)
{
	prism_sp_rx_queue_writel(queue, SP_IDR, 1 << PRISM_SP_RX_IXR_DONE_BITN);
}

static inline int prism_sp_gem_queue_read_rx_irq(struct macb_queue *queue)
{
	return (prism_sp_rx_queue_readl(queue, SP_ISR) & (1 << PRISM_SP_RX_IXR_DONE_BITN)) != 0;
}

static inline void prism_sp_desc_set_rbqp(struct macb_queue *queue, dma_addr_t addr)
{
	prism_sp_rx_queue_writel(queue, SP_QP_LSB, lower_32_bits(addr));
	prism_sp_rx_queue_writel(queue, SP_QP_MSB, upper_32_bits(addr));
}

static inline void prism_sp_desc_set_tbqp(struct macb_queue *queue, dma_addr_t addr)
{
	prism_sp_tx_queue_writel(queue, SP_QP_LSB, lower_32_bits(addr));
	prism_sp_tx_queue_writel(queue, SP_QP_MSB, upper_32_bits(addr));
}

static inline void prism_sp_enable_queues(struct macb *bp)
{
	struct macb_queue *queue;
	unsigned int q;

	for (q = 0, queue = bp->queues; q < bp->num_queues; ++q, ++queue) {

		printk("Prism: %s: Writing 0x%08x to %px\n",
			__func__,
			1 << PRISM_SP_CONTROL_ENABLE_OFFSET,
			(uint8_t *)queue->sp_rx->regs + PRISM_SP_CONTROL);
		prism_sp_writel(queue->sp_rx, PRISM_SP_CONTROL, 1 << PRISM_SP_CONTROL_ENABLE_OFFSET);

		printk("Prism: %s: Writing 0x%08x to %px\n",
			__func__,
			1 << PRISM_SP_CONTROL_ENABLE_OFFSET,
			(uint8_t *)queue->sp_tx->regs + PRISM_SP_CONTROL);
		prism_sp_writel(queue->sp_tx, PRISM_SP_CONTROL, 1 << PRISM_SP_CONTROL_ENABLE_OFFSET);
	}
	printk("Prism: %s: Done enabling queues.\n",
			__func__
	);
}


static int prism_macb_init_queue(
	struct platform_device *pdev,
	struct macb_queue *queue,
	int q
);
static int prism_sp_count_queues(struct platform_device *pdev);
static void gem_assign_no(struct macb *bp, struct resource *regs);

#endif
