#ifndef _LINUX_PRISM_SP_H_
#define _LINUX_PRISM_SP_H_

#include <linux/compiler_types.h>		// for __iomem.h
#include <linux/remoteproc.h>

#define PRISM_SP_CONTROL			0x00
#define PRISM_SP_STATUS				0x04
#define PRISM_SP_INFO				0x08
#define PRISM_SP_BRAM_ADDR			0x10
#define PRISM_SP_BRAM_DATA			0x14
#define PRISM_SP_IO_AXI_AXCACHE		0x20
#define PRISM_SP_DMA_AXI_AXCACHE	0x24
#define PRISM_SP_QP_LSB				0x30
#define PRISM_SP_QP_MSB				0x34

#define PRISM_SP_TER				0x40
#define PRISM_SP_TSR				0x4c

#define PRISM_SP_IER				0x50
#define PRISM_SP_IDR				0x54
#define PRISM_SP_IMR				0x58
#define PRISM_SP_ISR				0x5c

#define PRISM_SP_RX_IXR_DONE_BITN		0
#define PRISM_SP_RX_IXR_NODESC_BITN		1
#define PRISM_SP_TX_IXR_DONE_BITN		0

#define PRISM_SP_RX_TXR_START_BITN		0
#define PRISM_SP_TX_TXR_START_BITN		0

#define PRISM_SP_CONTROL_ENABLE_OFFSET			0
#define PRISM_SP_CONTROL_CPU_RESET_OFFSET		31

#define prism_sp_readl(sp, reg)					readl((sp)->regs + (reg))
#define prism_sp_writel(sp, reg, value)			writel((value), (sp)->regs + (reg))

struct prism_sp {
	void __iomem *regs;
	void *vring_mem;
	struct rproc *rproc;
	int booted;
	int irq;
	int extsoftirqn;
	irqreturn_t (*irq_handler)(int irq, void *);
	void *irq_data;
	uint32_t id;
};

void prism_sp_enable_rx_irq(struct prism_sp *);
void prism_sp_enable_tx_irq(struct prism_sp *);
void prism_sp_disable_rx_irq(struct prism_sp *);
void prism_sp_disable_tx_irq(struct prism_sp *);

#endif
