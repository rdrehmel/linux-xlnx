// SPDX-License-Identifier: GPL-2.0
/*
 * PRISM SP Remote Processor driver
 *
 * Copyright (c) 2021-2023 Robert Drehmel
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/remoteproc.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/prism_sp.h>

#include "remoteproc_internal.h"

#define PRISM_SP_BRAM_BASE		0x20000

enum PRISM_SP_CONFIG_HWTYPE {
	PRISM_SP_CONFIG_HWTYPE_BX,
	PRISM_SP_CONFIG_HWTYPE_RX,
	PRISM_SP_CONFIG_HWTYPE_TX
};

struct prism_sp_config {
	int hwtype;
	const char *dir;
};

static const struct prism_sp_config prism_sp_rx_config = {
	.hwtype = PRISM_SP_CONFIG_HWTYPE_RX,
	.dir = "rx"
};

static const struct prism_sp_config prism_sp_tx_config = {
	.hwtype = PRISM_SP_CONFIG_HWTYPE_TX,
	.dir = "tx"
};

void
prism_sp_set_io_axi_axcache(struct prism_sp *sp, int x)
{
	printk("Prism: %s: Writing 0x%08x to %px\n",
		__func__,
		x,
		(uint8_t *)sp->regs + PRISM_SP_IO_AXI_AXCACHE);
}

void
prism_sp_set_dma_axi_axcache(struct prism_sp *sp, int x)
{
	printk("Prism: %s: 0x%08x to %px\n",
		__func__,
		x,
		(uint8_t *)sp->regs + PRISM_SP_DMA_AXI_AXCACHE);
	prism_sp_writel(sp, PRISM_SP_DMA_AXI_AXCACHE, x);
}

void
prism_sp_enable_rx_irq(struct prism_sp *sp)
{
	prism_sp_writel(sp, PRISM_SP_IER, (uint32_t)1 << PRISM_SP_RX_IXR_DONE_BITN);
}
EXPORT_SYMBOL(prism_sp_enable_rx_irq);
void
prism_sp_enable_tx_irq(struct prism_sp *sp)
{
	prism_sp_writel(sp, PRISM_SP_IER, (uint32_t)1 << PRISM_SP_TX_IXR_DONE_BITN);
}
EXPORT_SYMBOL(prism_sp_enable_tx_irq);

void
prism_sp_disable_rx_irq(struct prism_sp *sp)
{
	prism_sp_writel(sp, PRISM_SP_IDR, (uint32_t)1 << PRISM_SP_RX_IXR_DONE_BITN);
}
EXPORT_SYMBOL(prism_sp_disable_rx_irq);
void
prism_sp_disable_tx_irq(struct prism_sp *sp)
{
	prism_sp_writel(sp, PRISM_SP_IDR, (uint32_t)1 << PRISM_SP_TX_IXR_DONE_BITN);
}
EXPORT_SYMBOL(prism_sp_disable_tx_irq);

static int
prism_sp_rproc_start(struct rproc *rproc)
{
	struct prism_sp *sp = rproc->priv;
	u32 ctrl;

	ctrl = prism_sp_readl(sp, PRISM_SP_CONTROL);

	if ((ctrl & (u32)1 << PRISM_SP_CONTROL_CPU_RESET_OFFSET) == 0) {
		return -EBUSY;
	}

	// Clear the (active high) reset signal at bit #31.
	ctrl &= ~((u32)1 << PRISM_SP_CONTROL_CPU_RESET_OFFSET);
	prism_sp_writel(sp, PRISM_SP_CONTROL, ctrl);

	return 0;
}

static int
prism_sp_rproc_stop(struct rproc *rproc)
{
	struct prism_sp *sp = rproc->priv;
	u32 ctrl;

	ctrl = prism_sp_readl(sp, PRISM_SP_CONTROL);

	if ((ctrl & (u32)1 << PRISM_SP_CONTROL_CPU_RESET_OFFSET) != 0) {
		return -EBUSY;
	}

	// Set the (active high) reset signal at bit #31.
	ctrl |= (u32)1 << PRISM_SP_CONTROL_CPU_RESET_OFFSET;
	prism_sp_writel(sp, PRISM_SP_CONTROL, ctrl);

	return 0;
}

static int
prism_sp_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{
	int ret;

	/* If it's there, nice, but we don't really care about the
	 * resource table.
	 */
	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret == -EINVAL)
		ret = 0;
	return ret;
}

static irqreturn_t
prism_sp_interrupt(int irq, void *sp_)
{
	struct prism_sp *sp = (struct prism_sp *)sp_;

	if (sp == NULL) {
		printk("Prism: %s: This must never happen: sp not set.\n", __func__);
		return -ENODEV;
	}
	if (sp->irq_handler == NULL) {
		printk("Prism: %s: This must never happen: sp->irq_handler not set.\n", __func__);
		return -ENODEV;
	}
	return (*sp->irq_handler)(irq, sp->irq_data);
}

static int
prism_sp_rproc_elf_load_segments(struct rproc *rproc,
	const struct firmware *fw)
{
	struct device *dev = &rproc->dev;
	struct prism_sp *sp = rproc->priv;
	struct elf32_hdr *ehdr = (struct elf32_hdr *)fw->data;
	int i;
	int index;

	for (i = 0; i < ehdr->e_phnum; i++) {
		struct elf32_phdr *phdr = ((struct elf32_phdr *)(fw->data + ehdr->e_phoff) + i);
		if (phdr->p_type != PT_LOAD)
			continue;

		if ((phdr->p_filesz % sizeof(u32)) != 0) {
			dev_err(dev, "PRISM SP firmware section size not aligned to 4 bytes"
				" (size=%u).\n", phdr->p_filesz);
			return -EINVAL;
		}

		for (index = 0; index < phdr->p_filesz; index += 4) {
			u32 data = *(u32 *)(fw->data + phdr->p_offset + index);
			prism_sp_writel(sp, PRISM_SP_BRAM_ADDR, phdr->p_paddr - PRISM_SP_BRAM_BASE + index);
			prism_sp_writel(sp, PRISM_SP_BRAM_DATA, data);
		}
		for (; index < phdr->p_memsz - phdr->p_filesz; index += 4) {
			prism_sp_writel(sp, PRISM_SP_BRAM_ADDR, phdr->p_paddr - PRISM_SP_BRAM_BASE + index);
			prism_sp_writel(sp, PRISM_SP_BRAM_DATA, 0x0);
		}
	}

	return 0;
}

static struct rproc_ops prism_sp_rproc_ops = {
	.start = prism_sp_rproc_start,
	.stop = prism_sp_rproc_stop,
	.load = prism_sp_rproc_elf_load_segments,
	.parse_fw = prism_sp_rproc_parse_fw,
	// Use the default ELF rproc functions
	.find_loaded_rsc_table = rproc_elf_find_loaded_rsc_table,
	.sanity_check = rproc_elf_sanity_check,
	.get_boot_addr = rproc_elf_get_boot_addr
};

int
bstring_to_int(const char *s, int *retp)
{
	int ret = 0;
	int i;

	for (i = 0; s[i] != '\0'; i++) {
		ret <<= 1;
		switch (s[i]) {
		case '0':
			break;
		case '1':
			ret |= 0x1;
			break;
		default:
			printk("Prism: %s: Wrong character in binary string '%s'.\n", __func__, s);
			return -1;
		}
	}

	*retp = ret;
	return 0;
}

int
prism_sp_get_axi_axcache(struct prism_sp *sp, const char *prop, int *retp)
{
	struct rproc *rproc;
	struct device *parent_dev;
	struct device_node *np;
	const char *axi_axcache_str;
	int axi_axcache;
	int ret;

	rproc = sp->rproc;
	parent_dev = rproc->dev.parent;
	np = parent_dev->of_node;

	ret = of_property_read_string(np, prop, &axi_axcache_str);
	if (ret < 0) {
		if (ret != -EINVAL) {
			dev_err(parent_dev, "Couldn't read device-tree value '%s'", prop);
			return -1;
		}
		axi_axcache = 0x0;
	}
	else {
		ret = bstring_to_int(axi_axcache_str, &axi_axcache);
		if (ret < 0) {
			dev_err(parent_dev, "Value in '%s' isn't convertible.", prop);
			return -1;
		}
	}
	*retp = axi_axcache;
	return 0;
}

int
prism_sp_get_id(struct prism_sp *sp, uint32_t *retp)
{
	struct rproc *rproc;
	struct device *parent_dev;
	struct device_node *np;
	int ret;

	rproc = sp->rproc;
	parent_dev = rproc->dev.parent;
	np = parent_dev->of_node;

	ret = of_property_read_u32_index(np, "id", 0, retp);
	if (ret < 0) {
		dev_err(parent_dev, "No ID is specified.\n");
		return ret;
	}
	return 0;
}

extern const char *prism_construct_fwfilename(const char *dir);

static int
prism_sp_remoteproc_probe(struct platform_device *pdev)
{
	const struct prism_sp_config *sp_config;
	const char *fwfilename;
	struct rproc *rproc;
	struct prism_sp *sp;
	struct resource *mmr_res;
	struct resource *vring_res;
	int io_axi_axcache;
	int dma_axi_axcache;
	int ret;

	sp_config = of_device_get_match_data(&pdev->dev);
	if (sp_config == NULL) {
		dev_err(&pdev->dev, "sp_config-specific data is not defined\n");
		return -ENODEV;
	}

	// Create the string for the correct firmware.
	fwfilename = prism_construct_fwfilename(sp_config->dir);
	printk("Prism: %s: The magic firmware filename is '%s'.\n", __func__, fwfilename);

	rproc = rproc_alloc(&pdev->dev, dev_name(&pdev->dev),
		&prism_sp_rproc_ops, fwfilename, sizeof(struct prism_sp));
	if (rproc == NULL) {
		dev_err(&pdev->dev, "rproc_alloc() failed\n");
		ret = -ENOMEM;
		goto error;
	}
	rproc->auto_boot = 0;

	sp = rproc->priv;
	sp->rproc = rproc;

	platform_set_drvdata(pdev, rproc);
	// The platform_device object now owns the rproc object.

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "dma_set_coherent_mask() failed: %d\n", ret);
		goto error;
	}

	mmr_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mmr");
	if (mmr_res == NULL) {
		printk("Prism: No 'mmr' regs found.\n");
		goto error;
	}

	sp->regs = devm_ioremap_resource(&pdev->dev, mmr_res);
	if (IS_ERR(sp->regs)) {
		printk("Prism: devm_ioremap_resource() failed for mmr: %d\n", ret);
		ret = PTR_ERR(sp->regs);
		dev_err(&pdev->dev, "devm_ioremap_resource() failed for mmr: %d\n", ret);
		goto error;
	}

	vring_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vring");
	if (vring_res == NULL) {
		printk("Prism: No vring memory found.\n");
	}
	else {
		printk("Prism: Vring memory found @%px, size 0x%x!\n",
			(void *)vring_res->start, (int)resource_size(vring_res));
		sp->vring_mem = devm_memremap(&pdev->dev, vring_res->start, resource_size(vring_res),
			MEMREMAP_WC);
		if (IS_ERR(sp->vring_mem)) {
			printk("Prism: devm_ioremap_resource() failed for vring: %d\n", ret);
			ret = PTR_ERR(sp->vring_mem);
			dev_err(&pdev->dev, "devm_ioremap_resource() failed for vring: %d\n", ret);
			goto error;
		}
		printk("Prism: Vring memory mapped to %px\n", sp->vring_mem);
	}

	sp->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(&pdev->dev, sp->irq, prism_sp_interrupt, IRQF_SHARED,
		dev_name(&pdev->dev), sp);
	if (ret) {
		dev_err(&pdev->dev, "Unable to request IRQ %d (error %d)\n",
			sp->irq, ret);
		goto error;
	}

	prism_sp_get_axi_axcache(sp, "io_axi_axcache", &io_axi_axcache);
	prism_sp_get_axi_axcache(sp, "dma_axi_axcache", &dma_axi_axcache);

	printk("Prism: %s: Setting IO AXI AxCACHE to 0x%x\n", __func__, io_axi_axcache);
	prism_sp_set_io_axi_axcache(sp, io_axi_axcache);
	printk("Prism: %s: Setting DMA AXI AxCACHE to 0x%x\n", __func__, dma_axi_axcache);
	prism_sp_set_dma_axi_axcache(sp, dma_axi_axcache);

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(&pdev->dev, "rproc_add() failed\n");
		goto error;
	}

	printk("Prism: %s: Prism SP found at %px with interrupt %d\n",
		__func__, (void *)mmr_res->start, sp->irq);
	return 0;
error:
	platform_set_drvdata(pdev, NULL);
	rproc_free(rproc);
	return ret;
}

static int
prism_sp_remoteproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_free(rproc);

	return 0;
}

static const struct of_device_id prism_sp_remoteproc_match[] = {
	{ .compatible = "xlnx,prism-sp-rx-1.0", .data = &prism_sp_rx_config },
	{ .compatible = "xlnx,prism-sp-tx-1.0", .data = &prism_sp_tx_config },
	{ },
};
MODULE_DEVICE_TABLE(of, prism_sp_remoteproc_match);

static struct platform_driver prism_sp_remoteproc_driver = {
	.probe = prism_sp_remoteproc_probe,
	.remove = prism_sp_remoteproc_remove,
	.driver = {
		.name = "prism_sp_remoteproc",
		.of_match_table = prism_sp_remoteproc_match,
	}
};
module_platform_driver(prism_sp_remoteproc_driver);

MODULE_AUTHOR("Robert Drehmel <robert@zoot.drehmel.com>");
MODULE_DESCRIPTION("Prism SP remote processor driver");
MODULE_LICENSE("GPL v2");
