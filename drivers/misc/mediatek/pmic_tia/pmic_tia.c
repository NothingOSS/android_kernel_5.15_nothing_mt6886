// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2023 MediaTek Inc.

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define PMIC_DEVNAME "pmic_tia"

void __iomem *addr_virt;

static int pmic_tia_open(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t pmic_tia_read(struct file *file, char __user *buffer, size_t len, loff_t *offset)
{
	unsigned int val = 0;

	val = readl(addr_virt);
	if(copy_to_user(buffer, (void *)(&val), sizeof(val)) != 0) {
		pr_info("%s failed\n", __func__);
		return -EINVAL;
	}
	pr_info("%s success, val: 0x%x\n", __func__, val);

	return sizeof(val);
}

static int pmic_tia_release(struct inode *node, struct file *file)
{
	return 0;
}

static const struct proc_ops pmic_tia_fops = {
	.proc_open	= pmic_tia_open,
	.proc_read	= pmic_tia_read,
	.proc_release	= pmic_tia_release,
	.proc_lseek	= no_llseek,
};

static int mtk_pmic_tia_probe(struct platform_device *pdev)
{

	struct resource *res;
	struct proc_dir_entry *entry;
	static phys_addr_t phys;

	entry = proc_create(PMIC_DEVNAME, 0444, NULL, &pmic_tia_fops);
	if (entry == 0)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if(res == NULL) {
		dev_info(&pdev->dev, "Failed to get pmic_tia addr\n");
		return -ENXIO;
	}

	phys = res->start & ~(PAGE_SIZE - 1U);
	addr_virt = ioremap(phys, PAGE_SIZE);
	addr_virt = addr_virt + res->start - phys;

	return 0;
}

static int mtk_pmic_tia_remove(struct platform_device *pdev)
{
	iounmap(addr_virt);
	return 0;
}

static const struct of_device_id mtk_pmic_tia_of_match[] = {
	{ .compatible = "mediatek,pmic_tia",},
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_pmic_tia_of_match);

static struct platform_driver mtk_pmic_tia_driver = {
	.probe  = mtk_pmic_tia_probe,
	.remove	= mtk_pmic_tia_remove,
	.driver = {
		.name = "mtk-pmic-tia",
		.of_match_table = mtk_pmic_tia_of_match,
	},
};
module_platform_driver(mtk_pmic_tia_driver);

MODULE_AUTHOR("Zhigang Qin <zhigang.qin@mediatek.com>");
MODULE_DESCRIPTION("MTK PMIC READ TIA Interface Driver");
MODULE_LICENSE("GPL");
