// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2015 MediaTek Inc.
 * Author: Chaotian.Jing <chaotian.jing@mediatek.com>
 */

#include <linux/module.h>
#include <linux/blkdev.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/major.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/xarray.h>

#define MAX_WP_GROUPS	4
struct wp_info {
	sector_t args[2 * MAX_WP_GROUPS];
	unsigned int nargs;
};

static int wp_boot1;
static int wp_boot2;
static struct wp_info wp_user;

static void set_ro_on_bdev(dev_t dev)
{
	struct block_device *bdev;

	bdev = blkdev_get_by_dev(dev, FMODE_READ, NULL);
	if (IS_ERR(bdev)) {
		pr_info("[mtk-mmc-wp]failed to get bdev(%d,%d)\n", MAJOR(dev), MINOR(dev));
		return;
	}

	bdev->bd_read_only = true;

	blkdev_put(bdev, FMODE_READ);
	pr_info("[mtk-mmc-wp]set wp %pg\n", bdev);

}

static void set_ro_on_parts(dev_t devt, struct wp_info *info)
{
	struct block_device *bdev, *part;
	struct gendisk *disk;
	unsigned long idx;
	sector_t part_start, part_end;
	sector_t wp_start, wp_end;
	int i;

	bdev = blkdev_get_by_dev(devt, FMODE_READ, NULL);
	if (IS_ERR(bdev)) {
		pr_info("[mtk-mmc-wp]failed to get bdev(%d,%d)\n", MAJOR(devt), MINOR(devt));
		return;
	}

	disk = bdev->bd_disk;

	rcu_read_lock();
	xa_for_each(&disk->part_tbl, idx, part) {
		if (!bdev_is_partition(part))
			continue;

		part_start = part->bd_start_sect;
		part_end = part->bd_start_sect + bdev_nr_sectors(part);

		for (i = 0; i < info->nargs - 1; i = i + 2) {
			wp_start = info->args[i];
			wp_end = wp_start + info->args[i + 1];
			if (part_start >= wp_start && part_end <= wp_end) {
				part->bd_read_only = true;
				pr_info("[mtk-mmc-wp]set wp %pg\n", part);
				break;
			}
		}
	}
	rcu_read_unlock();

	blkdev_put(bdev, FMODE_READ);
}

#define MAX_RETRIES			100
#define NUM_WHOLE_BDEVS		4

static dev_t find_whole_bdev(const char *name, int start_minor, int *perdev_minors)
{
	dev_t devt = MKDEV(0, 0);
	struct block_device *bdev;
	char dev_name[BDEVNAME_SIZE];
	int minor = start_minor;
	int retries = 0;

	do {

retry:
		bdev = blkdev_get_by_dev(MKDEV(MMC_BLOCK_MAJOR, minor), FMODE_READ, NULL);
		retries++;
		if (IS_ERR_OR_NULL(bdev)) {
			if (retries >= MAX_RETRIES) {
				pr_info("[mtk-mmc-wp]failed to find bdev(%d,%d) for %s, err=%ld\n",
						MMC_BLOCK_MAJOR, minor, name, -PTR_ERR(bdev));
				goto exit;
			} else
				msleep(100);
			goto retry;
		}

		snprintf(dev_name, sizeof(dev_name), "%pg", bdev);
		if (strcmp(dev_name, name)) {
			if(bdev && bdev->bd_disk)
				minor += bdev->bd_disk->minors;
			blkdev_put(bdev, FMODE_READ);
			retries = 0;
			continue;
		}
		if(bdev)
			devt = bdev->bd_dev;
		if (perdev_minors && bdev && bdev->bd_disk)
			*perdev_minors = bdev->bd_disk->minors;

		blkdev_put(bdev, FMODE_READ);
		break;
	} while (minor < NUM_WHOLE_BDEVS * bdev->bd_disk->minors);

exit:
	return devt;
}

static int __init wp_policy_init(void)
{
	dev_t devt = MKDEV(0, 0);
	int start_minor = 0;
	int perdev_minors = 0;

	if (!wp_boot1 && !wp_boot2 && !wp_user.nargs)
		return 0;

	if (wp_user.nargs) {
		devt = find_whole_bdev("mmcblk0", start_minor, &perdev_minors);
		if (!devt){
			pr_info("[mtk-mmc-wp]not found mmcblk0\n");
			return -ENODEV;
		}
		set_ro_on_parts(devt, &wp_user);
	}

	if (wp_boot1) {
		start_minor = MINOR(devt) + perdev_minors;
		devt = find_whole_bdev("mmcblk0boot0", start_minor, &perdev_minors);
		if (!devt){
			pr_info("[mtk-mmc-wp]not found mmcblk0boot0\n");
			return -ENODEV;
		}
		set_ro_on_bdev(devt);
	}

	if (wp_boot2) {
		start_minor = MINOR(devt) + perdev_minors;
		devt = find_whole_bdev("mmcblk0boot1", start_minor, &perdev_minors);
		if (!devt){
			pr_info("[mtk-mmc-wp]not found mmcblk0boot1\n");
			return -ENODEV;
		}
		set_ro_on_bdev(devt);
	}

	return 0;
}
late_initcall(wp_policy_init);

static void __exit wp_policy_exit(void)
{
	pr_info("[mtk-mmc-wp]%s\n", __func__);
}
module_exit(wp_policy_exit);

module_param(wp_boot1, int, 0);
module_param(wp_boot2, int, 0);
module_param_array_named(wp_user, wp_user.args, ullong, &wp_user.nargs, 0);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MediaTek SD/MMC Card Driver for WP");
