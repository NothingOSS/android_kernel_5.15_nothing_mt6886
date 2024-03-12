#include <linux/kernel.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/compat.h>

#include "gh_core.h"


#define GH_MISC_TOOLS_NAME		"gh_tools"
#define GH_TOOLS_VERSION_MAJOR		1
#define GH_TOOLS_VERSION_MINOR		0
static const u16 gh_tools_ver = ((GH_TOOLS_VERSION_MAJOR << 8) +
			(GH_TOOLS_VERSION_MINOR));

struct gh_tools
{
	struct gh_core_data *cd;
	int opened;
} gh_tools;

static int gh_tools_open(struct inode *inode, struct file *filp)
{
	gh_info("gh tools opened");
	filp->private_data = &gh_tools;
	gh_tools.opened = 1;
	return 0;
}

static int gh_tools_release(struct inode *inode, struct file *filp)
{
	gh_info("tools release called");
	gh_tools.opened = 0;
	filp->private_data =NULL;
	return 0;
}

#define GH_IOC_MAGIC			'G'
#define NEGLECT_SIZE_MASK		(~(_IOC_SIZEMASK << _IOC_SIZESHIFT))

#define GH_IOCTL_CMD_VERSION		_IOR(GH_IOC_MAGIC, 0x01, u32)
#define GH_IOCTL_CMD_RESET		_IOW(GH_IOC_MAGIC, 0x02, u32)
#define GH_IOCTL_CMD_SET_IRQ		_IOW(GH_IOC_MAGIC, 0x03, u32)
#define GH_IOCTL_REG_READ		(_IOR(GH_IOC_MAGIC, 4, u8) & NEGLECT_SIZE_MASK)
#define GH_IOCTL_REG_WRITE		(_IOW(GH_IOC_MAGIC, 5, u8) & NEGLECT_SIZE_MASK)


#define I2C_MSG_HEAD_LEN	20
#define MAX_BUF_LENGTH		(16*1024)
/* read data asynchronous,
 * success return data length, otherwise return < 0
 */
static int gh_tools_reg_read(struct gh_tools *tools, void __user *arg)
{
	u8 *databuf = NULL;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	struct gh_hw_ops *hw_ops = tools->cd->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret)
		return -EFAULT;

	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		gh_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}
	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		gh_err("Alloc memory failed");
		return -ENOMEM;
	}

	if (hw_ops->read(tools->cd, (u16)reg_addr, databuf, length, 1)) {
		ret = -EBUSY;
		gh_err("Read i2c failed");
		goto err_out;
	}
	ret = copy_to_user((u8 *)arg + I2C_MSG_HEAD_LEN, databuf, length);
	if (ret) {
		ret = -EFAULT;
		gh_err("Copy_to_user failed");
		goto err_out;
	}
	ret = 0;
err_out:
	kfree(databuf);
	return ret;
}

/* write data to i2c asynchronous,
 * success return bytes write, else return <= 0
 */
static int gh_tools_reg_write(struct gh_tools *tools, void __user *arg)
{
	u8 *databuf;
	int ret = 0;
	u32 reg_addr, length;
	u8 i2c_msg_head[I2C_MSG_HEAD_LEN];
	const struct gh_hw_ops *hw_ops = tools->cd->hw_ops;

	ret = copy_from_user(&i2c_msg_head, arg, I2C_MSG_HEAD_LEN);
	if (ret) {
		gh_err("Copy data from user failed");
		return -EFAULT;
	}
	reg_addr = i2c_msg_head[0] + (i2c_msg_head[1] << 8)
			+ (i2c_msg_head[2] << 16) + (i2c_msg_head[3] << 24);
	length = i2c_msg_head[4] + (i2c_msg_head[5] << 8)
			+ (i2c_msg_head[6] << 16) + (i2c_msg_head[7] << 24);
	if (length > MAX_BUF_LENGTH) {
		gh_err("buffer too long:%d > %d", length, MAX_BUF_LENGTH);
		return -EINVAL;
	}

	databuf = kzalloc(length, GFP_KERNEL);
	if (!databuf) {
		gh_err("Alloc memory failed");
		return -ENOMEM;
	}
	ret = copy_from_user(databuf, (u8 *)arg + I2C_MSG_HEAD_LEN, length);
	if (ret) {
		ret = -EFAULT;
		gh_err("Copy data from user failed");
		goto err_out;
	}

	if (hw_ops->write(tools->cd, (u16)reg_addr, databuf, length, 1)) {
		ret = -EBUSY;
		gh_err("Write data to device failed");
	} else {
		ret = 0;
	}

err_out:
	kfree(databuf);
	return ret;
}

static long gh_tools_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct gh_tools *tools = filp->private_data;
	struct gh_core_data *cd = tools->cd;
	int ret = 0;

	switch (cmd & NEGLECT_SIZE_MASK) {
	case GH_IOCTL_CMD_VERSION:
		gh_info("get tools version");
		ret = copy_to_user((u8 *)arg, &gh_tools_ver,
					sizeof(gh_tools_ver));
		if (ret)
			gh_err("failed copy driver version info to user");
		break;
	case GH_IOCTL_CMD_SET_IRQ:
		gh_info("tools set irq %ld", arg);
		if (arg == 0) {
			gh_irq_enable(cd, 1);
			gh_info("IRQ enabled");
		} else {
			gh_irq_enable(cd, 0);
			gh_info("IRQ disabled");
		}
		ret = 0;
		break;
	case GH_IOCTL_CMD_RESET:
		gh_info("tools set reset pin %ld", arg);
		break;
	case GH_IOCTL_REG_READ:
		ret = gh_tools_reg_read(tools, (void __user *)arg);
		if (ret)
			gh_err("Async data read failed");
		break;
	case GH_IOCTL_REG_WRITE:
		ret = gh_tools_reg_write(tools, (void __user *)arg);
		if (ret)
			gh_err("Async data write failed");
		break;
	default:
		gh_err("cmd unknown. 0x%x", cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long gh_tools_compat_ioctl(struct file *file, unsigned int cmd,
				  unsigned long arg)
{
	void __user *arg32 = compat_ptr(arg);

	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	return file->f_op->unlocked_ioctl(file, cmd, (unsigned long)arg32);
}
#endif

static const struct file_operations gh_tools_fops = {
	.owner		= THIS_MODULE,
	.open		= gh_tools_open,
	.release	= gh_tools_release,
	.unlocked_ioctl	= gh_tools_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = gh_tools_compat_ioctl,
#endif
};

static struct miscdevice gh_tools_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= GH_MISC_TOOLS_NAME,
	.fops	= &gh_tools_fops,
};

int gh_tools_init(struct gh_core_data *cd)
{
	int ret;

	gh_tools.cd = cd;
	ret = misc_register(&gh_tools_miscdev);
	if (ret)
		gh_err("failed to register misc device");

	gh_info("success register gh tools");
	return ret;
}

int gh_tools_exit(struct gh_core_data *cd)
{
	misc_deregister(&gh_tools_miscdev);
	gh_info("deregister gh tools dev");
	return 0;
}