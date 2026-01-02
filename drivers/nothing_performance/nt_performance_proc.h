#ifndef _NT_PERFORMANCE_PROC_H
#define _NT_PERFORMANCE_PROC_H

#include <linux/proc_fs.h>

#define CREATE_PROC_ENTRY_AND_FOPS(name) \
	static struct proc_dir_entry *name##_node_pentry = NULL; \
	static int name##_proc_open(struct inode *inode, struct file *file); \
	static ssize_t name##_proc_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos); \
	static const struct proc_ops name##_proc_fops = { \
		.proc_open       = name##_proc_open, \
		.proc_write      = name##_proc_write, \
		.proc_read       = seq_read, \
		.proc_lseek      = seq_lseek, \
		.proc_release    = single_release, \
	};

#define REMOVE_PROC_ENTRY(name) \
	if (name) { \
		proc_remove(name); \
		name = NULL; \
	}

#endif /* _NT_PERFORMANCE_PROC_H */
