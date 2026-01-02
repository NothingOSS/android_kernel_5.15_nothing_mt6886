#ifndef _NT_PERFORMANCE_PRINT_H
#define _NT_PERFORMANCE_PRINT_H

#include <linux/printk.h>

#define TAG "nt_performance"

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(fmt) TAG ": " fmt
#endif /* pr_fmt */

#endif /* _NT_PERFORMANCE_PRINT_H */
