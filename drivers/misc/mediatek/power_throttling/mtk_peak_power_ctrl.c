// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_qos.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/sched/clock.h>
#include <trace/events/power.h>
#include <linux/tracepoint.h>
#include <linux/kallsyms.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/nvmem-consumer.h>

#include "mtk_peak_power_ctrl.h"
#if IS_ENABLED(CONFIG_MTK_GPUFREQ_V2)
#include <mtk_gpufreq.h>
#endif

static LIST_HEAD(ppc_policy_list);

static bool mt_ppc_debug;
static bool ppc_enable;
static unsigned int g_p_active;
static DEFINE_MUTEX(ppc_mutex);

enum PPC_LIMIT_SOURCE {
	PPC_P_ACTIVE,
	PPC_C_BOOST,
	PPC_LIMIT_MAX
};

struct cpu_freq_thr_t cpu_freq_thr = {
	.cluster_num = 0,
	.cur_freq = {{0, 0}, {1, 0}, {2, 0}, {3, 0}},
	.thr_freq = {{0, 0}, {1, 2100000}, {3, 2100000}, {3, 0}},
};

static s32 limit_freq[PPC_MAX_CLUSTER_NUM] = {1800000, 2850000, 3050000,
	FREQ_QOS_MAX_DEFAULT_VALUE};

static struct ppc_ctrl_t ppc_ctrl = {
	.gpu_limit_state = 0,
	.cpu_limit_state = 0,
	.source_state = 0,
};

static void mtk_ppc_limit_cpu(unsigned int limit)
{
	struct cpu_ppc_policy *ppc_policy;
	s32 frequency;
	unsigned int i = 0;

	if (mt_ppc_debug)
		pr_info("%s: limit=%u\n", __func__, limit);

	list_for_each_entry(ppc_policy, &ppc_policy_list, cpu_ppc_list) {
		if (limit)
			frequency = limit_freq[i];
		else
			frequency = FREQ_QOS_MAX_DEFAULT_VALUE;

		freq_qos_update_request(&ppc_policy->qos_req, frequency);
		i++;
	}
}
static void mtk_ppc_limit_gpu(int limit)
{
#if IS_ENABLED(CONFIG_MTK_GPUFREQ_V2)
	gpufreq_set_limit_nolock(TARGET_DEFAULT, LIMIT_PEAK_POWER, (limit == 1) ? 981000 : 0,
		GPUPPM_KEEP_IDX);
#endif
}

static unsigned int mtk_ppc_arbitrate_and_set_limit(enum PPC_LIMIT_SOURCE source,
	unsigned int enable)
{
	unsigned int gpu_pre_state, cpu_pre_state;

	if (source >= PPC_LIMIT_MAX)
		return 0;

	if (!ppc_enable)
		return 0;

	gpu_pre_state = ppc_ctrl.gpu_limit_state;
	cpu_pre_state = ppc_ctrl.cpu_limit_state;
	if (enable != (ppc_ctrl.source_state >> source & 0x1))
		ppc_ctrl.source_state ^= (1 << source);

	ppc_ctrl.gpu_limit_state = (ppc_ctrl.source_state) ? 1 : 0;
	ppc_ctrl.cpu_limit_state = ppc_ctrl.source_state & 0x1;

	if (ppc_ctrl.gpu_limit_state != gpu_pre_state)
		mtk_ppc_limit_gpu(ppc_ctrl.gpu_limit_state);

	if (ppc_ctrl.cpu_limit_state != cpu_pre_state)
		mtk_ppc_limit_cpu(ppc_ctrl.cpu_limit_state);

	return (ppc_ctrl.gpu_limit_state != gpu_pre_state);
}

struct tracepoints_table {
	const char *name;
	void *func;
	struct tracepoint *tp;
	bool registered;
};

static void ppc_cpu_frequency_tracer(void *ignore, unsigned int frequency, unsigned int cpu_id)
{
	struct cpufreq_policy *policy = NULL;
	struct cpu_ppc_policy *ppc_policy;
	unsigned int cpu, i, state, change;
	int len = 0;
	char buf[512];

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	u64 ts[2];

	ts[0] = sched_clock();
#endif

	policy = cpufreq_cpu_get(cpu_id);
	if (!policy)
		return;

	if (cpu_id != cpumask_first(policy->related_cpus)) {
		cpufreq_cpu_put(policy);
		return;
	}

	i = 0;
	list_for_each_entry(ppc_policy, &ppc_policy_list, cpu_ppc_list) {
		cpu = ppc_policy->policy->cpu;
		if (cpu == cpu_id) {
			cpu_freq_thr.cur_freq[i].freq = frequency;
			break;
		}
		i++;
	}

	cpufreq_cpu_put(policy);

	state = 1;
	for (i = 0; i < cpu_freq_thr.cluster_num; i++) {
		if (cpu_freq_thr.cur_freq[i].freq <= cpu_freq_thr.thr_freq[i].freq) {
			state = 0;
			break;
		}
	}

	change = mtk_ppc_arbitrate_and_set_limit(PPC_C_BOOST, state);

	if (change && mt_ppc_debug) {
		for (i = 0; i < cpu_freq_thr.cluster_num; i++) {
			len += snprintf(buf + len, 512, "cl[%d] cur:%d limit>%d ", i,
				cpu_freq_thr.cur_freq[i].freq / 1000,
				cpu_freq_thr.thr_freq[i].freq / 1000);
		}

		len += snprintf(buf + len, 512, "==>state = %d ", state);
		pr_info("%s\n", buf);
	}

#if IS_ENABLED(CONFIG_MTK_IRQ_MONITOR_DEBUG)
	ts[1] = sched_clock();
	if ((ts[1] - ts[0] > 2000000ULL) && in_hardirq()) {
		printk_deferred("%s duration %llu, ts[0]=%llu, ts[1]=%llu\n",
			__func__, ts[1] - ts[0], ts[0], ts[1]);
	}
#endif
}

struct tracepoints_table ppc_tracepoints[] = {
	{.name = "cpu_frequency", .func = ppc_cpu_frequency_tracer},
};

#define FOR_EACH_INTEREST(i) \
	for (i = 0; i < sizeof(ppc_tracepoints) / sizeof(struct tracepoints_table); i++)

static void ppc_lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (strcmp(ppc_tracepoints[i].name, tp->name) == 0)
			ppc_tracepoints[i].tp = tp;
	}
}

void tracepoint_cleanup(void)
{
	int i;

	FOR_EACH_INTEREST(i) {
		if (ppc_tracepoints[i].registered) {
			tracepoint_probe_unregister(
				ppc_tracepoints[i].tp,
				ppc_tracepoints[i].func, NULL);
			ppc_tracepoints[i].registered = false;
		}
	}
}

static int mt_ppc_debug_proc_show(struct seq_file *m, void *v)
{
	if (mt_ppc_debug)
		seq_puts(m, "ppc debug enabled\n");
	else
		seq_puts(m, "ppc debug disabled\n");

	return 0;
}

static ssize_t mt_ppc_debug_proc_write
(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	int debug = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (kstrtoint(desc, 10, &debug) == 0) {
		if (debug == 0)
			mt_ppc_debug = 0;
		else if (debug == 1)
			mt_ppc_debug = 1;
		else
			pr_notice("should be [0:disable,1:enable]\n");
	} else
		pr_notice("should be [0:disable,1:enable]\n");

	return count;
}

static int mt_p_active_proc_show(struct seq_file *m, void *v)
{
	if (g_p_active)
		seq_puts(m, "1\n");
	else
		seq_puts(m, "0\n");

	return 0;
}

static ssize_t mt_p_active_proc_write
(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	char desc[32];
	int len = 0;
	int input = 0;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return 0;
	desc[len] = '\0';

	if (kstrtoint(desc, 10, &input) == 0) {
		if (input == 0 && g_p_active != 0) {
			g_p_active = 0;
			//mtk_ppc_arbitrate_and_set_limit(PPC_P_ACTIVE, g_p_active);
		} else if (input == 1 && g_p_active == 0) {
			g_p_active = 1;
			//mtk_ppc_arbitrate_and_set_limit(PPC_P_ACTIVE, g_p_active);
		}
	}

	return count;
}

#define PROC_FOPS_RW(name)						\
static int mt_ ## name ## _proc_open(struct inode *inode, struct file *file)\
{									\
	return single_open(file, mt_ ## name ## _proc_show, PDE_DATA(inode));\
}									\
static const struct proc_ops mt_ ## name ## _proc_fops = {	\
	.proc_open		= mt_ ## name ## _proc_open,			\
	.proc_read		= seq_read,					\
	.proc_lseek		= seq_lseek,					\
	.proc_release		= single_release,				\
	.proc_write		= mt_ ## name ## _proc_write,			\
}

#define PROC_ENTRY(name)	{__stringify(name), &mt_ ## name ## _proc_fops}
PROC_FOPS_RW(ppc_debug);
PROC_FOPS_RW(p_active);

static int mt_ppc_create_procfs(void)
{
	struct proc_dir_entry *dir = NULL;
	int i;

	struct pentry {
		const char *name;
		const struct proc_ops *fops;
	};

	const struct pentry entries[] = {
		PROC_ENTRY(ppc_debug),
		PROC_ENTRY(p_active),
	};

	dir = proc_mkdir("ppc", NULL);

	if (!dir) {
		pr_notice("fail to create /proc/ppc @ %s()\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(entries); i++) {
		if (!proc_create(entries[i].name, 0660, dir, entries[i].fops))
			pr_notice("@%s: create /proc/ppc/%s failed\n", __func__,
				    entries[i].name);
	}

	return 0;
}

static int get_segment_id(struct platform_device *pdev)
{
	unsigned int id = 0;
	struct nvmem_cell *efuse_cell;
	unsigned int *efuse_buf;
	size_t efuse_len;

	efuse_cell = nvmem_cell_get(&pdev->dev, "ppc_segment");
	if (IS_ERR(efuse_cell)) {
		pr_info("fail to get ppc_segment (%ld)", PTR_ERR(efuse_cell));
		id = 0;
		goto done;
	}

	efuse_buf = (unsigned int *)nvmem_cell_read(efuse_cell, &efuse_len);
	nvmem_cell_put(efuse_cell);
	if (IS_ERR(efuse_buf)) {
		pr_info("fail to get efuse_buf (%ld)", PTR_ERR(efuse_buf));
		goto done;
	}

	id = (*efuse_buf & 0xFF);
	kfree(efuse_buf);
done:
	return (id == 3);
}


static int ppc_probe(struct platform_device *pdev)
{
	struct cpufreq_policy *policy;
	struct cpu_ppc_policy *ppc_policy;
	unsigned int i;
	int cpu, ret;

	mt_ppc_create_procfs();

	ppc_enable = get_segment_id(pdev);
	ppc_enable = 0;
	if (!ppc_enable) {
		pr_info("ppc disable due to segment\n");
		return 0;
	}

	for_each_kernel_tracepoint(ppc_lookup_tracepoints, NULL);

	FOR_EACH_INTEREST(i) {
		if (ppc_tracepoints[i].tp == NULL) {
			pr_info("ppc Error, %s not found\n", ppc_tracepoints[i].name);
			tracepoint_cleanup();
			return -1;
		}
	}

	ret = tracepoint_probe_register(ppc_tracepoints[0].tp, ppc_tracepoints[0].func,  NULL);
	ppc_tracepoints[0].registered = true;

	if (!ret)
		ppc_tracepoints[0].registered = true;
	else
		pr_info("cpu_frequency: Couldn't activate tracepoint\n");

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;

		if (policy->cpu == cpu) {
			ppc_policy = kzalloc(sizeof(*ppc_policy), GFP_KERNEL);
			if (!ppc_policy)
				return -ENOMEM;

			i = cpufreq_table_count_valid_entries(policy);
			if (!i) {
				pr_info("%s: CPUFreq table not found or has no valid entries\n",
					 __func__);
				return -ENODEV;
			}

			ppc_policy->policy = policy;
			ppc_policy->cpu = cpu;

			ret = freq_qos_add_request(&policy->constraints,
				&ppc_policy->qos_req, FREQ_QOS_MAX,
				FREQ_QOS_MAX_DEFAULT_VALUE);

			if (ret < 0) {
				pr_notice("%s: Fail to add freq constraint (%d)\n",
					__func__, ret);
				return ret;
			}
			list_add_tail(&ppc_policy->cpu_ppc_list, &ppc_policy_list);
			cpu_freq_thr.cluster_num++;
		}
	}

	ppc_ctrl.cpu_freq_thr_info = &cpu_freq_thr;

	return ret;
}

static int ppc_remove(struct platform_device *pdev)
{
	struct cpu_ppc_policy *ppc_policy, *ppc_policy_t;

	mtk_ppc_limit_cpu(0);
	mtk_ppc_limit_gpu(0);

	list_for_each_entry_safe(ppc_policy, ppc_policy_t, &ppc_policy_list, cpu_ppc_list) {
		cpufreq_cpu_put(ppc_policy->policy);
		list_del(&ppc_policy->cpu_ppc_list);
		kfree(ppc_policy);
	}

	return 0;
}

static const struct of_device_id ppc_of_match[] = {
	{ .compatible = "mediatek,mt6985-ppc", },
	{},
};
MODULE_DEVICE_TABLE(of, ppc_of_match);

static struct platform_driver ppc_driver = {
	.probe = ppc_probe,
	.remove = ppc_remove,
	.driver = {
		.name = "mtk-peak-power-ctrl",
		.of_match_table = ppc_of_match,
	},
};
module_platform_driver(ppc_driver);

MODULE_AUTHOR("Samuel Hsieh");
MODULE_DESCRIPTION("MTK peak power control");
MODULE_LICENSE("GPL");
