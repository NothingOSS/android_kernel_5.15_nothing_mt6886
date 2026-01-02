#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include <trace/hooks/vendor_hooks.h>
#include <trace/hooks/sys.h>
#include <trace/hooks/sched.h>

#include "nt_performance_common.h"
#include "nt_performance_print.h"

#if IS_ENABLED(CONFIG_NOTHING_NAMED_THREAD_AFFINITY)
#include "nothing_named_thread_affinity.h"
#endif /* CONFIG_NOTHING_NAMED_THREAD_AFFINITY */

#define DEFINE_HOOK_TRACE_FLAG(name) \
	static bool hook_trace_##name = false; \
	void set_hook_trace_##name(bool enable) { \
		hook_trace_##name = enable; \
		pr_info("set status of register vendor hook: %s, %s", \
			#name, enable ? "true" : "false"); \
	} \
	EXPORT_SYMBOL_GPL(set_hook_trace_##name); \
	bool get_hook_trace_##name(void) { \
		return hook_trace_##name; \
	} \
	EXPORT_SYMBOL_GPL(get_hook_trace_##name);


#define REGISTER_ANDROID_VENDOR_HOOK(name) \
	{ \
		ret = register_trace_##name( \
				nt_probe_##name, NULL); \
		if (ret) { \
			pr_err("Register vendor hook %s failed: %d", #name, ret); \
		} else { \
			set_hook_trace_##name(true); \
		} \
	}

DEFINE_HOOK_TRACE_FLAG(android_vh_syscall_prctl_finished)
DEFINE_HOOK_TRACE_FLAG(android_vh_sched_setaffinity_early)

static void nt_probe_android_vh_sched_setaffinity_early(void *ignore, struct task_struct *p, const struct cpumask *in_mask, int *skip)
{
#if IS_ENABLED(CONFIG_NOTHING_NAMED_THREAD_AFFINITY)
	check_setaffinity_skip_by_named_thread_affinity(ignore, p, in_mask, skip);
#endif /* CONFIG_NOTHING_NAMED_THREAD_AFFINITY */
}

void nt_probe_android_vh_syscall_prctl_finished(void *ignore, int option, struct task_struct *p)
{
#if IS_ENABLED(CONFIG_NOTHING_NAMED_THREAD_AFFINITY)
	set_thread_affinity_on_set_name(ignore, option, p);
#endif /* CONFIG_NOTHING_NAMED_THREAD_AFFINITY */
}
EXPORT_SYMBOL_GPL(nt_probe_android_vh_syscall_prctl_finished);

void register_nt_performance_vendor_hooks(void)
{
	int ret;

	/* android_vh_syscall_prctl_finished is registered by mtk task_turbo */
	//REGISTER_ANDROID_VENDOR_HOOK(android_vh_syscall_prctl_finished)
	REGISTER_ANDROID_VENDOR_HOOK(android_vh_sched_setaffinity_early)
}

static int __init nt_performance_init(void)
{
	int ret = 0;

	pr_info("Module init");

	register_nt_performance_vendor_hooks();

	/* Init nt_named_thread_affinity after registering android_vh_syscall_prctl_finished */
#if IS_ENABLED(CONFIG_NOTHING_NAMED_THREAD_AFFINITY)
	nt_named_thread_affinity_init();
#endif /* CONFIG_NOTHING_NAMED_THREAD_AFFINITY */

	return ret;
}

static void __exit nt_performance_exit(void)
{
	/*
	 * vendor hook cannot unregister
	 */
}

module_init(nt_performance_init);
module_exit(nt_performance_exit);
MODULE_LICENSE("GPL v2");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("<BSP_CORE@nothing.tech>");
MODULE_DESCRIPTION("NOTHING PERFORMANCE COMMON");
