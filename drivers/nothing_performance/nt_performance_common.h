#ifndef _NT_PERFORMANCE_COMMON_H
#define _NT_PERFORMANCE_COMMON_H

#include <linux/types.h>

extern void set_hook_trace_android_vh_syscall_prctl_finished(bool);
extern bool get_hook_trace_android_vh_syscall_prctl_finished(void);

extern void set_hook_trace_android_vh_sched_setaffinity_early(bool);
extern bool get_hook_trace_android_vh_sched_setaffinity_early(void);

extern void nt_probe_android_vh_syscall_prctl_finished(void *, int, struct task_struct *);

#endif /* _NT_PERFORMANCE_COMMON_H */
