
#ifndef _NOTHING_NAMED_THREAD_AFFINITY_H
#define _NOTHING_NAMED_THREAD_AFFINITY_H

#include <linux/sched.h>
#include <linux/types.h>

extern int nt_named_thread_affinity_init(void);
extern void set_thread_affinity_on_set_name(void *, int , struct task_struct *);
extern void check_setaffinity_skip_by_named_thread_affinity(void *, struct task_struct *, const struct cpumask *, int *);
extern bool is_skip_process_setaffinity(struct task_struct *, const struct cpumask *);

#endif /* _NOTHING_NAMED_THREAD_AFFINITY_H */
