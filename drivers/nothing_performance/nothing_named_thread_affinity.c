#include <linux/bitmap.h>
#include <linux/cpumask.h>
#include <linux/cpuset.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sched/cputime.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include <linux/atomic/atomic-instrumented.h>

#include <kernel/sched/sched.h>

#include <uapi/linux/prctl.h>

#include <trace/hooks/vendor_hooks.h>
#include <trace/hooks/sys.h>
#include <trace/hooks/sched.h>

#include "nothing_named_thread_affinity.h"
#include "nt_performance_proc.h"
#include "nt_performance_print.h"
#include "nt_performance_common.h"

#ifdef TAG
#undef TAG
#define TAG "nt_named_thread_affinity"
#endif /* TAG */

#define PID_BUF_SIZE                           (SZ_64)
#define NAMED_THREAD_AFFINITY_BUF_SIZE         (SZ_64)
#define MAX_THREAD_NAME_SIZE                   (TASK_COMM_LEN)
#define DEBUG_BUF_SIZE                         (SZ_4)

#define MAX_POLICY_COUNT                       (16)
#define MAX_SET_TASK_AFFINITY_WORK_COUNT       (256)

#define SET_TASK_AFFINITY_WORK_DELAY_MS        (50)

#define DISABLE     (0)
#define ENABLE      (1)

enum {
	NAMED_THREAD_AFFINITY_TARGET_PID         = 0,
	NAMED_THREAD_AFFINITY_TARGET_POLICY_LIST = 1,
	NAMED_THREAD_AFFINITY_TARGET_RESET       = 2,
	NAMED_THREAD_AFFINITY_TARGET_DEBUG       = 3,
	NAMED_THREAD_AFFINITY_TARGET_MAX,
};

static struct proc_dir_entry *nt_named_thread_affinity_dir_pentry = NULL;
CREATE_PROC_ENTRY_AND_FOPS(target_pid)
CREATE_PROC_ENTRY_AND_FOPS(target_named_thread_affinity)
CREATE_PROC_ENTRY_AND_FOPS(reset)
CREATE_PROC_ENTRY_AND_FOPS(debug)

typedef struct {
	bool policy_in_use;
	char thread_name[MAX_THREAD_NAME_SIZE];
	cpumask_t cpumask;
} named_thread_affinity_policy;

#define DEFINE_NAMED_THREAD_AFFINITY_POLICY_LIST(x) \
	named_thread_affinity_policy x[MAX_POLICY_COUNT];

DEFINE_NAMED_THREAD_AFFINITY_POLICY_LIST(target_policy_list)

static pid_t target_pid = 0;
static atomic_t target_info_sufficient = ATOMIC_INIT(DISABLE);
static atomic_t target_policy_count = ATOMIC_INIT(0);
static atomic_t set_task_affinity_work_count = ATOMIC_INIT(0);
static atomic_t debug = ATOMIC_INIT(DISABLE);

typedef struct {
	atomic_t in_use;
	pid_t pid;
	cpumask_t mask;
	bool silence;
	struct delayed_work work;
} set_task_affinity_work;
static set_task_affinity_work set_task_affinity_work_list[MAX_SET_TASK_AFFINITY_WORK_COUNT];
static struct workqueue_struct *set_task_affinity_wq = NULL;

void init_target_info_value(void);
void reset_target_info_and_work(void);
bool is_vendor_hooks_registered(void);

DEFINE_SPINLOCK(target_info_spinlock);
void inline lock_target_info(void)
{
	unsigned long flags = 0;
	spin_lock_irqsave(&target_info_spinlock, flags);
}

void inline unlock_target_info(void)
{
	unsigned long flags = 0;
	spin_unlock_irqrestore(&target_info_spinlock, flags);
}

int get_target_info(pid_t *pid, named_thread_affinity_policy *list)
{
	int ret = 0;

	if (!pid || !list) {
		ret = -EINVAL;
		goto out;
	}

	lock_target_info();

	*pid = target_pid;
	memcpy(list, target_policy_list, sizeof(target_policy_list));

	unlock_target_info();

out:
	return ret;
}

bool inline is_target_info_sufficient(void)
{
	return (atomic_read(&target_info_sufficient) == ENABLE);
}

bool inline update_target_info_sufficient_locked(void)
{
	if (target_pid && atomic_read(&target_policy_count)) {
		atomic_set(&target_info_sufficient, ENABLE);
		return true;
	}

	atomic_set(&target_info_sufficient, DISABLE);
	return false;
}

bool check_intersects_with_online_cpu(cpumask_t *mask)
{
	return cpumask_intersects(mask, cpu_online_mask);
}

static int nt_sched_setaffinity(struct task_struct *p, const struct cpumask *in_mask, bool silence)
{
	int ret = 0;
	cpumask_var_t cpus_allowed, new_mask;

	if (!p || !in_mask) {
		ret = -EINVAL;
		goto out;
	}

	get_task_struct(p);

	if (p->flags & PF_NO_SETAFFINITY) {
		ret = -EINVAL;
		goto out_put_task;
	}

	if (task_has_dl_policy(p)) {
		ret = -EPERM;
		goto out_put_task;
	}

	if (!alloc_cpumask_var(&cpus_allowed, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto out_put_task;
	}

	if (!alloc_cpumask_var(&new_mask, GFP_KERNEL)) {
		ret = -ENOMEM;
		goto out_free_cpus_allowed;
	}

	cpuset_cpus_allowed(p, cpus_allowed);
	if (!cpumask_intersects(cpus_allowed, in_mask)) {
		ret = -EPERM;
		goto out_free_new_mask;
	}
	cpumask_and(new_mask, in_mask, cpus_allowed);

	ret = set_cpus_allowed_ptr(p, new_mask);
	if (!ret && !silence) {
		pr_info("Set cpu affinity to task %s:%d %*pbl", p->comm, p->pid, cpumask_pr_args(new_mask));
	}

out_free_new_mask:
	free_cpumask_var(new_mask);

out_free_cpus_allowed:
	free_cpumask_var(cpus_allowed);

out_put_task:
	put_task_struct(p);

out:
	return ret;
}

void inline clean_delay_work(set_task_affinity_work *delay_work)
{
	cpumask_t all_mask = CPU_MASK_ALL;
	if (!delay_work) {
		return;
	}

	delay_work->pid = 0;
	cpumask_copy(&delay_work->mask, &all_mask);
	delay_work->silence = false;
	atomic_set(&delay_work->in_use, DISABLE);
}

static void set_task_affinity_delay_work(struct work_struct *work)
{
	set_task_affinity_work *delay_work = NULL;
	struct task_struct *t;
	pid_t pid;
	cpumask_t new_mask;
	bool silence;

	if (!work) {
		return;
	}
	delay_work = container_of(to_delayed_work(work), set_task_affinity_work, work);

	if (atomic_read(&delay_work->in_use) != ENABLE) {
		return;
	}

	if (!is_target_info_sufficient()) {
		goto done;
	}

	pid = delay_work->pid;
	cpumask_copy(&new_mask, &delay_work->mask);
	silence = delay_work->silence;

	rcu_read_lock();
	t = find_task_by_vpid(pid);
	rcu_read_unlock();
	if (!t) {
		pr_err("Task not found after delay: %d", pid);
		goto done;
	}

	if (!check_intersects_with_online_cpu(&new_mask)) {
		goto done;
	}

	if (!cpumask_test_cpu(task_cpu(t), &new_mask)
			&& (task_is_running(t) || READ_ONCE(t->__state) == TASK_UNINTERRUPTIBLE)) {
		goto requeue;
	} else {
		nt_sched_setaffinity(t, &new_mask, silence);
	}

done:
	clean_delay_work(delay_work);
	atomic_dec(&set_task_affinity_work_count);
	return;

requeue:
	queue_delayed_work(set_task_affinity_wq, &delay_work->work, msecs_to_jiffies(SET_TASK_AFFINITY_WORK_DELAY_MS));
	return;
}

static int queue_set_task_affinity_delay_work(int pid, cpumask_t *new_mask, bool silence)
{
	int ret, i;
	set_task_affinity_work *delay_work = NULL;

	if (pid <= 0 || !new_mask) {
		ret = -EINVAL;
		goto out;
	}

	if (atomic_read(&set_task_affinity_work_count) >= MAX_SET_TASK_AFFINITY_WORK_COUNT) {
		pr_err("Exceed delay work count limit: %d", MAX_SET_TASK_AFFINITY_WORK_COUNT);
		ret = -EBUSY;
		goto out;
	}
	atomic_inc(&set_task_affinity_work_count);

	for (i = 0; i < MAX_SET_TASK_AFFINITY_WORK_COUNT; i++) {
		/* Find free one */
		if (atomic_read(&set_task_affinity_work_list[i].in_use) == DISABLE) {
			atomic_set(&set_task_affinity_work_list[i].in_use, ENABLE);
			delay_work = &set_task_affinity_work_list[i];
			break;
		}
	}

	if (!delay_work) {
		pr_err("Failed to find free delay work");
		ret = -EAGAIN;
		goto out;
	}

	delay_work->pid = pid;
	cpumask_copy(&delay_work->mask, new_mask);
	delay_work->silence = silence;
	queue_delayed_work(set_task_affinity_wq, &delay_work->work, msecs_to_jiffies(SET_TASK_AFFINITY_WORK_DELAY_MS));

	ret = 0;

out:
	return ret;
}

int set_task_affinity(struct task_struct *t, cpumask_t *new_mask, bool silence)
{
	int ret = 0;

	if (!t) {
		pr_err("Empty task struct input");
		ret = -EINVAL;
		goto out;
	}

	if (!new_mask) {
		pr_err("Empty cpumask input");
		ret = -EINVAL;
		goto out;
	}

	/* Bottom-half check to prevent online cpu changed */
	if (!check_intersects_with_online_cpu(new_mask)) {
		ret = -EPERM;
		goto out;
	}

	ret = queue_set_task_affinity_delay_work(t->pid, new_mask, silence);

out:
	return ret;
}

int set_task_affinity_silence(struct task_struct *t, cpumask_t *new_mask)
{
	return set_task_affinity(t, new_mask, true);
}

int set_task_affinity_notify(struct task_struct *t, cpumask_t *new_mask)
{
	return set_task_affinity(t, new_mask, false);
}

int set_target_process_named_thread_affinity(char *thread_name, cpumask_t *cpumask)
{
	int ret = 0;
	struct task_struct *p, *t;
	pid_t pid;

	if (!thread_name || !cpumask) {
		ret = -EINVAL;
		goto out;
	}

	if (!is_target_info_sufficient()) {
		ret = -EPERM;
		goto out;
	}

	lock_target_info();
	pid = target_pid;
	unlock_target_info();

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (p) {
		for_each_thread(p, t) {
			if (!t) {
				continue;
			}

			if (strnstr(t->comm, thread_name, strlen(thread_name))) {
				set_task_affinity_notify(t, cpumask);
			}
		}
	} else {
		ret = -ESRCH;
	}
	rcu_read_unlock();

out:
	return ret;
}

void set_named_thread_affinity(struct task_struct *p)
{
	pid_t pid;
	DEFINE_NAMED_THREAD_AFFINITY_POLICY_LIST(policy_list)
	int i;
	cpumask_t all_mask = CPU_MASK_ALL;

	if (!p) {
		return;
	}

	if (!is_target_info_sufficient()) {
		return;
	}

	if (get_target_info(&pid, policy_list)) {
		return;
	}

	if (p->tgid != pid) {
		return;
	}

	for (i = 0; i < MAX_POLICY_COUNT; i++)  {

		if (!policy_list[i].policy_in_use) {
			continue;
		}

		if (strnstr(p->comm, policy_list[i].thread_name, strlen(policy_list[i].thread_name))) {
			/* match */
			if (atomic_read(&debug)) {
				pr_info("Check match %s: %d, %s", policy_list[i].thread_name, p->pid, p->comm);
			}
			set_task_affinity_notify(p, &policy_list[i].cpumask);
			break;
		}
	}

	if (i == MAX_POLICY_COUNT) {
		/* not match */
		/* Prevent inherit parent set_named_thread_affinity result */
		if (atomic_read(&debug)) {
			pr_info("Use default: %d, %s", p->pid, p->comm);
			set_task_affinity_notify(p, &all_mask);
		} else {
			set_task_affinity_silence(p, &all_mask);
		}
	}
}

static int show_named_thread_affinity_target_value(struct seq_file *m, int type)
{
	int ret, i;
	pid_t pid;
	DEFINE_NAMED_THREAD_AFFINITY_POLICY_LIST(policy_list)

	if (type < 0 || type >= NAMED_THREAD_AFFINITY_TARGET_MAX) {
		pr_err("Invalid target info type %d", type);
		ret = -EINVAL;
		goto out;
	}

	if (get_target_info(&pid, policy_list)) {
		pr_err("Failed to get target info");
		ret = -EAGAIN;
		goto out;
	}

	switch (type) {
		case NAMED_THREAD_AFFINITY_TARGET_PID:
			seq_printf(m, "%d\n", pid);
			break;

		case NAMED_THREAD_AFFINITY_TARGET_POLICY_LIST:
			for (i = 0; i < MAX_POLICY_COUNT; i++) {

				if (!policy_list[i].policy_in_use) {
					continue;
				}

				seq_printf(m, "%s %*pbl\n",
						policy_list[i].thread_name,
						cpumask_pr_args(&policy_list[i].cpumask));
			}
			break;

		case NAMED_THREAD_AFFINITY_TARGET_DEBUG:
			seq_printf(m, "%d\n", atomic_read(&debug));
			break;

		case NAMED_THREAD_AFFINITY_TARGET_RESET:
		default:
			break;
	}

out:
	return 0;
}

int add_target_named_thread_affinity_policy(char *thread_name, cpumask_t *cpumask)
{
	int ret = -1;
	int i;

	if (!thread_name || !cpumask) {
		goto out;
	}

	lock_target_info();
	/* Replace same name and is in use */
	for (i = 0; i < MAX_POLICY_COUNT; i++)  {
		if (strncmp(thread_name, target_policy_list[i].thread_name, MAX_THREAD_NAME_SIZE)) {
			continue;
		}

		/* Match */
		/* Check is in use */
		if (!target_policy_list[i].policy_in_use) {
			break;
		}

		cpumask_copy(&target_policy_list[i].cpumask, cpumask);
		ret = 0;
		goto done;
	}

	/* Not found */
	if (atomic_read(&target_policy_count) >= MAX_POLICY_COUNT) {
		/* Exceed policy limit, cannot add */
		pr_err("Exceed policy count limit: %d", MAX_POLICY_COUNT);
		goto fail;
	}

	for (i = 0; i < MAX_POLICY_COUNT; i++) {
		if (target_policy_list[i].policy_in_use) {
			continue;
		}

		strncpy(target_policy_list[i].thread_name, thread_name, MAX_THREAD_NAME_SIZE);
		cpumask_copy(&target_policy_list[i].cpumask, cpumask);
		target_policy_list[i].policy_in_use = true;

		atomic_inc(&target_policy_count);
		update_target_info_sufficient_locked();

		ret = 0;
		goto done;
	}

fail:
	ret = -EBUSY;

done:
	unlock_target_info();

out:
	return ret;
}

static ssize_t debug_proc_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	int ret = count;
	int debug_val;
	char buf[DEBUG_BUF_SIZE];

	if (!count || count > DEBUG_BUF_SIZE - 1) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	buf[count] = '\0';
	if (kstrtoint(buf, 10, &debug_val)) {
		ret = -EINVAL;
		goto out;
	}

	if (debug_val > 0) {
		atomic_set(&debug, ENABLE);
	} else {
		atomic_set(&debug, DISABLE);
	}

out:
	return ret;
}

static int debug_proc_show(struct seq_file *m, void *v)
{
	return show_named_thread_affinity_target_value(m, NAMED_THREAD_AFFINITY_TARGET_DEBUG);
}

static int debug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, debug_proc_show, NULL);
}

static ssize_t reset_proc_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	int ret = count;

	reset_target_info_and_work();
	return ret;
}

static int reset_proc_show(struct seq_file *m, void *v)
{
	return show_named_thread_affinity_target_value(m, NAMED_THREAD_AFFINITY_TARGET_RESET);
}

static int reset_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, reset_proc_show, NULL);
}

#define DELIMITER       (" ")
#define LINE_FEED_UNIX  ('\n')
#define LINE_FEED_DOS   ('\r')
static ssize_t target_named_thread_affinity_proc_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	int ret = count;
	char buf[NAMED_THREAD_AFFINITY_BUF_SIZE];
	char *p_buf = buf;
	char *thread_name_raw = NULL;
	size_t thread_name_length = 0;
	char *cpumask_raw = NULL;
	cpumask_t cpumask;
	size_t cpumask_length = 0;

	if (!is_vendor_hooks_registered()) {
		ret = -EPERM;
		if (atomic_read(&debug)) {
			pr_err("Vendor hook not ready, skip setting named_thread_affinity");
		}
		goto out;
	}

	if (!count || count > NAMED_THREAD_AFFINITY_BUF_SIZE - 1) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	/* Handle line feed */
	if (buf[count - 1] == LINE_FEED_UNIX || buf[count - 1] == LINE_FEED_DOS) {
		buf[count - 1] = '\0';
	} else {
		buf[count] = '\0';
	}

	/* Split string */
	thread_name_raw = strsep(&p_buf, DELIMITER);
	if (!thread_name_raw) {
		pr_err("Failed to split target thread name");
		ret = -EINVAL;
		goto out;
	}

	cpumask_raw = strsep(&p_buf, DELIMITER);
	if (!cpumask_raw) {
		pr_err("Failed to split target cpumask");
		ret = -EINVAL;
		goto out;
	}

	/* Handle data */
	thread_name_length = strlen(thread_name_raw);
	if (!thread_name_length) {
		pr_err("Failed to get target thread name");
		ret = -EINVAL;
		goto out;
	}
	if (thread_name_length > MAX_THREAD_NAME_SIZE - 1) {
		pr_err("Error, thread name too long");
		ret = -EINVAL;
		goto out;
	}

	cpumask_length = strlen(cpumask_raw);
	if (!cpumask_length) {
		pr_err("Failed to get target cpumask");
		ret = -EINVAL;
		goto out;
	}

	if (bitmap_parselist(cpumask_raw, cpumask_bits(&cpumask), nr_cpumask_bits)) {
		pr_err("Failed to parse target cpumask");
		ret = -EIO;
		goto out;
	}

	/* Prevent set unsuitable cpumask */
	if (!check_intersects_with_online_cpu(&cpumask)) {
		pr_err("Target cpumask is empty intersects with online cpu");
		ret = -EINVAL;
		goto out;
	}

	ret = add_target_named_thread_affinity_policy(thread_name_raw, &cpumask);
	if (ret) {
		pr_err("Failed to add target policy %s %s", thread_name_raw, cpumask_raw);
		goto out;
	}

	if (!is_target_info_sufficient()) {
		goto done;
	}

	pr_info("Add target policy: %s %s", thread_name_raw, cpumask_raw);
	ret = set_target_process_named_thread_affinity(thread_name_raw, &cpumask);
	if (ret) {
		pr_err("Failed to set target policy %s %s", thread_name_raw, cpumask_raw);
		goto out;
	}

done:
	ret = count;

out:
	return ret;
}

static int target_named_thread_affinity_proc_show(struct seq_file *m, void *v)
{
	return show_named_thread_affinity_target_value(m, NAMED_THREAD_AFFINITY_TARGET_POLICY_LIST);
}

static int target_named_thread_affinity_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, target_named_thread_affinity_proc_show, NULL);
}

static ssize_t target_pid_proc_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
	int ret = count;
	int pid_val;
	pid_t pid;
	struct task_struct *p;
	char buf[PID_BUF_SIZE];

	if (!is_vendor_hooks_registered()) {
		ret = -EPERM;
		if (atomic_read(&debug)) {
			pr_err("Vendor hook not ready, skip setting pid");
		}
		goto out;
	}

	if (!count || count > PID_BUF_SIZE - 1) {
		ret = -EINVAL;
		goto out;
	}

	if (copy_from_user(buf, ubuf, count)) {
		ret = -EFAULT;
		goto out;
	}

	buf[count] = '\0';
	if (kstrtoint(buf, 10, &pid_val)) {
		ret = -EINVAL;
		goto out;
	}

	if (pid_val <= 0) {
		ret = -EINVAL;
		goto out;
	}

	pid = pid_val;
	rcu_read_lock();
	p = find_task_by_vpid(pid);
	rcu_read_unlock();

	if (!p) {
		pr_err("Failed to find process of pid: %d", target_pid);
		ret = -ESRCH;
	}

	/* Input pid is correct ! reset all info */
	reset_target_info_and_work();

	/* Set new target pid */
	lock_target_info();
	target_pid = pid;
	unlock_target_info();
	pr_info("Set target pid: %d", target_pid);


out:
	return ret;
}

static int target_pid_proc_show(struct seq_file *m, void *v)
{
	return show_named_thread_affinity_target_value(m, NAMED_THREAD_AFFINITY_TARGET_PID);
}

static int target_pid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, target_pid_proc_show, NULL);
}

void remove_nt_named_thread_affinity_proc_node(void)
{
	REMOVE_PROC_ENTRY(debug_node_pentry)
	REMOVE_PROC_ENTRY(reset_node_pentry)
	REMOVE_PROC_ENTRY(target_named_thread_affinity_node_pentry)
	REMOVE_PROC_ENTRY(target_pid_node_pentry)
	REMOVE_PROC_ENTRY(nt_named_thread_affinity_dir_pentry)
}

int create_nt_named_thread_affinity_proc_node(void)
{
	int ret;

	nt_named_thread_affinity_dir_pentry = proc_mkdir("nt_named_thread_affinity", NULL);
	if (!nt_named_thread_affinity_dir_pentry)
	{
		pr_err("Mkdir nt_named_thread_affinity dir failed");
		ret = -ENOMEM;
		goto out;
	}

	target_pid_node_pentry = proc_create(
			"pid",
			S_IRUGO | S_IWUGO,
			nt_named_thread_affinity_dir_pentry,
			&target_pid_proc_fops);
	if (!target_pid_node_pentry) {
		pr_err("Create pid node failed");
		ret = -ENOMEM;
		goto clean;
	}

	target_named_thread_affinity_node_pentry = proc_create(
			"named_thread_affinity",
			S_IRUGO | S_IWUGO,
			nt_named_thread_affinity_dir_pentry,
			&target_named_thread_affinity_proc_fops);
	if (!target_named_thread_affinity_node_pentry) {
		pr_err("Create named_thread_affinity node failed");
		ret = -ENOMEM;
		goto clean;
	}

	reset_node_pentry = proc_create(
			"reset",
			S_IRUGO | S_IWUGO,
			nt_named_thread_affinity_dir_pentry,
			&reset_proc_fops);
	if (!reset_node_pentry) {
		pr_err("Create reset node failed");
		ret = -ENOMEM;
		goto clean;
	}

	debug_node_pentry = proc_create(
			"debug",
			S_IRUGO | S_IWUGO,
			nt_named_thread_affinity_dir_pentry,
			&debug_proc_fops);
	if (!debug_node_pentry) {
		pr_err("Create debug node failed");
		ret = -ENOMEM;
		goto clean;
	}

	ret = 0;
	goto out;

clean:
	remove_nt_named_thread_affinity_proc_node();

out:
	return ret;
}

bool is_skip_process_setaffinity(struct task_struct *p, const struct cpumask *in_mask)
{
	int i;
	pid_t pid;
	DEFINE_NAMED_THREAD_AFFINITY_POLICY_LIST(policy_list)

	if (!p || !in_mask) {
		return false;
	}

	if (!is_target_info_sufficient()) {
		return false;
	}

	if (get_target_info(&pid, policy_list)) {
		return false;
	}

	if (p->tgid != pid) {
		return false;
	}

	for (i = 0; i < MAX_POLICY_COUNT; i++)  {

		if (!policy_list[i].policy_in_use) {
			continue;
		}

		if (strnstr(p->comm, policy_list[i].thread_name, strlen(policy_list[i].thread_name))) {
			/* match */
			return true;
		}
	}

	return false;
}
EXPORT_SYMBOL_GPL(is_skip_process_setaffinity);

bool is_vendor_hooks_registered(void)
{
	bool ret = false;

	if (!get_hook_trace_android_vh_syscall_prctl_finished()) {
		if (atomic_read(&debug)) {
			pr_err("Check trace_android_vh_syscall_prctl_finished register failed");
		}
		goto out;
	}

	if (!get_hook_trace_android_vh_sched_setaffinity_early()) {
		if (atomic_read(&debug)) {
			pr_err("Check trace_android_vh_sched_setaffinity_early register failed");
		}
		goto out;
	}

	ret = true;

out:
	return ret;
}

/*
 * Hook in probe_android_vh_sched_setaffinity_early
 *
 * This function will skip setaffinity action if match currnet policy.
 */
void check_setaffinity_skip_by_named_thread_affinity(void *ignore, struct task_struct *p, const struct cpumask *in_mask, int *skip)
{
	if (!p || !in_mask || !skip) {
		return;
	}

	if (!is_target_info_sufficient()) {
		return;
	}

	if (!is_vendor_hooks_registered()) {
		return;
	}

	*skip = is_skip_process_setaffinity(p, in_mask) ? 1 : 0;
}

/*
 * Hook in probe_android_vh_syscall_prctl_finished
 *
 * This function will trigger set_affinity_to_thread to thread just being named
 * if relative info is sufficient.
 */
void set_thread_affinity_on_set_name(void *ignore, int option, struct task_struct *p)
{

	if (!p) {
		return;
	}

	/* Filter and reduce error logs */
	if (!is_target_info_sufficient()) {
		return;
	}

	if (!is_vendor_hooks_registered()) {
		return;
	}

	if (option == PR_SET_NAME) {
		set_named_thread_affinity(p);
	}
}

void cancel_all_delay_work(void)
{
	int i;

	/* Stop insert new  */
	atomic_set(&set_task_affinity_work_count, MAX_SET_TASK_AFFINITY_WORK_COUNT);
	for (i = 0; i < MAX_SET_TASK_AFFINITY_WORK_COUNT; i++) {
		if (atomic_read(&set_task_affinity_work_list[i].in_use)) {
			cancel_delayed_work_sync(&set_task_affinity_work_list[i].work);
			clean_delay_work(&set_task_affinity_work_list[i]);
		}
	}
	atomic_set(&set_task_affinity_work_count, 0);
}

/* Can only use in init stage */
void init_set_task_affinity_work_list(void)
{
	int i;
	for (i = 0; i < MAX_SET_TASK_AFFINITY_WORK_COUNT; i++) {
		clean_delay_work(&set_task_affinity_work_list[i]);
		INIT_DELAYED_WORK(&set_task_affinity_work_list[i].work, set_task_affinity_delay_work);
	}
}

int init_set_task_affinity_wq(void)
{
	int ret = 0;

	set_task_affinity_wq = alloc_workqueue("nt_set_task_affinity_wq", WQ_UNBOUND , 0);
	if (!set_task_affinity_wq) {
		ret = -ENOMEM;
		goto out;
	}

out:
	return ret;
}

void release_set_task_affinity_wq(void)
{
	if (set_task_affinity_wq) {
		destroy_workqueue(set_task_affinity_wq);
		set_task_affinity_wq = NULL;
	}
}

void inline init_target_policy_list(void)
{
	int i;
	cpumask_t all_mask = CPU_MASK_ALL;

	atomic_set(&target_policy_count, 0);

	lock_target_info();
	for (i = 0; i < MAX_POLICY_COUNT; i++) {
		target_policy_list[i].policy_in_use = false;
		memset(target_policy_list[i].thread_name, 0, MAX_THREAD_NAME_SIZE);
		cpumask_copy(&target_policy_list[i].cpumask, &all_mask);
	}
	unlock_target_info();
}

void inline init_target_pid(void)
{
	lock_target_info();
	target_pid = 0;
	unlock_target_info();
}

/* init pid and policy list */
void init_target_info_value(void)
{
	atomic_set(&target_info_sufficient, DISABLE);

	init_target_pid();
	init_target_policy_list();
}

void reset_target_info_and_work(void)
{
	pr_info("Reset target information and work");
	init_target_info_value();
	cancel_all_delay_work();
}

int nt_named_thread_affinity_init(void)
{
	int ret = 0;

	pr_info("Module init");

	init_target_info_value();

	init_set_task_affinity_work_list();
	ret = init_set_task_affinity_wq();
	if (ret) {
		pr_err("Failed to init set_task_affinity_wq");
		goto out;
	}

	ret = create_nt_named_thread_affinity_proc_node();
	if (ret) {
		pr_err("Failed to create proc nodes");
		goto clean_wq;
	}

	pr_info("Module init successfully");
	goto out;


clean_wq:
	release_set_task_affinity_wq();

out:
	return ret;
}
