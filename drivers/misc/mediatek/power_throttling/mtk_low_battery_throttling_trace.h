/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author: Victor Lin <victor-wc.lin@mediatek.com>
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM mtk_low_battery_throttling
#if !defined(_TRACE_MTK_LOW_BATTERY_THROTTLING_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_MTK_LOW_BATTERY_THROTTLING_H
#include <linux/tracepoint.h>
#include "mtk_low_battery_throttling.h"
TRACE_EVENT(low_battery_throttling_level,
	TP_PROTO(int low_bat_thl_level),
	TP_ARGS(low_bat_thl_level),
	TP_STRUCT__entry(
		__field(int, low_bat_thl_level)
	),
	TP_fast_assign(
		__entry->low_bat_thl_level = low_bat_thl_level;
	),
	TP_printk("exec_throttle=%d\n", __entry->low_bat_thl_level)
);

TRACE_EVENT(low_battery_throttling_cpu_freq,
	TP_PROTO(int cpu, int lbat_limit_freq),
	TP_ARGS(cpu, lbat_limit_freq),
	TP_STRUCT__entry(
		__field(int, cpu)
		__field(int, lbat_limit_freq)
	),
	TP_fast_assign(
		__entry->cpu = cpu;
		__entry->lbat_limit_freq = lbat_limit_freq;
	),
	TP_printk("CPU[%d] limit_freq=%d\n", __entry->cpu, __entry->lbat_limit_freq)
);

TRACE_EVENT(low_battery_throttling_gpu_freq,
	TP_PROTO(int lbat_limit_freq),
	TP_ARGS(lbat_limit_freq),
	TP_STRUCT__entry(
		__field(int, lbat_limit_freq)
	),
	TP_fast_assign(
		__entry->lbat_limit_freq = lbat_limit_freq;
	),
	TP_printk("limit_freq=%d\n", __entry->lbat_limit_freq)
);
#endif /* _TRACE_MTK_LOW_BATTERY_THROTTLING_H */
/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH ../../drivers/misc/mediatek/power_throttling/
#define TRACE_INCLUDE_FILE mtk_low_battery_throttling_trace
#include <trace/define_trace.h>
