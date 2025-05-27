/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM power
#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE ged_perfetto

#if !defined(_GED_PERFETTO_H) || defined(TRACE_HEADER_MULTI_READ)
#define _GED_PERFETTO_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(gpu,

		    TP_PROTO(unsigned int state, unsigned int gpu_id),

		    TP_ARGS(state, gpu_id),

		    TP_STRUCT__entry(__field(u32, state) __field(u32, gpu_id)),

		    TP_fast_assign(__entry->state = state; __entry->gpu_id = gpu_id;),

		    TP_printk("state=%lu gpu_id=%lu", (unsigned long)__entry->state,
			      (unsigned long)__entry->gpu_id));

DEFINE_EVENT(gpu, gpu_frequency,

	     TP_PROTO(unsigned int frequency, unsigned int gpu_id),

	     TP_ARGS(frequency, gpu_id));

void ged_perfetto_update_frequency(unsigned int frequency, unsigned int gpu_id);
#endif /* _GED_PERFETTO_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
