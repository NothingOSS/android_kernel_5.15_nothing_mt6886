// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#define CREATE_TRACE_POINTS
#include "ged_perfetto.h"

void ged_perfetto_update_frequency(unsigned int frequency, unsigned int gpu_id)
{
	trace_gpu_frequency(frequency, gpu_id);
}
