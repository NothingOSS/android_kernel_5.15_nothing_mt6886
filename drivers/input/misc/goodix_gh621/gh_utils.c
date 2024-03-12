
#include "gh_core.h"

#define MAX_LOG_STR_LEN				160

static u8 g_log_level = GH621X_LOG_LEVEL_INFO;//GH621X_LOG_LEVEL_EX2;

void gh_log_level_set(int level)
{
	g_log_level = level;
}

void gh_log_print(int level, const char *fmt, ...)
{
	char log_msg[MAX_LOG_STR_LEN] = {0};
	va_list args;

	if (level <= g_log_level) {
		va_start(args, fmt);
		vsnprintf(log_msg, sizeof(log_msg), fmt, args);
		va_end(args);
		pr_info("[gh621x] %s", log_msg);
	}
}

void gh_be16_to_cpu_array(u16 *buffer, int len)
{
	int i;

	for (i = 0; i < len; i++)
		buffer[i] = be16_to_cpu(buffer[i]);
}

/* convert u16 data buffer to u32 data
 * @src: pointer to u16 buffer
 * @target: pointer to u32 target array
 * @num: u32 data nums
 */
void gh_be32_to_cpu_array(u16 *src, u32 *target, int num)
{
	int i, j;

	// TODO: does not work in big endian
	for (i = 0, j = 0; j < num; j++) {
		target[j] = src[i] | (src[i + 1] << 16);
		i += 2;
	}
}