#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <sys/wait.h>


#define AWINIC_SMARTPA_CALI_RE   "/sys/class/smartpa/re25_calib"
#define AWINIC_SMARTPA_CALI_F0    "/sys/class/smartpa/f0_calib"
#define AWINIC_SMARTPA_CALI_TIEM "/sys/class/smartpa/calib_time"

enum {
	AW_DEV_CH_PRI_L = 0,
	AW_DEV_CH_PRI_R = 1,
	AW_DEV_CH_SEC_L = 2,
	AW_DEV_CH_SEC_R = 3,
	AW_DEV_CH_MAX,
};

static char *ch_name[AW_DEV_CH_MAX] = {"pri_l", "pri_r", "sec_l", "sec_r"};

static int aw882xx_cali_re(int *buf, int buf_size)
{
	int fd_re = 0;
	int ret, i;
	char read_buf[100] = { 0 };
	int cali_re[AW_DEV_CH_MAX] = {0};

	fd_re = open (AWINIC_SMARTPA_CALI_RE, O_RDWR);
	if (fd_re < 0) {
		printf("%s:can not open : %s \n", __func__, AWINIC_SMARTPA_CALI_RE);
		return -1;
	}

	ret = read(fd_re, read_buf, 100);
	if (ret <= 0) {
		printf("%s: read re failed \n", __func__);
		ret = -1;
		goto exit;
	}

	ret = sscanf(read_buf, "pri_l:%d mOhms pri_r:%d mOhms sec_l:%d mOhms sec_r:%d mOhms ",
				&cali_re[AW_DEV_CH_PRI_L], &cali_re[AW_DEV_CH_PRI_R],
				&cali_re[AW_DEV_CH_SEC_L], &cali_re[AW_DEV_CH_SEC_R]);
	if (!ret) {
		printf("%s:get cali_re failed ,[%s]", __func__, read_buf);
		ret = -1;
		goto exit;
	}

	for (i = 0; i < ret; i++) {
		buf[i] = cali_re[i];
	}

exit:
	if (fd_re) {
		close(fd_re);
	}
	return ret;
}

static int aw882xx_write_cali_re_to_dirver(int *cali_re, int buf_size)
{
	int fd_re = 0, len = 0;
	int ret, i;
	char write_buf[256] = { 0 };

	fd_re = open (AWINIC_SMARTPA_CALI_RE, O_RDWR);
	if (fd_re < 0) {
		printf("%s:can not open : %s \n", __func__, AWINIC_SMARTPA_CALI_RE);
		return -1;
	}

	for (i = 0; i < buf_size; i ++) {
		len += snprintf(write_buf+len, 100-len, "%s:%d ", ch_name[i], cali_re[i]);
	}

	ret = write(fd_re, write_buf, len);
	if (ret < 0) {
		printf("%s: write [%s] failed \n", __func__, write_buf);
		ret = -1;
		goto exit;
	}

	printf("write [%s] success \n", write_buf);
exit:
	if (fd_re) {
		close(fd_re);
	}
	return ret;
}

static int aw882xx_cali_f0(int *buf, int buf_siz)
{
	int fd_f0 = 0;
	int ret,i;
	char read_buf[100] = { 0 };
	int cali_f0[AW_DEV_CH_MAX] = {0};

	fd_f0 = open (AWINIC_SMARTPA_CALI_F0, O_RDWR);
	if (fd_f0 < 0) {
		printf("%s:can not open : %s \n", __func__, AWINIC_SMARTPA_CALI_F0);
		return -1;
	}

	ret = read(fd_f0, read_buf, 100);
	if (ret <= 0) {
		printf("%s: read f0 failed \n", __func__);
		ret = -1;
		goto exit;
	}

	ret = sscanf(read_buf, "pri_l:%d pri_r:%d sec_l:%d  sec_r:%d ",
				&cali_f0[AW_DEV_CH_PRI_L], &cali_f0[AW_DEV_CH_PRI_R],
				&cali_f0[AW_DEV_CH_SEC_L], &cali_f0[AW_DEV_CH_SEC_R]);
	if (!ret) {
		printf("%s:get cali_f0 failed ,[%s]", __func__, read_buf);
		ret = -1;
		goto exit;
	}

	for (i = 0; i < ret; i++) {
		buf[i] = cali_f0[i];
	}

exit:
	if (fd_f0) {
		close(fd_f0);
	}
	return ret;
}

int main(void)
{
	int cali_re[AW_DEV_CH_MAX] = {0};
	int cali_f0[AW_DEV_CH_MAX] = {0};
	int ret = 0, i;

	ret = aw882xx_cali_re(cali_re, AW_DEV_CH_MAX);
	if (ret < 0) {
		printf("cali_re failed");
		return -1;
	}

	for (i = 0; i < ret; i++) {
		printf("%s: %d mOhms \n", ch_name[i], cali_re[i]);
	}

	aw882xx_write_cali_re_to_dirver(cali_re, ret);

	ret = aw882xx_cali_f0(cali_f0, AW_DEV_CH_MAX);
	if (ret < 0) {
		printf("cali_f0 failed");
		return -1;
	}

	for (i = 0; i < ret; i++) {
		printf("%s: %d Hz \n", ch_name[i], cali_f0[i]);
	}
	return 0;
}

