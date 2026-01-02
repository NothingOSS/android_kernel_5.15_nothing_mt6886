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


#define AWINIC_SMARTPA_CALI_RE   "/sys/bus/i2c/drivers/aw882xx_smartpa/6-0034/cali_re"
#define AWINIC_SMARTPA_CALI_F0    "/sys/bus/i2c/drivers/aw882xx_smartpa/6-0034/cali_f0"


static int aw882xx_cali_re(int *buf, int buf_size)
{
	int fd_re = 0;
	int ret;
	char read_buf[100] = { 0 };
	int cali_re = {0};

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

	ret = sscanf(read_buf, "%d", &cali_re);
	if (!ret) {
		printf("%s:get cali_re failed ,[%s]", __func__, read_buf);
		ret = -1;
		goto exit;
	}

	buf[0] = cali_re;


exit:
	if (fd_re) {
		close(fd_re);
	}
	return ret;
}

static int aw882xx_write_cali_re_to_dirver(int *cali_re)
{
	int fd_re = 0, len = 0;
	int ret, i;
	char write_buf[100] = { 0 };

	fd_re = open (AWINIC_SMARTPA_CALI_RE, O_RDWR);
	if (fd_re < 0) {
		printf("%s:can not open : %s \n", __func__, AWINIC_SMARTPA_CALI_RE);
		return -1;
	}


	len += snprintf(write_buf+len, 100-len, "%d", cali_re[0]);


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
	int cali_f0 = {0};

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

	ret = sscanf(read_buf, "%d", &cali_f0);
	if (!ret) {
		printf("%s:get cali_f0 failed ,[%s]", __func__, read_buf);
		ret = -1;
		goto exit;
	}

	buf[0] = cali_f0;


exit:
	if (fd_f0) {
		close(fd_f0);
	}
	return ret;
}

int main(void)
{
	int cali_re[4] = {0};
	int cali_f0[4] = {0};
	int ret = 0, i;

	ret = aw882xx_cali_re(cali_re, 4);
	if (ret < 0) {
		printf("cali_re failed");
		return -1;
	}


	printf("%d mOhms \n", cali_re[0]);

	aw882xx_write_cali_re_to_dirver(cali_re);

	ret = aw882xx_cali_f0(cali_f0, 4);
	if (ret < 0) {
		printf("cali_f0 failed");
		return -1;
	}

	printf("%d Hz \n", cali_f0[0]);
	return 0;
}

