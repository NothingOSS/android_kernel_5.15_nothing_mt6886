#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define GAIN_VAL  10

extern int nt_board_id;
static unsigned int GPIO_SEL1;
static unsigned int board_id_version = 0;
enum {
    ADC_VAL_PreT0 = 50,
    ADC_VAL_T0_1  = 210,
    ADC_VAL_T0_2  = 420,
    ADC_VAL_EVT_1  = 620,
    ADC_VAL_EVT_2  = 820,
    ADC_VAL_EVT_3  = 1040,
    ADC_VAL_EVT_4  = 1240,
    ADC_VAL_DVT_1  = 1450,
    ADC_VAL_PVT  = 50,
    ADC_VAL_MP   = 210,
};

enum {
    ADC_VAL_PreT0_GAIN_VAL = 5,
    ADC_VAL_T0_1_GAIN_VAL  = 14,
    ADC_VAL_T0_2_GAIN_VAL  = 30,
    ADC_VAL_EVT_1_GAIN_VAL  = 43,
    ADC_VAL_EVT_2_GAIN_VAL  = 57,
    ADC_VAL_EVT_3_GAIN_VAL  = 72,
    ADC_VAL_EVT_4_GAIN_VAL  = 86,
    ADC_VAL_DVT_1_GAIN_VAL  = 100,
    ADC_VAL_PVT_GAIN_VAL  = 5,
    ADC_VAL_MP_GAIN_VAL   = 14,
};

enum {
    IDX_PreT0 = 0,
    IDX_T0_1,
    IDX_T0_2,
    IDX_EVT_1,
    IDX_EVT_2,
    IDX_EVT_3,
    IDX_EVT_4,
    IDX_DVT_1,
    IDX_PVT,
    IDX_MP,
    IDX_UNKNOW,
};
static const char * const hwid_type_text[] = {
    "PreT0",
    "T0_1",
    "T0_2",
    "EVT_1",
    "EVT_2",
    "EVT_3",
    "EVT_4",
    "DVT_1",
    "PVT",
    "MP",
    "Unknow",
};
static unsigned int sel1 = 0;
static int adc_val = 0;
extern int ccci_get_adc_mV(void);

static int prize_pcb_version_show(struct seq_file *m, void *data)
{
    seq_printf(m, "version = %s    gpio8_val=%d  AUXADC_VIN1_val=%d.\n", hwid_type_text[board_id_version], sel1, adc_val);

    return 0;
}

static int prize_pcb_version_open(struct inode *node, struct file *file)
{
    return single_open(file, prize_pcb_version_show, PDE_DATA(node));
}

static const struct proc_ops prize_pcb_version_fops = {
    //.owner = THIS_MODULE,
    .proc_open = prize_pcb_version_open,
    .proc_read = seq_read,
    .proc_lseek = seq_lseek,
    .proc_release = single_release,
    .proc_write = NULL,
};

static int setup_board_id_proc_files(struct platform_device *pdev)
{
    struct proc_dir_entry *entry = NULL;

    entry = proc_create("hwid", 0644, NULL, &prize_pcb_version_fops);
    if (!entry) {
        return -ENODEV;
    }
    return 0;
}

#define DPS_DEV_NAME  "prize,board-id"
#ifdef CONFIG_OF
static const struct of_device_id board_id_of_match[] = {
    {.compatible = DPS_DEV_NAME},
    {},
};
MODULE_DEVICE_TABLE(of, board_id_of_match);
#endif

static int board_id_probe(struct platform_device *pdev)
{
    struct device	*dev = &pdev->dev;
    const struct of_device_id *match;
    int ret = 0;

    pr_err("%s:%d start\n",__func__,__LINE__);

    if (dev->of_node) {
        match = of_match_device(of_match_ptr(board_id_of_match), dev);
        if (!match) {
            pr_err("[board_id_probe][ERROR] No device match found\n");
            return -ENODEV;
        }
    }

    GPIO_SEL1 = of_get_named_gpio(dev->of_node, "board_id_sel1", 0);

    ret = gpio_request(GPIO_SEL1, "board_id_sel1");
    if (ret < 0)
        pr_err("[BOARD_ID]Unable to request board_id_sel1\n");
    else
        pr_err("[BOARD_ID]success to request board_id_sel1\n");

    ret = gpio_direction_input(GPIO_SEL1);
    if (ret < 0) {
        pr_err("Failed to set GPIO%d as input pin(ret = %d)\n",GPIO_SEL1, ret);
    }
    sel1 = gpio_get_value(GPIO_SEL1);
    if (sel1 > 0) {         //PVT/MP
        adc_val = ccci_get_adc_mV();
        switch(adc_val)
        {
            case (0) ... (ADC_VAL_PVT + ADC_VAL_PVT_GAIN_VAL):
                board_id_version = IDX_PVT;
                break;
            case (ADC_VAL_MP - ADC_VAL_MP_GAIN_VAL) ... (ADC_VAL_MP + ADC_VAL_MP_GAIN_VAL):
                board_id_version = IDX_MP;
                break;
            default:
                board_id_version = IDX_UNKNOW;
                break;
        }
    } else if (sel1 == 0){
        adc_val = ccci_get_adc_mV();
        switch(adc_val)
        {
            case (0) ... (ADC_VAL_PreT0 + ADC_VAL_PreT0_GAIN_VAL):
                board_id_version = IDX_PreT0;
                break;
            case (ADC_VAL_T0_1 - ADC_VAL_T0_1_GAIN_VAL) ... (ADC_VAL_T0_1 + ADC_VAL_T0_1_GAIN_VAL):
                board_id_version = IDX_T0_1;
                break;
            case (ADC_VAL_T0_2 - ADC_VAL_T0_2_GAIN_VAL) ... (ADC_VAL_T0_2 + ADC_VAL_T0_2_GAIN_VAL) :
                board_id_version = IDX_T0_2;
                break;
            case (ADC_VAL_EVT_1 - ADC_VAL_EVT_1_GAIN_VAL) ... (ADC_VAL_EVT_1 + ADC_VAL_EVT_1_GAIN_VAL):
                board_id_version = IDX_EVT_1;
                break;
            case (ADC_VAL_EVT_2 - ADC_VAL_EVT_2_GAIN_VAL) ... (ADC_VAL_EVT_2 + ADC_VAL_EVT_2_GAIN_VAL):
                board_id_version = IDX_EVT_2;
                break;
            case (ADC_VAL_EVT_3 - ADC_VAL_EVT_3_GAIN_VAL) ... (ADC_VAL_EVT_3 + ADC_VAL_EVT_3_GAIN_VAL):
                board_id_version = IDX_EVT_3;
                break;
            case (ADC_VAL_EVT_4 - ADC_VAL_EVT_4_GAIN_VAL) ... (ADC_VAL_EVT_4 + ADC_VAL_EVT_4_GAIN_VAL):
                board_id_version = IDX_EVT_4;
                break;
            case (ADC_VAL_DVT_1 - ADC_VAL_DVT_1_GAIN_VAL) ... (ADC_VAL_DVT_1 + ADC_VAL_DVT_1_GAIN_VAL):
                board_id_version = IDX_DVT_1;
                break;
            default:
                board_id_version = IDX_UNKNOW;
                break;
        }
    }
    else {
        adc_val = 0;
        board_id_version = IDX_UNKNOW;
        pr_err("Failed to get GPIO%d value:%d\n", GPIO_SEL1, ret);
    }
    pr_err("[BOARD_ID]sel1:%d, ,adc_val:%d\n", sel1, adc_val);

    if (board_id_version == IDX_UNKNOW)
        nt_board_id = -1;
    else
        nt_board_id = board_id_version;
    setup_board_id_proc_files(pdev);

    return 0;
}

static int board_id_remove(struct platform_device *pdev)
{
    return 0;
}

static struct platform_driver board_id_platform_driver = {
    .probe = board_id_probe,
    .remove = board_id_remove,
    .driver = {
        .name = DPS_DEV_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = board_id_of_match,
#endif
    },
};

static int __init board_id_init(void)
{
    int ret;

    pr_err("%s-%d start\n",__func__,__LINE__);

    ret = platform_driver_register(&board_id_platform_driver);
    if (ret) {
        pr_err("Failed to register board_id platform driver\n");
        return ret;
    }

    pr_err("%s-%d end\n",__func__,__LINE__);

    return 0;
}

static void __exit board_id_exit(void)
{
    platform_driver_unregister(&board_id_platform_driver);
}

module_init(board_id_init);
module_exit(board_id_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("xiaojun.niu");
MODULE_DESCRIPTION("board_id Driver");
