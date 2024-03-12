// SPDX-License-Identifier: GPL-2.0
/*
* Copyright (c) 2021 Rockchip Electronics Co. Ltd.
*
* Author: Shunqing Chen <csq@rock-chips.com>
*/

#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define DIO8018_DEVICE_D1          0x00
#define DIO8018_DEVICE_D2          0x01
#define DIO8018_DISCHARGE_RESISTORS     0x10
#define DIO8018_LDO1_VOUT        0x04
#define DIO8018_LDO2_VOUT        0x05
#define DIO8018_LDO3_VOUT        0x06
#define DIO8018_LDO4_VOUT        0x07
#define DIO8018_LDO5_VOUT        0x08
#define DIO8018_LDO6_VOUT        0x09
#define DIO8018_LDO7_VOUT        0x0A
#define DIO8018_LDO1_LDO2_SEQ        0x0B
#define DIO8018 _LDO3_LDO4_SEQ        0x0C
#define DIO8018_LDO5_LDO6_SEQ        0x0D
#define DIO8018_LDO7_SEQ           0x0E
#define DIO8018_LDO_EN               0x03
#define RESERVED_2                 0x1e
#define DIO8018_VSEL_MASK         0xff
#define DIO8018_WRITE_EN_START    0x02
#define DIO8018_RESERVED          0x14
#define DIO8018_MINT1              0x1c
#define DIO8018_RESET            0x11
#define DIO8018_WRITE_EN_END    0x1E
#define DIO8018_ENABLE            1
static int __maybe_unused dio8018_suspend(struct device *dev , int value);
static int __maybe_unused dio8018_resume(struct device *dev,int value);
struct device *ldo_enable;
int dio8018_flag;
enum dio8018_regulators {

       DIO8018_REGULATOR_LDO1 = 0,
       DIO8018_REGULATOR_LDO2,
       DIO8018_REGULATOR_LDO3,
       DIO8018_REGULATOR_LDO4,
       DIO8018_REGULATOR_LDO5,
       DIO8018_REGULATOR_LDO6,
       DIO8018_REGULATOR_LDO7,
       DIO8018_MAX_REGULATORS,
};

struct dio8018 {
       struct device *dev;
       struct regmap *regmap;
       struct regulator_dev *rdev;
       struct gpio_desc *reset_gpio;
       struct gpio_desc *dcdc_gpio;
       int min_dropout_uv;
       int ldo_vout[7];
       int ldo_en;
};

enum LDO_NUM {
       DIO8018_LDO1 = 0,
       DIO8018_LDO2,
       DIO8018_LDO3,
       DIO8018_LDO4,
       DIO8018_LDO5,
       DIO8018_LDO6,
       DIO8018_LDO7,
};

static const struct regulator_ops dio8018_reg_ops = {

       .list_voltage          = regulator_list_voltage_linear,
       .map_voltage        = regulator_map_voltage_linear,
       .get_voltage_sel    = regulator_get_voltage_sel_regmap,
       .set_voltage_sel    = regulator_set_voltage_sel_regmap,
       .enable                 = regulator_enable_regmap,
       .disable          = regulator_disable_regmap,
       .is_enabled           = regulator_is_enabled_regmap,
};

#define DIO8018_DESC(_id, _match, _supply, _min, _max, _step, _vreg,      \
       _vmask, _ereg, _emask, _enval, _disval)             \
       {                                                       \
              .id          = (_id),                        \
              .name            = (_match),                         \
              .of_match      = of_match_ptr(_match),                   \
              .supply_name = (_supply),                            \
              .min_uV         = (_min) * 1000,                 \
              .uV_step = (_step) * 1000,                 \
              .n_voltages    = (((_max) - (_min)) / (_step) + 1),      \
              .regulators_node = of_match_ptr("regulators"),       \
              .type              = REGULATOR_VOLTAGE,                \
              .vsel_reg = (_vreg),                            \
              .vsel_mask     = (_vmask),                         \
              .enable_reg   = (_ereg),                           \
              .enable_mask = (_emask),                        \
              .enable_val     = (_enval),                      \
              .disable_val     = (_disval),                     \
              .ops        = &dio8018_reg_ops,                \
              .owner           = THIS_MODULE,                      \
       }

static const struct regulator_desc dio8018_reg[] = {
       DIO8018_DESC(DIO8018_REGULATOR_LDO1, "dio8018-ldo1", "ldo1", 496, 2536, 8,
                   DIO8018_LDO1_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(0), BIT(0), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO2, "dio8018-ldo2", "ldo2", 496, 2536, 8,
                   DIO8018_LDO2_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(1), BIT(1), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO3, "dio8018-ldo3", "ldo3", 1504, 3544, 8,
                   DIO8018_LDO3_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(2), BIT(2), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO4, "dio8018-ldo4", "ldo4", 1504, 3544, 8,
                   DIO8018_LDO4_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(3), BIT(3), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO5, "dio8018-ldo5", "ldo5", 1504, 3544, 8,
                   DIO8018_LDO5_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(4), BIT(4), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO6, "dio8018-ldo6", "ldo6", 1504, 3544, 8,
                   DIO8018_LDO6_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(5), BIT(5), 0),

       DIO8018_DESC(DIO8018_REGULATOR_LDO7, "dio8018-ldo7", "ldo7", 496, 2536, 8,
                   DIO8018_LDO7_VOUT, DIO8018_VSEL_MASK, DIO8018_LDO_EN, BIT(6), BIT(6), 0),
};

static const struct regmap_range dio8018_writeable_ranges[] = {
       regmap_reg_range(DIO8018_WRITE_EN_START, DIO8018_RESERVED),
       regmap_reg_range(DIO8018_MINT1, DIO8018_WRITE_EN_END),
};

static const struct regmap_range dio8018_readable_ranges[] = {
       regmap_reg_range(DIO8018_DEVICE_D1, RESERVED_2),
};

static const struct regmap_range dio8018_volatile_ranges[] = {

       regmap_reg_range(DIO8018_WRITE_EN_START, DIO8018_RESERVED),
       regmap_reg_range(DIO8018_MINT1, DIO8018_WRITE_EN_END),
};

static const struct regmap_access_table dio8018_writeable_table = {
       .yes_ranges   = dio8018_writeable_ranges,
       .n_yes_ranges = ARRAY_SIZE(dio8018_writeable_ranges),
};

static const struct regmap_access_table dio8018_readable_table = {
       .yes_ranges   = dio8018_readable_ranges,
       .n_yes_ranges = ARRAY_SIZE(dio8018_readable_ranges),

};

static const struct regmap_access_table dio8018_volatile_table = {
       .yes_ranges   = dio8018_volatile_ranges,
       .n_yes_ranges = ARRAY_SIZE(dio8018_volatile_ranges),

};

static const struct regmap_config dio8018_regmap_config = {
       .reg_bits = 8,
       .val_bits = 8,
       .max_register = RESERVED_2,
       .wr_table = &dio8018_writeable_table,
       .rd_table = &dio8018_readable_table,
       .cache_type = REGCACHE_RBTREE,
       .volatile_table = &dio8018_volatile_table,
};

// static void dio8018_dcdc(struct dio8018 *dio8018)
// {
//        gpiod_set_value_cansleep(dio8018->reset_gpio, 0);
//        usleep_range(100000, 110000);
//        gpiod_set_value_cansleep(dio8018->dcdc_gpio, 0);
// }

// static void dio8018_reset(struct dio8018 *dio8018)
// {
//        gpiod_set_value_cansleep(dio8018->reset_gpio, 1);
//        usleep_range(100000, 110000);
//        gpiod_set_value_cansleep(dio8018->dcdc_gpio, 1);
// }
static int dio8018_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
       struct device *dev = &client->dev;
       struct regulator_config config = {};
       struct regulator_dev *rdev;
       const struct regulator_desc *regulators;
       struct dio8018 *dio8018;
       int ret, i;

       dio8018 = devm_kzalloc(dev, sizeof(struct dio8018), GFP_KERNEL);
       if (!dio8018)
              return -ENOMEM;
       i2c_set_clientdata(client, dio8018);
       dio8018->dev = dev;
       ldo_enable = dev;
       dio8018->regmap = devm_regmap_init_i2c(client, &dio8018_regmap_config);
       if (IS_ERR(dio8018->regmap)) {
              ret = PTR_ERR(dio8018->regmap);
              dev_err(dev, "Failed to allocate register map: %d\n", ret);
              return ret;
       }

       config.dev = &client->dev;
       config.regmap = dio8018->regmap;
       regulators = dio8018_reg;

       /* Instantiate the regulators */
       for (i = 0; i < DIO8018_MAX_REGULATORS; i++) {
              rdev = devm_regulator_register(&client->dev,
                                          &regulators[i], &config);
              if (IS_ERR(rdev)) {
                     dev_err(&client->dev,
                            "failed to register %d regulator\n", i);
                     return PTR_ERR(rdev);
              }
       }
       dio8018_flag = DIO8018_ENABLE;
       if(regmap_write(dio8018->regmap, DIO8018_RESET, 0xb6) != 0) {
              dev_err(&client->dev,"dio8018 write reset fail");
              dio8018_flag = 0;
       }
       regmap_write(dio8018->regmap, DIO8018_RESET, 0xb6);
       usleep_range(1000, 1100);
       regmap_write(dio8018->regmap, DIO8018_RESET, 0xb6);
       usleep_range(1000, 1100);

       return 0;
}

static void dio8018_regulator_shutdown(struct i2c_client *client)
{
       struct dio8018 *dio8018 = i2c_get_clientdata(client);
       if (system_state == SYSTEM_POWER_OFF)
              regmap_write(dio8018->regmap, DIO8018_LDO_EN, 0x80);
}

int  dio8018_enable_ldo(int value)
{
    if(dio8018_flag)
        dio8018_resume(ldo_enable , value);
    return 0;
}
EXPORT_SYMBOL_GPL(dio8018_enable_ldo);

int  dio8018_disable_ldo(int value)
{
    if(dio8018_flag)
        dio8018_suspend(ldo_enable , value);
    return 0;
}
EXPORT_SYMBOL_GPL(dio8018_disable_ldo);

void setLdoValue(int value ,int *ldo_value , struct dio8018 *dio8018)
{
    int read_ldo_value;
    regmap_read(dio8018->regmap, DIO8018_LDO_EN,&read_ldo_value);
    switch (value)
    {
    case DIO8018_LDO1:
        /* code */
        regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x01);
        dio8018->ldo_vout[DIO8018_LDO1]=ldo_value[DIO8018_LDO1];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT,
                      dio8018->ldo_vout[DIO8018_LDO1]);
        break;
    case DIO8018_LDO2:
        regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x02);
        dio8018->ldo_vout[DIO8018_LDO2]=ldo_value[DIO8018_LDO2];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO2,
                      dio8018->ldo_vout[DIO8018_LDO2]);
        break;
    case DIO8018_LDO3:
        regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x04);
        dio8018->ldo_vout[DIO8018_LDO3]=ldo_value[DIO8018_LDO3];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO3,
                      dio8018->ldo_vout[DIO8018_LDO3]);
       break;
    case DIO8018_LDO4:
        regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x08);
        dio8018->ldo_vout[DIO8018_LDO4]=ldo_value[DIO8018_LDO4];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO4,
                      dio8018->ldo_vout[DIO8018_LDO4]);
        break;
    case DIO8018_LDO6:
        regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x20);
        dio8018->ldo_vout[DIO8018_LDO6]=ldo_value[DIO8018_LDO6];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO6,
                      dio8018->ldo_vout[DIO8018_LDO6]);
        break;
    case DIO8018_LDO7:
       regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x40);
        dio8018->ldo_vout[DIO8018_LDO7]=ldo_value[DIO8018_LDO7];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO7,
                      dio8018->ldo_vout[DIO8018_LDO7]);
        break;
    default:
       regmap_write(dio8018->regmap, DIO8018_LDO_EN, read_ldo_value | 0x10);
        dio8018->ldo_vout[DIO8018_LDO5]=ldo_value[DIO8018_LDO5];
        regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + DIO8018_LDO5,
                      dio8018->ldo_vout[DIO8018_LDO5]);
        break;
    }
}

static int __maybe_unused dio8018_suspend(struct device *dev ,int value)
{
       struct i2c_client *client = to_i2c_client(dev);
       struct dio8018 *dio8018 = i2c_get_clientdata(client);
       // int i;
       int value_ldo[7] = {0};
       int ldo_value;


       setLdoValue(value , value_ldo,dio8018);
       regmap_read(dio8018->regmap, DIO8018_LDO_EN,&ldo_value);
       switch (value)
       {
           case DIO8018_LDO1:
               /* code */
               ldo_value = ldo_value & 0xFE;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           case DIO8018_LDO2:
               ldo_value = ldo_value & 0xFD;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           case DIO8018_LDO3:
               ldo_value = ldo_value & 0xFB;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           case DIO8018_LDO4:
               ldo_value = ldo_value & 0xF7;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           case DIO8018_LDO6:
               ldo_value = ldo_value & 0xDF;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           case DIO8018_LDO7:
               ldo_value = ldo_value & 0xBF;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
           default:
               ldo_value = ldo_value & 0xEF;
               regmap_write(dio8018->regmap, DIO8018_LDO_EN, ldo_value);
               break;
    }

       //regmap_read(dio8018->regmap, DIO8018_LDO_EN, &dio8018->ldo_en);
       // for (i = 0; i < ARRAY_SIZE(dio8018->ldo_vout); i++){
       //    dio8018->ldo_vout[i]=ldo_value[i];
       //    regmap_read(dio8018->regmap, DIO8018_LDO1_VOUT + i
       //                &dio8018->ldo_vout[i]);
       //   regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + i,
       //               dio8018->ldo_vout[i]);
       //    }
       //dio8018_dcdc(dio8018);
       // for (i = 0; i < ARRAY_SIZE(dio8018->ldo_vout); i++){
       //        regmap_read(dio8018->regmap, DIO8018_LDO1_VOUT + i,
       //                   &dio8018->ldo_vout[i]);
       // }
       return 0;
}

static int __maybe_unused dio8018_resume(struct device *dev,int value)
{
       struct i2c_client *client = to_i2c_client(dev);
       struct dio8018 *dio8018 = i2c_get_clientdata(client);
	int ldo_value[7] = {0x83,0x83,0xD8,0xB3,0xD8,0x36,0xB3};
	   //ldo1=main_dvdd ldo2=wide_dvdd ldo3=main_af ldo4=main_avdd ldo5=main_ois
	   //ldo6=iovdd ldo7=wide_avdd
       // int i;
       //dio8018_reset(dio8018);

       //regmap_write(dio8018->regmap, DIO8018_LDO_EN, 0xFF);
       setLdoValue(value , ldo_value,dio8018);
       // for (i = 0; i < ARRAY_SIZE(dio8018->ldo_vout); i++){
       //        //dio8018->ldo_vout[i]=ldo_value[i];
       //         //regmap_write(dio8018->regmap, DIO8018_LDO1_VOUT + i,
       //                    //dio8018->ldo_vout[i]);
       //        regmap_read(dio8018->regmap, DIO8018_LDO1_VOUT + i,
       //                    &dio8018->ldo_vout[i]);
       // }
       return 0;
}

//static SIMPLE_DEV_PM_OPS(dio8018_pm_ops, dio8018_suspend, dio8018_resume);
static const struct i2c_device_id dio8018_i2c_id[] = {
       { "dio8018", 0 },
       { }
};

MODULE_DEVICE_TABLE(i2c, dio8018_i2c_id);

static const struct of_device_id dio8018_of_match[] = {
       { .compatible = "willsemi,dio8018" },
       {}
};

MODULE_DEVICE_TABLE(of, dio8018_of_match);

static struct i2c_driver dio8018_i2c_driver = {
       .driver = {
       .name = "dio8018",
       .of_match_table = of_match_ptr(dio8018_of_match),
       //.pm = &dio8018_pm_ops,
       },
       .id_table = dio8018_i2c_id,
       .probe    = dio8018_i2c_probe,
       .shutdown = dio8018_regulator_shutdown,
};

module_i2c_driver(dio8018_i2c_driver);

MODULE_DESCRIPTION("dio8018 regulator driver");
MODULE_AUTHOR("willsemi");
MODULE_LICENSE("GPL");