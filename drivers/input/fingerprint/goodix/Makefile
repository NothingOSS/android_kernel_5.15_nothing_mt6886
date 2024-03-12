#
# Makefile for the fingerprint device.
#

# Each configuration option enables a list of files.
#include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)

include $(srctree)/drivers/tee/teei/Makefile.include

#obj-m	+= gf_tee.o
#gf_tee-y := gf_spi_tee.o gf_spi_factory.o
#gf_tee-y := gf_spi_tee.o

obj-m += goodix_fp.o
goodix_fp-y := gf_spi_tee.o
