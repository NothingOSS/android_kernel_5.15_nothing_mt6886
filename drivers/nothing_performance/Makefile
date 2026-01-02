ccflags-y += -I$(srctree)
ccflags-y += -I$(srctree)/$(src)

obj-$(CONFIG_NOTHING_PERFORMANCE_FEATURE) += nothing_performance.o
nothing_performance-y += nt_performance_common.o
nothing_performance-$(CONFIG_NOTHING_NAMED_THREAD_AFFINITY)    += nothing_named_thread_affinity.o
