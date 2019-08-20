EXTRA_CFLAGS += -I$(srctree)/arch/arm/mach-qcom/include
obj-y			:= board.o
obj-$(CONFIG_SMP)	+= platsmp.o
obj-$(CONFIG_QCOM_SCM)	+= scm-boot.o
obj-y			+= headsmp.o
obj-$(CONFIG_MSM_PCIE)	+= pcie.o pcie_irq.o pcie_phy.o
obj-y			+= qfprom.o
obj-$(CONFIG_MSM_RPM_LOG) += rpm_log.o
