obj-$(CONFIG_QCOM_GSBI)	+=	qcom_gsbi.o
CFLAGS_scm.o :=$(call as-instr,.arch_extension sec,-DREQUIRES_SEC=1)
obj-$(CONFIG_QCOM_SCM) += scm.o
obj-$(CONFIG_QCOM_TCSR)	+=	qcom_tcsr.o
obj-$(CONFIG_QCOM_RESTART_REASON) += restart_reason.o
obj-$(CONFIG_QCA_SCM_RESTART_REASON) += scm_restart_reason.o
obj-$(CONFIG_QCOM_CACHE_DUMP) += cache_dump.o
obj-$(CONFIG_QCOM_TZ_LOG) += tz_log.o
obj-$(CONFIG_MSM_GLINK) += glink.o glink_debugfs.o
obj-$(CONFIG_MSM_GLINK_LOOPBACK_SERVER) += glink_loopback_server.o
obj-$(CONFIG_MSM_GLINK_SPI_XPRT)	+=	glink_spi_xprt.o
obj-$(CONFIG_WCD_DSP_GLINK) += wcd-dsp-glink.o