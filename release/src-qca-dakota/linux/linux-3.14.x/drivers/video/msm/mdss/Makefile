ccflags-y += -I$(src)
ccflags-y += -I$(srctree)/arch/arm/mach-qcom/include
ccflags-y += -I$(srctree)/arch/arm/mach-qcom

mdss-qpic-objs := mdss_qpic.o mdss_fb.o mdss_qpic_panel.o
obj-$(CONFIG_FB_MSM_QPIC) += mdss-qpic.o
obj-$(CONFIG_FB_MSM_QPIC_ILI_QVGA_PANEL) += qpic_panel_ili_qvga.o
obj-$(CONFIG_FB_MSM_QPIC_ER_SSD1963_PANEL) += qpic_panel_er_ssd1963.o
