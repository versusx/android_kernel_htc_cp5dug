ifeq ($(CONFIG_ARCH_SC8810),y)
sprd_scale-objs := sin_cos.o gen_scale_coef.o scale_sc8810.o
obj-y += sprd_scale.o
endif

ifeq ($(CONFIG_ARCH_SC8825),y)
sprd_scale-objs := sc8825/sin_cos.o sc8825/gen_scale_coef.o sc8825/scale_drv.o sc8825/img_scale.o
obj-y += sprd_scale.o
endif

ifeq ($(CONFIG_ARCH_SCX35),y)
sprd_scale-objs := sc8830/sin_cos.o sc8830/gen_scale_coef.o sc8830/scale_drv.o sc8830/img_scale.o
obj-y += sprd_scale.o
endif

