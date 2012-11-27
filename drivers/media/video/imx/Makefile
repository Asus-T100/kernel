obj-$(CONFIG_VIDEO_IMX) += imx1x5.o

# Select the correct sensor objects.
imx1x5-objs-$(CONFIG_VIDEO_IMX175)	+= drv201.o
imx1x5-objs-$(CONFIG_VIDEO_IMX135)	+= ad5816g.o

imx1x5-objs	:= imx.o $(imx1x5-objs-y)

ccflags-y += -Werror
