# IPC Devices
obj-y							+= platform_ipc.o
obj-y							+= platform_msic_audio.o
obj-y							+= platform_clvs_audio.o
obj-y							+= platform_msic_gpio.o
obj-y							+= platform_msic_ocd.o
obj-$(CONFIG_MSIC_GPADC)				+= platform_msic_adc.o
obj-$(CONFIG_BATTERY_INTEL_MDF)				+= platform_msic_battery.o
obj-$(CONFIG_INTEL_MID_POWER_BUTTON)			+= platform_msic_power_btn.o
obj-$(CONFIG_GPIO_INTEL_PMIC)				+= platform_pmic_gpio.o
obj-$(CONFIG_MID_PWM)					+= platform_mid_pwm.o
obj-$(CONFIG_LEDS_INTEL_KPD)				+= platform_kpd_led.o
obj-$(CONFIG_INTEL_MFLD_THERMAL)			+= platform_msic_thermal.o
# I2C Devices
obj-$(CONFIG_A1026)					+= platform_a1026.o
obj-$(CONFIG_TOUCHSCREEN_CYTTSP_I2C)			+= platform_cyttsp.o
obj-$(CONFIG_SENSORS_EMC1403)				+= platform_emc1403.o
obj-$(CONFIG_SENSORS_HMC5883)				+= platform_hmc5883.o
obj-$(CONFIG_SENSORS_LIS3DH_ACC)			+= platform_lis3dh.o
obj-$(CONFIG_SENSORS_LTR502ALS)				+= platform_ltr502als.o
obj-$(CONFIG_SENSORS_L3G4200D_POLL)			+= platform_l3g4200d.o
obj-$(CONFIG_SENSORS_LSM303_MAG)			+= platform_lsm303.o
obj-$(CONFIG_SENSORS_APDS990X)				+= platform_apds990x.o
obj-$(CONFIG_TOUCHSCREEN_MAX11871)			+= platform_max11871.o
ifdef CONFIG_BATTERY_MAX17042
obj-$(CONFIG_BATTERY_MAX17042)				+= platform_max17042.o
else
obj-$(CONFIG_BATTERY_MAX17050)				+= platform_max17042.o
endif
obj-$(CONFIG_CHARGER_BQ24192)				+= platform_bq24192.o
obj-$(CONFIG_GPIO_PCA953X)				+= platform_max7315.o
obj-$(CONFIG_SENSORS_MPU3050)				+= platform_mpu3050.o
obj-$(CONFIG_SENSORS_MS5607)				+= platform_ms5607.o
obj-$(CONFIG_TOUCHSCREEN_ATMEL_MXT224)			+= platform_mxt224.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT))	+= platform_mxt_ts.o
ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C)		+= platform_s3202.o
else
obj-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4)		+= platform_s3202.o
endif
obj-$(CONFIG_PN544_NFC)					+= platform_pn544.o
obj-$(CONFIG_GPIO_PCA953X)				+= platform_tca6416.o
obj-$(CONFIG_SENSORS_LIS3LV02D)				+= platform_lis331.o
obj-$(CONFIG_CHARGER_SMB347)				+= platform_smb347.o
# Camera
obj-$(subst m,y,$(CONFIG_VIDEO_ATOMISP))		+= platform_camera.o
ifdef CONFIG_VIDEO_ATOMISP
obj-$(subst m,y,$(CONFIG_VIDEO_MT9E013))		+= platform_mt9e013.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9M114))		+= platform_mt9m114.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV8830))			+= platform_ov8830.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3554))			+= platform_lm3554.o
endif
# SPI Devices
obj-$(CONFIG_SERIAL_MRST_MAX3110)			+= platform_max3111.o
# HSI Devices
obj-$(CONFIG_HSI_ARASAN)				+= platform_hsi_modem.o
# Bluetooth
obj-$(subst m,y,$(CONFIG_TI_ST))			+= platform_btwilink.o
# WIFI
obj-$(CONFIG_WL12XX_PLATFORM_DATA)			+= platform_wl12xx.o
# MISC Devices
obj-$(CONFIG_KEYBOARD_GPIO)				+= platform_gpio_keys.o
obj-$(CONFIG_SWITCH_MID)				+= platform_switch_mid.o
obj-$(CONFIG_I2C_GPIO)					+= platform_i2c_gpio.o
obj-$(CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE)		+= platform_tc35876x.o
obj-$(CONFIG_USB_PENWELL_OTG)				+= platform_usb_otg.o
obj-$(CONFIG_SERIAL_MFD_HSU)				+= platform_hsu.o
obj-$(CONFIG_LEDS_INTEL_KPD_GPIO)			+= platform_kpd_led_gpio.o
