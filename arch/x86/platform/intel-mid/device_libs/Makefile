
CFLAGS_platform_clv_regulator.o			:= -Werror

# IPC Devices
obj-y									+= platform_mrfld_audio.o
obj-y									+= platform_ipc.o
obj-y									+= platform_msic_audio.o
obj-y									+= platform_ctp_audio.o
obj-y									+= platform_msic_gpio.o
obj-y									+= platform_msic_ocd.o
obj-y									+= platform_sst_audio.o
obj-$(subst m,y,$(CONFIG_SENSORS_MID_VDD))				+= platform_msic_vdd.o
obj-$(subst m,y,$(CONFIG_MSIC_GPADC))					+= platform_msic_adc.o
obj-$(subst m,y,$(CONFIG_BATTERY_INTEL_MDF))				+= platform_msic_battery.o
obj-$(subst m,y,$(CONFIG_INTEL_MID_POWER_BUTTON))			+= platform_msic_power_btn.o
obj-$(subst m,y,$(CONFIG_GPIO_INTEL_PMIC))				+= platform_pmic_gpio.o
obj-$(subst m,y,$(CONFIG_MID_PWM))					+= platform_mid_pwm.o
obj-$(subst m,y,$(CONFIG_LEDS_INTEL_KPD))				+= platform_kpd_led.o
obj-$(subst m,y,$(CONFIG_INTEL_MFLD_THERMAL))				+= platform_msic_thermal.o
obj-$(subst m,y,$(CONFIG_INTEL_SCU_FLIS))				+= platform_scu_flis.o
# IPC Devices for Merrifield Platform
obj-$(subst m,y,$(CONFIG_BASINCOVE_GPADC))				+= platform_mrfl_adc.o
obj-$(subst m,y,$(CONFIG_SENSORS_MRFLD_CURRENT))			+= platform_mrfl_ocd.o
# I2C Devices
obj-$(subst m,y,$(CONFIG_A1026))					+= platform_a1026.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_CYTTSP_I2C))			+= platform_cyttsp.o
obj-$(subst m,y,$(CONFIG_SENSORS_EMC1403))				+= platform_emc1403.o
obj-$(subst m,y,$(CONFIG_SENSORS_HMC5883))				+= platform_hmc5883.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3DH_ACC))				+= platform_lis3dh.o
obj-$(subst m,y,$(CONFIG_SENSORS_LTR502ALS))				+= platform_ltr502als.o
obj-$(subst m,y,$(CONFIG_INPUT_LTR301ALS))				+= platform_ltr301als.o
obj-$(subst m,y,$(CONFIG_SENSORS_L3G4200D_POLL))			+= platform_l3g4200d.o
obj-$(subst m,y,$(CONFIG_SENSORS_LSM303_MAG))				+= platform_lsm303.o
obj-$(subst m,y,$(CONFIG_SENSORS_APDS990X))				+= platform_apds990x.o
obj-$(subst m,y,$(CONFIG_SENSORS_APDS9300))				+= platform_apds9300.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_MAX11871))				+= platform_max11871.o
ifdef CONFIG_BATTERY_MAX17042
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17042))				+= platform_max17042.o
else
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17050))				+= platform_max17042.o
endif
obj-$(subst m,y,$(CONFIG_CHARGER_BQ24192))				+= platform_bq24192.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X))					+= platform_max7315.o
obj-$(subst m,y,$(CONFIG_SENSORS_MPU3050))				+= platform_mpu3050.o
obj-$(subst m,y,$(CONFIG_SENSORS_MS5607))				+= platform_ms5607.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT224))			+= platform_mxt224.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_FOCALTECH_FT5406))			+= platform_ft5406.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT))			+= platform_mxt_ts.o
ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_RMI4_I2C))		+= platform_s3202.o
else
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4))		+= platform_s3202.o
endif
obj-$(subst m,y,$(CONFIG_HID_NTRIG))					+= platform_ntrig.o
obj-$(subst m,y,$(CONFIG_HID_NTRIG_G4))                                 += platform_ntrig_g4.o
obj-$(subst m,y,$(CONFIG_PN544_NFC))					+= platform_pn544.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X))					+= platform_tca6416.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3LV02D))				+= platform_lis331.o
obj-$(subst m,y,$(CONFIG_CHARGER_SMB347))				+= platform_smb347.o
# Camera
obj-$(subst m,y,$(CONFIG_VIDEO_ATOMISP))				+= platform_camera.o
ifdef CONFIG_VIDEO_ATOMISP
obj-$(subst m,y,$(CONFIG_VIDEO_MT9E013))				+= platform_mt9e013.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9D113))				+= platform_mt9d113.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9M114))				+= platform_mt9m114.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9V113))				+= platform_mt9v113.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV8830))					+= platform_ov8830.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX175))					+= platform_imx175.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX135))					+= platform_imx135.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV9724))					+= platform_ov9724.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3554))					+= platform_lm3554.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3559))					+= platform_lm3559.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV5640))					+= platform_ov5640.o
endif
# SPI Devices
obj-$(subst m,y,$(CONFIG_SERIAL_MRST_MAX3110))				+= platform_max3111.o
# HSI Devices
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_hsi_modem.o
# Bluetooth
obj-$(subst m,y,$(CONFIG_TI_ST))					+= platform_btwilink.o
obj-$(subst m,y,$(CONFIG_BCM_BT_LPM))					+= platform_btlpm.o
# MMC Sdhci pci host controller platform data
obj-$(subst m,y,$(CONFIG_MMC_SDHCI_PCI))				+= platform_mmc_sdhci_pci.o
# WIFI
obj-$(subst m,y,$(CONFIG_WL12XX_PLATFORM_DATA))				+= platform_wl12xx.o
obj-$(subst m,y,$(CONFIG_BCM43XX_PLATFORM_DATA))			+= platform_bcm43xx.o
# MISC Devices
obj-$(subst m,y,$(CONFIG_KEYBOARD_GPIO))				+= platform_gpio_keys.o
obj-$(subst m,y,$(CONFIG_SWITCH_MID))					+= platform_switch_mid.o
obj-$(subst m,y,$(CONFIG_I2C_GPIO))					+= platform_i2c_gpio.o
obj-$(subst m,y,$(CONFIG_SUPPORT_TOSHIBA_MIPI_LVDS_BRIDGE))		+= platform_tc35876x.o
obj-$(subst m,y,$(CONFIG_USB_PENWELL_OTG))				+= platform_usb_otg.o
obj-$(subst m,y,$(CONFIG_SERIAL_MFD_HSU))				+= platform_hsu.o
obj-$(subst m,y,$(CONFIG_LEDS_INTEL_KPD_GPIO))				+= platform_kpd_led_gpio.o
obj-y									+= platform_clv_regulator.o
obj-$(subst m,y,$(CONFIG_TCG_TPM))					+= platform_tpm502i.o
obj-$(subst m,y,$(CONFIG_BQ24261_CHARGER))                            += platform_bq24261.o
obj-$(subst m,y,$(CONFIG_PMIC_CCSM))                                    += platform_mrfl_pmic.o
obj-$(subst m,y,$(CONFIG_DRM_MRFLD))                  += platform_hdmi.o
obj-$(subst m,y,$(CONFIG_SENSOR_MID_THERMAL_PROBE))			+= platform_mid_thermal_probe.o
