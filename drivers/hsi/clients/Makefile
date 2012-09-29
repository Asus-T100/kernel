#
# Makefile for HSI clients
#

CFLAGS_hsi_ffl_tty.o := -DDEBUG
EXTRA_CFLAGS 	+= -DDEBUG

obj-$(CONFIG_SSI_PROTOCOL)	+= ssi_protocol.o
obj-$(CONFIG_HSI_CHAR)		+= hsi_char.o
obj-$(CONFIG_HSI_CMT_SPEECH)	+= cmt_speech.o
obj-$(CONFIG_HSI_FFL_TTY)	+= hsi_ffl_tty.o
obj-$(CONFIG_HSI_DLP) 		+= hsi_dlp.o
hsi_dlp-objs := dlp_flash.o dlp_main.o dlp_ctrl.o dlp_tty.o dlp_net.o dlp_trace.o
