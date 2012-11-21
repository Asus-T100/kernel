/*
 * Copyright (c) 2010, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors:
 *	jim liu <jim.liu@intel.com>
 */


#ifndef MDFLD_HDMI_AUDIO_IF_H
#define MDFLD_HDMI_AUDIO_IF_H

/* HDMI AUDIO INTERRUPT TYPE */
#define HDMI_AUDIO_UNDERRUN	(1UL<<0)
#define HDMI_AUDIO_BUFFER_DONE	(1UL<<1)

enum had_caps_list {
	HAD_GET_ELD = 1,
	HAD_GET_SAMPLING_FREQ,
	HAD_GET_DISPLAY_RATE,
	HAD_GET_HDCP_STATUS,
	HAD_GET_AUDIO_STATUS,
	HAD_SET_ENABLE_AUDIO,
	HAD_SET_DISABLE_AUDIO,
	HAD_SET_ENABLE_AUDIO_INT,
	HAD_SET_DISABLE_AUDIO_INT,
	OTHERS_TBD,
};

enum had_event_type {
	HAD_EVENT_HOT_PLUG = 1,
	HAD_EVENT_HOT_UNPLUG,
	HAD_EVENT_MODE_CHANGING,
	HAD_EVENT_PM_CHANGING,
	HAD_EVENT_AUDIO_BUFFER_DONE,
	HAD_EVENT_AUDIO_BUFFER_UNDERRUN,
	HAD_EVENT_QUERY_IS_AUDIO_BUSY,
	HAD_EVENT_QUERY_IS_AUDIO_SUSPENDED,
};

/**
 * HDMI Display Controller Audio Interface 
 * 
 */
typedef int (*had_event_call_back)(enum had_event_type event_type, void * ctxt_info);

struct  hdmi_audio_registers_ops {
	int (*hdmi_audio_read_register)(uint32_t reg_addr, uint32_t *data);
	int (*hdmi_audio_write_register) (uint32_t reg_addr, uint32_t data);
	int (*hdmi_audio_read_modify)(uint32_t reg_addr, uint32_t data, uint32_t mask);
};

struct hdmi_audio_query_set_ops {
	int (*hdmi_audio_get_caps)(enum had_caps_list query_element , void *capabilties);
	int (*hdmi_audio_set_caps)(enum had_caps_list set_element , void *capabilties);
};

typedef struct hdmi_audio_event {
	int type;
} hdmi_audio_event_t;

struct snd_intel_had_interface {
	const char *name;
	int (*query) (void *had_data, hdmi_audio_event_t event);
	int (*suspend) (void *had_data, hdmi_audio_event_t event);
	int (*resume) (void *had_data);
};

extern int mid_hdmi_audio_setup(
	had_event_call_back audio_callbacks,
	struct hdmi_audio_registers_ops *reg_ops,
	struct hdmi_audio_query_set_ops *query_ops);
extern int mid_hdmi_audio_register(struct snd_intel_had_interface *driver, void *had_data);
extern bool mid_hdmi_audio_is_busy(struct drm_device *dev);
extern bool mid_hdmi_audio_suspend(struct drm_device *dev);
extern void mid_hdmi_audio_resume(struct drm_device *dev);
extern void mid_hdmi_audio_signal_event(struct drm_device *dev, enum had_event_type event);


#endif /* MDFLD_HDMI_AUDIO_IF_H */
