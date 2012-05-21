#ifndef PSB_INTEL_HDMI_I2C_H
#define PSB_INTEL_HDMI_I2C_H

#if 0				/* Not needed for MDFLD & MRST HDMI */
struct mdfld_hdmi_i2c {
	struct i2c_client *ddc_client;
	struct i2c_client *hdcp_client;
	int (*open) (struct i2c_client *, void *);
	int (*close) (struct i2c_client *, void *);
	char (*read_byte_data) (struct i2c_client *, unsigned char adr);
	int (*write_byte_data) (struct i2c_client *, unsigned char adr,
				unsigned char data);
	int (*read_data) (struct i2c_adapter *adapter, unsigned char adr,
			  unsigned char *data, int size);
	int (*write_data) (struct i2c_adapter *adapter, unsigned char adr,
			   unsigned char *data, int size);
	struct i2c_adapter *(*get_adapter) (struct i2c_client *c);
	struct mutex lock;
};

extern struct mdfld_hdmi_i2c *hdmi_i2c_init(void);

#endif
#endif
