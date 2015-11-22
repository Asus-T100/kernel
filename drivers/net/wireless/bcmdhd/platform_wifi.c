#include <linux/fs.h>
#include <linux/random.h>

#define WIFI_FAC_MAC_ADDR_FILE	"/factory/wifi/mac.txt"
#define WIFI_CON_MAC_ADDR_FILE	"/config/wifi/mac.txt"
#define EMMC_ID			"/proc/emmc0_id_entry"

int check_mac(char *str)
{
	int i;

	if (strlen(str) != 12) {
		pr_err("%s: bad mac address file len %zu < 12\n",
				__func__, strlen(str));
		return -1;
	}
	for (i = 0; i < strlen(str); i++) {
		if (!strchr("1234567890abcdefABCDEF", str[i])) {
			pr_err("%s: illegal wifi mac\n", __func__);
			return -1;
		}
	}
	return 0;
}

void string_to_mac(char *str, unsigned char *buf)
{
	char temp[3]="\0";
	int mac[6];
	int i;

	for (i = 0; i < 6; i++) {
		strncpy(temp, str+(i*2), 2);
		sscanf(temp, "%x", &mac[i]);
	}
	pr_info("%s: using wifi mac %02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	buf[0] = (unsigned char) mac[0];
	buf[1] = (unsigned char) mac[1];
	buf[2] = (unsigned char) mac[2];
	buf[3] = (unsigned char) mac[3];
	buf[4] = (unsigned char) mac[4];
	buf[5] = (unsigned char) mac[5];
}

#define ETHER_ADDR_LEN 6

int platform_wifi_get_mac_addr(unsigned char *buf)
{
	struct file *fp;
	char str[32];
	//char default_mac[12] = "00904C";
	uint rand_mac;
	//uint rand_mac;

	pr_debug("%s\n", __func__);

	memset(buf, 0x00, ETHER_ADDR_LEN);

	/* open wifi mac address file */
	fp = filp_open(WIFI_CON_MAC_ADDR_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: cannot open %s\n", __func__, WIFI_CON_MAC_ADDR_FILE);
		fp = filp_open(WIFI_FAC_MAC_ADDR_FILE, O_RDONLY, 0);
		if (IS_ERR(fp)) {
			pr_err("%s: cannot open %s\n", __func__, WIFI_FAC_MAC_ADDR_FILE);
			goto random_mac;
		}
	}

	/* read wifi mac address file */
	memset(str, 0, sizeof(str));
	kernel_read(fp, fp->f_pos, str, 12);

	if (check_mac(str)) {
		filp_close(fp, NULL);
		goto random_mac;
	}
	string_to_mac(str, buf);
	filp_close(fp, NULL);
	return 0;

random_mac:
	/* random create wifi mac address */

	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	/* open wifi mac address file */
	/*
	fp = filp_open(EMMC_ID, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: cannot open %s\n", __func__, EMMC_ID);
		return -1;
	}
	*/
	/* read wifi mac address file */
	/*
	memset(str, 0, sizeof(str));
	kernel_read(fp, fp->f_pos, str, 32);
	strcat(default_mac, str+strlen(str)-6);

	if (check_mac(default_mac)) {
		filp_close(fp, NULL);
		return -1;
	}
	string_to_mac(default_mac, buf);
	filp_close(fp, NULL);
	*/
	return 0;
}

#define DEFAULT_CCODE "/factory/country"
#define COUNTRY_BUF_SZ 4

const char *get_nvram_path(void)
{
	const char *nvram_path_CE = "/etc/nvram_CE.txt";
	const char *nvram_path_FCC = "/etc/nvram_FCC.txt";
	const char *nvram_path = "/etc/nvram.txt";
	struct file *fp;
	char ccode[COUNTRY_BUF_SZ];
	memset(ccode, 0, COUNTRY_BUF_SZ);

	fp = filp_open(DEFAULT_CCODE, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		kernel_read(fp, fp->f_pos, ccode, 2);
		ccode[2] = '\0';
		filp_close(fp, NULL);
	}

	if (strcmp(ccode, "US") == 0)
		return nvram_path_FCC;
	else if (strcmp(ccode, "WW") == 0)
		return nvram_path_CE;

	return nvram_path;
}
