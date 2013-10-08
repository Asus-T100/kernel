#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>



#define HYDIS    0
#define INX   1

static int panel_id_read(char *page, char **start, off_t off,
          int count, int *eof, void *data)
{
     int len=1;
     int gpio_value=-1;
     int gpio_lcd_id0,gpio_lcd_id1,gpio_lcd_id2;

	 gpio_lcd_id0=acpi_get_gpio("\\_SB.GPO2",29);
	 gpio_lcd_id1=acpi_get_gpio("\\_SB.GPO2",30);
	 gpio_lcd_id2=acpi_get_gpio("\\_SB.GPO2",27);

     //By retrun different Panel_ID value to detect which panel used now.
     //0 = Hydis, 1=INX
	 if(gpio_get_value(gpio_lcd_id0) == 0 && gpio_get_value(gpio_lcd_id1) == 0 && gpio_get_value(gpio_lcd_id2) == 0){ //Hydis   ID_0=L  ID_1=L ID_2=L
	    //pr_info("#########Panel Hydis#######\n");
		len = strlen ("0");
		strcpy(page,"0");
	 	}
	 else if(gpio_get_value(gpio_lcd_id0) == 1 && gpio_get_value(gpio_lcd_id1) == 1 && gpio_get_value(gpio_lcd_id2) == 0){ //Inx ID_0=L  ID_1=L ID_2=L
	    //pr_info("#########Panel INX#######\n");
		len = strlen ("1");
		strcpy(page,"1");
	 	}
	 else{// Unknown Panel
	    //pr_info("#########Unknown Panel#######\n");
		len = strlen ("-1");
		strcpy(page,"-1");
	 	}

	 
	if (len <= off+count)
		*eof = 1;
	
	*start = page + off;
	len -= off;
	
	if (len > count)
		len = count;
	
	if (len < 0)
		len = 0;
	

     
     return len;
}

static int __init asus_panel_id_init(void)
{
	int gpio_num;
	int ret=0;
	struct proc_dir_entry *res=NULL;

    pr_info("panel_id_init\n");
  

	res =  create_proc_read_entry ("Panel_ID", 0, NULL, panel_id_read, NULL);
	if (!res) {
		pr_err("failed to create /proc/Panel_ID entry\n");
		return 0;
	}

	res->read_proc = panel_id_read;
        
        
	return 0;
}

static void __exit asus_panel_id_exit(void)
{
    remove_proc_entry("Panel_ID", NULL);
	pr_info("asus_dualboot_exit\n");
	return;
}

module_init(asus_panel_id_init);
module_exit(asus_panel_id_exit);

