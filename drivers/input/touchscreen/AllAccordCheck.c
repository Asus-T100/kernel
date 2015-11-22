#include "176_openshort.h"

#define DRIVER_NUM 31
#define SENSOR_NUM 17
#define mul 100000
u32 all_accord_limit_Jtouch=22000;   //all_accord_limit for Jtouch
u32 all_accord_limit_Ofilm=22000;    //all_accord_limit for Ofilm
u32 all_accord_limit=22000;          //all_accord_limit = limit * mul
u32 all_accord_limit_default=22000;
u32 AllCheckResult=0;
u16 all_channel_status[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u16 beyond_all_accord_limit_num[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u32 beyond_all_accord_limit_val[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u32 average=0;

/*
*********************************************************************************************************
*********************************************************************************************************
*/
void AllAccordCheck(u16 *CurrentDataTemp)
{
    u32 j;
    u32 all;
    u32 accord_temp;

    for (j = 0; j < SENSOR_NUM * DRIVER_NUM; j++)
    {
        all += CurrentDataTemp[j];
    }
    average = all / (SENSOR_NUM * DRIVER_NUM);

    for (j = 0; j < SENSOR_NUM * DRIVER_NUM; j++)
    {
        accord_temp = 0;

        if (CurrentDataTemp[j] == 0)        // if equal to 0, no test
        {
            continue;
        }

        accord_temp = ((abs((s16)(CurrentDataTemp[j] - average))) * mul ) / average;

        if (accord_temp > all_accord_limit)     // fail
        {
            all_channel_status[j] |= _BEYOND_ALL_ACCORD_LIMIT;
            AllCheckResult |= _BEYOND_ALL_ACCORD_LIMIT;
            beyond_all_accord_limit_num[j]++;
            beyond_all_accord_limit_val[j] = accord_temp;
        }
    } // end of for (j = 0; j < SENSOR_NUM*DRIVER_NUM; j++)
}
