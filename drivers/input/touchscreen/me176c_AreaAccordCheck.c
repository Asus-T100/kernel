#include "me176c_openshort.h"

#define DRIVER_NUM 31
#define SENSOR_NUM 17
#define mul 100000
u32 accord_limit_Jtouch=22000;   //accord_limit for Jtouch
u32 accord_limit_Ofilm=22000;    //accord_limit for Ofilm
u32 accord_limit=22000;          //accord_limit = limit * mul
u32 AreaCheckResult=0;
u16 channel_status[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u16 beyond_accord_limit_num[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u32 beyond_accord_limit_val[MAX_SENSOR_NUM * MAX_DRIVER_NUM]={0};
u8  special_case_limit=0;
u32 special_case_limit_Innolux[][2]={{7, 50000}, {8, 50000}, {24, 40000}, {25, 40000}, {41, 40000}, {42, 40000}, {58, 40000}, {59, 40000}, {75, 40000}, {76, 40000}, 
                                   {92, 40000}, {93, 40000}, {109, 40000}, {110, 40000}, {126, 40000}, {127, 40000}, {143, 40000}, {144, 40000}, {330, 40000}, {331, 40000},
                                   {347, 40000}, {348, 40000}, {364, 40000}, {365, 40000}, {381, 40000}, {382, 40000}, {398, 40000}, {399, 40000}, {415, 40000}, {416, 40000},
                                   {432, 40000}, {433, 40000}, {449, 40000}, {450, 40000}, {466, 40000}, {467, 40000}, {483, 40000}, {484, 40000}, {500, 40000}, {501, 40000},
                                   {(SENSOR_NUM * DRIVER_NUM), 0}};

/*
*********************************************************************************************************
*********************************************************************************************************
*/
void AreaAccordCheck(u16 *CurrentDataTemp)
{
    u32 j;
    u32 temp;
    u32 accord_temp;
    u8 k;
    u8 special_case=0;

    for (j = 0; j < SENSOR_NUM * DRIVER_NUM; j++)			
    {
	accord_temp = 0;
	temp = 0;
        //special_case=0;
		
        if (CurrentDataTemp[j] == 0)		// if equal to 0, no test
        {
            continue;
        }

        if (j%SENSOR_NUM == 0)	// first row
        {
            accord_temp = ((abs((s16)(CurrentDataTemp[j+1] - CurrentDataTemp[j]))) * mul ) / CurrentDataTemp[j]; 
	}
        else if((j%SENSOR_NUM) == (SENSOR_NUM-1))		// last row
        {
            accord_temp = ((abs((s16)(CurrentDataTemp[j-1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
        }
        else		// others
        {
		
            accord_temp = ((abs((s16)(CurrentDataTemp[j+1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
            temp = ((abs((s16)(CurrentDataTemp[j-1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }

		
        if (j/SENSOR_NUM == 0)		// first column
        {
            temp = ((abs((s16)(CurrentDataTemp[j+SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
        else if ((j/SENSOR_NUM) == (DRIVER_NUM-1))		// last column
        {
            temp = ((abs((s16)(CurrentDataTemp[j-SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
        else		// others
        {
		
            temp = ((abs((s16)(CurrentDataTemp[j+SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j];
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
            temp = ((abs((s16)(CurrentDataTemp[j-SENSOR_NUM] - CurrentDataTemp[j]))) * mul) /CurrentDataTemp[j];
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
        if (accord_temp > accord_limit)		// fail
        {
            channel_status[j] |= _BEYOND_ACCORD_LIMIT;
            AreaCheckResult |= _BEYOND_ACCORD_LIMIT;
            beyond_accord_limit_num[j]++;
            beyond_accord_limit_val[j] = accord_temp;
        }
    } // end of for (j = 0; j < SENSOR_NUM*DRIVER_NUM; j++)
}
