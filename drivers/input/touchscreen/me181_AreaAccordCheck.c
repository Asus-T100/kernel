#include "me181_openshort.h"

#define mul 100000
u32 accord_limit_Jtouch=22000;   //accord_limit for Jtouch
u32 accord_limit_Ofilm=22000;    //accord_limit for Ofilm
u32 accord_limit_Soe = 22000;	  //accord_limit for Soe
u32 accord_limit=22000;          //accord_limit = limit * mul
u32 AreaCheckResult=1;
u32 channel_status[SENSOR_NUM * DRIVER_NUM];
u32 beyond_accord_limit_num[SENSOR_NUM * DRIVER_NUM];
u32 beyond_accord_limit_val[SENSOR_NUM * DRIVER_NUM];
u8  special_case_limit=5;
//u32 special_case_limit_Soe[][2]={{16, 26000}, {15, 26000}, {752, 26000}, {751, 26000},{(SENSOR_NUM * DRIVER_NUM), 22000}};
u32 special_case_limit_Soe[][2]={{360, 26000}, {383, 26000}, {384, 26000}, {407, 26000},{(SENSOR_NUM * DRIVER_NUM), 22000}};
/*
*********************************************************************************************************
*********************************************************************************************************
*/
void AreaAccordCheck(u16 *CurrentDataTemp)
{
    u32 j;
    u32 temp;
    u32 accord_temp;
    u16 k;
    u8 special_case=0;

    for (j = 0; j < (SENSOR_NUM * DRIVER_NUM); j++)			
    {
	accord_temp = 0;
	temp = 0;
    special_case=0;
		
        if (CurrentDataTemp[j] == 0)		// if equal to 0, no test
        {
            continue;
        }

        if ((j%SENSOR_NUM) == 0)	// first row
        {
            accord_temp = (((abs((s16)(CurrentDataTemp[j+1] - CurrentDataTemp[j]))) * mul ) / CurrentDataTemp[j]); 
		}
        else if((j%SENSOR_NUM) == (SENSOR_NUM-1))		// last row
        {
            accord_temp = (((abs((s16)(CurrentDataTemp[j-1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
        }
        else		// others
        {
		
            accord_temp = (((abs((s16)(CurrentDataTemp[j+1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
            temp = (((abs((s16)(CurrentDataTemp[j-1] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
	
        if ((j/SENSOR_NUM) == 0)		// first column
        {
            temp = (((abs((s16)(CurrentDataTemp[j+SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
        else if ((j/SENSOR_NUM) == (DRIVER_NUM-1))		// last column
        {
            temp = (((abs((s16)(CurrentDataTemp[j-SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }
        else		// others
        {	
            temp = (((abs((s16)(CurrentDataTemp[j+SENSOR_NUM] - CurrentDataTemp[j]))) * mul) / CurrentDataTemp[j]);
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
            temp = (((abs((s16)(CurrentDataTemp[j-SENSOR_NUM] - CurrentDataTemp[j]))) * mul) /CurrentDataTemp[j]);
            if (temp > accord_temp)
            {
                accord_temp = temp;
            }
        }

		
		if (special_case_limit == 5)
        {
            k = 0;
			special_case = 0;
            //for (k=0 ; special_case_limit_Soe[k][0] < (SENSOR_NUM * DRIVER_NUM) ; k++)
			for (k=0 ; special_case_limit_Soe[k][0] < (j+1) ; k++)//Miles
            {
                if(j == special_case_limit_Soe[k][0])
                {
                    special_case = 1;
                    break;
                }
            }
            if (special_case == 1)
            {
                if (accord_temp > special_case_limit_Soe[k][1])
                {
                    channel_status[j] |= _BEYOND_ACCORD_LIMIT;
                    AreaCheckResult |= _BEYOND_ACCORD_LIMIT;
                    beyond_accord_limit_num[j]++;
                    beyond_accord_limit_val[j] = accord_temp;
					
					 GTP_INFO("  accord_temp > special_case :[%d] , node:[%d], Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], k, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
					 //SET_INFO_LINE_INFO(" \n accord_temp > special_case :[%d] , node:[%d], Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], k, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
					GTP_INFO("Val:[%d],[%d],[%d],[%d],[%d]",CurrentDataTemp[j],CurrentDataTemp[j+1],CurrentDataTemp[j-1],CurrentDataTemp[j-SENSOR_NUM],CurrentDataTemp[j+SENSOR_NUM]);
                }
            }
            else
            {
	        if (accord_temp > accord_limit)		// fail
                {
                    channel_status[j] |= _BEYOND_ACCORD_LIMIT;
                    AreaCheckResult |= _BEYOND_ACCORD_LIMIT;
                    beyond_accord_limit_num[j]++;
                    beyond_accord_limit_val[j] = accord_temp;
					GTP_INFO("   accord_temp > accord_limit : [%d]  = => Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
					//SET_INFO_LINE_INFO(" \n accord_temp > special_case :[%d] , node:[%d], Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], k, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
					//GTP_INFO("Val:[%d],[%d],[%d],[%d],[%d]",CurrentDataTemp[j],CurrentDataTemp[j+1],CurrentDataTemp[j-1],CurrentDataTemp[j-SENSOR_NUM],CurrentDataTemp[j+SENSOR_NUM]) ;
                }
            }
        }
        else
        {	
		
			if (accord_temp > accord_limit)		// fail
			{
            channel_status[j] |= _BEYOND_ACCORD_LIMIT;
            AreaCheckResult |= _BEYOND_ACCORD_LIMIT;
            beyond_accord_limit_num[j]++;
            beyond_accord_limit_val[j] = accord_temp;
			GTP_INFO("   accord_temp > accord_limit : [%d]  = => Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
			//SET_INFO_LINE_INFO(" \n accord_temp > special_case :[%d] , node:[%d], Ch: [%d], T: [%d], Val: [%d]",CurrentDataTemp[j], k, j, beyond_accord_limit_num[j], beyond_accord_limit_val[j]);
			//GTP_INFO("Val:[%d],[%d],[%d],[%d],[%d]",CurrentDataTemp[j],CurrentDataTemp[j+1],CurrentDataTemp[j-1],CurrentDataTemp[j-SENSOR_NUM],CurrentDataTemp[j+SENSOR_NUM]) ;
			}
		} // end of for (j = 0; j < SENSOR_NUM*DRIVER_NUM; j++)
	}
	
	
	
	
	
	
	
	
	return;
}