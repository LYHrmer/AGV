#include "ui_update.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "CAN_receive.h"
#include "stdio.h"
#include "remote_control.h"
#include "referee.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "chassis_task.h"
extern ext_game_robot_state_t robot_state;
extern ext_bullet_remaining_t bullet_remaining_t;
extern gimbal_behaviour_e gimbal_behaviour;
extern gimbal_control_t gimbal_control;
extern ext_power_heat_data_t power_heat_data_t;
fp32 gimbal_cap;
Graph_Data Aim[7];
Graph_Data Aim_1[7];
String_Data strCAP;
String_Data strPOWER;
void UI_tasks(void const *pvParameters)
{
			vTaskDelay(200);
			char tmp1[30]={0},tmp2[30]={0},tmp3[30];
			memset(&strPOWER,0,sizeof(strPOWER));
			memset(&strCAP,0,sizeof(strCAP));
			for(int k=0;k<6;k++)
			{
				memset(&Aim[k],0,sizeof(Aim[k]));
			} 	
			for(int k=0;k<6;k++)
			{
				memset(&Aim_1[k],0,sizeof(Aim_1[k]));
			} 			
			//清空图形数据
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,940,200,940,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,3,920,485,960,485);
			Line_Draw(&Aim[2],"003",UI_Graph_ADD,5,UI_Color_Black,3,910,456,970,456);
			Line_Draw(&Aim[3],"004",UI_Graph_ADD,5,UI_Color_Yellow,2,900,425,980,425);
			//可通过宽度
			Line_Draw(&Aim[4],"005",UI_Graph_ADD,5,UI_Color_Green,2,505,0,696,232);
			Line_Draw(&Aim[5],"006",UI_Graph_ADD,5,UI_Color_Green,2,1450,0,1314,134);
			
			Line_Draw(&Aim[6],"007",UI_Graph_ADD,5,UI_Color_Orange,2,905,380,985,380);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);			
//		  Char_Draw(&strPOWER,"POWER",UI_Graph_ADD,8,UI_Color_Green,100,strlen(tmp1),6,600,100,tmp1);
//			Char_ReFresh(strPOWER);.vTaskDelay(2
			vTaskDelay(50);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_ReFresh(strCAP);
			vTaskDelay(50);
	while(1)
	{
		if(gimbal_control.gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_G)
		{
			Line_Draw(&Aim[0],"AL1",UI_Graph_ADD,5,UI_Color_Green,3,940,200,940,540);
			Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,5,UI_Color_Green,3,920,485,960,485);
			Line_Draw(&Aim[2],"003",UI_Graph_ADD,5,UI_Color_Black,3,910,456,970,456);
			Line_Draw(&Aim[3],"004",UI_Graph_ADD,5,UI_Color_Yellow,2,900,425,980,425);
			//可通过宽度
			Line_Draw(&Aim[4],"005",UI_Graph_ADD,5,UI_Color_Green,2,505,0,696,232);
			Line_Draw(&Aim[5],"006",UI_Graph_ADD,5,UI_Color_Green,2,1450,0,1314,134);
			
			Line_Draw(&Aim[6],"007",UI_Graph_ADD,5,UI_Color_Orange,2,905,380,985,380);
			UI_ReFresh(7,Aim[0],Aim[1],Aim[2],Aim[3],Aim[4],Aim[5],Aim[6]);
//			Char_Draw(&strPOWER,"POWER",UI_Graph_ADD,8,UI_Color_Green,100,strlen(tmp1),6,600,100,tmp1);
//			Char_ReFresh(strPOWER);
			vTaskDelay(50);
			Char_Draw(&strCAP,"CAP",UI_Graph_ADD,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
			Char_ReFresh(strCAP);
			vTaskDelay(50);
		}
		vTaskDelay(50);
		gimbal_cap=gimbal_control.cap_capvot;
		vTaskDelay(50);
		sprintf(tmp1,"Cap:%.1f(%.2fV)",((gimbal_cap-13.5f)/10.5f*100.f),gimbal_cap);
		vTaskDelay(50);
		if(gimbal_cap>18) //电容电压
		{
		Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Green,20,strlen(tmp1),2,860,100,tmp1);
		}
		else
		{
		Char_Draw(&strCAP,"CAP",UI_Graph_Change,8,UI_Color_Orange,20,strlen(tmp1),2,860,100,tmp1);
		}	
		Char_ReFresh(strCAP);	
//		if(power_heat_data_t.chassis_power_buffer< 8.0f) //功率预警
//		{
//				Char_Draw(&strPOWER,"POWER",UI_Graph_Change,8,UI_Color_Orange,100,strlen(tmp1),10,400,500,tmp1);
//				sprintf(tmp1,"WARNING");
//		}
//		else
//		{ 
//				Char_Draw(&strPOWER,"POWER",UI_Graph_Change,8,UI_Color_Green,50,strlen(tmp1),5,660,100,tmp1);
//				sprintf(tmp1,"POWER NORMAL");
//		}
		
//		Char_ReFresh(strPOWER);
		vTaskDelay(50);
		vTaskDelay(100);
	}
}
