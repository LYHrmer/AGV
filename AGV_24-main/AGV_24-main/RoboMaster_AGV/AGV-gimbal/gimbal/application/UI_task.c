#include "UI_task.h"
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
extern int anglesr;
extern ext_game_robot_state_t robot_state;
extern gimbal_behaviour_e gimbal_behaviour;
extern ext_bullet_remaining_t bullet_remaining_t;
extern chassis_move_t chassis_move;
extern int8_t R, QA, EA, turn_flags;
extern int key_G; // 开关弹舱盖
extern int Ready_Flag;
extern gimbal_control_t gimbal_control;
chassis_behaviour_e last_chassis_behaviour;
Graph_Data Aim[10];
String_Data strSHOOT, strCAP, strPILL, strMODE, strCOVER,strCUR_POWER,strYAW_Relative;
extern int32_t Cap_Voltage;
int32_t cur_output;
fp32 v = 0;
void UI_task(void const *pvParameters)
{
	//购买过的虚拟弹的总数，用于判断弹舱中实体弹是否有余
	int16_t sum_pill = 0, pill = 0, last_pill = 0;
	char tmp1[30] = {0}, tmp2[30] = {0}, tmp3[30] = {0}, tmp4[30] = {0}, tmp5[30] = {0},tmp6[30] = {0},tmp7[30] = {0};
	memset(&strCAP, 0, sizeof(strCAP));
	memset(&strSHOOT, 0, sizeof(strSHOOT));
	memset(&strPILL, 0, sizeof(strPILL));
	memset(&strMODE, 0, sizeof(strMODE));
//	memset(&strCOVER, 0, sizeof(strCOVER));
	memset(&strCUR_POWER, 0, sizeof(strCUR_POWER));
	memset(&strYAW_Relative, 0, sizeof(strYAW_Relative));
	for (int k = 0; k < 8; k++)
	{
		memset(&Aim[k], 0, sizeof(Aim[k]));
	} // 清空图形数据
//	Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,10,1700,600,1700+100*sin(-gimbal_control.gimbal_yaw_motor.relative_angle),600+100*cos(-gimbal_control.gimbal_yaw_motor.relative_angle));
//	Circle_Draw(&Aim[9],"CL9",UI_Graph_ADD,5,UI_Color_Orange,5,1700,600,120);
//	vTaskDelay(25);
	
	Line_Draw(&Aim[7],"AL8",UI_Graph_ADD,7,UI_Color_Purplish_red,4,955,505,975,505);
	Line_Draw(&Aim[2],"AL2",UI_Graph_ADD,2,UI_Color_Orange,2,955,520,975,520);
	Line_Draw(&Aim[3],"AL3",UI_Graph_ADD,2,UI_Color_Cyan,2,955,535,975,535);
	Line_Draw(&Aim[1],"AL2",UI_Graph_ADD,1,UI_Color_Purplish_red,2,955,650,955,450);
	
	Char_Draw(&strCAP, "CAP", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(tmp1), 2, 860, 200, tmp1);
	Char_Draw(&strSHOOT, "SHOOT", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp2), 2, 860, 300, tmp2);
	Char_Draw(&strPILL, "PILL", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp3), 2, 10, 760, tmp3);
	Char_Draw(&strMODE, "MODE", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp4), 2, 860, 250, tmp4);
//	Char_Draw(&strCOVER, "COVER", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp5), 2, 10, 710, tmp5);
	Char_Draw(&strCUR_POWER, "CUR_POWER", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp6), 2, 860, 150, tmp6);
	Char_Draw(&strYAW_Relative, "YAW_Relative", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp5), 2, 860, 100, tmp5);
	Char_ReFresh(strCAP);
	vTaskDelay(25);
	Char_ReFresh(strSHOOT);
	vTaskDelay(25);
	Char_ReFresh(strPILL);
	vTaskDelay(25);
	Char_ReFresh(strMODE);
	vTaskDelay(25);
//	Char_ReFresh(strCOVER);
//	vTaskDelay(25);
	Char_ReFresh(strCUR_POWER);
	vTaskDelay(25);
	Char_ReFresh(strYAW_Relative);
	vTaskDelay(25);
	UI_ReFresh(7, Aim[0], Aim[1], Aim[2], Aim[3], Aim[4], Aim[5], Aim[6]);
	vTaskDelay(25);
	UI_ReFresh(5, Aim[7], Aim[9], Aim[8]);

	while (1)
	{
		last_pill = pill;
		pill = bullet_remaining_t.bullet_remaining_num_17mm;

		if ((last_pill < pill) && (last_pill != pill))
		{
			sum_pill += (pill - last_pill);
		}
		
		if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
		{
			Char_Draw(&strCAP, "CAP", UI_Graph_ADD, 8, UI_Color_Green, 20, strlen(tmp1), 2, 860, 200, tmp1);
			Char_Draw(&strSHOOT, "SHOOT", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp2), 2, 860, 300, tmp2);
			Char_Draw(&strPILL, "PILL", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp3), 2, 10, 760, tmp3);
			Char_Draw(&strMODE, "MODE", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp4), 2, 860, 250, tmp4);
			Char_Draw(&strCUR_POWER, "CUR_POWER", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp6), 2, 860, 150, tmp6);
			Char_Draw(&strYAW_Relative, "YAW_Relative", UI_Graph_ADD, 9, UI_Color_Green, 20, strlen(tmp5), 2, 860, 100, tmp5);
			Char_ReFresh(strCAP);
			vTaskDelay(25);
			Char_ReFresh(strSHOOT);
			vTaskDelay(25);
			Char_ReFresh(strPILL);
			vTaskDelay(25);
			Char_ReFresh(strMODE);
			vTaskDelay(25);
			Char_ReFresh(strCOVER);
			vTaskDelay(25);
			Char_ReFresh(strCUR_POWER);
			vTaskDelay(25);
			Char_ReFresh(strYAW_Relative);
			vTaskDelay(25);
			UI_ReFresh(7, Aim[0], Aim[1], Aim[2], Aim[3], Aim[4], Aim[5], Aim[6]);
			vTaskDelay(25);
			UI_ReFresh(5, Aim[7], Aim[9], Aim[8]);
		}
		
//		Line_Draw(&Aim[7],"AL8",UI_Graph_Change,7,UI_Color_Purplish_red,10,1700,600,1700+100*sin(-gimbal_control.gimbal_yaw_motor.relative_angle),600+100*cos(-gimbal_control.gimbal_yaw_motor.relative_angle));
//		UI_ReFresh(1, Aim[7]);
//		vTaskDelay(25);
		
		// 超电电容组电压反馈
		sprintf(tmp1, "CAP:  %d V ", Cap_Voltage);
		if (Cap_Voltage > 16)
			Char_Draw(&strCAP, "CAP", UI_Graph_Change, 8, UI_Color_Green, 20, strlen(tmp1), 2, 860, 200, tmp1);
		else
			Char_Draw(&strCAP, "CAP", UI_Graph_Change, 8, UI_Color_Purplish_red, 20, strlen(tmp1), 2, 860, 200, tmp1);
		Char_ReFresh(strCAP);
		vTaskDelay(25);
		
		//当前设定底盘功率
//		sprintf(tmp6, "CUR_Power: %d W",cur_output);
//		Char_Draw(&strCUR_POWER, "CUR_Power", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp6), 2, 860, 150, tmp6);
//		Char_ReFresh(strCUR_POWER);
//		vTaskDelay(25);
		
		//底盘与yaw轴的相对角
//		sprintf(tmp5, "YAW_Relative: %d ",fabs(gimbal_control.gimbal_yaw_motor.relative_angle/3.14*180));
//		sprintf(tmp5, "QIQI");
//		Char_Draw(&strYAW_Relative, "YAW_Relative", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp5), 2, 860, 100, tmp5);
//		Char_ReFresh(strYAW_Relative);
//		vTaskDelay(25);
		
		// 摩擦轮开关反馈
		if (Ready_Flag)
		{
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			sprintf(tmp2, "SHOOT: ON");
			Char_Draw(&strSHOOT, "SHOOT", UI_Graph_Change, 9, UI_Color_Orange, 30, strlen(tmp2), 2, 860, 300, tmp2);
		}
		else
		{
//			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			sprintf(tmp2, "SHOOT:OFF");
			Char_Draw(&strSHOOT, "SHOOT", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp2), 2, 860, 300, tmp2);
		}
		Char_ReFresh(strSHOOT);
		vTaskDelay(25);
		// 底盘模式反馈
		if (chassis_move.chassis_behaviour == CHASSIS_SPIN)
		{
			sprintf(tmp4, "MODE:CHASSIS_SPIN");
			Char_Draw(&strMODE, "MODE", UI_Graph_Change, 9, UI_Color_Orange, 30, strlen(tmp4), 2, 860, 250, tmp4);
		}
		else if (chassis_move.chassis_behaviour == CHASSIS_FOLLOW_GIMBAL_YAW)
		{
			sprintf(tmp4, "MODE:CHA_F_GIM");
			Char_Draw(&strMODE, "MODE", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp4), 2, 860, 250, tmp4);
		}
		else if (chassis_move.chassis_behaviour == CHASSIS_RUDDER_FOLLOW_GIMBAL_YAW)
		{
			sprintf(tmp4, "MODE:RUD_F_GIM");
			Char_Draw(&strMODE, "MODE", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp4), 2, 860, 250, tmp4);
		}
		// 清空模式
		if (last_chassis_behaviour != chassis_move.chassis_behaviour)
		{
			sprintf(tmp4, "                  ");
			Char_Draw(&strMODE, "MODE", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp4), 2, 860, 250, tmp4);
		}
		last_chassis_behaviour = chassis_move.chassis_behaviour;
		Char_ReFresh(strMODE);
		vTaskDelay(25);

		// 弹丸剩余量反馈
		sprintf(tmp3, "SUM_PILL:%d", sum_pill);
		Char_Draw(&strPILL, "PILL", UI_Graph_Change, 9, UI_Color_Green, 20, strlen(tmp3), 2, 10, 760, tmp3);
		Char_ReFresh(strPILL);
		vTaskDelay(25);

		
		// 开关弹舱盖
//		if (key_G == 1)
//		{
//			sprintf(tmp5, "COVER: ON");
//			Char_Draw(&strCOVER, "COVER", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp5), 2, 10, 710, tmp5);
//		}
//		else
//		{
//			sprintf(tmp5, "COVER:OFF");
//			Char_Draw(&strCOVER, "COVER", UI_Graph_Change, 9, UI_Color_Green, 30, strlen(tmp5), 2, 10, 710, tmp5);
//		}
//		Char_ReFresh(strCOVER);
//		vTaskDelay(25);
	}
}
