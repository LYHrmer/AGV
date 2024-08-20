#include "bsp_laser.h"
#include "main.h"
int laster_flag=0;
int FLAG=0;
extern TIM_HandleTypeDef htim3;
void laser_on(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8399);
	if(FLAG==0)
	{
	laster_flag=1;
	FLAG=1;
	}
}
void laser_off(void)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
	if(FLAG==1)
	{
	laster_flag=0;
	FLAG=0;
	}
}
