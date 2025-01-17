#ifndef UI_TASK_H
#define UI_TASK_H
#include "main.h"
#include "chassis_task.h"
#include "shoot_task.h"
typedef struct
{
	chassis_move_t chassis_ui;
	Ammo booster_ui;
}ui_date;
extern void UI_task(void const *pvParameters);

#endif
