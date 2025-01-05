

#ifndef MATHH_H
#define MATHH_H

#include "main.h"

// rpm���㵽rad/s
#define RPM_TO_RADPS (2.0f * PI / 60.0f)
// deg���㵽rad
#define DEG_TO_RAD (PI / 180.0f)
// ���϶Ȼ��㵽���϶�
#define CELSIUS_TO_KELVIN (273.15f)
// Բ����PI
#define PI (3.14159265358979323846f)


float Math_Constrain(float *x, float Min, float Max);
uint16_t Math_Endian_Reverse_16(void *Source, void *Destination);
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max);
int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max);
float Float_Math_Abs(float x);


#endif
