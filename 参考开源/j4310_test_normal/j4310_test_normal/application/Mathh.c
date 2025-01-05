#include "Mathh.h"


/**
 * @brief �޷�����
 *
 * @tparam Type ����
 * @param x ��������
 * @param Min ��Сֵ
 * @param Max ���ֵ
 */
float Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return *x;
}

/**
 * @brief ��ת16λ���ݵ��ֽ���
 *
 * @param Source Դ����ָ��
 * @param Destination Ŀ������ָ�룬���Ϊ NULL �򲻽��п���
 * @return uint16_t ��ת���16λ����
 */
uint16_t Math_Endian_Reverse_16(void *Source, void *Destination)
{
    uint8_t *temp_address_8 = (uint8_t *)Source;
    uint16_t temp_address_16;
    temp_address_16 = (uint16_t)(temp_address_8[0] << 8 | temp_address_8[1]);

    if (Destination != NULL)
    {
        uint8_t *temp_destination = (uint8_t *)Destination;
        temp_destination[0] = temp_address_8[1];
        temp_destination[1] = temp_address_8[0];
    }

    return temp_address_16;
}

/**
 * @brief ��������Χӳ�䵽��������Χ
 *
 * @param x ���������ֵ
 * @param Int_Min ������Χ����Сֵ
 * @param Int_Max ������Χ�����ֵ
 * @param Float_Min ��������Χ����Сֵ
 * @param Float_Max ��������Χ�����ֵ
 * @return float ӳ���ĸ�����ֵ
 */
float Math_Int_To_Float(int32_t x, int32_t Int_Min, int32_t Int_Max, float Float_Min, float Float_Max)
{
    float tmp = (float)(x - Int_Min) / (float)(Int_Max - Int_Min);
    float out = tmp * (Float_Max - Float_Min) + Float_Min;
    return out;
}

/**
 * @brief ��������ӳ�䵽����
 *
 * @param x ������
 * @param Float_Min ��������Сֵ
 * @param Float_Max ���������ֵ
 * @param Int_Min ������Сֵ
 * @param Int_Max �������ֵ
 * @return int32_t ����
 */
int32_t Math_Float_To_Int(float x, float Float_Min, float Float_Max, int32_t Int_Min, int32_t Int_Max)
{
    float tmp = (x - Float_Min) / (Float_Max - Float_Min);
    int32_t out = tmp * (float) (Int_Max - Int_Min) + Int_Min;
    return (out);
}

float Float_Math_Abs(float x)
{
	return ((x > 0) ? x : -x);
}

