# CRCУ��

## ���

���ڶ��豸֮�������ͨ�ţ��ļ��ڰ���CRC8��CRC16����

## �������

### crc���㺯��

```c
/**
  * @brief          CRC8���㺯������һ��8bit��crcУ��ֵ
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      ucCRC8:��ʼCRC8
  * @retval         �������CRC8
  */
uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
```

```c
/**
  * @brief          CRC16���㺯������һ��16bit��crcУ��ֵ
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @param[in]      wCRC:��ʼCRC16
  * @retval         �������CRC16
  */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

```

### crc���麯��

�ڽ��յ����ݺ�ʹ�øú�������֤�����Ƿ������ȷ

```c
/**
  * @brief          CRC8У�麯��������У���Ƿ�ɹ�
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

```

```c
/**
  * @brief          CRC16У�麯��������У���Ƿ�ɹ�
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         ����߼�
  */
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength)
{
    uint16_t wExpected = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return 0;
    }
    wExpected = get_CRC16_check_sum(pchMessage, dwLength - 2, CRC16_INIT);
    return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
```

### crc���У�麯��

�����ݷ���֮ǰʹ�øú�����Ϊ����������Ӽ���ֵ

```c
/**
  * @brief          ���CRC8�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void append_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length)
{
    unsigned char ucCRC = 0;
    if ((pch_message == 0) || (dw_length <= 2))
    {
        return;
    }
    ucCRC = get_CRC8_check_sum((unsigned char *)pch_message, dw_length - 1, CRC8_INIT);
    pch_message[dw_length - 1] = ucCRC;
} 
```

```c
/**
  * @brief          ���CRC16�����ݵĽ�β
  * @param[in]      pch_message: ����
  * @param[in]      dw_length: ���ݺ�У��ĳ���
  * @retval         none
  */
void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength)
{
    uint16_t wCRC = 0;
    if ((pchMessage == NULL) || (dwLength <= 2))
    {
        return;
    }
    wCRC = get_CRC16_check_sum ( (uint8_t *)pchMessage, dwLength-2, CRC16_INIT );
    pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
    pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}
```