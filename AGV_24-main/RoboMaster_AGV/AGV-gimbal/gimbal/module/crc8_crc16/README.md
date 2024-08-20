# CRC校验

## 简介

用于多设备之间的数据通信，文件内包括CRC8和CRC16检验

## 函数简介

### crc计算函数

```c
/**
  * @brief          CRC8计算函数返回一个8bit的crc校验值
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @param[in]      ucCRC8:初始CRC8
  * @retval         计算完的CRC8
  */
uint8_t get_CRC8_check_sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
```

```c
/**
  * @brief          CRC16计算函数返回一个16bit的crc校验值
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @param[in]      wCRC:初始CRC16
  * @retval         计算完的CRC16
  */
extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);

```

### crc检验函数

在接收到数据后使用该函数，验证数据是否接收正确

```c
/**
  * @brief          CRC8校验函数，返回校验是否成功
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         真或者假
  */
uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

```

```c
/**
  * @brief          CRC16校验函数，返回校验是否成功
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
  * @retval         真或者假
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

### crc添加校验函数

在数据发送之前使用该函数，为发送数据添加检验值

```c
/**
  * @brief          添加CRC8到数据的结尾
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
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
  * @brief          添加CRC16到数据的结尾
  * @param[in]      pch_message: 数据
  * @param[in]      dw_length: 数据和校验的长度
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