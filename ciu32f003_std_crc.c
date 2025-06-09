/************************************************************************************************/
/**
* @file               ciu32f003_std_crc.c
* @author             MCU Ecosystem Development Team
* @brief              CRC STD��������
*                     ʵ��CRC��ʼ�����õȹ���API��
*                           
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @addtogroup CRC
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_CRC_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup CRC_External_Functions 
* @{
*
*/
/************************************************************************************************/
/**
* @brief  CRCȥ��ʼ��
* @retval ��
*/
void std_crc_deinit(void)
{
    /* ��λCRC */
    std_rcc_ahb_reset(RCC_PERIPH_RESET_CRC); 
}


/**
* @brief  ��CRC��ʼֵ��ת��д��Ĵ���
* @param  poly_sel   ����ʽѡ��
* @param  init_value �Զ����ʼֵ
* @retval ��
*/
void std_crc_set_init_value_invert(uint32_t poly_sel, uint32_t init_value)
{
    uint32_t index, temp; 
    uint32_t result = 0;
    
    /* ��ʼֵ��λ��ת */
    for (index = 0U; index < 4; index++)
    {
        temp = (init_value >> (8U * index));
        temp = (((temp & 0x55) << 1) | ((temp & 0xaa) >> 1));
        temp = (((temp & 0x33) << 2) | ((temp & 0xcc) >> 2));
        temp = (((temp & 0x0f) << 4) | ((temp & 0xf0) >> 4));
        result |= (temp << (8U * (3 - index)));
    }

    if (poly_sel == CRC_POLY_16) 
    {
        result >>= 16;
    }
    CRC->RDR = result;
}

/**
* @}
*/

#endif /* STD_CRC_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/
