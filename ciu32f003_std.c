/************************************************************************************************/
/**
* @file               ciu32f003_std.c
* @author             MCU Ecosystem Development Team
* @brief              STD���������������
*                     ʵ��STD��ļ�ʱ�ȹ���API��
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
* @addtogroup STD 
* @{
*
*/
/************************************************************************************************/  

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"




/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup STD_External_Functions 
* @{
*
*/
/************************************************************************************************/

/**
* @brief  Systick��ʼ��
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @retval ��
*/
__weak void std_delay_init(void)
{
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/**
* @brief  us����ʱ����������ģʽ��
* @param  count ��������
* @note   ��ʱ�������ֵ������SysTick����ֵ�Ĵ��������ֵ0xFFFFFF��16777216��
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @retval ��
*/
__weak void std_delayus(uint32_t count)
{
    count = STD_DELAY_US * count;
    count = count > 16777216 ? 16777216 : count;
    SysTick->LOAD = count - 1;
    SysTick->VAL = 0;
    while(!((SysTick->CTRL >> 16) & 0x1));
}

/**
* @brief  ms����ʱ����������ģʽ��
* @param  count ��������
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @retval ��
*/
__weak void std_delayms(uint32_t count)
{
    while(count--)
    {
        std_delayus(1000);
    }
}

/**
* @brief  us����ʱ������������ģʽ��
* @param  count ��������
* @note   ��ʱ�������ֵ������SysTick����ֵ�Ĵ��������ֵ0xFFFFFF��16777216��
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @retval ��
*/
__weak void std_delayus_start(uint32_t count)
{
    count = STD_DELAY_US * count;
    count = count > 16777216 ? 16777216 : count;
    SysTick->LOAD = count - 1;
    SysTick->VAL = 0;
}

/**
* @brief  ms����ʱ������������ģʽ��
* @param  count ��������
* @note   ��ʱ�������ֵ������SysTick����ֵ�Ĵ��������ֵ0xFFFFFF��16777216��
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @retval ��
*/
__weak void std_delayms_start(uint32_t count)
{
    std_delayus_start(1000 * count);
}


/**
* @brief  ��ȡ��������״̬��������ģʽ��
* @note   �ú���Ϊweak�������û���ѡ��������ʱ�����¶���ʵ�ָú���
* @note   �ú�����std_delayus_start��std_delayms_start�������ʹ�ã������жϼ�������״̬
* @retval bool �����߼����ʽ���жϽ��
*             @arg true�� ��ʾ�����ѽ���
*             @arg false����ʾ�������ڽ�����
*/
__weak bool std_delay_end(void)
{
    return (((SysTick->CTRL >> 16) & 0x1) == 0x1);
}



/** 
* @} 
*/


/** 
* @} 
*/

/** 
* @} 
*/
