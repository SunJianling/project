/************************************************************************************************/
/**
* @file               ciu32f003_std_rcc.c
* @author             MCU Ecosystem Development Team
* @brief              RCC STD��������
*                     ʵ��RCCʱ��Ƶ�ʻ�ȡ��API��
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
* @addtogroup RCC 
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup RCC_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  ��ȡ��ǰϵͳʱ��Ƶ�ʣ�SYSCLK��
* @retval ����ϵͳʱ��Ƶ�ʣ�Hz��
*/
uint32_t std_rcc_get_sysclkfreq(void)
{
    uint32_t frequency = 0;
    
    /* ��ȡ��ǰϵͳʱ��Դ */
    switch(std_rcc_get_sysclk_source())
    {
        /* ϵͳʱ��ΪEXTCLK��Ĭ��Ϊ8MHz */
        case RCC_SYSCLK_SRC_STATUS_EXTCLK:
        {
            frequency = EXTCLK_VALUE;
        }break;
        
        /* ϵͳʱ��ΪRCHDIV3 */
        case RCC_SYSCLK_SRC_STATUS_RCHDIV3:
        {
            frequency = RCH_VALUE/3;
        }break;
        
        /* ϵͳʱ��ΪRCH */
        case RCC_SYSCLK_SRC_STATUS_RCH:
        {
            frequency = RCH_VALUE;
        }break;
        
        /* ϵͳʱ��ΪRCL */
        case RCC_SYSCLK_SRC_STATUS_RCL:
        {
            frequency = RCL_VALUE;
        }break;       
        
        /* ϵͳʱ��ΪRCHDIV6 */
        case RCC_SYSCLK_SRC_STATUS_RCHDIV6:
        default:
        {
            frequency = RCH_VALUE/6;
        }break;
    }
    return frequency;
}


/**
* @brief  ��ȡAHBʱ��Ƶ�ʣ�HCLK��
* @retval ����HCLKʱ��Ƶ�ʣ�Hz��
*/
uint32_t std_rcc_get_hclkfreq(void)
{
    uint32_t frequency = 0;
    uint32_t tmp, hclk_div;
    
    tmp = std_rcc_get_sysclkfreq();
    
    /* ��ȡAHB��Ƶ���� */
    hclk_div = std_rcc_get_ahbdiv()>>RCC_CFG_HPRE_POS;
    frequency = tmp >> hclk_div;    

    return frequency;
}

/**
* @brief  ��ȡAPBʱ��Ƶ�ʣ�PCLK��
* @retval ����PCLK1ʱ��Ƶ�ʣ�Hz��
*/
uint32_t std_rcc_get_pclkfreq(void)
{
    uint32_t frequency = 0;
    uint32_t tmp, pclk_div;
    
    tmp = std_rcc_get_hclkfreq();
    
    /* ��ȡAPB��Ƶ���� */
    pclk_div = std_rcc_get_apbdiv()>>RCC_CFG_PPRE_POS;
    
    if (pclk_div < 3)
    {
        frequency = tmp;
    }
    else
    {
        pclk_div -= 3;
        frequency = tmp >> pclk_div;
    }
    
    return frequency;
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
