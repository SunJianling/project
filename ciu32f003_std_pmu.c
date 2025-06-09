/************************************************************************************************/
/**
* @file               ciu32f003_std_pmu.c
* @author             MCU Ecosystem Development Team
* @brief              PMU STD��������
*                     ʵ�ֵ͹���ģʽ���빦��API��
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
* @addtogroup PMU 
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"


#ifdef STD_PMU_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/
/************************************************************************************************/
/**
* @addtogroup PMU_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  ����sleepģʽ
* @param  mode_entry ����͹���ģʽ�ķ�ʽ
*             @arg PMU_ENTRY_LOWPOWER_MODE_WFE
*             @arg PMU_ENTRY_LOWPOWER_MODE_WFI
* @retval ��
*/
void std_pmu_enter_sleep(uint32_t mode_entry)
{    
    /* ��� SLEEPDEEP ��־ */
    SCB->SCR &= (~SCB_SCR_SLEEPDEEP_Msk); 
    
    /* ����͹���ģʽ�Ľ��뷽ʽ*/
    if(PMU_ENTRY_LOWPOWER_MODE_WFI == mode_entry)
    {
        __WFI();
    }
    else
    {
        /* ������һ���¼���ͨ��WFE����¼������µ���WFE����͹���ģʽ*/
        __SEV();
        __WFE();
        __WFE();
    }
}

/**
* @brief  ����stop/Deepstopģʽ
* @param  stop_mode �͹���ģʽѡ��
*             @arg PMU_MODE_STOP
*             @arg PMU_MODE_DEEPSTOP
* @param  mode_entry ����͹���ģʽ�ķ�ʽ
*             @arg PMU_ENTRY_LOWPOWER_MODE_WFE
*             @arg PMU_ENTRY_LOWPOWER_MODE_WFI
* @retval ��
*/
void std_pmu_enter_stop(uint32_t stop_mode, uint32_t mode_entry)
{    
    std_rcc_apb1_clk_enable(RCC_PERIPH_CLK_PMU);
    
    /* ���õ͹���ģʽ  */
    MODIFY_REG(PMU->CR, PMU_CR_LP_MODE, stop_mode);
    
    /* ��λSLEEPDEEP��־ */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;   
    
     /* ����͹���ģʽ�Ľ��뷽ʽ*/
    if(PMU_ENTRY_LOWPOWER_MODE_WFI == mode_entry)
    {
        __WFI();
    }
    else
    {
        /* ������һ���¼���ͨ��WFE����¼������µ���WFE����͹���ģʽ*/
        __SEV();
        __WFE();
        __WFE();
    }   
    
    /* ��ԭSLEEPDEEP��־ */
    SCB->SCR &= (~SCB_SCR_SLEEPDEEP_Msk);
}



/** 
* @} 
*/

#endif /* STD_PMU_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/

