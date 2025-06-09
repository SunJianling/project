/************************************************************************************************/
/**
* @file               ciu32f003_std_pmu.h
* @author             MCU Ecosystem Development Team
* @brief              PMU STD������ͷ�ļ���
*                     �ṩPMU���STD������������������������Լ������Ķ��塣 
*                      
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*����ͷ�ļ��ظ�����*/
#ifndef CIU32F003_STD_PMU_H
#define CIU32F003_STD_PMU_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup PMU PMU
* @brief ��Դ����Ԫ��STD������
* @{
*/
/************************************************************************************************/

#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std_common.h"


/*--------------------------------------------define--------------------------------------------*/
/************************************************************************************************/
/** 
* @defgroup PMU_Constants PMU Constants
* @brief PMU�������弰�궨��
* @{
*/
/************************************************************************************************/
/* �͹��Ľ��뷽ʽ���� */
#define PMU_ENTRY_LOWPOWER_MODE_WFI         (0x00UL)                            /**< WFI��ʽ����͹���   */
#define PMU_ENTRY_LOWPOWER_MODE_WFE         (0x01UL)                            /**< WFE��ʽ����͹���   */
     
/* �͹���ģʽ���� */     
#define PMU_MODE_STOP                       PMU_CR_LP_MODE_STOP                 /**< Stopģʽ            */
#define PMU_MODE_DEEPSTOP                   PMU_CR_LP_MODE_DEEPSTOP             /**< Deepstopģʽ        */

/* Deepstopģʽ���ѹ�����Flash�Ļ��ѵȴ�ʱ�䶨�� */
#define PMU_DEEPSTOP_FLASH_WAKEUP_TIME_0       (0x3UL << PMU_FLASH_WAKEUP_FLASH_WAKEUP_POS)       /**< Deepstopģʽ���ѹ�����Flash�Ļ��ѵȴ�ʱ��Ϊ0us   */
#define PMU_DEEPSTOP_FLASH_WAKEUP_TIME_10      (0x00000000U)                                      /**< Deepstopģʽ���ѹ�����Flash�Ļ��ѵȴ�ʱ��Ϊ10us  */


/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/
/************************************************************************************************/
/**
* @defgroup PMU_External_Functions PMU External Functions
* @brief    PMU���⺯��
* @{
*/
/************************************************************************************************/
/**
* @brief  ����Deepstopģʽ���ѹ�����Flash�Ļ��ѵȴ�ʱ��
* @param  time_value �ȴ�ʱ��
*             @arg PMU_DEEPSTOP_FLASH_WAKEUP_TIME_0 : 0us
*             @arg PMU_DEEPSTOP_FLASH_WAKEUP_TIME_10: 10us
* @retval ��
*/
__STATIC_INLINE void std_pmu_deepstop_flash_wakeup_time_config(uint32_t time_value)           
{
    MODIFY_REG(PMU->FLASH_WAKEUP, PMU_FLASH_WAKEUP_FLASH_WAKEUP, time_value);
}


/* PMU�͹���ģʽ��غ��� */
void std_pmu_enter_sleep(uint32_t mode_entry);
void std_pmu_enter_stop(uint32_t stop_mode, uint32_t mode_entry);


/**
* @}
*/


#ifdef __cplusplus
}
#endif

/**
* @}
*/

/**
* @}
*/

#endif /* CIU32F003_STD_PMU_H */

