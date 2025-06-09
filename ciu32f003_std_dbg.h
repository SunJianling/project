/************************************************************************************************/
/**
* @file               ciu32f003_std_dbg.h
* @author             MCU Ecosystem Development Team
* @brief              DBG STD������ͷ�ļ���
*                     �ṩDBG��ص�STD������������������������Լ������Ķ��塣                         
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_DBG_H
#define CIU32F003_STD_DBG_H

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup DBG DBG
* @brief ���Խӿڵ�STD������
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
* @defgroup DBG_Constants DBG Constants 
* @brief  DBG�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* ����ļ�������ѡ�� */
#define DBG_PERIPH_TIM3                DBG_APB_FZ1_TIM3_HOLD                /**< TIM3   ��������λ */
#define DBG_PERIPH_IWDG                DBG_APB_FZ1_IWDG_HOLD                /**< IWDG   ��������λ */
#define DBG_PERIPH_LPTIM1              DBG_APB_FZ1_LPTIM1_HOLD              /**< LPTIM1 ��������λ */

/**
* @}
*/

/*------------------------------------functions-------------------------------------------------*/
/************************************************************************************************/
/**
* @defgroup DBG_External_Functions DBG External Functions
* @brief    DBG���⺯��
* @{
*
*/
/************************************************************************************************/
/**
* @brief  ʹ��Stopģʽ���Թ���
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_stop_enable(void)
{
    DBG->CR = DBG_CR_DBG_STOP;
}

/**
* @brief  ��ֹStopģʽ���Թ���
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_stop_disable(void)
{
    DBG->CR = (~DBG_CR_DBG_STOP);
}

/**
* @brief  �ں�ֹͣʱֹͣ����
* @param  periph_hold ָ��ֹͣ�ļ�������
*             @arg DBG_PERIPH_TIM3
*             @arg DBG_PERIPH_IWDG
*             @arg DBG_PERIPH_LPTIM1
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_apb1_hold_enable(uint32_t periph_hold)
{
    DBG->APB_FZ1 |= periph_hold;
}

/**
* @brief  �ں�ֹͣʱ��������
* @param  periph_hold ָ�������ļ�������
*             @arg DBG_PERIPH_TIM3
*             @arg DBG_PERIPH_IWDG
*             @arg DBG_PERIPH_LPTIM1
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_apb1_hold_disable(uint32_t periph_hold)
{
    DBG->APB_FZ1 &= (~periph_hold);
}

/**
* @brief  �ں�ֹͣʱTIM1ֹͣ����
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_tim1_hold_enable(void)
{
    DBG->APB_FZ2 = DBG_APB_FZ2_TIM1_HOLD;
}

/**
* @brief  �ں�ֹͣʱTIM1��������
* @note   ֻ��ͨ��POR��λ
* @retval ��
*/
__STATIC_INLINE void std_dbg_tim1_hold_disable(void)
{
    DBG->APB_FZ2 = (~DBG_APB_FZ2_TIM1_HOLD);
}

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
#endif /* CIU32F003_STD_DBG_H */
