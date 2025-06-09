/************************************************************************************************/
/**
* @file               ciu32f003_std_iwdg.h
* @author             MCU Ecosystem Development Team
* @brief              IWDG STD������ͷ�ļ���
*                     �ṩIWDG��ص�STD������������������������Լ������Ķ��塣 
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_IWDG_H
#define CIU32F003_STD_IWDG_H

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup IWDG IWDG
* @brief �������Ź���STD������
* @{
*
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
* @defgroup IWDG_Constants IWDG Constants
* @brief IWDG�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* IWDG����ֵ���� */
#define IWDG_RELOAD                      (0x0000AAAAUL)                 /**< IWDG ι��             */
#define IWDG_ENABLE                      (0x0000CCCCUL)                 /**< IWDG ʹ��             */
#define IWDG_WRITE_ACCESS_ENABLE         (0x00005555UL)                 /**< IWDG дȨ��ʹ��       */
#define IWDG_WRITE_ACCESS_DISABLE        (0x00000000UL)                 /**< IWDG дȨ�޹ر�       */

/* IWDG�������ʱ�䶨�� */
#define IWDG_OVERFLOW_PERIOD_128         IWDG_CFG_OVP_128               /**< IWDG���ʱ��Ϊ128ms   */
#define IWDG_OVERFLOW_PERIOD_256         IWDG_CFG_OVP_256               /**< IWDG���ʱ��Ϊ256ms   */
#define IWDG_OVERFLOW_PERIOD_512         IWDG_CFG_OVP_512               /**< IWDG���ʱ��Ϊ512ms   */
#define IWDG_OVERFLOW_PERIOD_1024        IWDG_CFG_OVP_1024              /**< IWDG���ʱ��Ϊ1.024s  */
#define IWDG_OVERFLOW_PERIOD_2048        IWDG_CFG_OVP_2048              /**< IWDG���ʱ��Ϊ2.048s  */
#define IWDG_OVERFLOW_PERIOD_4096        IWDG_CFG_OVP_4096              /**< IWDG���ʱ��Ϊ4.096s  */
#define IWDG_OVERFLOW_PERIOD_8192        IWDG_CFG_OVP_8192              /**< IWDG���ʱ��Ϊ8.192s  */
#define IWDG_OVERFLOW_PERIOD_16384       IWDG_CFG_OVP_16384             /**< IWDG���ʱ��Ϊ16.384s */


/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/
/************************************************************************************************/
/**
* @defgroup IWDG_External_Functions IWDG External Functions
* @brief    IWDG���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ����IWDG����
* @note   ʹ��IWDG��,IWDG�޷�ֹͣ
* @retval ��
*/
__STATIC_INLINE void std_iwdg_start(void)
{
    IWDG->CR = IWDG_ENABLE;
}

/** 
* @brief  ʹ��IWDG���üĴ���дȨ��
* @retval ��
*/
__STATIC_INLINE void std_iwdg_write_access_enable(void)
{
    IWDG->CR = IWDG_WRITE_ACCESS_ENABLE;
}

/** 
* @brief  ��ֹIWDG���üĴ���дȨ��
* @retval ��
*/
__STATIC_INLINE void std_iwdg_write_access_disable(void)
{
    IWDG->CR = IWDG_WRITE_ACCESS_DISABLE;
}

/** 
* @brief  IWDGι��
* @retval ��
*/
__STATIC_INLINE void std_iwdg_refresh(void)
{
    IWDG->CR = IWDG_RELOAD;
}

/**
* @brief  �������ʱ��
* @param  overflow_period IWDG�����ʱ�䣺
*             @arg IWDG_OVERFLOW_PERIOD_128
*             @arg IWDG_OVERFLOW_PERIOD_256
*             @arg IWDG_OVERFLOW_PERIOD_512
*             @arg IWDG_OVERFLOW_PERIOD_1024
*             @arg IWDG_OVERFLOW_PERIOD_2048
*             @arg IWDG_OVERFLOW_PERIOD_4096
*             @arg IWDG_OVERFLOW_PERIOD_8192
*             @arg IWDG_OVERFLOW_PERIOD_16384
* @retval ��
*/
__STATIC_INLINE void std_iwdg_set_overflow_period(uint32_t overflow_period)
{
    IWDG->CFG = overflow_period;
}

/**
* @brief  ��ȡ���ʱ��
* @retval uint32_t IWDG�����ʱ�䣺
*             @arg IWDG_OVERFLOW_PERIOD_128
*             @arg IWDG_OVERFLOW_PERIOD_256
*             @arg IWDG_OVERFLOW_PERIOD_512
*             @arg IWDG_OVERFLOW_PERIOD_1024
*             @arg IWDG_OVERFLOW_PERIOD_2048
*             @arg IWDG_OVERFLOW_PERIOD_4096
*             @arg IWDG_OVERFLOW_PERIOD_8192
*             @arg IWDG_OVERFLOW_PERIOD_16384
*/
__STATIC_INLINE uint32_t std_iwdg_get_overflow_period(void)
{
    return(IWDG->CFG);
}

/**
* @brief ��ȡ����ֵ
* @retval uint32_t IWDG�ļ���ֵ
*/
__STATIC_INLINE uint32_t std_iwdg_get_counter(void)
{
    return(IWDG->CNT);
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

#endif /* CIU32F003_STD_IWDG_H */
