/************************************************************************************************/
/**
* @file               ciu32f003_std_lptim.h
* @author             MCU Ecosystem Development Team
* @brief              LPTIM STD������ͷ�ļ���
*                     �ṩLPTIM��ص�STD������������������������Լ������Ķ��塣
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*����ͷ�ļ��ظ�����*/
#ifndef CIU32F003_STD_LPTIM_H
#define CIU32F003_STD_LPTIM_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup LPTIM LPTIM
* @brief �͹��Ķ�ʱ����STD������
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
* @defgroup LPTIM_Constants LPTIM Constants
* @brief    LPTIM�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* ����������ģʽ */
#define LPTIM_COUNT_CONTINUOUS          LPTIM_CR_CNTSTRT            /**< ��������ģʽ    */
#define LPTIM_COUNT_SINGLE              LPTIM_CR_SNGSTRT            /**< ���μ���ģʽ    */

/* LPTIM PSCԤ��Ƶ����Ƶϵ��ѡ�� */
#define LPTIM_PRESCALER_DIV1            LPTIM_CFG_PRESC_1           /**< PSCԤ��Ƶ��1��Ƶ   */
#define LPTIM_PRESCALER_DIV2            LPTIM_CFG_PRESC_2           /**< PSCԤ��Ƶ��2��Ƶ   */
#define LPTIM_PRESCALER_DIV4            LPTIM_CFG_PRESC_4           /**< PSCԤ��Ƶ��4��Ƶ   */
#define LPTIM_PRESCALER_DIV8            LPTIM_CFG_PRESC_8           /**< PSCԤ��Ƶ��8��Ƶ   */
#define LPTIM_PRESCALER_DIV16           LPTIM_CFG_PRESC_16          /**< PSCԤ��Ƶ��16��Ƶ  */
#define LPTIM_PRESCALER_DIV32           LPTIM_CFG_PRESC_32          /**< PSCԤ��Ƶ��32��Ƶ  */
#define LPTIM_PRESCALER_DIV64           LPTIM_CFG_PRESC_64          /**< PSCԤ��Ƶ��64��Ƶ  */
#define LPTIM_PRESCALER_DIV128          LPTIM_CFG_PRESC_128         /**< PSCԤ��Ƶ��128��Ƶ */

/* LPTIM�ж�Դ */
#define LPTIM_INTERRUPT_ARRM            LPTIM_IER_ARRM_IE           /**< �Զ�����ƥ���ж�ʹ�� */
#define LPTIM_INTERRUPT_ITRF            LPTIM_IER_ITRF_IE           /**< ���������ж�ʹ��     */

/* LPTIM�ж�״̬��־ */
#define LPTIM_FLAG_ARRM                 LPTIM_ISR_ARRM              /**< �Զ�����ƥ���־     */
#define LPTIM_FLAG_ITRF                 LPTIM_ISR_ITRF              /**< ����������־         */

/* LPTIM�ж������־ */
#define LPTIM_CLEAR_ARRM                LPTIM_ICR_ARRM_CF           /**< �Զ�����ƥ���־���� */
#define LPTIM_CLEAR_ITRF                LPTIM_ICR_ITRF_CF           /**< ����������־����     */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/
/************************************************************************************************/
/**
* @defgroup LPTIM_External_Functions LPTIM External Functions
* @brief    LPTIM���⺯��
* @{
*
*/
/************************************************************************************************/


/** 
* @brief  ʹ��LPTIM
* @note   ��LPTIMʹ��λ��λ����Ҫ�������������ں������Ч
* @retval ��
*/
__STATIC_INLINE void std_lptim_enable(void)
{
    LPTIM1->CR |= LPTIM_CR_ENABLE;
}


/** 
* @brief  ��ֹLPTIM
* @retval ��
*/
__STATIC_INLINE void std_lptim_disable(void)
{
    LPTIM1->CR &= (~LPTIM_CR_ENABLE);
}


/** 
* @brief  ����LPTIM����������Ԥ��ģʽ��ʼ����
* @param  operate_mode ����ģʽѡ��
*             @arg LPTIM_COUNT_CONTINUOUS:    ��������ģʽ
*             @arg LPTIM_COUNT_SINGLE:        ���μ���ģʽ
* @note   ����ʹ��LPTIM�������������������
* @retval ��
*/
__STATIC_INLINE void std_lptim_start_counter(uint32_t operate_mode)
{
    MODIFY_REG(LPTIM1->CR, (LPTIM_CR_CNTSTRT | LPTIM_CR_SNGSTRT), operate_mode);
}


/** 
* @brief  ����LPTIM�Զ���װ��ֵ
* @param  auto_reload �Զ���װ��ֵ���ñ����ķ�ΧΪ0x0~0xFFFF��
* @retval ��
*/
__STATIC_INLINE void std_lptim_set_auto_reload(uint32_t auto_reload)
{
    LPTIM1->ARR = auto_reload;
}


/** 
* @brief  ��ȡLPTIM�Զ���װ��ֵ
* @retval uint32_t �Զ���װ��ֵ���ñ����ķ�ΧΪ0x0~0xFFFF��
*/
__STATIC_INLINE uint32_t std_lptim_get_auto_reload(void)
{
    return (LPTIM1->ARR);
}


/** 
* @brief  ��ȡLPTIM����ֵ
* @note   ��LPTIMΪ�첽ʱ�Ӽ���ʱ��Ϊȷ����ȡ��ȷ�ļ���ֵ����Ҫȷ�����ζ�ȡ�ļ���ֵһ�¡�
* @retval uint32_t LPTIM����ֵ���ñ����ķ�ΧΪ0x0~0xFFFF��
*/
__STATIC_INLINE uint32_t std_lptim_get_count(void)
{
    return (LPTIM1->CNT);
}


/** 
* @brief  ����LPTIMԤ��Ƶ����Ƶϵ��
* @param  prescaler Ԥ��Ƶϵ��ѡ��
*             @arg LPTIM_PRESCALER_DIV1
*             @arg LPTIM_PRESCALER_DIV2
*             @arg LPTIM_PRESCALER_DIV4
*             @arg ...
*             @arg LPTIM_PRESCALER_DIV128
* @retval ��
*/
__STATIC_INLINE void std_lptim_set_prescaler(uint32_t prescaler)
{
    MODIFY_REG(LPTIM1->CFG, LPTIM_CFG_PRESC, prescaler);
}


/** 
* @brief  ��ȡLPTIMԤ��Ƶ����Ƶϵ��
* @retval uint32_t Ԥ��Ƶϵ��ѡ��
*             @arg LPTIM_PRESCALER_DIV1
*             @arg LPTIM_PRESCALER_DIV2
*             @arg LPTIM_PRESCALER_DIV4
*             @arg ...
*             @arg LPTIM_PRESCALER_DIV128
*/
__STATIC_INLINE uint32_t std_lptim_get_prescaler(void)
{
    return (LPTIM1->CFG & LPTIM_CFG_PRESC);
}


/** 
* @brief  ʹ��LPTIM��������
* @retval ��
*/
__STATIC_INLINE void std_lptim_internal_trigger_enable(void)
{
    LPTIM1->CFG |= LPTIM_CFG_ITREN;
}


/** 
* @brief  ��ֹLPTIM��������
* @retval ��
*/
__STATIC_INLINE void std_lptim_internal_trigger_disable(void)
{
    LPTIM1->CFG &= ~LPTIM_CFG_ITREN;
}


/** 
* @brief  ʹ��LPTIM�ж�
* @param  interrupt LPTIM�ж�Դ
*             @arg LPTIM_INTERRUPT_ARRM:      �Զ�����ƥ���ж�
*             @arg LPTIM_INTERRUPT_ITRF:      ���������ж�ʹ��
* @retval ��
*/
__STATIC_INLINE void std_lptim_interrupt_enable(uint32_t interrupt)
{
    LPTIM1->IER |= interrupt;
}


/** 
* @brief  ��ֹLPTIM�ж�
* @param  interrupt LPTIM�ж�Դ
*             @arg LPTIM_INTERRUPT_ARRM:      �Զ�����ƥ���ж�
*             @arg LPTIM_INTERRUPT_ITRF:      ���������ж�ʹ��
* @retval ��
*/
__STATIC_INLINE void std_lptim_interrupt_disable(uint32_t interrupt)
{
    LPTIM1->IER &= (~interrupt);
}


/** 
* @brief  ��ȡLPTIM�ж�״̬
* @param  interrupt LPTIM�ж�Դ
*             @arg LPTIM_INTERRUPT_ARRM:      �Զ�����ƥ���ж�
*             @arg LPTIM_INTERRUPT_ITRF:      ���������ж�ʹ��
* @retval uint32_t LPTIM�ж�Դʹ��״̬
*             @arg ��0: ʹ��
*             @arg 0:   ��ֹ
*/
__STATIC_INLINE uint32_t std_lptim_get_interrupt_status(uint32_t interrupt)
{
    return (LPTIM1->IER & interrupt);
}


/** 
* @brief  ��ȡLPTIM״̬��־λ
* @param  flag LPTIM״̬��־λ
*             @arg LPTIM_FLAG_ARRM:        �Զ�����ƥ���־
*             @arg LPTIM_FLAG_ITRF:        ����������־
* @retval uint32_t LPTIM��־λ��״̬
*             @arg ��0: ��־λ��λ
*             @arg 0:   ��־λ���
*/
__STATIC_INLINE uint32_t std_lptim_get_flag(uint32_t flag)
{
    return (LPTIM1->ISR & flag);
}


/** 
* @brief  ���LPTIM״̬��־λ
* @param  flag LPTIM״̬��־λ
*             @arg LPTIM_CLEAR_ARRM:        �Զ�����ƥ���־
*             @arg LPTIM_CLEAR_ITRF:        ����������־����
* @retval ��
*/
__STATIC_INLINE void std_lptim_clear_flag(uint32_t flag)
{
    LPTIM1->ICR = flag;
}


/* LPTIMȥ��ʼ������ */
void std_lptim_deinit(void);

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
     
#endif /* CIU32F003_STD_LPTIM_H */
