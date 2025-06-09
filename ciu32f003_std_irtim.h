/************************************************************************************************/
/**
* @file               ciu32f003_std_irtim.h
* @author             MCU Ecosystem Development Team
* @brief              IRTIM STD������ͷ�ļ���
*                     �ṩIRTIM��ص�STD������������������������Լ������Ķ��塣                         
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_IRTIM_H
#define CIU32F003_STD_IRTIM_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup IRTIM IRTIM
* @brief �������ģ���STD������
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
* @defgroup IRTIM_Constants  IRTIM Constants
* @brief    IRTIM�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* IRTIM �����ź�Դѡ��  */    
#define ITRIM_SIGNAL_SOURCE_TIM3_OC1                       IRTIM_CR_IR_MODE_TIM3_OC1                /**< �����ź�Դ��TIM3_OC1  */
#define ITRIM_SIGNAL_SOURCE_UART1_TX                       IRTIM_CR_IR_MODE_UART1_TX                /**< �����ź�Դ��UART1_TX  */
#define ITRIM_SIGNAL_SOURCE_UART2_TX                       IRTIM_CR_IR_MODE_UART2_TX                /**< �����ź�Դ��UART2_TX  */
                    
/* IRTIM ����źż���ѡ�� */
#define IRTIM_POLARITY_DIRECT                              (0x00000000U)                            /**< IRTIM ����ź�δ����  */
#define IRTIM_POLARITY_INVERSE                             IRTIM_CR_IR_POL                          /**< IRTIM ����źŷ���    */
     
     
/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup IRTIM_External_Functions IRTIM External Functions
* @brief    IRTIM���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ����IRTIM�����ź�Դ
* @param  source �����ź�Դѡ��
*             @arg ITRIM_SIGNAL_SOURCE_TIM3_OC1�� �����ź�ԴΪTIM3��OC1
*             @arg ITRIM_SIGNAL_SOURCE_UART1_TX�� �����ź�ԴΪUART1
*             @arg ITRIM_SIGNAL_SOURCE_UART2_TX�� �����ź�ԴΪUART2
* @retval ��
*/
__STATIC_INLINE void std_irtim_set_signal_source(uint32_t source)
{
    MODIFY_REG(IRTIM->CR, IRTIM_CR_IR_MODE, source);
}

/**
* @brief  ��ȡIRTIM�����ź�Դ
* @retval uint32_t �����ź�Դ
*             @arg ITRIM_SIGNAL_SOURCE_TIM3_OC1�� �����ź�ԴΪTIM3��OC1
*             @arg ITRIM_SIGNAL_SOURCE_UART1_TX�� �����ź�ԴΪUART1
*             @arg ITRIM_SIGNAL_SOURCE_UART2_TX�� �����ź�ԴΪUART2
*/
__STATIC_INLINE uint32_t std_irtim_get_signal_source(void)
{
    return(IRTIM->CR & IRTIM_CR_IR_MODE);
}

/**
* @brief  IR_OUT����źż���ѡ��
* @param  polarity ����źż���
*             @arg IRTIM_POLARITY_DIRECT:  ����ź�δ����
*             @arg IRTIM_POLARITY_INVERSE: ����źŷ���
* @retval ��
*/
__STATIC_INLINE void std_irtim_set_polarity(uint32_t polarity)
{
    MODIFY_REG(IRTIM->CR, IRTIM_CR_IR_POL, polarity);
}

/**
* @brief  ��ȡIR_OUT����źż���״̬
* @retval uint32_t �����߼����ʽ���жϽ��
*             @arg ��0�� ��ʾIRTIM����źŷ���
*             @arg  0��  ��ʾIRTIM����ź�δ�෴
*/
__STATIC_INLINE uint32_t std_irtim_get_polarity(void)
{
    return(IRTIM->CR & IRTIM_CR_IR_POL);
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

#endif /* CIU32F003_STD_IRTIM_H */
