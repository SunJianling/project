/************************************************************************************************/
/**
* @file               ciu32f003_std_comp.h
* @author             MCU Ecosystem Development Team
* @brief              COMP STD������ͷ�ļ���
*                     �ṩCOMP��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_COMP_H
#define CIU32F003_STD_COMP_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup COMP COMP
* @brief �Ƚ�����STD������
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
* @defgroup COMP_Constants COMP Constants 
* @brief  COMP�������弰�궨��
* @{
*
*/
/************************************************************************************************/
     
/* �Ƚ����ڲ��ο���ѹVDDA ��ѹ */     
#define COMP_VDDA_DIV_1DIV16             COMP_CR_VCDIV_1DIV16         /**< VDDA ��ѹѡ��1/16    */
#define COMP_VDDA_DIV_2DIV16             COMP_CR_VCDIV_2DIV16         /**< VDDA ��ѹѡ��2/16    */     
#define COMP_VDDA_DIV_3DIV16             COMP_CR_VCDIV_3DIV16         /**< VDDA ��ѹѡ��3/16    */     
#define COMP_VDDA_DIV_4DIV16             COMP_CR_VCDIV_4DIV16         /**< VDDA ��ѹѡ��4/16    */     
#define COMP_VDDA_DIV_5DIV16             COMP_CR_VCDIV_5DIV16         /**< VDDA ��ѹѡ��5/16    */
#define COMP_VDDA_DIV_6DIV16             COMP_CR_VCDIV_6DIV16         /**< VDDA ��ѹѡ��6/16    */
#define COMP_VDDA_DIV_7DIV16             COMP_CR_VCDIV_7DIV16         /**< VDDA ��ѹѡ��7/16    */
#define COMP_VDDA_DIV_8DIV16             COMP_CR_VCDIV_8DIV16         /**< VDDA ��ѹѡ��8/16    */
#define COMP_VDDA_DIV_9DIV16             COMP_CR_VCDIV_9DIV16         /**< VDDA ��ѹѡ��9/16    */     
#define COMP_VDDA_DIV_10DIV16            COMP_CR_VCDIV_10DIV16        /**< VDDA ��ѹѡ��10/16   */     
#define COMP_VDDA_DIV_11DIV16            COMP_CR_VCDIV_11DIV16        /**< VDDA ��ѹѡ��11/16   */     
#define COMP_VDDA_DIV_12DIV16            COMP_CR_VCDIV_12DIV16        /**< VDDA ��ѹѡ��12/16   */
#define COMP_VDDA_DIV_13DIV16            COMP_CR_VCDIV_13DIV16        /**< VDDA ��ѹѡ��13/16   */

/* �Ƚ����������� */
#define COMP_INPSEL_IO1                   COMP_CSR_INP_IO1            /**< �������� COMP1 = PB0, COMP2 = PA3 */
#define COMP_INPSEL_IO2                   COMP_CSR_INP_IO2            /**< �������� COMP1 = PB1, COMP2 = PA4 */

/* �Ƚ����������� */
#define COMP_INMSEL_INVREF                COMP_CSR_INM_INT_VREF       /**< �������� �ڲ��ο���ѹ */
#define COMP_INMSEL_IO                    COMP_CSR_INM_IO             /**< �������� COMP1 = PB1, COMP2 = PA4 */                      

/* �Ƚ�����������ģʽѡ�񣬿�����ѡ�񴰿ڱȽ������� */
#define COMP_INPMODE_EACH_INPUT           (0x00000000U)               /**< ����������Զ������ɸ���inpsel���� */
#define COMP_INPMODE_COMMON_INPUT         COMP_CSR_INPMOD             /**< ���������໥���ӣ������ڱȽ���ģʽ */

/* �Ƚ����˲�ʱ�� */
#define COMP_FLTIME_1CYCLE                COMP_CSR_FLTIME_1CYCLE      /**< �˲�ʱ�� 1��CYCLE    */
#define COMP_FLTIME_3CYCLE                COMP_CSR_FLTIME_3CYCLE      /**< �˲�ʱ�� 3��CYCLE    */
#define COMP_FLTIME_7CYCLE                COMP_CSR_FLTIME_7CYCLE      /**< �˲�ʱ�� 7��CYCLE    */
#define COMP_FLTIME_15CYCLE               COMP_CSR_FLTIME_15CYCLE     /**< �˲�ʱ�� 15��CYCLE   */
#define COMP_FLTIME_31CYCLE               COMP_CSR_FLTIME_31CYCLE     /**< �˲�ʱ�� 31��CYCLE   */
#define COMP_FLTIME_63CYCLE               COMP_CSR_FLTIME_63CYCLE     /**< �˲�ʱ�� 63��CYCLE   */
#define COMP_FLTIME_255CYCLE              COMP_CSR_FLTIME_255CYCLE    /**< �˲�ʱ�� 255��CYCLE  */
#define COMP_FLTIME_1023CYCLE             COMP_CSR_FLTIME_1023CYCLE   /**< �˲�ʱ�� 1023��CYCLE */

/* �Ƚ���������� */
#define COMP_OUTPOL_NON_INVERTED          (0x00000000U)               /**< ���״̬������ */
#define COMP_OUTPOL_INVERTED              COMP_CSR_POL                /**< ���״̬����   */

/* �Ƚ������ѡ�� */
#define COMP_OUTMODE_EACH_OUT             (0x00000000U)               /**< ���Ϊ��ԭʼ��ѹ�ȽϽ��         */
#define COMP_OUTMODE_COMMON_XOR_OUT       COMP_CSR_OUTMOD             /**< ���Ϊ�Ƚ���1��2��ѹ�ȽϽ����� */

/* �Ƚ��������� */
#define COMP_OUTPUT_LEVEL_LOW             (0x00000000UL)              /**< �Ƚ��������ƽ�� */
#define COMP_OUTPUT_LEVEL_HIGH            (0x00000001UL)              /**< �Ƚ��������ƽ�� */

/* �Ƚ����ڲ��ο���ѹԴ */
#define COMP_REFERENCE_VBGR               (0x00000000UL)              /**< �Ƚ����ڲ��ο���ѹԴѡ��VBGR         */
#define COMP_REFERENCE_VDDA_DIV           COMP_CR_VCSEL               /**< �Ƚ����ڲ��ο���ѹԴѡ��VDDA 16����ѹ */

/* COMP�����ȶ�ʱ�� */
#define COMP_EN_DELAY                     (1U)                        /**< COMPʹ������ʱ�� */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup COMP_External_Functions COMP External Functions
* @brief    COMP���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ʹ��COMP
* @param  compx COMP����
* @retval ��
*/
__STATIC_INLINE void std_comp_enable(COMP_t *compx)
{
    compx->CSR |= (COMP_CSR_EN);
}

/** 
* @brief  ��ֹCOMP
* @param  compx COMP����
* @retval ��
*/
__STATIC_INLINE void std_comp_disable(COMP_t *compx)
{
    compx->CSR &= (~COMP_CSR_EN);
}

/** 
* @brief  ����COMP��������
* @param  compx COMP����
* @param  input_minus ��������ѡ��
*             @arg COMP_INMSEL_INVREF
*             @arg COMP_INMSEL_IO
* @retval ��
*/
__STATIC_INLINE void std_comp_set_input_minus(COMP_t *compx, uint32_t input_minus)
{
    MODIFY_REG(compx->CSR, COMP_CSR_INM, input_minus);
}

/** 
* @brief  ��ȡCOMP��������
* @param  compx COMP����
* @retval uint32_t ���ط�������
*             @arg COMP_INMSEL_INVREF
*             @arg COMP_INMSEL_IO
*/
__STATIC_INLINE uint32_t std_comp_get_input_minus(COMP_t *compx)
{
    return(compx->CSR & COMP_CSR_INM);
}

/** 
* @brief  ����COMP��������
* @param  compx COMP����
* @param  input_plus ��������ѡ��
*             @arg COMP_INPSEL_IO1
*             @arg COMP_INPSEL_IO2
* @retval ��
*/
__STATIC_INLINE void std_comp_set_input_plus(COMP_t *compx, uint32_t input_plus)
{
    MODIFY_REG(compx->CSR, COMP_CSR_INP, input_plus);
}

/** 
* @brief  ��ȡCOMP��������
* @param  compx COMP����
* @retval uint32_t ������������
*             @arg COMP_INPSEL_IO1
*             @arg COMP_INPSEL_IO2
*/
__STATIC_INLINE uint32_t std_comp_get_input_plus(COMP_t *compx)
{
    return(compx->CSR & COMP_CSR_INP);
}

/** 
* @brief  ����COMP��������ģʽ
* @param  compx COMP����
* @param  input_mode ��������ģʽѡ��
*             @arg COMP_INPMODE_EACH_INPUT
*             @arg COMP_INPMODE_COMMON_INPUT
* @retval ��
*/
__STATIC_INLINE void std_comp_set_input_plus_mode(COMP_t *compx, uint32_t input_mode)
{
    MODIFY_REG(compx->CSR, COMP_CSR_INPMOD, input_mode);
}

/** 
* @brief  ����COMP���ģʽ
* @param  compx COMP����
* @param  output_mode ���ģʽѡ��
*             @arg COMP_OUTMODE_EACH_OUT
*             @arg COMP_OUTMODE_COMMON_XOR_OUT
* @retval ��
*/
__STATIC_INLINE void std_comp_set_output_mode(COMP_t *compx, uint32_t output_mode)
{
    MODIFY_REG(compx->CSR, COMP_CSR_OUTMOD, output_mode);
}

/** 
* @brief  ����COMP�������
* @param  compx COMP����
* @param  output_polarity �������ѡ��
*             @arg COMP_OUTPOL_NON_INVERTED
*             @arg COMP_OUTPOL_INVERTED
* @retval ��
*/
__STATIC_INLINE void std_comp_set_output_polarity(COMP_t *compx, uint32_t output_polarity)
{
    MODIFY_REG(compx->CSR, COMP_CSR_POL, output_polarity);
}

/** 
* @brief  ��ȡCOMP�������
* @param  compx COMP����
* @retval uint32_t �����������
*             @arg COMP_OUTPOL_NON_INVERTED
*             @arg COMP_OUTPOL_INVERTED
*/
__STATIC_INLINE uint32_t std_comp_get_output_polarity(COMP_t *compx)
{
    return(compx->CSR & COMP_CSR_POL);
}

/** 
* @brief  ʹ��COMP���������
* @retval ��
*/
__STATIC_INLINE void std_comp_input_hysteresis_enable(void)
{
    COMP_COMMON->CR |= COMP_CR_HYST;
}

/** 
* @brief  ��ֹCOMP���������
* @retval ��
*/
__STATIC_INLINE void std_comp_input_hysteresis_disable(void)
{
    COMP_COMMON->CR &= ~COMP_CR_HYST;
}

/** 
* @brief  ����COMP����˲�ʱ��
* @param  compx COMP����
* @param  filter_time �˲�ʱ��ѡ��
*             @arg COMP_FLTIME_1CYCLE
*             @arg COMP_FLTIME_3CYCLE
*             @arg ...
*             @arg COMP_FLTIME_1023CYCLE
* @retval ��
*/
__STATIC_INLINE void std_comp_set_output_filter_time(COMP_t *compx, uint32_t filter_time)
{
    MODIFY_REG(compx->CSR, COMP_CSR_FLTIME, filter_time);
}

/** 
* @brief  COMP����˲�ʹ��
* @param  compx COMP����
* @retval ��
*/
__STATIC_INLINE void std_comp_output_filter_enable(COMP_t *compx)
{
    compx->CSR |= (COMP_CSR_FLTEN);
}

/** 
* @brief  COMP����˲���ֹ
* @param  compx COMP����
* @retval ��
*/
__STATIC_INLINE void std_comp_output_filter_disable(COMP_t *compx)
{
    compx->CSR &= (~COMP_CSR_FLTEN);
}

/** 
* @brief  ��ȡCOMPʹ��״̬
* @param  compx COMP����
* @retval uint32_t �����жϽ��
*             @arg ��0�� ��ʾCOMP��ʹ��
*             @arg 0��   ��ʾCOMPδʹ��
*/
__STATIC_INLINE uint32_t std_comp_get_enable_status(COMP_t *compx)
{
    return (compx->CSR & COMP_CSR_EN);
}

/** 
* @brief  ��ȡCOMP������
* @param  compx COMP����
* @retval uint32_t ����COMP������
*             @arg COMP_OUTPUT_LEVEL_LOW
*             @arg COMP_OUTPUT_LEVEL_HIGH
*/
__STATIC_INLINE uint32_t std_comp_get_output_result(COMP_t *compx)
{
    return ((compx->CSR & COMP_CSR_VAL) >> COMP_CSR_VAL_POS);
}

/** 
* @brief  COMP�ο���ѹԴ����
*             @arg COMP_REFERENCE_VBGR
*             @arg COMP_REFERENCE_VDDA_DIV
* @note   VDDA��ѹ��Ϊ�ο���ѹԴ��������VDDA��ѹ����;
* @retval ��
*/
__STATIC_INLINE void std_comp_set_reference_source(uint32_t reference_voltage)
{
    MODIFY_REG(COMP_COMMON->CR, COMP_CR_VCSEL, reference_voltage);
}

/** 
* @brief  ��ȡCOMP�ο���ѹԴ
* @retval uint32_t  ���زο���ԴԴ
*             @arg COMP_REFERENCE_VBGR
*             @arg COMP_REFERENCE_VDDA_DIV
*/
__STATIC_INLINE uint32_t std_comp_get_reference_source(void)
{
    return (COMP_COMMON->CR & COMP_CR_VCSEL);
}

/** 
* @brief  ����VDDA 16����ѹ
* @param  vdda_div VDDA 16����ѹ
*             @arg COMP_VDDA_DIV_1DIV16
*             @arg COMP_VDDA_DIV_2DIV16
*             @arg ...
*             @arg COMP_VDDA_DIV_13DIV16
* @retval ��
*/
__STATIC_INLINE void std_comp_set_ref_vdda_div(uint32_t vdda_div)
{
    MODIFY_REG(COMP_COMMON->CR, COMP_CR_VCDIV, vdda_div);
}

/** 
* @brief  ��ȡVDDA 16����ѹ
* @retval uint32_t ���� VDDA 16����ѹ
*             @arg COMP_VDDA_DIV_1DIV16
*             @arg COMP_VDDA_DIV_2DIV16
*             @arg ...
*             @arg COMP_VDDA_DIV_13DIV16
*/
__STATIC_INLINE uint32_t std_comp_get_ref_vdda_div(void)
{
    return (COMP_COMMON->CR & COMP_CR_VCDIV);
}

/** 
* @brief  ����COMP����������ͷ�������
* @param  compx COMP����
* @param  input_plus ��������ѡ��
*             @arg COMP_INPSEL_IO1
*             @arg COMP_INPSEL_IO2
* @param  input_minis ����������ѡ��
*             @arg COMP_INMSEL_INVREF
*             @arg COMP_INMSEL_IO
*
* @retval ��
*/
__STATIC_INLINE void std_comp_input_config(COMP_t *compx, uint32_t input_plus, uint32_t input_minis)
{
    MODIFY_REG(compx->CSR,
               (COMP_CSR_INP | COMP_CSR_INM),
               (input_plus | input_minis));
}

/* COMPȥ��ʼ������ */
void std_comp_deinit(COMP_t *compx);

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

#endif /* CIU32F003_STD_COMP_H */
