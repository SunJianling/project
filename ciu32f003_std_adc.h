/************************************************************************************************/
/**
* @file               ciu32f003_std_adc.h
* @author             MCU Ecosystem Development Team
* @brief              ADC STD������ͷ�ļ���
*                     �ṩADC��ص�STD������������������������Լ������Ķ��塣                         
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_ADC_H
#define CIU32F003_STD_ADC_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup ADC ADC
* @brief ģ��ת������STD������
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
* @defgroup ADC_Constants  ADC Constants
* @brief    ADC�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* ADC_CKʱ�ӷ�Ƶϵ�� */
#define ADC_CK_DIV1                          ADC_CFG2_PRESC_DIV1                            /**< ADC_CKʱ��: ����Ƶ */
#define ADC_CK_DIV2                          ADC_CFG2_PRESC_DIV2                            /**< ADC_CKʱ��: 2��Ƶ  */
#define ADC_CK_DIV3                          ADC_CFG2_PRESC_DIV3                            /**< ADC_CKʱ��: 3��Ƶ  */
#define ADC_CK_DIV4                          ADC_CFG2_PRESC_DIV4                            /**< ADC_CKʱ��: 4��Ƶ  */
#define ADC_CK_DIV8                          ADC_CFG2_PRESC_DIV8                            /**< ADC_CKʱ��: 8��Ƶ  */
#define ADC_CK_DIV16                         ADC_CFG2_PRESC_DIV16                           /**< ADC_CKʱ��: 16��Ƶ */
#define ADC_CK_DIV32                         ADC_CFG2_PRESC_DIV32                           /**< ADC_CKʱ��: 32��Ƶ */
#define ADC_CK_DIV64                         ADC_CFG2_PRESC_DIV64                           /**< ADC_CKʱ��: 64��Ƶ */

/* ADCת��ģʽ */
#define ADC_SINGLE_CONVER_MODE               ADC_CFG1_CONV_MOD_SINGLE                       /**< ADC����ɨ��ת�� */
#define ADC_CONTINUOUS_CONVER_MODE           ADC_CFG1_CONV_MOD_CONTINUOUS                   /**< ADCѭ��ɨ��ת�� */
#define ADC_DISCONTINUOUS_CONVER_MODE        ADC_CFG1_CONV_MOD_DISCONTINUOUS                /**< ADCѭ�����ת�� */

/* ADCͨ��ɨ�跽�� */
#define ADC_SCAN_DIR_FORWARD                 (0x00000000U)                                  /**< ADCת��ͨ��: ����ɨ�� */
#define ADC_SCAN_DIR_BACKWARD                ADC_CFG1_SDIR                                  /**< ADCת��ͨ��: ����ɨ�� */

/* ADCת������ģʽ: ������ */
#define ADC_TRIG_SW                          ADC_CFG1_TRIGEN_SW                             /**< ���������ʽ                 */
#define ADC_TRIG_HW_EDGE_RISING              ADC_CFG1_TRIGEN_HW_EDGE_RISING                 /**< �ⲿӲ������ʹ��: �����ش��� */
#define ADC_TRIG_HW_EDGE_FALLING             ADC_CFG1_TRIGEN_HW_EDGE_FALLING                /**< �ⲿӲ������ʹ��: �½��ش��� */
#define ADC_TRIG_HW_EDGE_BOTH                ADC_CFG1_TRIGEN_HW_EDGE_BOTH                   /**< �ⲿӲ������ʹ��: ˫�ش���   */

/*  ADCת������Դ */
#define ADC_EXTRIG_TIM1_TRGO                 ADC_CFG1_TRIG_TIM1_TRGO                        /**< ADC����Դ: TIM1 TRGO      */
#define ADC_EXTRIG_TIM1_OC4_ADC              ADC_CFG1_TRIG_TIM1_OC4_ADC                     /**< ADC����Դ: TIM1 OC4       */
#define ADC_EXTRIG_TIM3_TRGO                 ADC_CFG1_TRIG_TIM3_TRGO                        /**< ADC����Դ: TIM3 TRGO      */
#define ADC_EXTRIG_EXTI7                     ADC_CFG1_TRIG_EXTI7                            /**< ADC����Դ: �ⲿ�ж�EXTI_7 */

/* ADCת�������ADC_DR�Ĵ������ݴ洢��ʽ */
#define ADC_OVRN_MODE_PRESERVED              (0x00000000U)                                  /**< ADC_DR�Ĵ�������ԭ���� */
#define ADC_OVRN_MODE_OVERWRITTEN            ADC_CFG1_OVRN_MOD                              /**< ADC_DR�Ĵ������������� */

/* ADC����ʱ����� */
#define ADC_SAMPTIME_3CYCLES                 ADC_SAMPT_SAMPT_3CYCLES                        /**< ����ʱ��Ϊ3��ʱ������    */
#define ADC_SAMPTIME_7CYCLES                 ADC_SAMPT_SAMPT_7CYCLES                        /**< ����ʱ��Ϊ7��ʱ������    */
#define ADC_SAMPTIME_12CYCLES                ADC_SAMPT_SAMPT_12CYCLES                       /**< ����ʱ��Ϊ12��ʱ������   */
#define ADC_SAMPTIME_19CYCLES                ADC_SAMPT_SAMPT_19CYCLES                       /**< ����ʱ��Ϊ19��ʱ������   */
#define ADC_SAMPTIME_39CYCLES                ADC_SAMPT_SAMPT_39CYCLES                       /**< ����ʱ��Ϊ39��ʱ������   */
#define ADC_SAMPTIME_79CYCLES                ADC_SAMPT_SAMPT_79CYCLES                       /**< ����ʱ��Ϊ79��ʱ������   */
#define ADC_SAMPTIME_119CYCLES               ADC_SAMPT_SAMPT_119CYCLES                      /**< ����ʱ��Ϊ119��ʱ������  */
#define ADC_SAMPTIME_159CYCLES               ADC_SAMPT_SAMPT_159CYCLES                      /**< ����ʱ��Ϊ159��ʱ������  */
#define ADC_SAMPTIME_239CYCLES               ADC_SAMPT_SAMPT_239CYCLES                      /**< ����ʱ��Ϊ239��ʱ������  */
#define ADC_SAMPTIME_319CYCLES               ADC_SAMPT_SAMPT_319CYCLES                      /**< ����ʱ��Ϊ319��ʱ������  */
#define ADC_SAMPTIME_479CYCLES               ADC_SAMPT_SAMPT_479CYCLES                      /**< ����ʱ��Ϊ479��ʱ������  */
#define ADC_SAMPTIME_639CYCLES               ADC_SAMPT_SAMPT_639CYCLES                      /**< ����ʱ��Ϊ639��ʱ������  */
#define ADC_SAMPTIME_959CYCLES               ADC_SAMPT_SAMPT_959CYCLES                      /**< ����ʱ��Ϊ959��ʱ������  */
#define ADC_SAMPTIME_1279CYCLES              ADC_SAMPT_SAMPT_1279CYCLES                     /**< ����ʱ��Ϊ1279��ʱ������ */
#define ADC_SAMPTIME_1919CYCLES              ADC_SAMPT_SAMPT_1919CYCLES                     /**< ����ʱ��Ϊ1919��ʱ������ */

/* ADCת��ͨ�� */
#define ADC_CHANNEL_NONE                     (0x00000000U)                                  /**< ADC ת��ͨ�����     */ 
#define ADC_CHANNEL_0                        ADC_CHCFG_CHN0                                 /**< ADC ת��ͨ��IN0      */
#define ADC_CHANNEL_1                        ADC_CHCFG_CHN1                                 /**< ADC ת��ͨ��IN1      */
#define ADC_CHANNEL_2                        ADC_CHCFG_CHN2                                 /**< ADC ת��ͨ��IN2      */
#define ADC_CHANNEL_3                        ADC_CHCFG_CHN3                                 /**< ADC ת��ͨ��IN3      */
#define ADC_CHANNEL_4                        ADC_CHCFG_CHN4                                 /**< ADC ת��ͨ��IN4      */
#define ADC_CHANNEL_5                        ADC_CHCFG_CHN5                                 /**< ADC ת��ͨ��IN5      */
#define ADC_CHANNEL_6                        ADC_CHCFG_CHN6                                 /**< ADC ת��ͨ��IN6      */
#define ADC_CHANNEL_7                        ADC_CHCFG_CHN7                                 /**< ADC ת��ͨ��IN7      */
#define ADC_CHANNEL_8                        ADC_CHCFG_CHN8                                 /**< ADC ת��ͨ��IN8      */
#define ADC_CHANNEL_VBGR                     ADC_CHCFG_CHN8                                 /**< ADC �ڲ�ת��ͨ��VBGR */
#define ADC_CHANNEL_ALL                      ADC_CHCFG_CHN                                  /**< ADC ȫ��ת��ͨ��     */ 

/* ADCģ�⿴�Ź����ͨ�� */
#define ADC_AWDG_CHANNEL_NONE                (0x00000000U)                                  /**< ADC ģ�⿴�Ź����ͨ����� */ 
#define ADC_AWDG_CHANNEL_0                   ADC_AWDGCR_CHN0                                /**< ADC ģ�⿴�Ź����ͨ��0    */ 
#define ADC_AWDG_CHANNEL_1                   ADC_AWDGCR_CHN1                                /**< ADC ģ�⿴�Ź����ͨ��1    */ 
#define ADC_AWDG_CHANNEL_2                   ADC_AWDGCR_CHN2                                /**< ADC ģ�⿴�Ź����ͨ��2    */ 
#define ADC_AWDG_CHANNEL_3                   ADC_AWDGCR_CHN3                                /**< ADC ģ�⿴�Ź����ͨ��3    */ 
#define ADC_AWDG_CHANNEL_4                   ADC_AWDGCR_CHN4                                /**< ADC ģ�⿴�Ź����ͨ��4    */ 
#define ADC_AWDG_CHANNEL_5                   ADC_AWDGCR_CHN5                                /**< ADC ģ�⿴�Ź����ͨ��5    */ 
#define ADC_AWDG_CHANNEL_6                   ADC_AWDGCR_CHN6                                /**< ADC ģ�⿴�Ź����ͨ��6    */ 
#define ADC_AWDG_CHANNEL_7                   ADC_AWDGCR_CHN7                                /**< ADC ģ�⿴�Ź����ͨ��7    */ 
#define ADC_AWDG_CHANNEL_8                   ADC_AWDGCR_CHN8                                /**< ADC ģ�⿴�Ź����ͨ��8    */ 
#define ADC_AWDG_CHANNEL_VBGR                ADC_AWDGCR_CHN8                                /**< ADC ģ�⿴�Ź����ͨ��VBGR */ 
#define ADC_AWDG_CHANNEL_ALL                 ADC_AWDGCR_CHN                                 /**< ADC ȫ��ת��ͨ��           */ 

/* ADC�ڲ�ͨ��ʹ�� */
#define ADC_INTERNAL_CHANNEL_VBGREN          ADC_CFG2_VBGREN                                /**< ADC �ڲ�ͨ��VBGRʹ�� */ 

/* ADC�ж�Դ���壺IER�жϼĴ��� */
#define ADC_INTERRUPT_EOSAMP                 ADC_IER_EOSAMPIE                               /**< ADC���������ж�           */
#define ADC_INTERRUPT_EOC                    ADC_IER_EOCIE                                  /**< ADC��ͨ��ת������ж�     */
#define ADC_INTERRUPT_EOS                    ADC_IER_EOSIE                                  /**< ADCͨ������ת������ж�   */
#define ADC_INTERRUPT_OVRN                   ADC_IER_OVRNIE                                 /**< ADC��������ж�           */
#define ADC_INTERRUPT_AWDG                   ADC_IER_AWDGIE                                 /**< ADCģ�⿴�Ź���ѹ����ж� */
#define ADC_INTERRUPT_EOCAL                  ADC_IER_EOCALIE                                /**< ADCУ׼�����ж�           */

/* ADC״̬���壺ISR״̬�Ĵ��� */
#define ADC_FLAG_EOSAMP                      ADC_ISR_EOSAMP                                 /**< ADC��������״̬           */
#define ADC_FLAG_EOC                         ADC_ISR_EOC                                    /**< ADC��ͨ��ת�����״̬     */
#define ADC_FLAG_EOS                         ADC_ISR_EOS                                    /**< ADCͨ������ת�����״̬   */
#define ADC_FLAG_OVRN                        ADC_ISR_OVRN                                   /**< ADC�������״̬           */
#define ADC_FLAG_AWDG                        ADC_ISR_AWDG                                   /**< ADCģ�⿴�Ź���ص�ѹ״̬ */
#define ADC_FLAG_EOCAL                       ADC_ISR_EOCAL                                  /**< ADCУ׼״̬               */
#define ADC_FLAG_ALL                         (ADC_ISR_EOSAMP | ADC_ISR_EOC | ADC_ISR_EOS \
                                              | ADC_ISR_OVRN | ADC_ISR_AWDG | ADC_ISR_EOCAL)/**< ADCȫ��״̬               */
                                              
/* ADC����ģʽ���� */
#define ADC_MODE_INTERVAL                    (0x00000000U)                                  /**< ADC��Ъ����ģʽ           */
#define ADC_MODE_NORMAL                      ADC_CFG3_MODE                                  /**< ADC��������ģʽ           */
                                              

/* VBGRУ׼�������� */  
#define VBGR_CAL_ADDR                        ((uint16_t *)(BGR_CAL))                        /**< VBGRУ׼�����洢��ַ       */
#define VBGR_CAL_VREF                        (3300U)                                        /**< VBGRУ׼���òο���ѹ��3.3V */

/* �ȴ��ڲ�ͨ��VBGR�������ȶ����ȶ�ʱ�����ֵ�ο�CIU32F003�������ֲ�(tADC_BUF����) */
#define ADC_VBGR_CHANNEL_DELAY               (22U)                                          /**< VBGRͨ�������ȶ�ʱ�䣺22us */

/* ADCʹ���ȶ�ʱ�䣬��ȴ�1us�ȶ�ʱ�� */
#define ADC_EN_DELAY                         (1U)                                           /**< ADCʹ���ȶ�ʱ�� */

/* ADC������ֵ */
#define ADC_CONVER_SCALE                     (4095U)                                        /**< ADC������ֵ������ADC��ѹת������ */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup ADC_External_Functions ADC External Functions
* @brief    ADC���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ʹ��ADC
* @retval ��
*/
__STATIC_INLINE void std_adc_enable(void)
{
    ADC->CR = (ADC_CR_ADEN);
}

/** 
* @brief  ��ֹADC
* @retval ��
*/
__STATIC_INLINE void std_adc_disable(void)
{
    ADC->CR = (ADC_CR_ADDIS);
}

/** 
* @brief  ��ȡADCʹ��λ״̬
* @retval uint32_t �����߼����ʽ���жϽ��
*             @arg ��0�� ��ʾADC����ʹ��״̬
*             @arg 0����ʾADC���ڽ�ֹ״̬
*/
__STATIC_INLINE uint32_t std_adc_get_enable_status(void)
{
    return ((ADC->CR & ADC_CR_ADEN));
}

/** 
* @brief  ʹ��ADCУ׼
* @note   ��ADEN=1��ADC�ȶ�����START=0��STOP=0��ADDIS=0��������ͨ�������CALENλ��1
* @retval ��
*/
__STATIC_INLINE void std_adc_calibration_enable(void)
{
    ADC->CR = (ADC_CR_CALEN);
}

/** 
* @brief  ����ADCת�� 
* @retval ��
*/
__STATIC_INLINE void std_adc_start_conversion(void)
{
    ADC->CR = (ADC_CR_START);
}

/** 
* @brief  ��ȡADC����״̬
* @retval uint32_t ����ADC����״̬
*             @arg ��0�� ��ʾADC�����ڹ���״̬
*             @arg 0����ʾADC����δ����״̬
*/
__STATIC_INLINE uint32_t std_adc_get_conversion_status(void)
{
    return ((ADC->CR & ADC_CR_START));
}

/** 
* @brief  ADCֹͣת��
* @note   ����START=1��ADDIS=0ʱ�������STOPλ��1��Ч
* @retval ��
*/
__STATIC_INLINE void std_adc_stop_conversion(void)
{
    ADC->CR = (ADC_CR_STOP);
}

/** 
* @brief  ADC�ж�ʹ��
* @param  interrupt ADC�ж�Դ   
*             @arg ADC_INTERRUPT_EOSAMP
*             @arg ADC_INTERRUPT_EOC
*             @arg ADC_INTERRUPT_EOS
*             @arg ADC_INTERRUPT_OVRN
*             @arg ADC_INTERRUPT_AWDG
*             @arg ADC_INTERRUPT_EOCAL
* @retval ��
*/
__STATIC_INLINE void std_adc_interrupt_enable(uint32_t interrupt)
{
    ADC->IER |= (interrupt);
}

/** 
* @brief  ADC�жϽ�ֹ
* @param  interrupt ADC�ж�Դ   
*             @arg ADC_INTERRUPT_EOSAMP
*             @arg ADC_INTERRUPT_EOC
*             @arg ADC_INTERRUPT_EOS
*             @arg ADC_INTERRUPT_OVRN
*             @arg ADC_INTERRUPT_AWDG
*             @arg ADC_INTERRUPT_EOCAL
* @retval ��
*/
__STATIC_INLINE void std_adc_interrupt_disable(uint32_t interrupt)
{
    ADC->IER &= (~interrupt);
}

/** 
* @brief  ��ȡADC�ж�Դʹ��״̬
* @param  interrupt ADC�ж�Դ��Ϣ 
*             @arg ADC_INTERRUPT_EOSAMP
*             @arg ADC_INTERRUPT_EOC
*             @arg ADC_INTERRUPT_EOS
*             @arg ADC_INTERRUPT_OVRN
*             @arg ADC_INTERRUPT_AWDG
*             @arg ADC_INTERRUPT_EOCAL
* @retval uint32_t �����ж�Դʹ��״̬
*             @arg ��0�� ��ʾָ�����ж�ʹ��
*             @arg 0����ʾָ�����ж�δʹ��
*/

__STATIC_INLINE uint32_t std_adc_get_interrupt_enable(uint32_t interrupt)
{
    return((ADC->IER & (interrupt)));
}

/** 
* @brief  ��ȡADC��־״̬
* @param  flag ��ȡADC��־
*             @arg ADC_FLAG_EOSAMP
*             @arg ADC_FLAG_EOC
*             @arg ADC_FLAG_EOS
*             @arg ADC_FLAG_OVRN
*             @arg ADC_FLAG_AWDG
*             @arg ADC_FLAG_EOCAL
* @retval uint32_t ���ر�־λ״̬
*             @arg ��0����ʾ��ǰ��־Ϊ��λ״̬
*             @arg 0����ʾ��ǰ��־Ϊ���״̬
*/
__STATIC_INLINE uint32_t std_adc_get_flag(uint32_t flag)
{
    return((ADC->ISR & (flag)));
}

/** 
* @brief  ���ADC��־
* @param  flag ���ADC��־
*             @arg ADC_FLAG_EOSAMP
*             @arg ADC_FLAG_EOC
*             @arg ADC_FLAG_EOS
*             @arg ADC_FLAG_OVRN
*             @arg ADC_FLAG_AWDG
*             @arg ADC_FLAG_EOCAL
* @retval ��
*/
__STATIC_INLINE void std_adc_clear_flag(uint32_t flag)
{
    ADC->ISR = (flag);
}

/** 
* @brief  ����ADCת��ģʽ
* @param  conversion_mode ת��ģʽѡ��
*             @arg ADC_SINGLE_CONVER_MODE
*             @arg ADC_CONTINUOUS_CONVER_MODE
*             @arg ADC_DISCONTINUOUS_CONVER_MODE
* @note   ��START=0ʱ������Դ�λ��ִ��д������Ч
* @retval ��
*/
__STATIC_INLINE void std_adc_conversion_mode_config(uint32_t conversion_mode)
{
    MODIFY_REG(ADC->CFG1, ADC_CFG1_CONV_MOD, conversion_mode);
}

/** 
* @brief  ADCͨ��ʹ��
* @param  channel ͨ����ѡ��
*             @arg ADC_CHANNEL_0
*             @arg ADC_CHANNEL_1
*             @arg ...
*             @arg ADC_CHANNEL_ALL
* @retval ��
*/
__STATIC_INLINE void std_adc_fix_sequence_channel_enable(uint32_t channel)
{
    ADC->CHCFG |= (channel);
}

/** 
* @brief  ADCͨ����ֹ
* @param  channel ��ֹͨ��ѡ��
*             @arg ADC_CHANNEL_0
*             @arg ADC_CHANNEL_1
*             @arg ...
*             @arg ADC_CHANNEL_ALL
* @retval ��
*/
__STATIC_INLINE void std_adc_fix_sequence_channel_disable(uint32_t channel)
{
    ADC->CHCFG &= (~channel);
}

/** 
* @brief  ����ADC����ԴΪ�������
* @retval ��
*/
__STATIC_INLINE void std_adc_trig_sw(void)
{
    MODIFY_REG(ADC->CFG1, ADC_CFG1_TRIGEN, ADC_TRIG_SW);
}

/** 
* @brief  ����ADC�����ź�Դ�ͼ���
* @param  trig_edge ��������ѡ��
*             @arg ADC_TRIG_SW(���������ʽ)
*             @arg ADC_TRIG_HW_EDGE_RISING
*             @arg ADC_TRIG_HW_EDGE_FALLING
*             @arg ADC_TRIG_HW_EDGE_BOTH
* @param  trig_source �ⲿ����Դ��ѡ��
*             @arg ADC_EXTRIG_TIM1_TRGO
*             @arg ADC_EXTRIG_TIM1_OC4_ADC
*             @arg ADC_EXTRIG_TIM3_TRGO
*             @arg ADC_EXTRIG_EXTI7
* @note   ��ѡ��ADC_TRIG_SW(���������ʽ)ʱ������Դ���������塣
* @retval ��
*/
__STATIC_INLINE void std_adc_trig_config(uint32_t trig_edge,uint32_t trig_source)
{
    MODIFY_REG(ADC->CFG1, (ADC_CFG1_TRIGEN | ADC_CFG1_TRIG_SEL), (trig_edge | trig_source));
}

/** 
* @brief  ADC����ʱ�����1����
* @param  sample_time ����ʱ��ѡ��
*           @arg ADC_SAMPTIME_3CYCLES
*           @arg ...
*           @arg ADC_SAMPTIME_1919CYCLES 
* @retval ��
*/
__STATIC_INLINE void std_adc_sampt_time_config(uint32_t sample_time)
{
    MODIFY_REG(ADC->SAMPT, ADC_SAMPT_SAMPT, sample_time);
}

/** 
* @brief  ����ADC_CKʱ�ӷ�Ƶ
* @param  presc ADC_CKʱ��Դ��Ƶϵ��
*             @arg  ADC_CK_DIV1�� ADC_CK ����Ƶ
*             @arg  ADC_CK_DIV2�� ADC_CK 2��Ƶ
*             @arg  ADC_CK_DIV3�� ADC_CK 3��Ƶ
*             @arg ...
*             @arg  ADC_CK_DIV64��ADC_CK 64��Ƶ
* @retval ��
*/
__STATIC_INLINE void std_adc_clock_config(uint32_t presc)
{
    MODIFY_REG(ADC->CFG2, ADC_CFG2_PRESC, presc);
}

/** 
* @brief  ����ADCУ׼ϵ��
* @param  calibration_factor ������ΧΪ0x00~0x3F
* @retval ��
*/
__STATIC_INLINE void std_adc_calibration_factor_config(uint32_t calibration_factor)
{
    ADC->CALFACT = calibration_factor;
}

/** 
* @brief  ��ȡADCУ׼ϵ��
* @retval uint16_t ADCУ׼ϵ��
*/
__STATIC_INLINE uint16_t std_adc_get_calibration_factor(void)
{
    return ((uint16_t)(ADC->CALFACT));
}

/** 
* @brief  ʹ��ADC�ȴ�ģʽ
* @note   ���ڱ�������δ��ʱ��ȡ��ת�����
* @retval ��
*/
__STATIC_INLINE void std_adc_wait_mode_enable(void)
{
    ADC->CFG1 |= (ADC_CFG1_WAIT_MOD);
}

/** 
* @brief  ��ֹADC�ȴ�ģʽ
* @retval ��
*/
__STATIC_INLINE void std_adc_wait_mode_disable(void)
{
    ADC->CFG1 &= (~ADC_CFG1_WAIT_MOD);
}

/** 
* @brief  ����ͨ������ɨ�跽��
* @param  dir ADCͨ��ɨ�跽��
*             @arg ADC_SCAN_DIR_FORWARD
*             @arg ADC_SCAN_DIR_BACKWARD
* @retval ��
*/
__STATIC_INLINE void std_adc_scan_direction_config(uint32_t dir)
{
    MODIFY_REG(ADC->CFG1, ADC_CFG1_SDIR, dir);
}

/** 
* @brief  ����ADC�����������ʽ
* @param  ovrn_mode ADC�����������ʽ
*             @arg ADC_OVRN_MODE_PRESERVED
*             @arg ADC_OVRN_MODE_OVERWRITTEN
* @retval ��
*/
__STATIC_INLINE void std_adc_ovrn_mode_config(uint32_t ovrn_mode)
{
    MODIFY_REG(ADC->CFG1, ADC_CFG1_OVRN_MOD, ovrn_mode);
}

/** 
* @brief  ADC�ڲ�ͨ��VBGRʹ��
* @retval ��
*/
__STATIC_INLINE void std_adc_internal_channel_vbgr_enable(void)
{
    ADC->CFG2 |= (ADC_INTERNAL_CHANNEL_VBGREN);
}

/** 
* @brief  ADC�ڲ�ͨ��VBGR��ֹ
* @retval ��
*/
__STATIC_INLINE void std_adc_internal_channel_vbgr_disable(void)
{
    ADC->CFG2 &= (~ADC_INTERNAL_CHANNEL_VBGREN);
}

/** 
* @brief  ѡ��ADCģ�⿴�Ź����ͨ��
* @param  channel ADC���Ź����ͨ��ѡ��
*             @arg ADC_AWDG_CHANNEL_NONE
*             @arg ADC_AWDG_CHANNEL_0
*             @arg ...
*             @arg ADC_AWDG_CHANNEL_ALL
* @retval ��
*/
__STATIC_INLINE void std_adc_analog_watchdog_monit_channel(uint32_t channel)
{
    MODIFY_REG(ADC->AWDGCR, ADC_AWDGCR_CHN, channel);
}

/** 
* @brief  ���ÿ��Ź����ͨ����ѹ��ֵ
* @param  high_threshold ��ֵ����ѡ��ΧΪ 0x000~0xFFF
* @param  low_threshold  ��ֵ����ѡ��ΧΪ 0x000~0xFFF
* @retval ��
*/
__STATIC_INLINE void std_adc_analog_watchdog_thresholds_config(uint32_t high_threshold, uint32_t low_threshold)
{
    MODIFY_REG(ADC->AWDGTR, (ADC_AWDGTR_AWDG_LT | ADC_AWDGTR_AWDG_HT), (high_threshold << ADC_AWDGTR_AWDG_HT_POS) | (low_threshold));
}

/** 
* @brief  ��ȡADC����ֵ
* @retval uint16_t ADCת��ֵ
*/
__STATIC_INLINE uint16_t std_adc_get_conversion_value(void)
{
    return ((uint16_t)(ADC->DR));
}

/** 
* @brief  ��ȡУ׼��Ĳο���ѹֵ
* @param  vbgr_sample_data ADC�ɼ�BGR��ת��ֵ
* @retval uint32_t У׼��Ĳο���ѹֵ
*/
__STATIC_INLINE uint32_t std_adc_calc_vref_voltage(uint16_t vbgr_sample_data)
{
    return ((VBGR_CAL_VREF * (*VBGR_CAL_ADDR))/vbgr_sample_data);
}

/** 
* @brief  ����ADC����ģʽ
* @param  mode_sel ADC����ģʽ
*             @arg ADC_MODE_INTERVAL
*             @arg ADC_MODE_NORMAL
* @retval ��
*/
__STATIC_INLINE void std_adc_mode_config(uint32_t mode_sel)
{
    MODIFY_REG(ADC->CFG3, ADC_CFG3_MODE_MASK, mode_sel);
}

/* ADCȥ��ʼ������ */
void std_adc_deinit(void);

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

#endif /* CIU32F003_STD_ADC_H */
