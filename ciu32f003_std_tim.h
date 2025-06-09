/************************************************************************************************/
/**
* @file               ciu32f003_std_tim.h          
* @author             MCU Ecosystem Development Team
* @brief              TIM STD������ͷ�ļ���
*                     �ṩTIM��ص�STD������������������������Լ������Ķ��塣 
* 
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*����ͷ�ļ��ظ�����*/
#ifndef CIU32F003_STD_TIM_H
#define CIU32F003_STD_TIM_H

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup TIM TIM
* @brief    �߼����ƶ�ʱ��/ͨ�ö�ʱ����STD������
* @{
*
*/
/************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std_common.h"


/*-----------------------------------------type define------------------------------------------*/

/************************************************************************************************/
/** 
* @defgroup TIM_Types TIM Types
* @brief    TIM�������Ͷ���
* @{
*/
/************************************************************************************************/

/**
* @brief  TIM�����������ýṹ�嶨��
*/
typedef struct
{    
    uint32_t prescaler;              /**< TIMʱ�ӵ�Ԥ��Ƶ����                               
                                              @note TIM1Ԥ��Ƶ������ΧΪ��0x0000~0xFFFF    
                                                    TIM3Ԥ��Ƶ������ΧΪ��0x0000~0x000F                    */
                                   
    uint32_t counter_mode;           /**< ������ģʽѡ��
                                              @arg TIM_COUNTER_MODE_UP ...                                 */
                                   
    uint32_t period;                 /**< ���´θ����¼�ʱ���ص��Զ����¼��ؼĴ����е����ֵ
                                          ��ֵ������0x0000~0xFFFF֮��                                      */
                                   
    uint32_t clock_div;              /**< TIMʱ�ӷ�Ƶ����
                                              @arg TIM_CLOCK_DTS_DIV1 ...                                  */
                                   
    uint8_t repeat_counter;         /**< �ظ��������������壬ÿ��RCR���¼����ﵽ0ʱ�������һ�������¼���
                                         ����RCR��ֵ(N)��ʼ���¼������ò���������0x0000~0x00FF֮�䡣       */
                                   
}std_tim_basic_init_t;


/**
* @brief  TIM���벶��������ýṹ�嶨��
*/
typedef struct
{
    uint32_t  input_capture_pol;         /**< �����źŵ���Ч����ѡ��                           
                                                  @arg TIM_INPUT_POL_RISING ...                */
                                
    uint32_t input_capture_sel;          /**< ����ģʽ����
                                                  @arg TIM_INPUT_CAPTURE_SEL_DIRECTTI ...      */
                                
    uint32_t input_capture_prescaler;    /**< ���벶��Ԥ��Ƶ����
                                                  @arg TIM_INPUT_CAPTURE_PSC_DIV1 ...          */
                                
    uint32_t input_capture_filter;       /**< ���벶���˲������壬��ֵ������0x0~0x7֮��        */
    
}std_tim_input_capture_init_t;      


/**
* @brief  TIM����Ƚϲ������ýṹ�嶨��
*/
typedef struct
{
    uint32_t output_compare_mode;        /**< TIM����Ƚ�ģʽ����
                                                  @arg TIM_OUTPUT_MODE_ACTIVE ...                           */
    
    uint32_t output_state;               /**< ���ʹ�ܶ���                                               
                                                  @arg TIM_OUTPUT_DISABLE ...                               */
 
    uint32_t output_negtive_state;       /**< �������ʹ�ܶ���                                               
                                                  @arg TIM_OUTPUT_NEGTIVE_DISABLE ...                       */
                                   
    uint32_t pulse;                      /**< TIM���벶��ȽϼĴ���������ֵ����ֵ������0x0000~0xFFFF֮��    */    
                                   
    uint32_t output_pol;                 /**< ������Զ���
                                                  @arg TIM_OUTPUT_POL_HIGH ...                              */
                                   
    uint32_t output_negtive_pol;         /**< ����������Զ���
                                                  @arg TIM_OUTPUT_NEGTIVE_POL_HIGH ...                      */                                                                     
                                   
    uint32_t output_idle_state;          /**< TIM����״̬�����״̬����
                                                  @arg  TIM_OUTPUT_IDLE_SET ...
                                                  @note �ò�������֧��Break���ܵĶ�ʱ��ʵ����Ч             */
                                   
    uint32_t output_negtive_idle_state;  /**< TIM����״̬�»������״̬����
                                                  @arg  TIM_OUTPUT_NEGTIVE_IDLE_SET ...
                                                  @note �ò�������֧��Break���ܵĶ�ʱ��ʵ����Ч             */
    
}std_tim_output_compare_init_t;


/**
* @brief  TIM��·����������������ýṹ�嶨��
*/
typedef struct
{
    uint32_t off_state_run_mode;                        /**< ����ģʽ�µĹر�״̬
                                                                 @arg TIM_OSSR_ENABLE ...            */
    
    uint32_t off_state_idle_mode;                       /**< ����ģʽ�µĹر�״̬
                                                                 @arg TIM_OSSI_ENABLE ...            */
    
    uint32_t lock_level;                                /**< LOCK����
                                                                 @arg TIM_LOCK_LEVEL1 ...            */  
    
    uint32_t dead_time;                                 /**< �����������壬��ֵ������0x00~0xFF֮��   */
       
    uint32_t break_state;                              /**< ��·����ʹ�ܿ���
                                                                  @arg TIM_BREAK_ENABLE
                                                                  @arg TIM_BREAK_DISABLE             */ 
    
}std_tim_break_init_t;



/**
* @}
*/
  
/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup TIM_Constants TIM Constants 
* @brief    TIM�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* TIM����ģʽ���� */
#define TIM_COUNTER_MODE_UP                (0x00000000U)                      /**< ���ϼ���                      */
#define TIM_COUNTER_MODE_DOWN              TIM_CR1_DIR                        /**< ���¼���                      */
#define TIM_COUNTER_MODE_CENT_MODE1        TIM_CR1_CMS_CENTER_UP              /**< ���Ķ���ģʽ1                 */
#define TIM_COUNTER_MODE_CENT_MODE2        TIM_CR1_CMS_CENTER_DOWN            /**< ���Ķ���ģʽ2                 */
#define TIM_COUNTER_MODE_CENT_MODE3        TIM_CR1_CMS_CENTER_UP_DOWN         /**< ���Ķ���ģʽ3                 */

/* TIM�����¼�Դѡ�� */
#define TIM_UPDATE_SOURCE_REGULAR          (0x00000000U)                      /**< ������������紥��                       */
#define TIM_UPDATE_SOURCE_COUNTER          TIM_CR1_URS                        /**< ������������硢UG��λ����ģʽ����������  */

/* TIMʱ�ӷ�Ƶ */
#define TIM_CLOCK_DTS_DIV1                 (0x00000000U)                      /**< tDTS=tTIMx_KCLK          */
#define TIM_CLOCK_DTS_DIV2                 TIM_CR1_CLK_DIV2                   /**< tDTS=2*tTIMx_KCLK        */
#define TIM_CLOCK_DTS_DIV4                 TIM_CR1_CLK_DIV4                   /**< tDTS=4*tTIMx_KCLK        */

/* TIMͨ������ */
#define TIM_CHANNEL_1                      (0x00U)                            /**< ͨ��1����            */
#define TIM_CHANNEL_2                      (0x01U)                            /**< ͨ��2����            */
#define TIM_CHANNEL_3                      (0x02U)                            /**< ͨ��3����            */
#define TIM_CHANNEL_4                      (0x03U)                            /**< ͨ��4����            */

/* TIM����ͨ�����Զ��� */
#define TIM_INPUT_POL_RISING               (0x00000000U)                        /**< δ����/�����ش���             */
#define TIM_INPUT_POL_FALLING              TIM_CCEN_CC1P                        /**< ����/�½��ش���               */
#define TIM_INPUT_POL_BOTH                 (TIM_CCEN_CC1P | TIM_CCEN_CC1NP)     /**< δ����/�����½�������         */

/* TIM���벶��ѡ�� */
#define TIM_INPUT_CAPTURE_SEL_DIRECTTI         TIM_CCM1_CC1S_DIRECTTI             /**< TIM����1, 2, 3 or 4���ұ�ӳ�䵽IC1, IC2, IC3 or IC4��һһ��Ӧ�� */
#define TIM_INPUT_CAPTURE_SEL_INDIRECTTI       TIM_CCM1_CC1S_INDIRECTTI           /**< TIM����1, 2, 3 or 4���ұ�ӳ�䵽IC2, IC1, IC4 or IC3��һһ��Ӧ�� */
#define TIM_INPUT_CAPTURE_SEL_TRC              TIM_CCM1_CC1S_TRC                  /**< TIM����1, 2, 3 or 4���ұ�ӳ�䵽TRC                              */

/* TIM���벶��Ԥ��Ƶ�������� */
#define TIM_INPUT_CAPTURE_PSC_DIV1             (0x00000000U)                      /**< ��Ԥ��Ƶ��������������ÿ��⵽һ�����ر�ִ�в��� */
#define TIM_INPUT_CAPTURE_PSC_DIV2             TIM_CCM1_IC1PSC_DIV2               /**< ÿ����2���¼�ִ��һ�β���        */
#define TIM_INPUT_CAPTURE_PSC_DIV4             TIM_CCM1_IC1PSC_DIV4               /**< ÿ����4���¼�ִ��һ�β���        */
#define TIM_INPUT_CAPTURE_PSC_DIV8             TIM_CCM1_IC1PSC_DIV8               /**< ÿ����8���¼�ִ��һ�β���        */

/* TIM����Ƚϼ��Զ��� */
#define TIM_OUTPUT_POL_HIGH                (0x00000000U)                      /**< �Ƚ�������ԣ��ߵ�ƽΪ��Ч��ƽ      */
#define TIM_OUTPUT_POL_LOW                 TIM_CCEN_CC1P                      /**< �Ƚ�������ԣ��͵�ƽΪ��Ч��ƽ      */

/* TIM����Ƚ����ʹ�ܶ��� */
#define TIM_OUTPUT_DISABLE                 (0x00000000U)                      /**< �Ƚ�ͨ�������ֹ                    */
#define TIM_OUTPUT_ENABLE                  TIM_CCEN_CC1E                      /**< �Ƚ�ͨ�����ʹ��                    */

/* TIM����Ƚϲ�������*/   
#define TIM_OUTPUT_MODE_FROZEN                  (0x00000000U)                                /**< ����                          */
#define TIM_OUTPUT_MODE_ACTIVE                  TIM_CCM1_OC1M_ACTIVE                         /**< ͨ��1����Ϊƥ��ʱ�����Ч��ƽ */
#define TIM_OUTPUT_MODE_INACTIVE                TIM_CCM1_OC1M_INACTIVE                       /**< ͨ��1����Ϊƥ��ʱ�����Ч��ƽ */
#define TIM_OUTPUT_MODE_TOGGLE                  TIM_CCM1_OC1M_TOGGLE                         /**< ��ת                          */
#define TIM_OUTPUT_MODE_FORCED_INACTIVE         TIM_CCM1_OC1M_FORCED_INACTIVE                /**< ǿ�Ʊ�Ϊ��Ч��ƽ              */
#define TIM_OUTPUT_MODE_FORCED_ACTIVE           TIM_CCM1_OC1M_FORCED_ACTIVE                  /**< ǿ�Ʊ�Ϊ��Ч��ƽ              */
#define TIM_OUTPUT_MODE_PWM1                    TIM_CCM1_OC1M_PWM1                           /**< PWMģʽ1                      */
#define TIM_OUTPUT_MODE_PWM2                    TIM_CCM1_OC1M_PWM2                           /**< PWMģʽ2                      */

/*  TIMʱ��Դѡ�� */ 
#define TIM_CLKSRC_INT                     (0x00000000U)                      /**< TIMʱ��Դ���ڲ�ʱ��           */
#define TIM_CLKSRC_MODE1                   TIM_SMC_SM_SEL_EXT_MODE1           /**< TIMʱ��Դ���ⲿʱ��Դģʽ1    */

/* TIMʱ�Ӽ��Զ��� */
#define TIM_CLK_TIX_POL_RISING             (0x00000000U)                      /**< TIxʱ��Դ�ļ��ԣ���������Ч   */
#define TIM_CLK_TIX_POL_FALLING            TIM_CCEN_CC1P                      /**< TIxʱ��Դ�ļ��ԣ��½�����Ч   */
#define TIM_CLK_TIX_POL_BOTH               (TIM_CCEN_CC1P | TIM_CCEN_CC1NP)   /**< TIxʱ��Դ�ļ��ԣ�˫����Ч     */

/* TIM�¼�Դ */
#define TIM_EVENT_SRC_UPDATE               TIM_EVTG_UG                       /**< ���³�ʼ��������������һ�������¼�      */
#define TIM_EVENT_SRC_CC1                  TIM_EVTG_CC1G                     /**< ��ͨ��1������һ������/�Ƚ��¼�          */
#define TIM_EVENT_SRC_CC2                  TIM_EVTG_CC2G                     /**< ��ͨ��2������һ������/�Ƚ��¼�          */ 
#define TIM_EVENT_SRC_CC3                  TIM_EVTG_CC3G                     /**< ��ͨ��3������һ������/�Ƚ��¼�          */ 
#define TIM_EVENT_SRC_CC4                  TIM_EVTG_CC4G                     /**< ��ͨ��4������һ������/�Ƚ��¼�          */ 
#define TIM_EVENT_SRC_COM                  TIM_EVTG_COMG                     /**< ����һ�������¼�                        */
#define TIM_EVENT_SRC_TRIG                 TIM_EVTG_TG                       /**< ����һ�������¼�                        */
#define TIM_EVENT_SRC_BREAK                TIM_EVTG_BG                       /**< ����һ����·�¼�                        */

/* TIM�ж϶��� */
#define TIM_INTERRUPT_UPDATE               TIM_DIER_UIE                       /**< �����ж�                */
#define TIM_INTERRUPT_CC1                  TIM_DIER_CC1IE                     /**< ����/�Ƚ�1�ж�          */
#define TIM_INTERRUPT_CC2                  TIM_DIER_CC2IE                     /**< ����/�Ƚ�1�ж�2         */
#define TIM_INTERRUPT_CC3                  TIM_DIER_CC3IE                     /**< ����/�Ƚ�1�ж�3         */
#define TIM_INTERRUPT_CC4                  TIM_DIER_CC4IE                     /**< ����/�Ƚ�1�ж�4         */
#define TIM_INTERRUPT_COM                  TIM_DIER_COMIE                     /**< �����ж�                */
#define TIM_INTERRUPT_TRIG                 TIM_DIER_TIE                       /**< �����ж�                */
#define TIM_INTERRUPT_BREAK                TIM_DIER_BIE                       /**< ��·�ж�                */

/* TIM��־���� */
#define TIM_FLAG_UPDATE                    TIM_SR_UIF                         /**< �����жϱ�־              */
#define TIM_FLAG_CC1                       TIM_SR_CC1IF                       /**< ����/�Ƚ�1�¼���־        */
#define TIM_FLAG_CC2                       TIM_SR_CC2IF                       /**< ����/�Ƚ�2�¼���־        */
#define TIM_FLAG_CC3                       TIM_SR_CC3IF                       /**< ����/�Ƚ�3�¼���־        */
#define TIM_FLAG_CC4                       TIM_SR_CC4IF                       /**< ����/�Ƚ�4�¼���־        */
#define TIM_FLAG_COM                       TIM_SR_COMIF                       /**< �����¼���־              */
#define TIM_FLAG_TRIG                      TIM_SR_TIF                         /**< �����¼���־              */
#define TIM_FLAG_BREAK                     TIM_SR_BIF                         /**< ��·�¼���־              */
#define TIM_FLAG_CC1OF                     TIM_SR_CC1OF                       /**< ����/�Ƚ�1�ظ������־    */
#define TIM_FLAG_CC2OF                     TIM_SR_CC2OF                       /**< ����/�Ƚ�2�ظ������־    */

#define TIM_FLAG_ALL                       (0xFFFF)                           /**< TIM�¼���־               */
#define TIM_FLAG_CCX_ALL                   (0x001E)                           /**< ȫͨ������/�Ƚ��¼���־   */

/* TIM���OCxREF���������Դ */                               
#define TIM_CLEAR_INPUT_SRC_COMP1          (0x00000000U)                    /**< OCREF_CLR_INPUT���ӵ�COMP1�����  */
#define TIM_CLEAR_INPUT_SRC_COMP2          TIM_CFG_OCREF_CLR                /**< OCREF_CLR_INPUT���ӵ�COMP2�����  */

/* TIM�����¼�Դ */
#define TIM_COM_SOFTWARE                  (0x00000000U)                      /**< �������/�ȽϿ���λ����Ԥװ�أ�CCPC=1������ͨ����COMGλ��1���������¼�               */
#define TIM_COM_TRGI                      TIM_CR2_CCU_SEL                    /**< �������/�ȽϿ���λ����Ԥװ�أ�CCPC=1������ͨ����COMGλ��1��TRIG�������ش��������¼� */

/* TIM��ģʽѡ��(TRIG_OUT)��������*/
#define TIM_TRIG_OUT_RESET                 (0x00000000U)                         /**< TIM1_EVTG�Ĵ����е�UGλ�������������TRIG_OUT��  */
#define TIM_TRIG_OUT_ENABLE                TIM_CR2_MM_SEL_ENABLE                 /**< ������ʹ���ź�CEN�������������TRIG_OUT��        */
#define TIM_TRIG_OUT_UPDATE                TIM_CR2_MM_SEL_UPDATE                 /**< ѡ������¼���Ϊ���������TRIG_OUT��             */
#define TIM_TRIG_OUT_CC1                   TIM_CR2_MM_SEL_CC1IF                  /**< �����Ƚ�ƥ�䣬���������TRIG_OUT��             */
#define TIM_TRIG_OUT_OC1REF                TIM_CR2_MM_SEL_OC1REF                 /**< OC1REF�ź������������(TRIG_OUT)                 */
#define TIM_TRIG_OUT_OC2REF                TIM_CR2_MM_SEL_OC2REF                 /**< OC2REF�ź������������(TRIG_OUT)                 */
#define TIM_TRIG_OUT_OC3REF                TIM_CR2_MM_SEL_OC3REF                 /**< OC3REF�ź������������(TRIG_OUT)                 */
#define TIM_TRIG_OUT_OC4REF                TIM_CR2_MM_SEL_OC4REF                 /**< OC4REF�ź������������(TRIG_OUT)                 */

/* TIM��ģʽ��������*/
#define TIM_SLAVE_MODE_DISABLE             (0x00000000U)                              /**< ��ֹ��ģʽ          */
#define TIM_SLAVE_MODE_RESET               TIM_SMC_SM_SEL_RESET                       /**< ��λģʽ            */
#define TIM_SLAVE_MODE_GATED               TIM_SMC_SM_SEL_GATED                       /**< �ſ�ģʽ            */
#define TIM_SLAVE_MODE_TRIG                TIM_SMC_SM_SEL_TRIG                        /**< ����ģʽ            */

/* TIM����ѡ���������*/
#define TIM_TRIG_SOURCE_ITR0               TIM_SMC_TS_ITR0                              /**< �ڲ�����0��ITR0��               */
#define TIM_TRIG_SOURCE_TI1F_ED            TIM_SMC_TS_TI1F_ED                           /**< TI1���ؼ������TI1F_ED��        */
#define TIM_TRIG_SOURCE_TI1FP1             TIM_SMC_TS_TI1FP1                            /**< �˲���Ķ�ʱ������1��TI1FP1��   */
#define TIM_TRIG_SOURCE_TI2FP2             TIM_SMC_TS_TI2FP2                            /**< �˲���Ķ�ʱ������2��TI1FP2��   */

/* TIM�������Բ�������*/
#define TIM_TRIG_TIX_POL_RISING            (0x00000000U)                        /**< TIxFPx��TI1F_ED�������ԣ��ߵ�ƽ����������Ч   */
#define TIM_TRIG_TIX_POL_FALLING           TIM_CCEN_CC1P                        /**< TIxFPx��TI1F_ED�������ԣ��͵�ƽ���½�����Ч   */
#define TIM_TRIG_TIX_POL_BOTH              (TIM_CCEN_CC1P | TIM_CCEN_CC1NP)     /**< TIxFPx��TI1F_ED�������ԣ������½�������       */
                                                                                       
/* TIM����Ƚϻ���������Զ��� */
#define TIM_OUTPUT_NEGTIVE_POL_HIGH        (0x00000000U)                      /**< �����������Ϊ�ߵ�ƽ��Ч      */
#define TIM_OUTPUT_NEGTIVE_POL_LOW         TIM_CCEN_CC1NP                     /**< �����������Ϊ�͵�ƽ��Ч      */

/* TIM����Ƚϻ������ʹ�ܶ��� */
#define TIM_OUTPUT_NEGTIVE_DISABLE         (0x00000000U)                      /**< �������ͨ�������ֹ          */
#define TIM_OUTPUT_NEGTIVE_ENABLE          TIM_CCEN_CC1NE                     /**< �������ͨ�����ʹ��          */

/* TIM����״̬�����״̬���� */
#define TIM_OUTPUT_IDLE_RESET              (0x00000000U)                      /**< ����״̬Ϊ: ��MOEN=0ʱOCx����͵�ƽ   */
#define TIM_OUTPUT_IDLE_SET                TIM_CR2_OIS1                       /**< ����״̬Ϊ: ��MOEN=0ʱOCx����ߵ�ƽ   */

/* TIM����״̬�»������״̬���� */
#define TIM_OUTPUT_NEGTIVE_IDLE_RESET      (0x00000000U)                      /**< �����������״̬Ϊ: ��MOEN=0ʱOCxN����͵�ƽ  */
#define TIM_OUTPUT_NEGTIVE_IDLE_SET        TIM_CR2_OIS1N                      /**< �����������״̬Ϊ: ��MOEN=0ʱOCxN����ߵ�ƽ  */

/* TIM����ģʽ�¹ر�״̬ѡ��������� */                                         
#define TIM_OSSR_DISABLE                   (0x00000000U)               /**< ������Ч״̬ʱ��OC/OCN�����ֹ(�����ܶ�ʱ������)     */
#define TIM_OSSR_ENABLE                    TIM_BDT_OSSR                /**< ������Ч״̬ʱ��OC/OCN���ʹ��(��Ȼ�ܶ�ʱ������)     */
                                                                               
/* TIM����ģʽ�¹ر�״̬ѡ��������� */                                         
#define TIM_OSSI_DISABLE                   (0x00000000U)               /**< ������Ч״̬ʱ��OC/OCN�����ֹ(�����ܶ�ʱ������)     */
#define TIM_OSSI_ENABLE                    TIM_BDT_OSSI                /**< ������Ч״̬ʱ��OC/OCN���ʹ��(��Ȼ�ܶ�ʱ������)     */

/* TIM��������*/
#define TIM_LOCK_LEVEL_OFF                 (0x00000000U)                      /**< ������0         */
#define TIM_LOCK_LEVEL1                    TIM_BDT_LOCK_LEVEL1                /**< ������1         */
#define TIM_LOCK_LEVEL2                    TIM_BDT_LOCK_LEVEL2                /**< ������2         */
#define TIM_LOCK_LEVEL3                    TIM_BDT_LOCK_LEVEL3                /**< ������3         */

/* TIM��·����ʹ��*/                                                            
#define TIM_BREAK_DISABLE                  (0x00000000U)                      /**< ��ֹ��·����    */
#define TIM_BREAK_ENABLE                   TIM_BDT_BKEN                       /**< ʹ�ܶ�·����    */                                                                              
                                                                              
/* TIM��·����Դ���� */                                                       
#define TIM_BREAK_INPUT_SRC_GPIO           TIM1_AF1_BKINE                      /**< GPIO�ӵ�BKIN������          */    
#define TIM_BREAK_INPUT_SRC_COMP1          TIM1_AF1_BKCMP1E                    /**< COMP1����ӵ���·����       */
#define TIM_BREAK_INPUT_SRC_COMP2          TIM1_AF1_BKCMP2E                    /**< COMP2����ӵ���·����       */       
  
/* TIM��·���뼫�Զ��� */                                                                                                    
#define TIM_BREAK_INPUT_POL_HIGH           TIM1_AF1_BKINP                      /**< ��·����ԴΪ�ߵ�ƽ      */
#define TIM_BREAK_INPUT_POL_LOW            (0x00000000U)                       /**< ��·����ԴΪ�͵�ƽ      */ 
                                                                                                                                                                              
/* TIM�ⲿʱ������ѡ��  */                                                                                                                                                    
#define TIM_TIM3_TI1_GPIO                  TIM_TISEL_TI1_SEL_CH1                      /**< TIM3_TI1���ӵ�GPIO       */
#define TIM_TIM3_TI1_COMP1                 TIM_TISEL_TI1_SEL_COMP1                    /**< TIM3_TI1���ӵ�COMP1���  */    
                                                                                       
#define TIM_TIM3_TI2_GPIO                  TIM_TISEL_TI2_SEL_CH2                      /**< TIM3_TI2���ӵ�GPIO       */  
#define TIM_TIM3_TI2_COMP2                 TIM_TISEL_TI2_SEL_COMP2                    /**< TIM3_TI2���ӵ�COMP2���  */                                                                                                                                                 


/**
* @}
*/


/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup TIM_External_Functions TIM External Functions
* @brief    TIM���⺯��
* @{
*
*/
/************************************************************************************************/
/**
* @brief  TIM��������
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_enable(TIM_t *timx)
{
    timx->CR1 |= TIM_CR1_CEN;    
}

/**
* @brief  TIMֹͣ����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_disable(TIM_t *timx)
{
    timx->CR1 &= (~TIM_CR1_CEN);
}


/**
* @brief  ����TIMԤ��Ƶ����
* @param  timx TIM����
* @param  presc Ԥ��Ƶ��ֵ
* @note   TIM1Ԥ��Ƶ������ΧΪ��0x0000~0xFFFF    
*         TIM3Ԥ��Ƶ������ΧΪ��0x0000~0x000F
* @retval ��
*/
__STATIC_INLINE void std_tim_set_psc(TIM_t *timx, uint32_t presc)          
{
    timx->PSC = (presc);
}


/**
* @brief  ��ȡTIMԤ��Ƶ����
* @param  timx TIM����
* @retval uint32_t TIMԤ��Ƶֵ
*/
__STATIC_INLINE uint32_t std_tim_get_psc(TIM_t *timx)          
{
    return (timx->PSC);    
}


/**
* @brief  ����TIM����ֵ
* @param  timx TIM���� 
* @param  counter ������ֵ
* @retval ��
*/
__STATIC_INLINE void std_tim_set_counter(TIM_t *timx, uint32_t counter)     
{
    timx->CNT = (counter);
}

/**
* @brief  ��ȡTIM����ֵ
* @param  timx TIM����
* @retval uint32_t TIM����ֵ
*/
__STATIC_INLINE uint32_t std_tim_get_counter(TIM_t *timx)   
{
    return (timx->CNT);
}

/**
* @brief  ����TIM ARRֵ
* @param  timx TIM����
* @param  autoreload ARRֵ
* @retval ��
*/
__STATIC_INLINE void std_tim_set_autoreload(TIM_t *timx, uint32_t autoreload)
{
    timx->ARR = (autoreload); 
}
                                                            
/**
* @brief  ��ȡTIM ARRֵ
* @param  timx TIM����
* @retval uint32_t TIM ARRֵ
*/
__STATIC_INLINE uint32_t std_tim_get_autoreload(TIM_t *timx)
{
    return (timx->ARR);
}


/**
* @brief  ����TIM RCRֵ
* @param  timx TIM����
* @param  rcr TIM RCRֵ
* @retval ��
*/
__STATIC_INLINE void std_tim_set_repcounter(TIM_t *timx, uint32_t rcr)
{
    timx->RCR = (rcr);
}


/**
* @brief  ��ȡTIM RCRֵ
* @param  timx TIM����
* @retval uint32_t TIM RCRֵ
*/
__STATIC_INLINE uint32_t std_tim_get_repcounter(TIM_t *timx)
{
    return (timx->RCR);
}


/**
* @brief  ����TIMʱ�ӷ�Ƶֵ
* @param  timx TIM����
* @param  clk_div ʱ�ӷ�Ƶֵ
*             @arg TIM_CLOCK_DTS_DIV1:  tDTS=tTIM_KCLK
*             @arg TIM_CLOCK_DTS_DIV2:  tDTS=2*tTIM_KCLK
*             @arg TIM_CLOCK_DTS_DIV4:  tDTS=4*tTIM_KCLK
* @retval ��
*/
__STATIC_INLINE void std_tim_set_clock_div(TIM_t *timx, uint32_t clk_div) 
{
    MODIFY_REG(timx->CR1, TIM_CR1_CLK_DIV, clk_div);    
}

/**
* @brief  ��ȡTIMʱ�ӷ�Ƶֵ
* @param  timx TIM����
* @retval uint32_t ʱ�ӷ�Ƶֵ
*             @arg TIM_CLOCK_DIV1: tDTS=tTIM_KCLK
*             @arg TIM_CLOCK_DIV2: tDTS=2*tTIM_KCLK
*             @arg TIM_CLOCK_DIV4: tDTS=4*tTIM_KCLK
*/
__STATIC_INLINE uint32_t std_tim_get_clock_div(TIM_t *timx)           
{
    return (timx->CR1 & TIM_CR1_CLK_DIV);
}

/**
* @brief  ʹ���Զ����ع���
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_arrpreload_enable(TIM_t *timx)           
{
    timx->CR1 |= TIM_CR1_ARPE;
}

/**
* @brief  ��ֹ�Զ����ع���
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_arrpreload_disable(TIM_t *timx)           
{
    timx->CR1 &= (~TIM_CR1_ARPE);
}

/**
* @brief  ʹ�ܸ����¼�
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_updateevent_enable(TIM_t *timx)
{
    timx->CR1 |= TIM_CR1_UDIS;
}

/**
* @brief  ��ֹ�����¼�
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_updateevent_disable(TIM_t *timx)
{
    timx->CR1 &= (~TIM_CR1_UDIS);
}

/**
* @brief  ���ø����¼�Դ
* @param  timx TIM����
* @param  update_source �����¼�Դѡ��
*             @arg TIM_UPDATE_SOURCE_REGULAR
*             @arg TIM_UPDATE_SOURCE_COUNTER
* @retval ��
*/
__STATIC_INLINE void std_tim_set_update_source(TIM_t *timx, uint32_t update_source)
{
    MODIFY_REG(timx->CR1, TIM_CR1_URS, update_source);
}


/**
* @brief  ��ȡ�����¼�Դ
* @param  timx TIM����
* @retval uint32_t �����¼�Դѡ��
*             @arg TIM_UPDATE_SOURCE_REGULAR
*             @arg TIM_UPDATE_SOURCE_COUNTER
*/
__STATIC_INLINE uint32_t std_tim_get_update_source(TIM_t *timx)
{
    return (timx->CR1 & TIM_CR1_URS);
}


/**
* @brief  ʹ�ܹ���ģʽ1
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_work_mode1_enable(TIM_t *timx)
{
    timx->CR1 |= TIM_CR1_MODE;
}

/**
* @brief  ��ֹ����ģʽ1
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_work_mode1_disable(TIM_t *timx)
{
    timx->CR1 &= (~TIM_CR1_MODE);
}


/**
* @brief  ���ü���ģʽ
* @param  timx TIM����
* @param  counter_mode ������ʽ
*             @arg TIM_COUNTER_MODE_UP
*             @arg TIM_COUNTER_MODE_DOWN
*             @arg TIM_COUNTER_MODE_CENT_MODE1
*             @arg TIM_COUNTER_MODE_CENT_MODE2
*             @arg TIM_COUNTER_MODE_CENT_MODE3
* @note   ����DIR����λ�����Ķ���ģʽ��Ϊֻ��Ȩ�ޣ��������Ķ���ģʽ�л�������ģʽʱ��������������޸��쳣��Ӧ�ȸ�λһ��TIM
* @retval ��
*/
__STATIC_INLINE void std_tim_set_counter_mode(TIM_t *timx, uint32_t counter_mode)
{
    MODIFY_REG(timx->CR1, (TIM_CR1_DIR | TIM_CR1_CMS), counter_mode);
}

/**
* @brief  ��ȡ����ģʽ
* @param  timx TIM����
* @retval uint32_t ������ʽ
*             @arg TIM_COUNTER_MODE_UP
*             @arg TIM_COUNTER_MODE_DOWN
*             @arg TIM_COUNTER_MODE_CENT_MODE1
*             @arg TIM_COUNTER_MODE_CENT_MODE2
*             @arg TIM_COUNTER_MODE_CENT_MODE3
*/
__STATIC_INLINE uint32_t std_tim_get_counter_mode(TIM_t *timx)
{
    if(timx->CR1 & TIM_CR1_CMS)
    {
        return (timx->CR1 & TIM_CR1_CMS);
    }
    else
    {
        return (timx->CR1 & TIM_CR1_DIR);       
    }
}

/**
* @brief  ��ȡTIM��������
* @param  timx TIM����
* @retval uint32_t ����TIM���������־
*             @arg ��0: ��ǰ��������Ϊ���¼���
*             @arg 0:��ǰ��������Ϊ���ϼ���
*/
__STATIC_INLINE uint32_t std_tim_get_count_dir(TIM_t *timx)  
{
    return (timx->CR1 & TIM_CR1_DIR);
}


/**
* @brief  ʹ�ܵ�����ģʽ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_onepulse_enable(TIM_t *timx)
{
    timx->CR1 |= TIM_CR1_OPM;
}

/**
* @brief  ��ֹ������ģʽ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_onepulse_disable(TIM_t *timx)
{
    timx->CR1 &= (~TIM_CR1_OPM);
}

/**
* @brief  ��ȡ������ģʽ
* @param  timx TIM����
* @retval uint32_t ���ص��������ģʽ
*             @arg ��0: ��ǰ����Ϊ������ģʽ
*             @arg 0:��ǰ����Ϊ��������ģʽ
*/
__STATIC_INLINE uint32_t std_tim_get_onepulse_mode(TIM_t *timx)
{
    return (timx->CR1 & (TIM_CR1_OPM));
}


/**
* @brief  ���ü���ʱ��Դ����
* @param  timx TIM����
* @param  clock_source ʱ��Դѡ��
*             @arg TIM_CLKSRC_INT:   �ڲ�ʱ��Դ
*             @arg TIM_CLKSRC_MODE1: �ⲿʱ��ģʽ1
* @retval ��
*/
__STATIC_INLINE void std_tim_clock_source_config(TIM_t *timx, uint32_t clock_source)
{
    MODIFY_REG(timx->SMC, TIM_SMC_SM_SEL, clock_source);
}


/**
* @brief  ����һ������¼�
* @param  timx TIM����
* @param  event_src �¼�Դ
*             @arg TIM_EVENT_SRC_UPDATE:�����¼�Դ
*             @arg TIM_EVENT_SRC_CC1:   ����Ƚ�1�¼�Դ
*             @arg TIM_EVENT_SRC_CC2:   ����Ƚ�2�¼�Դ
*             @arg TIM_EVENT_SRC_CC3:   ����Ƚ�3�¼�Դ
*             @arg TIM_EVENT_SRC_CC4:   ����Ƚ�4�¼�Դ
*             @arg TIM_EVENT_SRC_COM:   �����¼�Դ
*             @arg TIM_EVENT_SRC_TRIG:  �����¼�Դ
*             @arg TIM_EVENT_SRC_BREAK: ��·�¼�Դ
* @retval ��
*/
__STATIC_INLINE void std_tim_set_sw_trig_event(TIM_t *timx, uint32_t event_src)
{
    timx->EVTG = event_src;
}


/** 
* @brief  TIM�ж�ʹ��
* @param  timx TIM����
* @param  interrupt TIM�ж�Դ
*             @arg TIM_INTERRUPT_UPDATE:  �����ж�
*             @arg TIM_INTERRUPT_CC1:     ����/�Ƚ�1�ж�
*             @arg TIM_INTERRUPT_CC2:     ����/�Ƚ�2�ж�
*             @arg TIM_INTERRUPT_CC3:     ����/�Ƚ�3�ж�
*             @arg TIM_INTERRUPT_CC4:     ����/�Ƚ�4�ж�
*             @arg TIM_INTERRUPT_COM:     �����ж�
*             @arg TIM_INTERRUPT_TRIG:    �����ж�
*             @arg TIM_INTERRUPT_BREAK:   ��·�ж�
* @retval ��
*/
__STATIC_INLINE void std_tim_interrupt_enable(TIM_t *timx, uint32_t interrupt)    
{
    timx->DIER |= (interrupt);
}

/** 
* @brief  TIM�жϽ�ֹ
* @param  timx TIM����
* @param  interrupt TIM�ж�Դ
*             @arg TIM_INTERRUPT_UPDATE:  �����ж�
*             @arg TIM_INTERRUPT_CC1:     ����/�Ƚ�1�ж�
*             @arg TIM_INTERRUPT_CC2:     ����/�Ƚ�2�ж�
*             @arg TIM_INTERRUPT_CC3:     ����/�Ƚ�3�ж�
*             @arg TIM_INTERRUPT_CC4:     ����/�Ƚ�4�ж�
*             @arg TIM_INTERRUPT_COM:     �����ж�
*             @arg TIM_INTERRUPT_TRIG:    �����ж�
*             @arg TIM_INTERRUPT_BREAK:   ��·�ж�
* @retval ��
*/
__STATIC_INLINE void std_tim_interrupt_disable(TIM_t *timx, uint32_t interrupt)   
{
    timx->DIER &= (~(interrupt));
}


/**
* @brief  ��ȡTIM�ж�Դ��״̬
* @param  timx TIM����
* @param  interrupt TIM�ж�Դ��Ϣ
*             @arg TIM_INTERRUPT_UPDATE:  �����ж�
*             @arg TIM_INTERRUPT_CC1:     ����/�Ƚ�1�ж�
*             @arg TIM_INTERRUPT_CC2:     ����/�Ƚ�2�ж�
*             @arg TIM_INTERRUPT_CC3:     ����/�Ƚ�3�ж�
*             @arg TIM_INTERRUPT_CC4:     ����/�Ƚ�4�ж�
*             @arg TIM_INTERRUPT_COM:     �����ж�
*             @arg TIM_INTERRUPT_TRIG:    �����ж�
*             @arg TIM_INTERRUPT_BREAK:   ��·�ж�
* @retval uint32_t �����ж�����Դ��״̬ 
*/  
__STATIC_INLINE uint32_t std_tim_get_interrupt_enable(TIM_t *timx, uint32_t interrupt)   
{
    return (timx->DIER & (interrupt));
}


/** 
* @brief  ��ȡTIM��־״̬
* @param  timx TIM����
* @param  flag TIM��־��Ϣ
*             @arg TIM_FLAG_UPDATE:       �����¼���־
*             @arg TIM_FLAG_CC1:          ����/�Ƚ�1�¼���־
*             @arg TIM_FLAG_CC2:          ����/�Ƚ�2�¼���־
*             @arg TIM_FLAG_CC3:          ����/�Ƚ�3�¼���־
*             @arg TIM_FLAG_CC4:          ����/�Ƚ�4�¼���־
*             @arg TIM_FLAG_COM:          �����¼���־
*             @arg TIM_FLAG_TRIG:         �����¼���־
*             @arg TIM_FLAG_BREAK:        ��·�¼���־
*             @arg TIM_FLAG_CC1OF:        ����/�Ƚ�1�ظ������־
*             @arg TIM_FLAG_CC2OF:        ����/�Ƚ�2�ظ������־
*             @arg TIM_FLAG_CCX_ALL:      ȫͨ������/�Ƚ��¼���־
*             @arg TIM_FLAG_ALL:          TIM�¼���־
* @retval uint32_t ���ر�־��״̬ 
*             @arg ��0: ��ǰ��־Ϊ����״̬ 
*             @arg 0:��ǰ��־Ϊ���״̬
*/
__STATIC_INLINE uint32_t std_tim_get_flag(TIM_t *timx, uint32_t flag)          
{
    return (timx->SR &(flag));
}


/** 
* @brief  ���TIM��־
* @param  timx TIM����
* @param  flag ���TIM��־
*             @arg TIM_FLAG_UPDATE:       �����¼���־
*             @arg TIM_FLAG_CC1:          ����/�Ƚ�1�¼���־
*             @arg TIM_FLAG_CC2:          ����/�Ƚ�2�¼���־
*             @arg TIM_FLAG_CC3:          ����/�Ƚ�3�¼���־
*             @arg TIM_FLAG_CC4:          ����/�Ƚ�4�¼���־
*             @arg TIM_FLAG_COM:          �����¼���־
*             @arg TIM_FLAG_TRIG:         �����¼���־
*             @arg TIM_FLAG_BREAK:        ��·�¼���־
*             @arg TIM_FLAG_CC1OF:        ����/�Ƚ�1�ظ������־
*             @arg TIM_FLAG_CC2OF:        ����/�Ƚ�2�ظ�������־
*             @arg TIM_FLAG_CCX_ALL:      ȫͨ������/�Ƚ��¼���־
*             @arg TIM_FLAG_ALL:          TIM�¼���־
* @retval ��
*/
__STATIC_INLINE void std_tim_clear_flag(TIM_t *timx, uint32_t flag)
{
    timx->SR = (~(flag));
}


/**
* @brief  ʹ��TIM�Ƚ�/ƥ��ͨ��
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_ccx_channel_enable(TIM_t *timx, uint32_t channel_id)
{  
    timx->CCEN |= (TIM_CCEN_CC1E << (channel_id << 2));
}


/**
* @brief  ��ֹTIM�Ƚ�/ƥ��ͨ��
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_ccx_channel_disable(TIM_t *timx, uint32_t channel_id)
{     
    timx->CCEN &= (~(TIM_CCEN_CC1E << (channel_id << 2)));
}


/**
* @brief  ʹ��TIM�������ͨ��
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @retval ��
*/
__STATIC_INLINE void std_tim_ccxn_channel_enable(TIM_t *timx, uint32_t channel_id)
{    
    timx->CCEN |= (TIM_CCEN_CC1NE << (channel_id << 2));
}


/**
* @brief  ��ֹTIM�������ͨ��
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @retval ��
*/
__STATIC_INLINE void std_tim_ccxn_channel_disable(TIM_t *timx, uint32_t channel_id)
{ 
    timx->CCEN &= (~(TIM_CCEN_CC1NE << (channel_id << 2)));
}


/**
* @brief  ����TIM���벶��ͨ������
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @param  input_capture_pol ���뼫��
*             @arg TIM_INPUT_POL_RISING
*             @arg TIM_INPUT_POL_FALLING
*             @arg TIM_INPUT_POL_BOTH
* @retval ��
*/
__STATIC_INLINE void std_tim_set_input_pol(TIM_t *timx, uint32_t channel_id, uint32_t input_capture_pol)
{ 
    MODIFY_REG(timx->CCEN, ((TIM_CCEN_CC1P | TIM_CCEN_CC1NP) << (channel_id << 2)), (input_capture_pol << (channel_id << 2)));
}

/**
* @brief  ��ȡTIM���벶��ͨ������
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval uint32_t ���뼫��
*             @arg TIM_INPUT_POL_RISING
*             @arg TIM_INPUT_POL_FALLING
*             @arg TIM_INPUT_POL_BOTH
*/
__STATIC_INLINE uint32_t std_tim_get_input_pol(TIM_t *timx, uint32_t channel_id)
{ 
    return (((timx->CCEN) >> (channel_id << 2)) & (TIM_CCEN_CC1P | TIM_CCEN_CC1NP));
}


/**
* @brief  ����TIMͨ������ļ���
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @param  output_commpare_pol �������
*             @arg TIM_OUTPUT_POL_HIGH
*             @arg TIM_OUTPUT_POL_LOW
* @retval ��
*/
__STATIC_INLINE void std_tim_set_output_pol(TIM_t *timx, uint32_t channel_id, uint32_t output_commpare_pol)
{ 
    MODIFY_REG(timx->CCEN, (TIM_CCEN_CC1P << (channel_id << 2)), (output_commpare_pol << (channel_id << 2)));
}

/**
* @brief  ��ȡTIMͨ������ļ���
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval uint32_t �������
*             @arg TIM_OUTPUT_POL_HIGH
*             @arg TIM_OUTPUT_POL_LOW
*/
__STATIC_INLINE uint32_t std_tim_get_output_pol(TIM_t *timx, uint32_t channel_id)
{ 
    return (((timx->CCEN) >> (channel_id << 2)) & TIM_CCEN_CC1P);
}


/**
* @brief  ����TIM����ͨ�����������
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @param  negtive_output_pol ����ͨ�����������
*             @arg TIM_OUTPUT_NEGTIVE_POL_HIGH
*             @arg TIM_OUTPUT_NEGTIVE_POL_LOW
* @retval ��
*/
__STATIC_INLINE void std_tim_set_negtive_output_pol(TIM_t *timx, uint32_t channel_id, uint32_t negtive_output_pol)
{ 
    MODIFY_REG(timx->CCEN, (TIM_CCEN_CC1NP << (channel_id << 2)), (negtive_output_pol << (channel_id << 2)));
}

/**
* @brief  ��ȡTIM����ͨ�����������
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @retval uint32_t ����ͨ�����������
*             @arg TIM_OUTPUT_NEGTIVE_POL_HIGH
*             @arg TIM_OUTPUT_NEGTIVE_POL_LOW
*/
__STATIC_INLINE uint32_t std_tim_get_negtive_output_pol(TIM_t *timx, uint32_t channel_id)
{ 
    return (((timx->CCEN) >> (channel_id << 2)) & TIM_CCEN_CC1NP);
}


/**
* @brief  ����TIMͨ���Ŀ���״̬
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @param  idle_state ͨ������״̬
*             @arg TIM_OUTPUT_IDLE_RESET
*             @arg TIM_OUTPUT_IDLE_SET
* @retval ��
*/
__STATIC_INLINE void std_tim_set_output_idlestate(TIM_t *timx, uint32_t channel_id, uint32_t idle_state)
{ 
    MODIFY_REG(timx->CR2, (TIM_CR2_OIS1 << (channel_id << 1)), (idle_state << (channel_id << 1)));
}

/**
* @brief  ��ȡTIMͨ���Ŀ���״̬
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval uint32_t ͨ������״̬
*             @arg TIM_OUTPUT_IDLE_RESET
*             @arg TIM_OUTPUT_IDLE_SET
*/
__STATIC_INLINE uint32_t std_tim_get_output_idlestate(TIM_t *timx, uint32_t channel_id)
{ 
    return (((timx->CR2) >> (channel_id << 1)) & TIM_CR2_OIS1);    
}

/**
* @brief  ����TIM����ͨ���Ŀ���״̬
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @param  negtive_idlestate ����ͨ������״̬
*             @arg TIM_OUTPUT_NEGTIVE_IDLE_RESET
*             @arg TIM_OUTPUT_NEGTIVE_IDLE_SET
* @retval ��
*/
__STATIC_INLINE void std_tim_set_negtive_output_idlestate(TIM_t *timx, uint32_t channel_id, uint32_t negtive_idlestate)
{ 
    MODIFY_REG(timx->CR2, (TIM_CR2_OIS1N << (channel_id << 1)), (negtive_idlestate << (channel_id << 1)));
}

/**
* @brief  ��ȡTIM����ͨ���Ŀ���״̬
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
* @retval uint32_t ����ͨ������״̬
*             @arg TIM_OUTPUT_NEGTIVE_IDLE_RESET
*             @arg TIM_OUTPUT_NEGTIVE_IDLE_SET
*/
__STATIC_INLINE uint32_t std_tim_get_negtive_output_idlestate(TIM_t *timx, uint32_t channel_id)
{ 
    return (((timx->CR2) >> (channel_id << 1)) & TIM_CR2_OIS1N);        
}

/**
* @brief  ʹ�����OCxREF����
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_channel_clear_ocxref_enable(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;    
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    /* ʹ�����OCxREF */ 
    *preg |= (TIM_CCM1_OC1CE << tmp_value);
}

/**
* @brief  ��ֹ���OCxREF����
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_channel_clear_ocxref_disable(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;      
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    /* ��ֹ���OCxREF */ 
    *preg &= (~(TIM_CCM1_OC1CE << tmp_value));
}

/**
* @brief  �������벶��Ԥ��Ƶֵ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @param  icxpsc_num ���벶��Ԥ��Ƶֵ
*             @arg TIM_INPUT_CAPTURE_PSC_DIV1
*             @arg TIM_INPUT_CAPTURE_PSC_DIV2
*             @arg TIM_INPUT_CAPTURE_PSC_DIV4
*             @arg TIM_INPUT_CAPTURE_PSC_DIV8
* @retval ��
*/
__STATIC_INLINE void std_tim_set_channel_icxpsc(TIM_t *timx, uint32_t channel_id, uint32_t icxpsc_num)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;      
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    /* �������벶��Ԥ��Ƶ */ 
    MODIFY_REG(*preg, (TIM_CCM1_IC1PSC << tmp_value), (icxpsc_num << tmp_value));
}


/**
* @brief  ��ȡ���벶��Ԥ��Ƶֵ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval uint32_t ���벶��Ԥ��Ƶֵ
*             @arg TIM_INPUT_CAPTURE_PSC_DIV1
*             @arg TIM_INPUT_CAPTURE_PSC_DIV2
*             @arg TIM_INPUT_CAPTURE_PSC_DIV4
*             @arg TIM_INPUT_CAPTURE_PSC_DIV8
*/
__STATIC_INLINE uint32_t std_tim_get_channel_icxpsc(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    /* ��ȡ���벶��Ԥ��Ƶ */ 
    return ((*preg >> tmp_value) & TIM_CCM1_IC1PSC);
    
}


/**
* @brief  ʹ��TIMͨ������Ƚϵ�Ԥװ�ع���
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_preloadccx_channel_enable(TIM_t *timx, uint32_t channel_id)
{ 
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);    
 
    /* ʹ������Ƚ�Ԥװ�ع��� */ 
    *preg |= (TIM_CCM1_OC1PE << tmp_value);
}


/**
* @brief  ��ֹTIMͨ������Ƚϵ�Ԥװ�ع���
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_preloadccx_channel_disable(TIM_t *timx, uint32_t channel_id)
{ 
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);    
 
    /* ʹ������Ƚ�Ԥװ�ع��� */ 
    *preg &= (~(TIM_CCM1_OC1PE << tmp_value));
}


/**
* @brief  ʹ��TIMͨ������ģʽ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_fastmode_channel_enable(TIM_t *timx, uint32_t channel_id)
{ 
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);    
 
    /* ʹ������Ƚ�Ԥװ�ع��� */ 
    *preg |= (TIM_CCM1_OC1FE << tmp_value);
}


/**
* @brief  ��ֹTIMͨ������ģʽ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
__STATIC_INLINE void std_tim_fastmode_channel_disable(TIM_t *timx, uint32_t channel_id)
{ 
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);    
 
    /* ��ֹ�������ģʽ */ 
    *preg &= (~(TIM_CCM1_OC1FE << tmp_value));
}


/**
* @brief  ����ͨ�����ģʽ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @param  ocmode ���ģʽѡ��
*             @arg TIM_OUTPUT_MODE_FROZEN         
*             @arg TIM_OUTPUT_MODE_ACTIVE         
*             @arg TIM_OUTPUT_MODE_INACTIVE       
*             @arg TIM_OUTPUT_MODE_TOGGLE         
*             @arg TIM_OUTPUT_MODE_FORCED_INACTIVE
*             @arg TIM_OUTPUT_MODE_FORCED_ACTIVE  
*             @arg TIM_OUTPUT_MODE_PWM1           
*             @arg TIM_OUTPUT_MODE_PWM2                   
* @retval ��
*/
__STATIC_INLINE void std_tim_set_ocmode(TIM_t *timx, uint32_t channel_id, uint32_t ocmode)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    MODIFY_REG(*preg, ((TIM_CCM1_OC1M  | TIM_CCM1_CC1S) << tmp_value), (ocmode << tmp_value));
}


/**
* @brief  ��ȡͨ�����ģʽ
* @param  timx TIM����
* @param  channel_id ָ��TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @note   �ú���ִ�к󣬻Ὣͨ������Ϊ���ģʽ
* @retval uint32_t ���ģʽѡ��
*             @arg TIM_OUTPUT_MODE_FROZEN         
*             @arg TIM_OUTPUT_MODE_ACTIVE         
*             @arg TIM_OUTPUT_MODE_INACTIVE       
*             @arg TIM_OUTPUT_MODE_TOGGLE         
*             @arg TIM_OUTPUT_MODE_FORCED_INACTIVE
*             @arg TIM_OUTPUT_MODE_FORCED_ACTIVE  
*             @arg TIM_OUTPUT_MODE_PWM1           
*             @arg TIM_OUTPUT_MODE_PWM2               
*/
__STATIC_INLINE uint32_t std_tim_get_ocmode(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;    
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);

    return ((*preg >> tmp_value) & TIM_CCM1_OC1M);
}

/**
* @brief  ���ò���/�ȽϼĴ�����ֵ
* @param  timx TIM����
* @param  ccx_value ����ȽϼĴ�����ֵ
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval std_status_t ����APIִ�н��
*/
__STATIC_INLINE void std_tim_set_ccx_value(TIM_t *timx, uint32_t channel_id, uint32_t ccx_value)
{    
    if ((timx->CR1 & TIM_CR1_MODE) != TIM_CR1_MODE)
    {
        __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)(&timx->CC1) + (channel_id << 2));
        MODIFY_REG(*pReg, TIM_CC1_CC1, ccx_value);    
    }
    else
    {
        uint32_t tmp_value = ((channel_id & 0x02) == 0)?0U:8U;   
        uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:2U;          
        __IO uint32_t *pReg = (__IO uint32_t *)((uint32_t)(&timx->CC1) + ((channel_id - shift_value) << 2));
        MODIFY_REG(*pReg, (TIM3_CC1_CC1_MODE1 << tmp_value), (ccx_value << tmp_value));
    }
}


/**
* @brief  ��ȡ����/�ȽϼĴ�����ֵ
* @param  timx TIM����
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval uint32_t ����ȽϼĴ�����ֵ
*/
__STATIC_INLINE uint32_t std_tim_get_ccx_value(TIM_t *timx, uint32_t channel_id)
{
    if ((timx->CR1 & TIM_CR1_MODE) != TIM_CR1_MODE)
    {
        __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CC1) + (channel_id << 2));
        return (*preg & TIM_CC1_CC1);
    }
    else
    {
        uint32_t tmp_value = ((channel_id & 0x02) == 0)?0U:8U;  
        uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:2U;     
        __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CC1) + ((channel_id - shift_value) << 2));
        
        return ((*preg >> tmp_value) & TIM3_CC1_CC1_MODE1);
    }
}


/**
* @brief  ����ͨ��Ϊ����ģʽ
* @param  timx TIM����
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @param  icmode ����ģʽѡ��
*             @arg TIM_INPUT_CAPTURE_SEL_DIRECTTI
*             @arg TIM_INPUT_CAPTURE_SEL_INDIRECTTI
*             @arg TIM_INPUT_CAPTURE_SEL_TRC
* @retval ��
*/
__STATIC_INLINE void std_tim_set_icmode(TIM_t *timx, uint32_t channel_id, uint32_t icmode)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    MODIFY_REG(*preg, (TIM_CCM1_CC1S << tmp_value), (icmode << tmp_value));
}

/**
* @brief  ��ȡͨ��������״̬
* @param  timx TIM����
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval uint32_t ����ģʽѡ��
*             @arg TIM_INPUT_CAPTURE_SEL_DIRECTTI
*             @arg TIM_INPUT_CAPTURE_SEL_INDIRECTTI
*             @arg TIM_INPUT_CAPTURE_SEL_TRC
*/
__STATIC_INLINE uint32_t std_tim_get_icmode(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    return ((*preg >> tmp_value) & TIM_CCM1_CC1S);
}

/**
* @brief  ��������ͨ�����˲�����
* @param  timx TIM����
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @param  icfilter �����˲���������ֵ�ķ�ΧΪ:0x00~0x0F
* @retval ��
*/
__STATIC_INLINE void std_tim_set_icfilter(TIM_t *timx, uint32_t channel_id, uint32_t icfilter)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    MODIFY_REG(*preg, (TIM_CCM1_IC1F << tmp_value), ((icfilter << tmp_value) << 4U));
}

/**
* @brief  ��ȡ����ͨ�����˲�����
* @param  timx TIM����
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval uint32_t �����˲���������ֵ�ķ�ΧΪ:0x00~0x0F
*/
__STATIC_INLINE uint32_t std_tim_get_icfilter(TIM_t *timx, uint32_t channel_id)
{
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    return (((*preg >> tmp_value) & TIM_CCM1_IC1F) >> 4U);
}


/**
* @brief  ������ģʽ��TRIG_OUT���������
* @param  timx TIM����
* @param  trigout_mode ��ģʽ�����������
*             @arg TIM_TRIG_OUT_RESET
*             @arg TIM_TRIG_OUT_ENABLE
*             @arg TIM_TRIG_OUT_UPDATE
*             @arg TIM_TRIG_OUT_CC1      
*             @arg TIM_TRIG_OUT_OC1REF   
*             @arg TIM_TRIG_OUT_OC2REF   
*             @arg TIM_TRIG_OUT_OC3REF   
*             @arg TIM_TRIG_OUT_OC4REF
* @retval ��
*/
__STATIC_INLINE void std_tim_trigout_mode_config(TIM_t *timx, uint32_t trigout_mode)
{
    MODIFY_REG(timx->CR2, TIM_CR2_MM_SEL, trigout_mode);
}


/**
* @brief  ���ô�ģʽ����
* @param  timx TIM����
* @param  slave_mode ��ģʽ��������
*             @arg TIM_SLAVE_MODE_DISABLE 
*             @arg TIM_SLAVE_MODE_RESET   
*             @arg TIM_SLAVE_MODE_GATED   
*             @arg TIM_SLAVE_MODE_TRIG    
* @retval ��
*/
__STATIC_INLINE void std_tim_slave_mode_config(TIM_t *timx, uint32_t slave_mode)
{
    MODIFY_REG(timx->SMC, TIM_SMC_SM_SEL, slave_mode);
}

/**
* @brief  ���ô�������Դ
* @param  timx TIM����
* @param  trig_source ��������Դ����
*             @arg TIM_TRIG_SOURCE_ITR0    
*             @arg TIM_TRIG_SOURCE_TI1F_ED
*             @arg TIM_TRIG_SOURCE_TI1FP1 
*             @arg TIM_TRIG_SOURCE_TI2FP2  
* @retval ��
*/
__STATIC_INLINE void std_tim_trig_source_config(TIM_t *timx, uint32_t trig_source)
{
    MODIFY_REG(timx->SMC, TIM_SMC_TS, trig_source);
}


/**
* @brief  ʹ����/��ģʽ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_master_mode_enable(TIM_t *timx)
{
    timx->SMC |= TIM_SMC_MS_MOD;
}


/**
* @brief  ��ֹ��/��ģʽ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_master_mode_disable(TIM_t *timx)
{
    timx->SMC &= (~TIM_SMC_MS_MOD);
}


/**
* @brief  ����TIM OCREF CLEAR����Դ
* @param  timx TIM����
* @param  ocrefclr_source OCREF CLRԴѡ��
*             @arg TIM_CLEAR_INPUT_SRC_COMP1: OCREF CLR���ӵ�COMP1
*             @arg TIM_CLEAR_INPUT_SRC_COMP2: OCREF CLR���ӵ�COMP2
* @retval ��
*/
__STATIC_INLINE void std_tim_ocrefclr_source_config(TIM_t *timx, uint32_t ocrefclr_source)
{
    MODIFY_REG(timx->CFG, TIM_CFG_OCREF_CLR, ocrefclr_source);
}


/**
* @brief  ʹ�����벶��/����Ƚϻ���Ԥװ��
* @param  timx TIM����
* @note   �ú������Ի���ͨ����Ч
* @retval ��
*/
__STATIC_INLINE void std_tim_ccreload_enable(TIM_t *timx)
{
    timx->CR2 |= TIM_CR2_CC_PRECR;
}

/**
* @brief  ��ֹ���벶��/����Ƚϻ���Ԥװ��
* @param  timx TIM����
* @note   �ú������Ի���ͨ����Ч
* @retval ��
*/
__STATIC_INLINE void std_tim_ccreload_disable(TIM_t *timx)
{
    timx->CR2 &= (~TIM_CR2_CC_PRECR);
}


/**
* @brief  ���û�����Ƹ���Դѡ��
* @param  timx TIM����
* @param  ccupdate_source �������Դ
*             @arg TIM_COM_SOFTWARE
*             @arg TIM_COM_TRGI
* @note   �ú������Ի���ͨ����Ч
* @retval ��
*/
__STATIC_INLINE void std_tim_cc_set_update_source(TIM_t *timx, uint32_t ccupdate_source)
{
    MODIFY_REG(timx->CR2, TIM_CR2_CCU_SEL, ccupdate_source);
}



/**
* @brief  ʹ��TI1��XOR����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ti1xor_enable(TIM_t *timx)
{
    timx->CR2 |= TIM_CR2_TI1_XOR_SEL;
}


/**
* @brief  ��ֹTI1��XOR����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ti1xor_disable(TIM_t *timx)
{
    timx->CR2 &= (~TIM_CR2_TI1_XOR_SEL);
}


/**
* @brief  ����TIM����ͨ����ӳ�书��
* @param  timx TIM����
* @param  ti_sel ͨ������Դѡ�����
*             @arg TIM_TIM3_TI1_GPIO:           TIM3 TI1���ӵ�GPIO
*             @arg TIM_TIM3_TI1_COMP1:          TIM3 TI1���ӵ�COMP1���
*             @arg TIM_TIM3_TI2_GPIO:           TIM3 TI2���ӵ�GPIO
*             @arg TIM_TIM3_TI2_COMP2:          TIM3 TI2���ӵ�COMP2���
* @param  channel_id TIMͨ��
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval ��
*/
__STATIC_INLINE void std_tim_set_channel_remap(TIM_t *timx, uint32_t ti_sel, uint32_t channel_id)
{
    MODIFY_REG(timx->TISEL, (TIM_TISEL_TI1_SEL << (channel_id << 3)), ti_sel);
}



/**
* @brief  TIM���ʹ��
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_moen_enable(TIM_t *timx)            
{
    timx->BDT |= (TIM_BDT_MOEN);
}

/**
* @brief  TIM�����ֹ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_moen_disable(TIM_t *timx)        
{
    timx->BDT &= (~TIM_BDT_MOEN);
}

/**
* @brief  TIM�Զ����ʹ��
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_aoen_enable(TIM_t *timx)            
{
    timx->BDT |= (TIM_BDT_AOEN);
}

/**
* @brief  TIM�Զ������ֹ
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_aoen_disable(TIM_t *timx)        
{
    timx->BDT &= (~TIM_BDT_AOEN);
}


/** 
* @brief  ʹ��OSSR����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ossr_enable(TIM_t *timx)                
{
    timx->BDT |= TIM_BDT_OSSR;
}

/** 
* @brief  ��ֹOSSR����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ossr_disable(TIM_t *timx)                
{
    timx->BDT &= (~TIM_BDT_OSSR);
}

/** 
* @brief  ʹ��OSSI����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ossi_enable(TIM_t *timx)                
{
    timx->BDT |= TIM_BDT_OSSI;
}

/** 
* @brief  ��ֹOSSI����
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_ossi_disable(TIM_t *timx)                
{
    timx->BDT &= (~TIM_BDT_OSSI);
}


/** 
* @brief  ʹ��TIM��·
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_bken_enable(TIM_t *timx)                
{
    timx->BDT |= TIM_BDT_BKEN;
}

/** 
* @brief  ��ֹTIM��·
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_bken_disable(TIM_t *timx)                
{
    timx->BDT &= (~TIM_BDT_BKEN);
}


/** 
* @brief  ʹ�ܶ�·Դ
* @param  timx TIM����
* @param  brk_source ��·����Դ����
*             @arg TIM_BREAK_INPUT_SRC_GPIO
*             @arg TIM_BREAK_INPUT_SRC_COMP1
*             @arg TIM_BREAK_INPUT_SRC_COMP2
* @retval ��
*/
__STATIC_INLINE void std_tim_brk_source_enable(TIM_t *timx, uint32_t brk_source)
{
    timx->AF1 |= brk_source;
}


/** 
* @brief  ���ö�·�ļ���
* @param  timx TIM����
* @param  brk_source ��·����Դ����
*             @arg TIM_BREAK_INPUT_SRC_GPIO
*             @arg TIM_BREAK_INPUT_SRC_COMP1
*             @arg TIM_BREAK_INPUT_SRC_COMP2
* @param  brk_pol ��·���뼫�Զ���
*             @arg TIM_BREAK_INPUT_POL_HIGH
*             @arg TIM_BREAK_INPUT_POL_LOW
* @retval ��
*/
__STATIC_INLINE void std_tim_set_brk_pol(TIM_t *timx, uint32_t brk_source, uint32_t brk_pol)
{   
    MODIFY_REG(timx->AF1, (TIM1_AF1_BKINP << (brk_source >> 1U)), (brk_pol << (brk_source >> 1U)));
}


/**
* @brief  ����TIM����ʱ��
* @param  timx TIM����
* @param  deadtime ����ʱ�䣬��ֵ�ķ�Χ:0x00~0xFF
* @retval ��
*/
__STATIC_INLINE void std_tim_set_deadtime(TIM_t *timx, uint32_t deadtime)
{
    MODIFY_REG(timx->BDT, TIM_BDT_DTG, deadtime);
}


/**
* @brief  ��ȡTIM����ʱ��
* @param  timx TIM����
* @retval uint32_t ����ʱ�䣬��ֵ��ΧΪ:0x00~0xFF
*/
__STATIC_INLINE uint32_t std_tim_get_deadtime(TIM_t *timx)
{
    return (timx->BDT & TIM_BDT_DTG);
}


/**
* @brief  ����TIM����������
* @param  timx TIM����
* @param  locklevel LOCK��������
*             @arg TIM_LOCK_LEVEL_OFF
*             @arg TIM_LOCK_LEVEL1
*             @arg TIM_LOCK_LEVEL2
*             @arg TIM_LOCK_LEVEL3
* @retval ��
*/
__STATIC_INLINE void std_tim_set_locklevel(TIM_t *timx, uint32_t locklevel)
{
    MODIFY_REG(timx->BDT, TIM_BDT_LOCK, locklevel);
}


/**
* @brief  ��ȡTIM��������
* @param  timx TIM����
* @retval uint32_t LOCK��������
*             @arg TIM_LOCK_LEVEL_OFF
*             @arg TIM_LOCK_LEVEL1
*             @arg TIM_LOCK_LEVEL2
*             @arg TIM_LOCK_LEVEL3
*/
__STATIC_INLINE uint32_t std_tim_get_locklevel(TIM_t *timx)
{
    return (timx->BDT & TIM_BDT_LOCK);
}


/** 
* @brief  ʹ��LOCKUP��������
* @param  timx TIM����
* @retval ��
*/
__STATIC_INLINE void std_tim_lockup_lock_enable(TIM_t *timx)
{
    timx->AF1 |= TIM1_AF1_LOCKUP_LOCK;
}




/* �����������ܳ�ʼ��/ȥ��ʼ�� */
void std_tim_deinit(TIM_t *timx);
void std_tim_init(TIM_t *timx, std_tim_basic_init_t *tim_init_param);
void std_tim_struct_init(std_tim_basic_init_t *tim_init_struct);

/* ���벶���ʼ�� */
void std_tim_input_capture_init(TIM_t *timx, std_tim_input_capture_init_t *input_config, uint32_t channel_id);
void std_tim_input_capture_struct_init(std_tim_input_capture_init_t *input_init_struct);

/* ���ģʽ��ʼ�����������ú��� */
void std_tim_output_compare_init(TIM_t *timx, std_tim_output_compare_init_t *output_config, uint32_t channel_id);
void std_tim_output_compare_struct_init(std_tim_output_compare_init_t *output_init_struct);

/* ��·���ܳ�ʼ�� */
void std_tim_bdt_init(TIM_t* timx, std_tim_break_init_t *bdt_init_param);
void std_tim_bdt_struct_init(std_tim_break_init_t *bdt_init_struct);


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
#endif /* CIU32F003_STD_TIM_H */

