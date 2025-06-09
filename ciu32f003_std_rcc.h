/************************************************************************************************/
/**
* @file               ciu32f003_std_rcc.h
* @author             MCU Ecosystem Development Team
* @brief              RCC STD������ͷ�ļ���
*                     �ṩRCC��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*����ͷ�ļ��ظ�����*/
#ifndef CIU32F003_STD_RCC_H
#define CIU32F003_STD_RCC_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup RCC RCC
* @brief ��λ��ʱ�ӿ�������STD������
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
* @defgroup RCC_Constants RCC Constants 
* @brief  RCC�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* ϵͳʱ��Դѡ�� */
#define RCC_SYSCLK_SRC_RCHDIV6         RCC_CFG_SYSW_RCHDIV6                         /**< ѡ��RCHDIV6��Ϊϵͳʱ��Դ   */
#define RCC_SYSCLK_SRC_RCHDIV3         RCC_CFG_SYSW_RCHDIV3                         /**< ѡ��RCHDIV3��Ϊϵͳʱ��Դ   */
#define RCC_SYSCLK_SRC_RCH             RCC_CFG_SYSW_RCH                             /**< ѡ��RCH��Ϊϵͳʱ��Դ       */
#define RCC_SYSCLK_SRC_RCL             RCC_CFG_SYSW_RCL                             /**< ѡ��RCL��Ϊϵͳʱ��Դ       */
#define RCC_SYSCLK_SRC_EXTCLK          RCC_CFG_SYSW_EXTCLK                          /**< ѡ��EXTCLK��Ϊϵͳʱ��Դ    */

/* ϵͳʱ��Դ״̬���� */
#define RCC_SYSCLK_SRC_STATUS_RCHDIV6      RCC_CFG_SYSWS_RCHDIV6                    /**< ϵͳʱ��ΪRCHDIV6  */
#define RCC_SYSCLK_SRC_STATUS_RCHDIV3      RCC_CFG_SYSWS_RCHDIV3                    /**< ϵͳʱ��ΪRCHDIV3  */
#define RCC_SYSCLK_SRC_STATUS_RCH          RCC_CFG_SYSWS_RCH                        /**< ϵͳʱ��ΪRCH      */
#define RCC_SYSCLK_SRC_STATUS_RCL          RCC_CFG_SYSWS_RCL                        /**< ϵͳʱ��ΪRCL      */
#define RCC_SYSCLK_SRC_STATUS_EXTCLK       RCC_CFG_SYSWS_EXTCLK                     /**< ϵͳʱ��ΪEXTCLK   */

/* AHBʱ�ӷ�Ƶ�������� */
#define RCC_HCLK_DIV1                      RCC_CFG_HPRE_1                           /**< HCLK����Ƶ         */
#define RCC_HCLK_DIV2                      RCC_CFG_HPRE_2                           /**< HCLK��Ƶ���� = 2   */
#define RCC_HCLK_DIV4                      RCC_CFG_HPRE_4                           /**< HCLK��Ƶ���� = 4   */
#define RCC_HCLK_DIV8                      RCC_CFG_HPRE_8                           /**< HCLK��Ƶ���� = 8   */
#define RCC_HCLK_DIV16                     RCC_CFG_HPRE_16                          /**< HCLK��Ƶ���� = 16  */
#define RCC_HCLK_DIV32                     RCC_CFG_HPRE_32                          /**< HCLK��Ƶ���� = 32  */
#define RCC_HCLK_DIV64                     RCC_CFG_HPRE_64                          /**< HCLK��Ƶ���� = 64  */
#define RCC_HCLK_DIV128                    RCC_CFG_HPRE_128                         /**< HCLK��Ƶ���� = 128 */

/* APBʱ�ӷ�Ƶ�������� */
#define RCC_PCLK_DIV1                      RCC_CFG_PPRE_1                           /**< APB����ʱ�Ӳ���Ƶ        */
#define RCC_PCLK_DIV2                      RCC_CFG_PPRE_2                           /**< APB����ʱ�ӷ�Ƶ���� = 2  */
#define RCC_PCLK_DIV4                      RCC_CFG_PPRE_4                           /**< APB����ʱ�ӷ�Ƶ���� = 4  */
#define RCC_PCLK_DIV8                      RCC_CFG_PPRE_8                           /**< APB����ʱ�ӷ�Ƶ���� = 8  */
#define RCC_PCLK_DIV16                     RCC_CFG_PPRE_16                          /**< APB����ʱ�ӷ�Ƶ���� = 16 */
        
/* MCOʱ��Դ�������� */                                                                            
#define RCC_MCO_SRC_DISABLE                RCC_CFG_MCOSEL_DISABLE                   /**< MCO�����Ч         */
#define RCC_MCO_SRC_SYSCLK                 RCC_CFG_MCOSEL_SYSCLK                    /**< MCOѡ��SYSCLK���   */
#define RCC_MCO_SRC_RCHDIV6                RCC_CFG_MCOSEL_RCHDIV6                   /**< MCOѡ��RCHDIV6���  */
#define RCC_MCO_SRC_EXTCLK                 RCC_CFG_MCOSEL_EXTCLK                    /**< MCOѡ��EXTCLK���   */
#define RCC_MCO_SRC_RCL                    RCC_CFG_MCOSEL_RCL                       /**< MCOѡ��RCL���      */

/* MCOʱ�ӷ�Ƶ�������� */
#define RCC_MCO_DIV1                       RCC_CFG_MCOPRE_DIV1                      /**< MCO����Ƶ         */
#define RCC_MCO_DIV2                       RCC_CFG_MCOPRE_DIV2                      /**< MCO��Ƶ���� = 2   */
#define RCC_MCO_DIV4                       RCC_CFG_MCOPRE_DIV4                      /**< MCO��Ƶ���� = 4   */
#define RCC_MCO_DIV8                       RCC_CFG_MCOPRE_DIV8                      /**< MCO��Ƶ���� = 8   */
#define RCC_MCO_DIV16                      RCC_CFG_MCOPRE_DIV16                     /**< MCO��Ƶ���� = 16  */
#define RCC_MCO_DIV32                      RCC_CFG_MCOPRE_DIV32                     /**< MCO��Ƶ���� = 32  */
#define RCC_MCO_DIV64                      RCC_CFG_MCOPRE_DIV64                     /**< MCO��Ƶ���� = 64  */
#define RCC_MCO_DIV128                     RCC_CFG_MCOPRE_DIV128                    /**< MCO��Ƶ���� = 128 */

/* RCC�жϿ���λ���� */
#define RCC_INTERRUPT_RCL_READY            RCC_IER_RCL_RDYIE                        /**< RCL Ready���жϿ���λ   */
#define RCC_INTERRUPT_RCH_READY            RCC_IER_RCH_RDYIE                        /**< RCH Ready���жϿ���λ   */

/* RCC�жϱ�־λ���� */
#define RCC_FLAG_RCL_READY                 RCC_ISR_RCL_RDYF                         /**< RCL Ready���жϱ�־λ   */
#define RCC_FLAG_RCH_READY                 RCC_ISR_RCH_RDYF                         /**< RCH Ready���жϱ�־λ   */

/* RCC�������λ���� */
#define RCC_CLEAR_RCL_READY                RCC_ICR_RCL_RDYC                         /**< RCL Ready���������λ   */
#define RCC_CLEAR_RCH_READY                RCC_ICR_RCH_RDYC                         /**< RCH Ready���������λ   */

/* IO�˿�ʱ��ѡ�� */
#define RCC_PERIPH_CLK_GPIOA              RCC_IOPEN_GPIOAEN                        /**< GPIOA ʱ�ӿ���λ  */
#define RCC_PERIPH_CLK_GPIOB              RCC_IOPEN_GPIOBEN                        /**< GPIOB ʱ�ӿ���λ  */
#define RCC_PERIPH_CLK_GPIOC              RCC_IOPEN_GPIOCEN                        /**< GPIOC ʱ�ӿ���λ  */

/* IO�˿ڸ�λѡ�� */
#define RCC_PERIPH_RESET_GPIOA            RCC_IOPRST_GPIOA_RST                            /**< GPIOA�˿ڸ�λ����λ  */
#define RCC_PERIPH_RESET_GPIOB            RCC_IOPRST_GPIOB_RST                            /**< GPIOB�˿ڸ�λ����λ  */
#define RCC_PERIPH_RESET_GPIOC            RCC_IOPRST_GPIOC_RST                            /**< GPIOC�˿ڸ�λ����λ  */

/* AHB����ʱ��ѡ�� */
#define RCC_PERIPH_CLK_CRC                RCC_AHBEN_CRCEN                                 /**< AHB����CRCʱ�ӿ���λ  */

/* AHB���踴λѡ�� */
#define RCC_PERIPH_RESET_CRC              RCC_AHBRST_CRC_RST                              /**< AHB����CRC��λ����λ  */

/* APB����ʱ��ѡ�� */
#define RCC_PERIPH_CLK_TIM3               RCC_APBEN1_TIM3EN                               /**< APB����TIM3ʱ�ӿ���λ    */
#define RCC_PERIPH_CLK_UART2              RCC_APBEN1_UART2EN                              /**< APB����UART2ʱ�ӿ���λ   */
#define RCC_PERIPH_CLK_I2C1               RCC_APBEN1_I2C1EN                               /**< APB����I2C1ʱ�ӿ���λ    */
#define RCC_PERIPH_CLK_PMU                RCC_APBEN1_PMUEN                                /**< APB����PMUʱ�ӿ���λ     */
#define RCC_PERIPH_CLK_LPTIM1             RCC_APBEN1_LPTIM1EN                             /**< APB����LPTIM1ʱ�ӿ���λ  */

#define RCC_PERIPH_CLK_COMP               RCC_APBEN2_COMPEN                               /**< APB����COMPʱ�ӿ���λ    */
#define RCC_PERIPH_CLK_TIM1               RCC_APBEN2_TIM1EN                               /**< APB����TIM1ʱ�ӿ���λ    */
#define RCC_PERIPH_CLK_SPI1               RCC_APBEN2_SPI1EN                               /**< APB����SPI1ʱ�ӿ���λ    */
#define RCC_PERIPH_CLK_UART1              RCC_APBEN2_UART1EN                              /**< APB����UART1ʱ�ӿ���λ   */
#define RCC_PERIPH_CLK_ADC                RCC_APBEN2_ADCEN                                /**< APB����ADCʱ�ӿ���λ     */
#define RCC_PERIPH_CLK_DBG                RCC_APBEN2_DBGEN                                /**< APB����DBGʱ�ӿ���λ     */

/* APB���踴λѡ�� */
#define RCC_PERIPH_RESET_TIM3             RCC_APBRST1_TIM3_RST                            /**< APB����TIM3��λ����λ    */
#define RCC_PERIPH_RESET_UART2            RCC_APBRST1_UART2_RST                           /**< APB����UART2��λ����λ   */
#define RCC_PERIPH_RESET_I2C1             RCC_APBRST1_I2C1_RST                            /**< APB����I2C1��λ����λ    */
#define RCC_PERIPH_RESET_LPTIM1           RCC_APBRST1_LPTIM1_RST                          /**< APB����LPTIM1��λ����λ  */
                                                                                                                                                                  
#define RCC_PERIPH_RESET_COMP             RCC_APBRST2_COMP_RST                            /**< APB����COMP��λ��λ      */
#define RCC_PERIPH_RESET_TIM1             RCC_APBRST2_TIM1_RST                            /**< APB����TIM1��λ����λ    */
#define RCC_PERIPH_RESET_SPI1             RCC_APBRST2_SPI1_RST                            /**< APB����SPI1��λ����λ    */
#define RCC_PERIPH_RESET_UART1            RCC_APBRST2_UART1_RST                           /**< APB����UART��λ����λ    */
#define RCC_PERIPH_RESET_ADC              RCC_APBRST2_ADC_RST                             /**< APB����ADC��λ����λ     */
#define RCC_PERIPH_RESET_DBG              RCC_APBRST2_DBG_RST                             /**< APB����DBG��λ����λ     */

/* LPTIM1�첽ʱ��Դѡ�� */
#define RCC_LPTIM1_ASYNC_CLK_SRC_PCLK         RCC_CLKSEL_LPTIM1_SEL_PCLK             /**< PCLKʱ����ΪLPTIM1ʱ��      */
#define RCC_LPTIM1_ASYNC_CLK_SRC_RCL          RCC_CLKSEL_LPTIM1_SEL_RCL              /**< RCLʱ����ΪLPTIM1ʱ��       */
#define RCC_LPTIM1_ASYNC_CLK_SRC_MCO          RCC_CLKSEL_LPTIM1_SEL_MCO              /**< MCOʱ����ΪLPTIM1ʱ��       */

/* COMP1�첽ʱ��Դѡ�� */
#define RCC_COMP1_ASYNC_CLK_SRC_PCLK          RCC_CLKSEL_COMP1_SEL_PCLK             /**< PCLKʱ����ΪCOMP1ʱ��      */
#define RCC_COMP1_ASYNC_CLK_SRC_RCL           RCC_CLKSEL_COMP1_SEL_RCL              /**< RCLʱ����ΪCOMP1ʱ��       */

/* COMP2�첽ʱ��Դѡ�� */
#define RCC_COMP2_ASYNC_CLK_SRC_PCLK          RCC_CLKSEL_COMP2_SEL_PCLK             /**< PCLKʱ����ΪCOMP2ʱ��      */
#define RCC_COMP2_ASYNC_CLK_SRC_RCL           RCC_CLKSEL_COMP2_SEL_RCL              /**< RCLʱ����ΪCOMP2ʱ��       */

/* ��λ��־���� */
#define RCC_RESET_FLAG_LOCKUP                   RCC_CSR2_LOCKUP_RSTF                  /**< LOCKUP��λ��־           */
#define RCC_RESET_FLAG_NRST                     RCC_CSR2_NRST_RSTF                    /**< NRST���Ÿ�λ��־         */
#define RCC_RESET_FLAG_PMU                      RCC_CSR2_PMU_RSTF                     /**< POR/PDR��BOR��λ��־     */
#define RCC_RESET_FLAG_SW                       RCC_CSR2_SW_RSTF                      /**< �����λ��־             */
#define RCC_RESET_FLAG_IWDG                     RCC_CSR2_IWDG_RSTF                    /**< IWDG��λ��־             */
#define RCC_RESET_FLAG_LPM                      RCC_CSR2_LPM_RSTF                     /**< �͹��ĸ�λ��־           */
#define RCC_RESET_FLAG_ALL                     (0xFFUL<<RCC_CSR2_LOCKUP_RSTF_POS)     /**< ���и�λ��־             */


/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup RCC_External_Functions RCC External Functions
* @brief    RCC���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ʹ��RCHʱ��
* @note   ʹ��RCH�����Ӧ�ȴ�RCHRDY��־��λ����ʹ�ø�ʱ��
* @retval ��
*/
__STATIC_INLINE void std_rcc_rch_enable(void)
{
    RCC->CSR1 |= RCC_CSR1_RCHON;
} 

/** 
* @brief  �ر�RCHʱ��
* @note   ���RCHΪϵͳʱ��Դ�����䲻�ܱ�ֹͣ����ʱ�û�Ӧ�Ƚ�ϵͳʱ��Դ�л�Ϊ����ʱ�ӣ��ٹرո�ʱ��Դ
* @note   ���ر�RCHʱ����ȴ�RCHRDY��־����
* @retval ��
*/
__STATIC_INLINE void std_rcc_rch_disable(void)
{
    RCC->CSR1 &= (~RCC_CSR1_RCHON);
}

/** 
* @brief  ��ȡRCH ready��־
* @retval uint32_t ����RCH RDY��־״̬
*             @arg ��0�� ��ʾRCH ready����λ
*             @arg 0����ʾRCH readyδ��λ
*/
__STATIC_INLINE uint32_t std_rcc_get_rch_ready(void) 
{
    return(RCC->CSR1 & RCC_CSR1_RCHRDY);
}        

/** 
* @brief  ����ϵͳʱ��Դ
* @param  clocksource ϵͳʱ��Դ
*             @arg RCC_SYSCLK_SRC_RCHDIV6
*             @arg RCC_SYSCLK_SRC_RCHDIV3   
*             @arg RCC_SYSCLK_SRC_RCH    
*             @arg RCC_SYSCLK_SRC_RCL     
*             @arg RCC_SYSCLK_SRC_EXTCLK
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_sysclk_source(uint32_t clocksource)
{
    MODIFY_REG(RCC->CFG, RCC_CFG_SYSW, clocksource);
}    

/** 
* @brief  ��ȡϵͳʱ��Դ��Ϣ
* @retval uint32_t ϵͳʱ��Դ
*             @arg RCC_SYSCLK_SRC_STATUS_RCHDIV6  
*             @arg RCC_SYSCLK_SRC_STATUS_RCHDIV3   
*             @arg RCC_SYSCLK_SRC_STATUS_RCH    
*             @arg RCC_SYSCLK_SRC_STATUS_RCL    
*             @arg RCC_SYSCLK_SRC_STATUS_EXTCLK 
*/
__STATIC_INLINE uint32_t std_rcc_get_sysclk_source(void)
{
    return(RCC->CFG & RCC_CFG_SYSWS);
}    

/** 
* @brief  ����AHBʱ�ӷ�Ƶ����
* @param  ahb_div AHB��Ƶ����
*             @arg RCC_HCLK_DIV1   
*             @arg RCC_HCLK_DIV2     
*             @arg ...   
*             @arg RCC_HCLK_DIV128
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_ahbdiv(uint32_t ahb_div)
{
    MODIFY_REG(RCC->CFG, RCC_CFG_HPRE, ahb_div);
}    

/** 
* @brief  ��ȡAHBʱ�ӷ�Ƶ����
* @retval uint32_t HCLKʱ�ӷ�Ƶ����
*             @arg RCC_HCLK_DIV1   
*             @arg RCC_HCLK_DIV2     
*             @arg ...   
*             @arg RCC_HCLK_DIV128
*/
__STATIC_INLINE uint32_t std_rcc_get_ahbdiv(void)
{
    return(RCC->CFG & RCC_CFG_HPRE);
} 

/** 
* @brief  ����APBʱ�ӷ�Ƶ����
* @param  apb_div APB��Ƶ����
*             @arg RCC_PCLK_DIV1   
*             @arg RCC_PCLK_DIV2     
*             @arg ...   
*             @arg RCC_PCLK_DIV16
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_apbdiv(uint32_t apb_div)
{
    MODIFY_REG(RCC->CFG, RCC_CFG_PPRE, apb_div);
}    

/** 
* @brief  ��ȡAPBʱ�ӷ�Ƶ����
* @retval uint32_t PCLK1ʱ�ӷ�Ƶ����
*             @arg RCC_PCLK_DIV1   
*             @arg RCC_PCLK_DIV2     
*             @arg ...   
*             @arg RCC_PCLK_DIV16
*/
__STATIC_INLINE uint32_t std_rcc_get_apbdiv(void)
{
    return(RCC->CFG & RCC_CFG_PPRE);
} 

/**
* @brief  ����MCO�����Ϣ
* @param  mco_source MCO���Դѡ��
*             @arg RCC_MCO_SRC_SYSCLK
*             @arg RCC_MCO_SRC_RCHDIV6
*             @arg RCC_MCO_SRC_EXTCLK
*             @arg RCC_MCO_SRC_RCL
* @param  mco_psc MCO�����Ƶ����
*             @arg RCC_MCO_DIV1
*             @arg RCC_MCO_DIV2
*             @arg ...
*             @arg RCC_MCO_DIV128
* @retval ��
*/
__STATIC_INLINE void std_rcc_mco_config(uint32_t mco_source, uint32_t mco_psc)
{
    MODIFY_REG(RCC->CFG, RCC_CFG_MCOSEL | RCC_CFG_MCOPRE, mco_source | mco_psc);
}

/** 
* @brief  ʹ��EXTCLKʱ��
* @retval ��
*/
__STATIC_INLINE void std_rcc_extclk_enable(void) 
{
    RCC->CSR1 |= RCC_CSR1_EXTCLKON;     
}

/** 
* @brief  �ر�EXTCLKʱ��
* @retval ��
*/
__STATIC_INLINE void std_rcc_extclk_disable(void)
{
    RCC->CSR1 &= (~RCC_CSR1_EXTCLKON);
} 

/** 
* @brief  ʹ��RCC�ж�
* @param  interrupt ʹ��RCC�ж�Դ����Ϣ
*             @arg  RCC_INTERRUPT_RCL_READY
*             @arg  RCC_INTERRUPT_RCH_READY
* @retval ��
*/
__STATIC_INLINE void std_rcc_interrupt_enable(uint32_t interrupt)     
{
    RCC->IER |= (interrupt);
}

/** 
* @brief  �ر�RCC�ж�
* @param  interrupt �ر�RCC�ж�Դ����Ϣ
*             @arg  RCC_INTERRUPT_RCL_READY
*             @arg  RCC_INTERRUPT_RCH_READY
* @retval ��
*/
__STATIC_INLINE void std_rcc_interrupt_disable(uint32_t interrupt)    
{
    RCC->IER &= (~(interrupt));
} 

/**
* @brief  ��ȡRCC�ж�ʹ��״̬
* @param  interrupt RCC�ж�Դ��Ϣ
*             @arg  RCC_INTERRUPT_RCL_READY
*             @arg  RCC_INTERRUPT_RCH_READY
* @retval uint32_t ���������жϱ�־��λ״̬
*             @arg ��0�� ��ʾָ�����жϴ���ʹ��״̬
*             @arg 0����ʾָ�����жϴ��ڽ�ֹ״̬
*/
__STATIC_INLINE uint32_t std_rcc_get_interrupt_enable(uint32_t interrupt)    
{
    return(RCC->IER & (interrupt));
}

/**
* @brief  ��ȡRCC�жϱ�־״̬
* @param  flag RCC�жϱ�־��Ϣ
*             @arg  RCC_FLAG_RCL_READY
*             @arg  RCC_FLAG_RCH_READY
* @retval uint32_t ���������жϱ�־��λ״̬
*             @arg ��0�� ��ʾָ�����ж�����λ
*             @arg 0����ʾָ�����ж�δ��λ
*/
__STATIC_INLINE uint32_t std_rcc_get_flag(uint32_t flag) 
{
    return(RCC->ISR & (flag));
}

/** 
* @brief  ���RCC��־
* @param  flags �����־λ
*             @arg  RCC_CLEAR_RCL_READY
*             @arg  RCC_CLEAR_RCH_READY
* @retval ��
*/
__STATIC_INLINE void std_rcc_clear_flag(uint32_t flags)   
{
    RCC->ICR = (flags);
}

/** 
* @brief  ʹ��RCLʱ��
* @note   ʹ��RCL�����Ӧ�ȴ�RCLRDY��־��λ����ʹ�ø�ʱ��
* @retval ��
*/
__STATIC_INLINE void std_rcc_rcl_enable(void)
{
    RCC->CSR2 |= RCC_CSR2_RCLON;
}          

/** 
* @brief  �ر�RCLʱ��
* @note   ���RCLΪϵͳʱ��Դ�����䲻�ܱ�ֹͣ����ʱ�û�Ӧ�Ƚ�ϵͳʱ��Դ�л�Ϊ����ʱ�ӣ��ٹرո�ʱ��Դ
* @note   ���ر�RCLʱ��RCLRDY��־����RCL�رպ��Զ�����
* @retval ��
*/
__STATIC_INLINE void std_rcc_rcl_disable(void) 
{
    RCC->CSR2 &= (~RCC_CSR2_RCLON);
}        

/** 
* @brief  ��ȡRCL ready��־
* @retval uint32_t ����RCL RDY��־״̬
*             @arg ��0�� ��ʾRCL ready����λ
*             @arg 0����ʾRCL readyδ��λ
*/
__STATIC_INLINE uint32_t std_rcc_get_rcl_ready(void)          
{
    return(RCC->CSR2 & RCC_CSR2_RCLRDY);
}

/** 
* @brief  ��ȡ��λ��־
* @param  reset_flag ָ��Ҫ��ȡ�ĸ�λ��־
*             @arg RCC_RESET_FLAG_LOCKUP
*             @arg RCC_RESET_FLAG_NRST  
*             @arg RCC_RESET_FLAG_PMU   
*             @arg RCC_RESET_FLAG_SW    
*             @arg RCC_RESET_FLAG_IWDG  
*             @arg RCC_RESET_FLAG_LPM   
*             @arg RCC_RESET_FLAG_ALL   
* @retval uint32_t ����״̬��־
*             @arg ��0�� ��ʾָ���ĸ�λ��־����λ
*             @arg 0����ʾָ���ĸ�λ��־δ��λ
*/
__STATIC_INLINE uint32_t std_rcc_get_reset_flag(uint32_t reset_flag)            
{
    return(RCC->CSR2 & (reset_flag));
} 

/** 
* @brief  �����λ��־
* @note   �ú�����������и�λ��־
* @retval ��
*/
__STATIC_INLINE void std_rcc_clear_reset_flags(void)           
{
    RCC->CSR2 |= RCC_CSR2_RMVF;
}

/** 
* @brief  GPIO�˿�ʱ��ʹ��
* @param  gpiox_clock ָ��ʹ�ܵ�GPIOʱ��
*             @arg RCC_PERIPH_CLK_GPIOA
*             @arg RCC_PERIPH_CLK_GPIOB
*             @arg RCC_PERIPH_CLK_GPIOC
* @retval ��
*/
__STATIC_INLINE void std_rcc_gpio_clk_enable(uint32_t gpiox_clock)
{
    RCC->IOPEN |= gpiox_clock; 
    
    /* RCC��Χʱ�����ú���ӳ� */  
    __NOP(); __NOP(); __NOP(); 
}

/** 
* @brief  GPIO�˿�ʱ�ӽ�ֹ
* @param  gpiox_clock ָ����ֹ��GPIOʱ��
*             @arg RCC_PERIPH_CLK_GPIOA
*             @arg RCC_PERIPH_CLK_GPIOB
*             @arg RCC_PERIPH_CLK_GPIOC
* @retval ��
*/
__STATIC_INLINE void std_rcc_gpio_clk_disable(uint32_t gpiox_clock) 
{
    RCC->IOPEN &= (~(gpiox_clock)); 
    
    /* RCC��Χʱ�����ú���ӳ� */  
    __NOP(); __NOP(); __NOP(); 
}

/** 
* @brief  GPIO�˿ڸ�λ
* @param  gpiox_rst ָ����λ��GPIO�˿�
*             @arg RCC_PERIPH_RESET_GPIOA
*             @arg RCC_PERIPH_RESET_GPIOB
*             @arg RCC_PERIPH_RESET_GPIOC
* @retval ��
*/
__STATIC_INLINE void std_rcc_gpio_reset(uint32_t gpiox_rst)
{
    RCC->IOPRST |= (gpiox_rst);
    RCC->IOPRST &= (~(gpiox_rst));
}         

/** 
* @brief  AHB����ʱ��ʹ��
* @param  periph_clock ָ��ʹ�ܵ�AHB����ʱ��
*             @arg RCC_PERIPH_CLK_CRC  
* @retval ��
*/
__STATIC_INLINE void std_rcc_ahb_clk_enable(uint32_t periph_clock)
{
    RCC->AHBEN |= periph_clock;
    
    /* RCC��Χʱ�����ú���ӳ� */    
    __NOP(); __NOP(); __NOP(); 
}   

/** 
* @brief  AHB����ʱ�ӽ�ֹ
* @param  periph_clock ָ����ֹ��AHB����ʱ��
*             @arg RCC_PERIPH_CLK_CRC  
* @retval ��
*/
__STATIC_INLINE void std_rcc_ahb_clk_disable(uint32_t periph_clock)            
{
    RCC->AHBEN &= (~(periph_clock));
}

/** 
* @brief  AHB���踴λ
* @param  periph_rst ָ����λ��AHB����
*             @arg RCC_PERIPH_RESET_CRC
* @retval ��
*/
__STATIC_INLINE void std_rcc_ahb_reset(uint32_t periph_rst)
{
    RCC->AHBRST |= (periph_rst);
    RCC->AHBRST &= (~(periph_rst));
}       

/** 
* @brief  APB����1ʱ��ʹ��
* @param  periph_clock ָ��ʹ�ܵ�APB����1ʱ��
*             @arg RCC_PERIPH_CLK_TIM3   
*             @arg RCC_PERIPH_CLK_UART2 
*             @arg RCC_PERIPH_CLK_I2C1   
*             @arg RCC_PERIPH_CLK_PMU       
*             @arg RCC_PERIPH_CLK_LPTIM1 
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb1_clk_enable(uint32_t periph_clock)
{
    RCC->APBEN1 |= periph_clock;
    
    /* RCC��Χʱ�����ú���ӳ� */    
    __NOP(); __NOP(); __NOP(); 
}

/** 
* @brief  APB����1ʱ�ӽ�ֹ
* @param  periph_clock ָ����ֹ��APB����1ʱ��
*             @arg RCC_PERIPH_CLK_TIM3      
*             @arg RCC_PERIPH_CLK_UART2 
*             @arg RCC_PERIPH_CLK_I2C1   
*             @arg RCC_PERIPH_CLK_PMU      
*             @arg RCC_PERIPH_CLK_LPTIM1 
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb1_clk_disable(uint32_t periph_clock)            
{
    RCC->APBEN1 &= (~(periph_clock));
}

/** 
* @brief  APB����1��λ
* @param  periph_rst ָ����λ��APB����1
*             @arg RCC_PERIPH_RESET_TIM3     
*             @arg RCC_PERIPH_RESET_UART2 
*             @arg RCC_PERIPH_RESET_I2C1   
*             @arg RCC_PERIPH_RESET_LPTIM1 
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb1_reset(uint32_t periph_rst)
{
    RCC->APBRST1 |= (periph_rst);
    RCC->APBRST1 &= (~(periph_rst));
}       

/** 
* @brief  APB����2ʱ��ʹ��
* @param  periph_clock ָ��ʹ�ܵ�APB����2ʱ��
*             @arg RCC_PERIPH_CLK_COMP
*             @arg RCC_PERIPH_CLK_TIM1   
*             @arg RCC_PERIPH_CLK_SPI1   
*             @arg RCC_PERIPH_CLK_UART1 
*             @arg RCC_PERIPH_CLK_ADC    
*             @arg RCC_PERIPH_CLK_DBG
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb2_clk_enable(uint32_t periph_clock)
{
    RCC->APBEN2 |= periph_clock;
    
    /* RCC��Χʱ�����ú���ӳ� */    
    __NOP(); __NOP(); __NOP(); 
}

/** 
* @brief  APB����2ʱ�ӽ�ֹ
* @param  periph_clock ָ����ֹ��APB����2ʱ��
*             @arg RCC_PERIPH_CLK_COMP
*             @arg RCC_PERIPH_CLK_TIM1   
*             @arg RCC_PERIPH_CLK_SPI1   
*             @arg RCC_PERIPH_CLK_UART1 
*             @arg RCC_PERIPH_CLK_ADC    
*             @arg RCC_PERIPH_CLK_DBG
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb2_clk_disable(uint32_t periph_clock)            
{
    RCC->APBEN2 &= (~(periph_clock));
}

/** 
* @brief  APB����2��λ
* @param  periph_rst ָ����λ��APB����2
*             @arg RCC_PERIPH_RESET_COMP   
*             @arg RCC_PERIPH_RESET_TIM1   
*             @arg RCC_PERIPH_RESET_SPI1   
*             @arg RCC_PERIPH_RESET_UART1 
*             @arg RCC_PERIPH_RESET_ADC    
*             @arg RCC_PERIPH_RESET_DBG  
* @retval ��
*/
__STATIC_INLINE void std_rcc_apb2_reset(uint32_t periph_rst)
{
    RCC->APBRST2 |= (periph_rst);
    RCC->APBRST2 &= (~(periph_rst));
}       

/** 
* @brief  ʹ��LOCKUP��λ
* @retval ��
*/
__STATIC_INLINE void std_rcc_lockup_reset_enable(void)
{
    RCC->CSR2 |= RCC_CSR2_LOCKUP_RSTEN;
}

/** 
* @brief  ��ֹLOCKUP��λ
* @retval ��
*/
__STATIC_INLINE void std_rcc_lockup_reset_disable(void)
{
    RCC->CSR2 &= (~RCC_CSR2_LOCKUP_RSTEN);
}

/** 
* @brief  LPTIM1ʱ��Դѡ��
* @param  lptim1clk_select LPTIM1ʱ��Դ
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_PCLK
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_RCL
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_MCO
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_lptim1clk_source(uint32_t lptim1clk_select)
{
    MODIFY_REG(RCC->CLKSEL, RCC_CLKSEL_LPTIM1_SEL, (lptim1clk_select));
}       

/** 
* @brief  ��ȡLPTIM1ʱ��Դ
* @retval uint32_t ����LPTIM1ʱ��Դ��Ϣ
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_PCLK1
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_RCL
*             @arg RCC_LPTIM1_ASYNC_CLK_SRC_MCO
*/
__STATIC_INLINE uint32_t std_rcc_get_lptim1clk_source(void)
{
    return(RCC->CLKSEL & RCC_CLKSEL_LPTIM1_SEL);
} 

/** 
* @brief  COMP1ʱ��Դѡ��
* @param  comp1clk_select COMP1ʱ��Դ
*             @arg RCC_COMP1_ASYNC_CLK_SRC_PCLK
*             @arg RCC_COMP1_ASYNC_CLK_SRC_RCL
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_comp1clk_source(uint32_t comp1clk_select)
{
    MODIFY_REG(RCC->CLKSEL, RCC_CLKSEL_COMP1_SEL, (comp1clk_select));
}       

/** 
* @brief  ��ȡCOMP1ʱ��Դ
* @retval uint32_t ����COMP1ʱ��Դ��Ϣ
*             @arg RCC_COMP1_ASYNC_CLK_SRC_PCLK
*             @arg RCC_COMP1_ASYNC_CLK_SRC_RCL
*/
__STATIC_INLINE uint32_t std_rcc_get_comp1clk_source(void)
{
    return(RCC->CLKSEL & RCC_CLKSEL_COMP1_SEL);
} 

/** 
* @brief  COMP2ʱ��Դѡ��
* @param  comp2clk_select COMP2ʱ��Դ
*             @arg RCC_COMP2_ASYNC_CLK_SRC_PCLK
*             @arg RCC_COMP2_ASYNC_CLK_SRC_RCL
* @retval ��
*/
__STATIC_INLINE void std_rcc_set_comp2clk_source(uint32_t comp2clk_select)
{
    MODIFY_REG(RCC->CLKSEL, RCC_CLKSEL_COMP2_SEL, (comp2clk_select));
}       

/** 
* @brief  ��ȡCOMP2ʱ��Դ
* @retval uint32_t ����COMP2ʱ��Դ��Ϣ
*             @arg RCC_COMP2_ASYNC_CLK_SRC_PCLK
*             @arg RCC_COMP2_ASYNC_CLK_SRC_RCL
*/
__STATIC_INLINE uint32_t std_rcc_get_comp2clk_source(void)
{
    return(RCC->CLKSEL & RCC_CLKSEL_COMP2_SEL);
} 

/** 
* @brief  дRCLУ׼ֵ
* @param  cal_value RCLУ׼ֵ
* @retval ��
*/
__STATIC_INLINE void std_rcc_write_rcl_calibration(uint32_t cal_value)
{
    RCC->RCLCAL = cal_value;
}

/** 
* @brief  ��ȡRCLУ׼ֵ
* @retval uint32_t RCLУ׼ֵ
*/
__STATIC_INLINE uint32_t std_rcc_read_rcl_calibration(void)
{
    return(RCC->RCLCAL);
}

/** 
* @brief  дRCHʱ�Ӵֵ�ֵ
* @param  cal_value RCHʱ�Ӵֵ�ֵ
* @retval ��
*/
__STATIC_INLINE void std_rcc_write_rch_coarse_calibration(uint32_t cal_value)
{
    MODIFY_REG(RCC->RCHCAL, RCC_RCHCAL_RCH_CAL_COARSE, cal_value << RCC_RCHCAL_RCH_CAL_COARSE_POS);
}

/** 
* @brief  ��ȡRCHʱ�Ӵֵ�ֵ
* @retval uint32_t RCHʱ�Ӵֵ�ֵ
*/
__STATIC_INLINE uint32_t std_rcc_read_rch_coarse_calibration(void)
{
    return((RCC->RCHCAL & RCC_RCHCAL_RCH_CAL_COARSE) >> RCC_RCHCAL_RCH_CAL_COARSE_POS);
}

/** 
* @brief  дRCHʱ��΢��ֵ
* @param  trim_value  RCHʱ��΢��ֵ
* @retval ��
*/
__STATIC_INLINE void std_rcc_write_rch_fine_calibration(uint32_t trim_value)
{
    MODIFY_REG(RCC->RCHCAL, RCC_RCHCAL_RCH_CAL_FINE, trim_value);
}

/** 
* @brief  ��ȡRCHʱ��΢��ֵ
* @retval uint32_t RCHʱ��΢��ֵ
*/
__STATIC_INLINE uint32_t std_rcc_read_rch_fine_calibration(void)
{
    return(RCC->RCHCAL & RCC_RCHCAL_RCH_CAL_FINE);
}


/* ��ȡʱ��Ƶ�ʺ��� */
uint32_t std_rcc_get_sysclkfreq(void);
uint32_t std_rcc_get_hclkfreq(void);
uint32_t std_rcc_get_pclkfreq(void); 


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

#endif /* CIU32F003_STD_RCC_H */
