/************************************************************************************************/
/**
* @file               ciu32f003_std_flash.h
* @author             MCU Ecosystem Development Team
* @brief              FLASH STD������ͷ�ļ���
*                     �ṩFLASH��ص�STD������������������������Լ������Ķ��塣                         
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*����ͷ�ļ��ظ�����*/
#ifndef CIU32F003_STD_FLASH_H
#define CIU32F003_STD_FLASH_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup FLASH FLASH
* @brief FLASH�洢����STD������
* @{
*/
/************************************************************************************************/



#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std_common.h"

/*-------------------------------------------define---------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup FLASH_Constants FLASH Constants 
* @brief  FLASH�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* Flash��ȡ���ʵȴ����� */      
#define FLASH_LATENCY_0CLK                FLASH_ACR_LATENCY_0CLK                     /**< �ȴ����ڣ�0 HCLK         */
#define FLASH_LATENCY_1CLK                FLASH_ACR_LATENCY_1CLK                     /**< �ȴ����ڣ�1 HCLK         */ 

/* Flash���ƼĴ���������Կ */                                                        
#define FLASH_CR_KEY1                     (0xE57A1A85U)                              /**< Flash���ƼĴ���������Կ1 */
#define FLASH_CR_KEY2                     (0x7C6E8391U)                              /**< Flash���ƼĴ���������Կ2 */

/* Flashѡ���ֽڽ�����Կ */                                                          
#define FLASH_OPT_KEY1                    (0x6A894D7BU)                              /**< Flashѡ���ֽڽ�����Կ1   */
#define FLASH_OPT_KEY2                    (0x7C311F5AU)                              /**< Flashѡ���ֽڽ�����Կ2   */

/* Flash����״̬ */                                                                  
#define FLASH_FLAG_EOP                    FLASH_SR_EOP                               /**< Flash������ɱ�־              */
#define FLASH_FLAG_BSY                    FLASH_SR_BSY                               /**< Flash����״̬��־              */
#define FLASH_FLAG_OPTVERR                FLASH_SR_OPTVERR                           /**< Flash option bytesУ������־ */
#define FLASH_FLAG_WRPERR                 FLASH_SR_WRPERR                            /**< Flashд���������־            */

/* Flash�ж�Դ */                      
#define FLASH_INTERRUPT_OPERR             FLASH_CR_OPERRIE                           /**< Flash�����쳣�ж�        */
#define FLASH_INTERRUPT_EOP               FLASH_CR_EOPIE                             /**< Flash��������ж�        */

/* Flash����ģʽ */                                                                  
#define FLASH_MODE_IDLE                   FLASH_CR_OP_MODE_IDLE                      /**< Flash�˳��������ģʽ    */
#define FLASH_MODE_PAGE_ERASE             FLASH_CR_OP_MODE_PAGE_ERASE                /**< Flashҳ����ģʽ          */
#define FLASH_MODE_MASS_ERASE             FLASH_CR_OP_MODE_MASS_ERASE                /**< Flash��������ģʽ        */
#define FLASH_MODE_PROGRAM                FLASH_CR_OP_MODE_PROGRAM                   /**< Flash���ģʽ            */

/* ѡ���ֽ�1 λ��ʹ�� */
#define FLASH_PIN_MODE_MASK               FLASH_OPTR1_NRST_SWD_MODE_MASK             /**< NRST SWD���Ź���ѡ��    */
#define FLASH_PIN_MODE_NRST_SWD           FLASH_OPTR1_NRST_SWD_MODE_0                /**< PC0: NRST  PB6: SWDIO   */
#define FLASH_PIN_MODE_GPIO_SWD           FLASH_OPTR1_NRST_SWD_MODE_2                /**< PC0: GPIO  PB6: SWDIO   */
#define FLASH_PIN_MODE_SWD_GPIO           FLASH_OPTR1_NRST_SWD_MODE_3                /**< PC0: SWDIO PB6: GPIO    */

#define FLASH_BOR_DISABLE                 (0x00000000U)                              /**< BOR��ֹ                             */
#define FLASH_BOR_ENABLE                  FLASH_OPTR1_BOR_EN                         /**< BORʹ��                             */

#define FLASH_BOR_LEVEL_0                 FLASH_OPTR1_BOR_LEVEL_0                    /**< BOR����ѹ����/�½���ֵ��2.0/1.9V  */
#define FLASH_BOR_LEVEL_1                 FLASH_OPTR1_BOR_LEVEL_1                    /**< BOR����ѹ����/�½���ֵ��2.4/2.3V  */
#define FLASH_BOR_LEVEL_2                 FLASH_OPTR1_BOR_LEVEL_2                    /**< BOR����ѹ����/�½���ֵ��2.8/2.7V  */
#define FLASH_BOR_LEVEL_3                 FLASH_OPTR1_BOR_LEVEL_3                    /**< BOR����ѹ����/�½���ֵ��3.2/3.1V  */

#define FLASH_RDP_LEVEL_MASK              FLASH_OPTR1_RDPRP_MASK                     /**< RDP������       */
#define FLASH_RDP_LEVEL_0                 (0x00000000U)                              /**< RDP�����ȼ�0    */
#define FLASH_RDP_LEVEL_1                 (0x00000001U)                              /**< RDP�����ȼ�1    */

/* ѡ���ֽ�2 λ��ʹ�� */
#define FLASH_IWDG_STOP_MODE_STOP         (0x00000000U)                              /**< IWDG��Stopģʽ��ֹͣ����                       */
#define FLASH_IWDG_STOP_MODE_NORMAL       FLASH_OPTR2_IWDG_STOP                      /**< IWDG��Stopģʽ����������                       */

#define FLASH_STOP_RESET_ENABLE           (0x00000000U)                              /**< ִ�н���Stopģʽ������������Stop��������λ     */
#define FLASH_STOP_RESET_DISABLE          FLASH_OPTR2_RST_STOP                       /**< ִ�н���Stopģʽ��������������Stop����������λ */

/* д������������ */
#define FLASH_WRP_AREA_0                  (0x0000003EU)                              /**< д��������0x00000000U ~ 0x00000FFF */
#define FLASH_WRP_AREA_1                  (0x0000003DU)                              /**< д��������0x00001000U ~ 0x00001FFF */
#define FLASH_WRP_AREA_2                  (0x0000003BU)                              /**< д��������0x00002000U ~ 0x00002FFF */
#define FLASH_WRP_AREA_3                  (0x00000037U)                              /**< д��������0x00003000U ~ 0x00003FFF */
#define FLASH_WRP_AREA_4                  (0x0000002FU)                              /**< д��������0x00004000U ~ 0x00004FFF */
#define FLASH_WRP_AREA_5                  (0x0000001FU)                              /**< д��������0x00005000U ~ 0x00005FFF */
#define FLASH_WRP_AREA_ALL                (0x00000000U)                              /**< User flash����ȫ��д����             */
#define FLASH_WRP_AREA_NONE               (0x0000003FU)                              /**< User flash����д�����ر�             */


/**
* @brief  ����Option Bytes�����ֵ
* @param  VAL OB����16λԤ�ڱ��ֵ 
* @retval OB��16λȡ�������16λ��ӵ�ֵ
*/
#define FLASH_OB_DATA_CALCULATE(VAL)      ((uint32_t)(((VAL) & 0xFFFF) | ((~(VAL & 0xFFFF)) << 16)))

/**
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup FLASH_External_Functions FLASH External Functions
* @brief    FLASH���⺯��
* @{
*
*/
/************************************************************************************************/

/** 
* @brief  Flash���ƼĴ�������
* @note   FLASH���������������CRKEY�Ĵ�����������ߴ�������д������ֱ���´θ�λ
* @retval ��
*/
__STATIC_INLINE void std_flash_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) == FLASH_CR_LOCK)
    {
        FLASH->CRKEY = FLASH_CR_KEY1;
        FLASH->CRKEY = FLASH_CR_KEY2; 
    }
}

/** 
* @brief  Flash���ƼĴ�������
* @retval ��
*/
__STATIC_INLINE void std_flash_lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK;
}

/** 
* @brief  ��ȡFlash����״̬
* @retval uint32_t Flash����״̬
*             @arg ��0: ����
*             @arg 0:   δ����
*/
__STATIC_INLINE uint32_t std_flash_get_lock_status(void)
{
    return (FLASH->CR & FLASH_CR_LOCK);
}

/** 
* @brief  Flashѡ���ֽڽ���
* @note   FLASH ѡ���ֽڽ��������OPTKEY�Ĵ�����������ߴ�������д������ֱ���´θ�λ
* @retval ��
*/
__STATIC_INLINE void std_flash_opt_unlock(void)
{
    if ((FLASH->CR & FLASH_CR_OPTLOCK) == FLASH_CR_OPTLOCK)
    {
        FLASH->OPTKEY = FLASH_OPT_KEY1;
        FLASH->OPTKEY = FLASH_OPT_KEY2; 
    }
}

/** 
* @brief  Flashѡ���ֽ�����
* @retval ��
*/
__STATIC_INLINE void std_flash_opt_lock(void)
{
    FLASH->CR |= FLASH_CR_OPTLOCK;
}

/** 
* @brief  ��ȡFlashѡ���ֽ�����״̬
* @retval uint32_t Flashѡ���ֽ�����״̬
*             @arg ��0: ����
*             @arg 0:   δ����
*/
__STATIC_INLINE uint32_t std_flash_get_opt_lock_status(void)
{
    return (FLASH->CR & FLASH_CR_OPTLOCK);
}

/**
* @brief  ����FLASH��ȡ���ʵȴ�����
* @param  latency ��ȡ���ʵȴ�����
*             @arg  FLASH_LATENCY_0CLK
*             @arg  FLASH_LATENCY_1CLK
* @retval ��
*/
__STATIC_INLINE void std_flash_set_latency(uint32_t latency)
{
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, latency);
} 

/**
* @brief  ��ȡFLASH��ȡ���ʵȴ�����
* @retval uint32_t ��ȡ���ʵȴ�����
*             @arg  FLASH_LATENCY_0CLK
*             @arg  FLASH_LATENCY_1CLK
*/
__STATIC_INLINE uint32_t std_flash_get_latency(void)
{
    return (FLASH->ACR & FLASH_ACR_LATENCY);
} 

/**
* @brief  ʹ��FLASH�ж�
* @param  interrupts �ж�Դ
*             @arg FLASH_INTERRUPT_OPERR
*             @arg FLASH_INTERRUPT_EOP 
* @retval ��
*/
__STATIC_INLINE void std_flash_interrupt_enable(uint32_t interrupts)
{
    FLASH->CR |= interrupts;
}

/**
* @brief  ��ֹFLASH�ж�
* @param  interrupts �ж�Դ
*             @arg FLASH_INTERRUPT_OPERR
*             @arg FLASH_INTERRUPT_EOP 
* @retval ��
*/
__STATIC_INLINE void std_flash_interrupt_disable(uint32_t interrupts)
{
    FLASH->CR &= (~interrupts);
}

/**
* @brief  ��ȡFLASH�ж�ʹ��״̬
* @param  interrupt �ж�Դ
*             @arg FLASH_INTERRUPT_OPERR
*             @arg FLASH_INTERRUPT_EOP 
* @retval uint32_t FLASH�ж�ʹ��״̬
*             @arg ��0: ʹ��
*             @arg 0:   ��ֹ
*/
__STATIC_INLINE uint32_t std_flash_get_interrupt_enable(uint32_t interrupt)
{
    return (FLASH->CR & interrupt);
}

/**
* @brief  ��ȡFLASH״̬��־
* @param  flag ״̬��־
*             @arg FLASH_FLAG_EOP
*             @arg FLASH_FLAG_BSY
*             @arg FLASH_FLAG_OPTVERR
*             @arg FLASH_FLAG_WRPERR
* @retval uint32_t FLASH״̬��־
*             @arg ��0: ��־λ��λ
*             @arg 0:   ��־λ���
*/
__STATIC_INLINE uint32_t std_flash_get_flag(uint32_t flag)
{
    return (FLASH->SR & flag);
}

/**
* @brief  ���FLASH״̬��־
* @param  flags ״̬��־���
*             @arg FLASH_FLAG_EOP
*             @arg FLASH_FLAG_WRPERR
* @retval ��
*/
__STATIC_INLINE void std_flash_clear_flag(uint32_t flags)
{
    FLASH->SR = flags;
}

/** 
* @brief  ����Flash����ģʽ
* @param  mode ����ģʽ
*             @arg FLASH_MODE_IDLE
*             @arg FLASH_MODE_PAGE_ERASE
*             @arg FLASH_MODE_MASS_ERASE
*             @arg FLASH_MODE_PROGRAM
* @retval ��
*/
__STATIC_INLINE void std_flash_set_operate_mode(uint32_t mode)
{
    MODIFY_REG(FLASH->CR, FLASH_CR_OP_MODE, mode);
}

/**
* @brief  ��ȡѡ���ֽڼĴ���1
* @retval uint32_t ѡ���ֽ�1����ֵ
*/
__STATIC_INLINE uint32_t std_flash_get_opt1(void)
{
    return (FLASH->OPTR1);
}

/**
* @brief  ��ȡFLASH�������ȼ�
* @retval uint32_t FLASH�������ȼ�
*             @arg FLASH_RDP_LEVEL_0
*             @arg FLASH_RDP_LEVEL_1
*/
__STATIC_INLINE uint32_t std_flash_get_rdp_level(void)
{
    return (FLASH->OPTR1 & FLASH_OPTR1_RDPRP);
}

/**
* @brief  ��ȡBOR��ֵ�ȼ�
* @retval uint32_t FLASH�������ȼ�
*             @arg FLASH_BOR_LEVEL_0
*             @arg ...
*             @arg FLASH_BOR_LEVEL_3
*/
__STATIC_INLINE uint32_t std_flash_get_bor_level(void)
{
    return (FLASH->OPTR1 & FLASH_OPTR1_BOR_LEVEL);
}

/** 
* @brief  ��ȡBORʹ�ܿ���
* @retval uint32_t BORʹ�ܿ���
*             @arg FLASH_BOR_DISABLE
*             @arg FLASH_BOR_ENABLE
*/
__STATIC_INLINE uint32_t std_flash_get_bor_en(void)
{
    return (FLASH->OPTR1 & FLASH_OPTR1_BOR_EN);
}

/** 
* @brief  ��ȡNRST SWD���Ź��ܿ���
* @retval uint32_t NRST SWD���Ź���ѡ��
*             @arg FLASH_PIN_MODE_NRST_SWD
*             @arg FLASH_PIN_MODE_GPIO_SWD
*             @arg FLASH_PIN_MODE_SWD_GPIO
*/
__STATIC_INLINE uint32_t std_flash_get_nrst_swd_mode(void)
{
    return (FLASH->OPTR1 & FLASH_PIN_MODE_MASK);
}

/**
* @brief  ��ȡѡ���ֽڼĴ���2
* @retval uint32_t ѡ���ֽ�2����ֵ
*/
__STATIC_INLINE uint32_t std_flash_get_opt2(void)
{
    return (FLASH->OPTR2);
}

/** 
* @brief  ��ȡIWDG��STOPģʽ�¼�����ֹͣ����
* @retval uint32_t IWDG_STOP����
*             @arg FLASH_IWDG_STOP_MODE_STOP
*             @arg FLASH_IWDG_STOP_MODE_NORMAL
*/
__STATIC_INLINE uint32_t std_flash_get_iwdg_stop(void)
{
    return (FLASH->OPTR2 & FLASH_OPTR2_IWDG_STOP);
}

/** 
* @brief  ��ȡ����STOPģʽ�Ƿ�λ������
* @retval uint32_t RST_STOP����
*             @arg FLASH_STOP_RESET_ENABLE
*             @arg FLASH_STOP_RESET_DISABLE
*/
__STATIC_INLINE uint32_t std_flash_get_rst_stop(void)
{
    return (FLASH->OPTR2 & FLASH_OPTR2_RST_STOP);
}

/**
* @brief  ��ȡд����������ֵ
* @retval uint32_t д����������ֵ
*/
__STATIC_INLINE uint32_t std_flash_get_wrp(void)
{
    return (FLASH->WRP & FLASH_WRP_WRP);
}

/** 
* @brief  ��ȡоƬ�ͺ�
* @retval uint32_t оƬ�ͺ�
*/
__STATIC_INLINE uint32_t std_flash_get_device_type(void)
{
    return (*(uint32_t *)DEVICE_TYPE);
}

/** 
* @brief  ��ȡFlash�ռ��С
* @retval uint32_t Flash�ռ��С
*/
__STATIC_INLINE uint32_t std_flash_get_flash_size(void)
{
    return (*(uint32_t *)USERFLASH_SIZE);
}

/** 
* @brief  ��ȡSRAM�ռ��С
* @retval uint32_t SRAM�ռ��С
*/
__STATIC_INLINE uint32_t std_flash_get_sram_size(void)
{
    return (*(uint32_t *)SRAM_SIZE);
}


std_status_t std_flash_erase(uint32_t mode, uint32_t address);
std_status_t std_flash_word_program(uint32_t address, uint32_t prog_data);

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

#endif /* CIU32F003_STD_FLASH_H */


