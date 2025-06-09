/************************************************************************************************/
/**
* @file               ciu32f003_std_exti.h
* @author             MCU Ecosystem Development Team
* @brief              EXTI STD������ͷ�ļ���
*                     �ṩEXTI��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_EXTI_H
#define CIU32F003_STD_EXTI_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup EXTI EXTI
* @brief ��չ�жϺ��¼���������STD������
* @{
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
* @defgroup EXTI_Types EXTI Types
* @brief EXTI�������Ͷ���
* @{
*/
/************************************************************************************************/

/**
* @brief  EXTI�������ýṹ�嶨��
*/
typedef struct
{
    uint32_t line_id;                 /**< EXTIͨ��ID
                                               @arg EXTI_LINE_GPIO_PIN0 ... */
    uint32_t mode;                    /**< EXTIͨ��ģʽ
                                               @arg EXTI_MODE_INTERRUPT ... */
    uint32_t trigger;                 /**< EXTIͨ����������
                                               @arg EXTI_TRIGGER_RISING ... */
    uint32_t gpio_id;                 /**< GPIO�˿�ID
                                               @arg EXTI_GPIOA ... */
} std_exti_init_t;


/**
* @}
*/

/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup EXTI_Constants EXTI Constants 
* @brief  EXTI�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* EXTI LINE ID���� */
#define EXTI_LINE_GPIO_PIN0                 (EXTI_GPIO     | 0x00U)                         /**< EXTI_LINE0  */
#define EXTI_LINE_GPIO_PIN1                 (EXTI_GPIO     | 0x01U)                         /**< EXTI_LINE1  */
#define EXTI_LINE_GPIO_PIN2                 (EXTI_GPIO     | 0x02U)                         /**< EXTI_LINE2  */
#define EXTI_LINE_GPIO_PIN3                 (EXTI_GPIO     | 0x03U)                         /**< EXTI_LINE3  */
#define EXTI_LINE_GPIO_PIN4                 (EXTI_GPIO     | 0x04U)                         /**< EXTI_LINE4  */
#define EXTI_LINE_GPIO_PIN5                 (EXTI_GPIO     | 0x05U)                         /**< EXTI_LINE5  */
#define EXTI_LINE_GPIO_PIN6                 (EXTI_GPIO     | 0x06U)                         /**< EXTI_LINE6  */
#define EXTI_LINE_GPIO_PIN7                 (EXTI_GPIO     | 0x07U)                         /**< EXTI_LINE7  */
#define EXTI_LINE_COMP1                     (EXTI_CONFIG   | 0x10U)                         /**< EXTI_LINE16 */
#define EXTI_LINE_COMP2                     (EXTI_CONFIG   | 0x11U)                         /**< EXTI_LINE17 */ 
#define EXTI_LINE_LPTIM1                    (EXTI_DIRECT   | 0x1EU)                         /**< EXTI_LINE30 */

/* EXTI LINE ���� */
#define EXTI_DIRECT                         (0x01000000)                                    /**< ֱ��ͨ��                    */
#define EXTI_CONFIG                         (0x02000000)                                    /**< ����������ͨ��(COMP1/COMP2) */
#define EXTI_GPIO                           (0x06000000)                                    /**< GPIO������ͨ��              */
#define EXTI_PROPERTY_MASK                  (EXTI_DIRECT | EXTI_CONFIG | EXTI_GPIO)         /**< ͨ����������                */

/* EXTI LINE�������룬����ɸѡEXTI LINE ID */ 
#define EXTI_LINE_MASK                      (0x0000001FU)                                   /**< EXTI LINE�������� */

/* EXTI LINEģʽ���жϡ��¼� */ 
#define EXTI_MODE_NONE                      (0x00000000U)                                   /**< ���ж�/�¼����� */
#define EXTI_MODE_INTERRUPT                 (0x00000001U)                                   /**< �жϻ���        */
#define EXTI_MODE_EVENT                     (0x00000002U)                                   /**< �¼�����        */
#define EXTI_MODE_INTERRUPT_EVENT           (EXTI_MODE_EVENT | EXTI_MODE_INTERRUPT)         /**< �ж�/�¼�����   */

/* EXTI LINE �������Ͷ��壬���ڿ�����ͨ�����źŴ����������� */
#define EXTI_TRIGGER_NONE                   (0x00000000U)                                   /**< ������            */
#define EXTI_TRIGGER_RISING                 (0x00000001U)                                   /**< �Ͻ��ش���        */
#define EXTI_TRIGGER_FALLING                (0x00000002U)                                   /**< �½��ش���        */
#define EXTI_TRIGGER_RISING_FALLING         (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)    /**< ������/�½��ش��� */

/* EXTI LINE ���������������룬����ɸѡEXTI ������ͨ���������� */
#define EXTI_TRIGGER_MASK                   (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)     /**< ���������������� */

/* GPIO�˿�ID���� */
#define EXTI_GPIOA                          (0x00000000UL)                                   /**< GPIOA ID */
#define EXTI_GPIOB                          (0x00000001UL)                                   /**< GPIOB ID */
#define EXTI_GPIOC                          (0x00000002UL)                                   /**< GPIOC ID */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup EXTI_External_Functions EXTI External Functions
* @brief    EXTI���⺯��
* @{
*
*/
/************************************************************************************************/

/**
* @brief  ��ȡEXTIͨ���жϹ���״̬
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t �������жϹ���״̬
*             @arg ��0  ��ʾ�ѹ���
*             @arg 0 ��ʾδ����
*/
__STATIC_INLINE uint32_t std_exti_get_pending_status(uint32_t exti_line)
{
    return (EXTI->PIR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}


/**
* @brief  ���EXTIͨ���жϹ���״̬
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval ��
*/
__STATIC_INLINE void std_exti_clear_pending(uint32_t exti_line)
{
    EXTI->PIR = (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  ʹ��EXTIͨ�������ش���
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval ��
*/
__STATIC_INLINE void std_exti_rising_trigger_enable(uint32_t exti_line)
{
    EXTI->RTSR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  ��ֹEXTIͨ�������ش���
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval ��
*/
__STATIC_INLINE void std_exti_rising_trigger_disable(uint32_t exti_line)
{
    EXTI->RTSR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));   
}

/**
* @brief  ʹ��EXTIͨ���½��ش���
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval ��
*/
__STATIC_INLINE void std_exti_falling_trigger_enable(uint32_t exti_line)
{
    EXTI->FTSR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  ��ֹEXTIͨ���½��ش���
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval ��
*/
__STATIC_INLINE void std_exti_falling_trigger_disable(uint32_t exti_line)
{
    EXTI->FTSR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  ��ȡEXTIͨ�������ش���״̬
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1 
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t �����ش���״̬
*             @arg ��0  ��ʾ��ʹ��
*             @arg 0 ��ʾ�ѽ�ֹ
*/
__STATIC_INLINE uint32_t std_exti_get_rising_trigger_enable(uint32_t exti_line)
{
    return (EXTI->RTSR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  ��ȡEXTIͨ���½��ش���״̬
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t �½��ش���״̬
*             @arg ��0  ��ʾ��ʹ��
*             @arg 0 ��ʾ�ѽ�ֹ
*/
__STATIC_INLINE uint32_t std_exti_get_falling_trigger_enable(uint32_t exti_line)
{
    return (EXTI->FTSR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  ʹ��EXTIͨ�������ж�
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval ��
*/
__STATIC_INLINE void std_exti_interrupt_enable(uint32_t exti_line)
{
    EXTI->IMR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  ��ֹEXTIͨ�������ж�
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval ��
*/
__STATIC_INLINE void std_exti_interrupt_disable(uint32_t exti_line)
{
    EXTI->IMR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  ʹ��EXTIͨ�������¼�
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval ��
*/
__STATIC_INLINE void std_exti_event_enable(uint32_t exti_line)
{
    EXTI->EMR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  ��ֹEXTIͨ�������¼�
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval ��
*/
__STATIC_INLINE void std_exti_event_disable(uint32_t exti_line)
{
    EXTI->EMR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  ����EXTIͨ������ӦGPIO�˿�
* @param  gpio_id  GPIO�˿�ID
*             @arg EXTI_GPIOA
*             @arg EXTI_GPIOB
*             @arg EXTI_GPIOC
* @param  exti_line  EXTIͨ��ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_GPIO_PIN7
* @retval ��
*/
__STATIC_INLINE void std_exti_set_gpio(uint32_t gpio_id, uint32_t exti_line)
{
    exti_line &= EXTI_LINE_MASK;
    MODIFY_REG(EXTI->EXTICR1,
              (EXTI_EXTICR1_EXTI0_MASK << (EXTI_EXTICR1_EXTI1_POS * exti_line)),
              (gpio_id << (EXTI_EXTICR1_EXTI1_POS * exti_line)));
}

void std_exti_init(std_exti_init_t* exti_init_param);
void std_exti_deinit(void);
void std_exti_struct_init(std_exti_init_t* exti_init_struct);

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

#endif /* CIU32F003_STD_EXTI_H */
