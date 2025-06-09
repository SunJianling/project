/************************************************************************************************/
/**
* @file               ciu32f003_std_gpio.h
* @author             MCU Ecosystem Development Team
* @brief              GPIO STD������ͷ�ļ���
*                     �ṩGPIO��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_GPIO_H
#define CIU32F003_STD_GPIO_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup GPIO GPIO
* @brief ͨ������/����ӿڵ�STD������
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
* @defgroup GPIO_Types GPIO Types
* @brief GPIO�������Ͷ���
* @{
*/
/************************************************************************************************/
/**
* @brief  GPIO��ʼ���ṹ�嶨��
*/
typedef struct
{                         
    uint32_t pin;                 /**< ָ��GPIO���ţ�������GPIO������� 
                                           @arg GPIO_PIN_0 ...           */

    uint32_t mode;                /**< ָ��GPIO����ģʽ������������� 
                                           @arg GPIO_MODE_INPUT
                                           @arg GPIO_MODE_ANALOG
                                           @arg GPIO_MODE_OUTPUT
                                           @arg GPIO_MODE_ALTERNATE      */

    uint32_t pull;                /**< ָ��GPIO������/��������  
                                           @arg GPIO_NOPULL ...          */

    uint32_t output_type;         /**< ָ��GPIO�������  
                                           @arg GPIO_OUTPUT_PUSHPULL
                                           @arg GPIO_OUTPUT_OPENDRAIN    */
    
    uint32_t alternate;           /**< ָ��GPIO���Ÿ��ù�������   
                                           @arg GPIO_AF0_SPI1 ...        */
} std_gpio_init_t;

/** 
* @} 
*/

/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup GPIO_Constants GPIO Constants 
* @brief  GPIO�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* GPIO PIN���� */
#define  GPIO_PIN_0                            ((uint16_t)0x0001U)        /**< ѡ��  PIN 0   */
#define  GPIO_PIN_1                            ((uint16_t)0x0002U)        /**< ѡ��  PIN 1   */
#define  GPIO_PIN_2                            ((uint16_t)0x0004U)        /**< ѡ��  PIN 2   */
#define  GPIO_PIN_3                            ((uint16_t)0x0008U)        /**< ѡ��  PIN 3   */
#define  GPIO_PIN_4                            ((uint16_t)0x0010U)        /**< ѡ��  PIN 4   */
#define  GPIO_PIN_5                            ((uint16_t)0x0020U)        /**< ѡ��  PIN 5   */
#define  GPIO_PIN_6                            ((uint16_t)0x0040U)        /**< ѡ��  PIN 6   */
#define  GPIO_PIN_7                            ((uint16_t)0x0080U)        /**< ѡ��  PIN 7   */
#define  GPIO_PIN_All                          ((uint16_t)0x00FFU)        /**< ѡ��  ȫ��    */

/* GPIO ģʽ���ó��� */
#define  GPIO_MODE_INPUT                       (0x00000000U)              /**< ���븡�� */
#define  GPIO_MODE_OUTPUT                      (0x00000001U)              /**< ������� */
#define  GPIO_MODE_ALTERNATE                   (0x00000002U)              /**< ���ù��� */
#define  GPIO_MODE_ANALOG                      (0x00000003U)              /**< ģ�⹦�� */

/* GPIO ���������ò��� */
#define  GPIO_NOPULL                           (0x00000000U)              /**< ���ϡ����� */
#define  GPIO_PULLUP                           (0x00000001U)              /**< ����       */
#define  GPIO_PULLDOWN                         (0x00000002U)              /**< ����       */

/* GPIO ������� */
#define  GPIO_OUTPUT_PUSHPULL                  (0x00000000U)              /**< ������� */
#define  GPIO_OUTPUT_OPENDRAIN                 (0x00000001U)              /**< ��©��� */

/* ���ù���0 */
#define  GPIO_AF0_SPI1                         ((uint8_t)0x00U)           /**< SPI1   ���ù���ӳ�� */
#define  GPIO_AF0_SWCLK                        ((uint8_t)0x00U)           /**< SWCLK  ���ù���ӳ�� */
#define  GPIO_AF0_SWDIO                        ((uint8_t)0x00U)           /**< SWDIO  ���ù���ӳ�� */

/* ���ù���1 */
#define  GPIO_AF1_UART1                        ((uint8_t)0x01U)           /**< UART1  ���ù���ӳ�� */

/* ���ù���2 */
#define  GPIO_AF2_TIM1                         ((uint8_t)0x02U)           /**< TIM1   ���ù���ӳ�� */

/* ���ù���3 */
#define  GPIO_AF3_TIM1                         ((uint8_t)0x03U)           /**< TIM1   ���ù���ӳ�� */
#define  GPIO_AF3_TIM3                         ((uint8_t)0x03U)           /**< TIM3   ���ù���ӳ�� */

/* ���ù���4 */
#define  GPIO_AF4_TIM1                         ((uint8_t)0x04U)           /**< TIM1   ���ù���ӳ�� */
#define  GPIO_AF4_COMP1                        ((uint8_t)0x04U)           /**< COMP1  ���ù���ӳ�� */
#define  GPIO_AF4_SPI1                         ((uint8_t)0x04U)           /**< SPI1   ���ù���ӳ�� */

/* ���ù���5 */
#define  GPIO_AF5_TIM1                         ((uint8_t)0x05U)           /**< TIM1   ���ù���ӳ�� */
#define  GPIO_AF5_UART1                        ((uint8_t)0x05U)           /**< UART1  ���ù���ӳ�� */
#define  GPIO_AF5_UART2                        ((uint8_t)0x05U)           /**< UART2  ���ù���ӳ�� */

/* ���ù���6 */
#define  GPIO_AF6_I2C1                         ((uint8_t)0x06U)           /**< I2C1   ���ù���ӳ�� */
#define  GPIO_AF6_MCO                          ((uint8_t)0x06U)           /**< MCO    ���ù���ӳ�� */

/* ���ù���7 */
#define  GPIO_AF7_COMP2                        ((uint8_t)0x07U)           /**< COMP2  ���ù���ӳ�� */
#define  GPIO_AF7_IR_OUT                       ((uint8_t)0x07U)           /**< IR_OUT ���ù���ӳ�� */
#define  GPIO_AF7_MCO                          ((uint8_t)0x07U)           /**< MCO    ���ù���ӳ�� */

/* ���ù��ܼĴ���GPIOԴѡ��ƫ��λ */
#define  GPIO_AF_SELECT_OFFSET                 (0x0000000FU)              /**< GPIOԴѡ��ƫ��λ */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup GPIO_External_Functions GPIO External Functions
* @brief    GPIO���⺯��
* @{
*
*/
/************************************************************************************************/
/**
* @brief  ����GPIO����ģʽ
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin  GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @param  mode GPIO����ģʽ
*             @arg GPIO_MODE_INPUT     ����ģʽ
*             @arg GPIO_MODE_OUTPUT    ���ģʽ
*             @arg GPIO_MODE_ALTERNATE ����ģʽ
*             @arg GPIO_MODE_ANALOG    ģ��ģʽ
* @note   ��֧�ֵ�������
* @retval ��
*/
__STATIC_INLINE void std_gpio_set_pin_mode(GPIO_t *gpiox, uint32_t pin, uint32_t mode)
{
    MODIFY_REG(gpiox->MODE, ((pin * pin) * GPIO_MODE_MODE0), ((pin * pin) * mode));
}

/**
* @brief  ��ȡGPIO����ģʽ
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval uint32_t GPIO����ģʽ
*             @arg GPIO_MODE_INPUT     ����ģʽ
*             @arg GPIO_MODE_OUTPUT    ���ģʽ
*             @arg GPIO_MODE_ALTERNATE ����ģʽ
*             @arg GPIO_MODE_ANALOG    ģ��ģʽ
*/
__STATIC_INLINE uint32_t std_gpio_get_pin_mode(GPIO_t *gpiox, uint32_t pin)
{
    return ((gpiox->MODE & ((pin * pin) * GPIO_MODE_MODE0)) / (pin * pin));
}

/**
* @brief  ����GPIO�����������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin_mask GPIO�������
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @param  output_type GPIO�����������
*             @arg GPIO_OUTPUT_PUSHPULL  �������
*             @arg GPIO_OUTPUT_OPENDRAIN ��©���
* @retval ��
*/
__STATIC_INLINE void std_gpio_set_pin_output_type(GPIO_t *gpiox, uint32_t pin_mask, uint32_t output_type)
{
    MODIFY_REG(gpiox->OTYPE, pin_mask, pin_mask * output_type);
}

/**
* @brief  ��ȡGPIO�����������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin  GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval uint32_t GPIO�����������
*             @arg GPIO_OUTPUT_PUSHPULL  �������
*             @arg GPIO_OUTPUT_OPENDRAIN ��©���
*/
__STATIC_INLINE uint32_t std_gpio_get_pin_output_type(GPIO_t *gpiox, uint32_t pin)
{
    return ((gpiox->OTYPE & (pin)) != 0U ? GPIO_OUTPUT_OPENDRAIN : GPIO_OUTPUT_PUSHPULL);
}

/**
* @brief  ����GPIO��������������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @param  pull GPIO��������������
*             @arg GPIO_NOPULL   ���ϡ�����
*             @arg GPIO_PULLUP   ����
*             @arg GPIO_PULLDOWN ����
* @note   ��֧�ֵ�������
* @retval ��
*/
__STATIC_INLINE void std_gpio_set_pin_pull(GPIO_t *gpiox, uint32_t pin, uint32_t pull)
{
    MODIFY_REG(gpiox->PUPD, ((pin * pin) * GPIO_PUPD_PUPD0), ((pin * pin) * pull));
}

/**
* @brief  ��ȡGPIO��������������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin  GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval uint32_t GPIO��������������
*             @arg GPIO_NOPULL   ���ϡ�����
*             @arg GPIO_PULLUP   ����
*             @arg GPIO_PULLDOWN ����
*/
__STATIC_INLINE uint32_t std_gpio_get_pin_pull(GPIO_t *gpiox, uint32_t pin)
{
  return ((gpiox->PUPD & ((pin * pin) * GPIO_PUPD_PUPD0)) / (pin * pin));
}

/**
* @brief  ����GPIO���ţ�0~7�����ù���
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin  GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @param  alternate GPIO���ţ�0~7�����ù���
*             @arg GPIO_AF0_SPI1                         
*             @arg GPIO_AF0_SWCLK                       
*             @arg GPIO_AF0_SWDIO             
*             @arg GPIO_AF1_UART1                        
*             @arg GPIO_AF2_TIM1                        
*             @arg GPIO_AF3_TIM1                         
*             @arg GPIO_AF3_TIM3                         
*             @arg GPIO_AF4_TIM1                         
*             @arg GPIO_AF4_COMP1                        
*             @arg GPIO_AF4_SPI1                         
*             @arg GPIO_AF5_TIM1                         
*             @arg GPIO_AF5_UART1                        
*             @arg GPIO_AF5_UART2                       
*             @arg GPIO_AF6_I2C1                         
*             @arg GPIO_AF6_MCO                          
*             @arg GPIO_AF7_COMP2                       
*             @arg GPIO_AF7_IR_OUT                       
*             @arg GPIO_AF7_MCO                          
* @note   ��֧�ֵ�������
* @retval ��
*/
__STATIC_INLINE void std_gpio_set_afpin_0_7(GPIO_t *gpiox, uint32_t pin, uint32_t alternate)
{
    MODIFY_REG(gpiox->AFL, ((((pin * pin) * pin) * pin) * GPIO_AF_SELECT_OFFSET),
               ((((pin * pin) * pin) * pin) * alternate)); 
}

/**
* @brief  ��ȡGPIO���ţ�0~7�����ù���
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval uint32_t GPIO���ţ�0~7�����ù���
*             @arg GPIO_AF0_SPI1                         
*             @arg GPIO_AF0_SWCLK                       
*             @arg GPIO_AF0_SWDIO             
*             @arg GPIO_AF1_UART1                        
*             @arg GPIO_AF2_TIM1                        
*             @arg GPIO_AF3_TIM1                         
*             @arg GPIO_AF3_TIM3                         
*             @arg GPIO_AF4_TIM1                         
*             @arg GPIO_AF4_COMP1                        
*             @arg GPIO_AF4_SPI1                         
*             @arg GPIO_AF5_TIM1                         
*             @arg GPIO_AF5_UART1                        
*             @arg GPIO_AF5_UART2                       
*             @arg GPIO_AF6_I2C1                         
*             @arg GPIO_AF6_MCO                          
*             @arg GPIO_AF7_COMP2                       
*             @arg GPIO_AF7_IR_OUT                       
*             @arg GPIO_AF7_MCO  
*/
__STATIC_INLINE uint32_t std_gpio_get_afpin_0_7(GPIO_t *gpiox, uint32_t pin)
{
    return ((gpiox->AFL & ((((pin * pin) * pin) * pin) * GPIO_AF_SELECT_OFFSET)) / 
            (((pin * pin) * pin) * pin));
}

/**
* @brief  ��ȡGPIO��������״̬
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin  GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval bool GPIO��������״̬
*             @arg true  ��ʾ�ߵ�ƽ
*             @arg false ��ʾ�͵�ƽ
*/
__STATIC_INLINE bool std_gpio_get_input_pin(GPIO_t* gpiox, uint32_t pin)
{
    return ((gpiox->IDR & (pin)) == (pin));
}

/**
* @brief  ��ȡGPIO�������״̬
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin GPIO����
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @note   ��֧�ֵ�������
* @retval bool GPIO�������״̬
*             @arg true  ��ʾ�ߵ�ƽ
*             @arg false ��ʾ�͵�ƽ
*/
__STATIC_INLINE bool std_gpio_get_output_pin(GPIO_t* gpiox, uint32_t pin)
{
    return ((gpiox->ODR & (pin)) == (pin));
}

/**
* @brief  ��ȡGPIO�˿���������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @retval uint32_t GPIO�˿���������
*/
__STATIC_INLINE uint32_t std_gpio_read_input_port(GPIO_t* gpiox)
{
    return (gpiox->IDR);
}

/**
* @brief  ��ȡGPIO�˿��������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @retval uint32_t GPIO�˿��������
*/
__STATIC_INLINE uint32_t std_gpio_read_output_port(GPIO_t* gpiox)
{
    return (gpiox->ODR);
}

/**
* @brief  д��GPIO�˿��������
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  value �������
* @retval ��
*/
__STATIC_INLINE void std_gpio_write_output_port(GPIO_t* gpiox, uint32_t value)
{
    gpiox->ODR = value;
}

/**
* @brief  ����GPIO����ߵ�ƽ
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin_mask GPIO�������
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @retval ��
*/
__STATIC_INLINE void std_gpio_set_pin(GPIO_t* gpiox, uint32_t pin_mask)
{
    gpiox->BSR = pin_mask;
}

/**
* @brief  ����GPIO����͵�ƽ
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin_mask  GPIO�������
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @retval ��
*/
__STATIC_INLINE void std_gpio_reset_pin(GPIO_t* gpiox, uint32_t pin_mask)
{
    gpiox->BR = pin_mask;
}

/**
* @brief  �л�GPIO�˿ڸ�/�͵�ƽ
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  pin_mask  GPIO�������
*             @arg GPIO_PIN_0
*             @arg GPIO_PIN_1
*             @arg ...
*             @arg GPIO_PIN_7
* @retval ��
*/
__STATIC_INLINE void std_gpio_toggle_pin(GPIO_t* gpiox, uint32_t pin_mask)
{
    gpiox->ODR ^= pin_mask;
}

void std_gpio_init(GPIO_t* gpiox, std_gpio_init_t* gpio_init_param);
void std_gpio_deinit(GPIO_t* gpiox);
void std_gpio_struct_init(std_gpio_init_t* gpio_init_struct);

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

#endif /* CIU32F003_STD_GPIO_H */


