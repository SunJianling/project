/************************************************************************************************/
/**
* @file               ciu32f003_std_gpio.c
* @author             MCU Ecosystem Development Team
* @brief              GPIO STD��������
*                     ʵ��GPIO��ʼ����ȥ��ʼ����API��
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @addtogroup GPIO
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_GPIO_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup GPIO_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  GPIO��ʼ��
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @param  gpio_init_param GPIO��ʼ�������ṹ��
* @retval ��
*/
void std_gpio_init(GPIO_t* gpiox, std_gpio_init_t* gpio_init_param)
{
    uint32_t offset = 0;
    uint32_t current_pin = 0;

    /* ��������GPIO���� */
    for (; ((gpio_init_param->pin) >> offset) != 0x00U; offset++)
    {
        /* ��ȡ��ǰ���������� */
        current_pin = (gpio_init_param->pin) & (0x00000001UL << offset);

        if (current_pin != 0x00U)
        {
            /* ����ģʽ */
            std_gpio_set_pin_mode(gpiox, current_pin, gpio_init_param->mode);

            /* ���������� */
            std_gpio_set_pin_pull(gpiox, current_pin, gpio_init_param->pull);

            if (gpio_init_param->mode == GPIO_MODE_ALTERNATE)
            {
                /* ����GPIO���ţ�0~7�����ù��� */
                std_gpio_set_afpin_0_7 (gpiox, current_pin, gpio_init_param->alternate);
            }
        }
    }

    if ((gpio_init_param->mode == GPIO_MODE_OUTPUT) || (gpio_init_param->mode == GPIO_MODE_ALTERNATE))
    {
        /* ����������� */
        std_gpio_set_pin_output_type(gpiox, gpio_init_param->pin, gpio_init_param->output_type);
    }
}

/**
* @brief  GPIOȥ��ʼ��
* @param  gpiox GPIO����
*             @arg GPIOA
*             @arg GPIOB
*             @arg GPIOC
* @retval ��
*/
void std_gpio_deinit(GPIO_t* gpiox)
{
    if(GPIOA == gpiox)
    {
        std_rcc_gpio_reset(RCC_PERIPH_RESET_GPIOA);    
    }
    else if(GPIOB == gpiox)
    {
        std_rcc_gpio_reset(RCC_PERIPH_RESET_GPIOB);    
    } 
    else if(GPIOC == gpiox)
    {
        std_rcc_gpio_reset(RCC_PERIPH_RESET_GPIOC);    
    }     
        
}

/**
* @brief  GPIO��ʼ���ṹ���ʼ��
* @param  gpio_init_struct GPIO��ʼ�������ṹ��
* @retval ��
*/
void std_gpio_struct_init(std_gpio_init_t* gpio_init_struct)
{
    gpio_init_struct->pin = GPIO_PIN_0;
    gpio_init_struct->mode = GPIO_MODE_ANALOG;
    gpio_init_struct->pull = GPIO_NOPULL;
    gpio_init_struct->output_type = GPIO_OUTPUT_PUSHPULL;
    gpio_init_struct->alternate = 0U;
}

/** 
* @} 
*/

#endif /* STD_GPIO_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/
