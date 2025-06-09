/************************************************************************************************/
/**
* @file               ciu32f003_std_exti.c
* @author             MCU Ecosystem Development Team
* @brief              EXTI STD������
*                     ʵ����EXTIģ����ź��߳�ʼ����ȥ��ʼ����API��
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
* @addtogroup EXTI 
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_EXTI_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup EXTI_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  EXTI��ʼ��
* @param  exti_init_param EXTI��ʼ�������ṹ��
* @retval ��
*/
void std_exti_init(std_exti_init_t* exti_init_param)
{
    uint32_t trigger;
    uint32_t exti_mode;

    /* ����GPIO EXTIͨ�� */
    if ((exti_init_param->line_id & EXTI_GPIO) != 0x00U)
    {
        trigger = exti_init_param->trigger & EXTI_TRIGGER_MASK;

        /* ��ֹEXTIͨ��������/�½��ش��� */
        std_exti_falling_trigger_disable(exti_init_param->line_id);
        std_exti_rising_trigger_disable (exti_init_param->line_id);

        /* ʹ��EXTIͨ�������ش��� */
        if ((trigger & EXTI_TRIGGER_RISING) == EXTI_TRIGGER_RISING)
        {
            std_exti_rising_trigger_enable (exti_init_param->line_id);
        }

        /* ʹ��EXTIͨ���½��ش��� */
        if ((trigger & EXTI_TRIGGER_FALLING) == EXTI_TRIGGER_FALLING)
        {
            std_exti_falling_trigger_enable(exti_init_param->line_id);
        }

        /* ����EXTIͨ������ӦGPIO�˿� */
        if ((exti_init_param->line_id & EXTI_GPIO) == EXTI_GPIO)
        {
            std_exti_set_gpio(exti_init_param->gpio_id, exti_init_param->line_id);
        }
    }

    /* ����EXTIͨ���ж�/�¼����� */
    exti_mode = exti_init_param->mode & EXTI_MODE_INTERRUPT_EVENT;

    /* ��ֹEXTIͨ���ж�/�¼����� */
    std_exti_interrupt_disable(exti_init_param->line_id);
    std_exti_event_disable(exti_init_param->line_id);

    /* ʹ��EXTIͨ���жϻ��� */
    if ((exti_mode & EXTI_MODE_INTERRUPT) == EXTI_MODE_INTERRUPT)
    {
        std_exti_interrupt_enable(exti_init_param->line_id);
    }

    /* ʹ��EXTIͨ���¼����� */
    if ((exti_mode & EXTI_MODE_EVENT) == EXTI_MODE_EVENT)
    {
        std_exti_event_enable (exti_init_param->line_id);
    }
}

/**
* @brief  EXTIȥ��ʼ��
* @retval ��
*/
void std_exti_deinit(void)
{
    EXTI->RTSR = 0x00000000U;
    EXTI->FTSR = 0x00000000U;
    EXTI->PIR = 0x300FFU;
    EXTI->EXTICR1 = 0x00000000U;
    EXTI->IMR = 0x40000000U;
    EXTI->EMR = 0x00000000U;
}

/**
* @brief  EXTI��ʼ���ṹ���ʼ��
* @param  exti_init_struct EXTI��ʼ�������ṹ��
* @retval ��
*/
void std_exti_struct_init(std_exti_init_t* exti_init_struct)
{
    exti_init_struct->line_id = EXTI_LINE_GPIO_PIN0;
    exti_init_struct->mode = EXTI_MODE_NONE;
    exti_init_struct->trigger = EXTI_TRIGGER_NONE;
    exti_init_struct->gpio_id = EXTI_GPIOA;
}

/** 
* @} 
*/

#endif /* STD_EXTI_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/
