/************************************************************************************************/
/**
* @file               ciu32f003_std_exti.h
* @author             MCU Ecosystem Development Team
* @brief              EXTI STD库驱动头文件。
*                     提供EXTI相关的STD库操作函数声明、数据类型以及常量的定义。                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* 避免头文件重复引用 */
#ifndef CIU32F003_STD_EXTI_H
#define CIU32F003_STD_EXTI_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup EXTI EXTI
* @brief 扩展中断和事件控制器的STD库驱动
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
* @brief EXTI数据类型定义
* @{
*/
/************************************************************************************************/

/**
* @brief  EXTI参数配置结构体定义
*/
typedef struct
{
    uint32_t line_id;                 /**< EXTI通道ID
                                               @arg EXTI_LINE_GPIO_PIN0 ... */
    uint32_t mode;                    /**< EXTI通道模式
                                               @arg EXTI_MODE_INTERRUPT ... */
    uint32_t trigger;                 /**< EXTI通道触发类型
                                               @arg EXTI_TRIGGER_RISING ... */
    uint32_t gpio_id;                 /**< GPIO端口ID
                                               @arg EXTI_GPIOA ... */
} std_exti_init_t;


/**
* @}
*/

/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup EXTI_Constants EXTI Constants 
* @brief  EXTI常量定义及宏定义
* @{
*
*/
/************************************************************************************************/

/* EXTI LINE ID定义 */
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

/* EXTI LINE 分组 */
#define EXTI_DIRECT                         (0x01000000)                                    /**< 直接通道                    */
#define EXTI_CONFIG                         (0x02000000)                                    /**< 其它可配置通道(COMP1/COMP2) */
#define EXTI_GPIO                           (0x06000000)                                    /**< GPIO可配置通道              */
#define EXTI_PROPERTY_MASK                  (EXTI_DIRECT | EXTI_CONFIG | EXTI_GPIO)         /**< 通道屏蔽掩码                */

/* EXTI LINE屏蔽掩码，用于筛选EXTI LINE ID */ 
#define EXTI_LINE_MASK                      (0x0000001FU)                                   /**< EXTI LINE屏蔽掩码 */

/* EXTI LINE模式：中断、事件 */ 
#define EXTI_MODE_NONE                      (0x00000000U)                                   /**< 无中断/事件唤醒 */
#define EXTI_MODE_INTERRUPT                 (0x00000001U)                                   /**< 中断唤醒        */
#define EXTI_MODE_EVENT                     (0x00000002U)                                   /**< 事件唤醒        */
#define EXTI_MODE_INTERRUPT_EVENT           (EXTI_MODE_EVENT | EXTI_MODE_INTERRUPT)         /**< 中断/事件唤醒   */

/* EXTI LINE 触发类型定义，用于可配置通道，信号触发类型配置 */
#define EXTI_TRIGGER_NONE                   (0x00000000U)                                   /**< 不触发            */
#define EXTI_TRIGGER_RISING                 (0x00000001U)                                   /**< 上降沿触发        */
#define EXTI_TRIGGER_FALLING                (0x00000002U)                                   /**< 下降沿触发        */
#define EXTI_TRIGGER_RISING_FALLING         (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)    /**< 上升沿/下降沿触发 */

/* EXTI LINE 触发类型屏蔽掩码，用于筛选EXTI 可配置通道触发类型 */
#define EXTI_TRIGGER_MASK                   (EXTI_TRIGGER_RISING | EXTI_TRIGGER_FALLING)     /**< 触发类型屏蔽掩码 */

/* GPIO端口ID定义 */
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
* @brief    EXTI对外函数
* @{
*
*/
/************************************************************************************************/

/**
* @brief  读取EXTI通道中断挂起状态
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t 上升沿中断挂起状态
*             @arg 非0  表示已挂起
*             @arg 0 表示未挂起
*/
__STATIC_INLINE uint32_t std_exti_get_pending_status(uint32_t exti_line)
{
    return (EXTI->PIR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}


/**
* @brief  清除EXTI通道中断挂起状态
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval 无
*/
__STATIC_INLINE void std_exti_clear_pending(uint32_t exti_line)
{
    EXTI->PIR = (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  使能EXTI通道上升沿触发
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval 无
*/
__STATIC_INLINE void std_exti_rising_trigger_enable(uint32_t exti_line)
{
    EXTI->RTSR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  禁止EXTI通道上升沿触发
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval 无
*/
__STATIC_INLINE void std_exti_rising_trigger_disable(uint32_t exti_line)
{
    EXTI->RTSR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));   
}

/**
* @brief  使能EXTI通道下降沿触发
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval 无
*/
__STATIC_INLINE void std_exti_falling_trigger_enable(uint32_t exti_line)
{
    EXTI->FTSR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  禁止EXTI通道下降沿触发
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval 无
*/
__STATIC_INLINE void std_exti_falling_trigger_disable(uint32_t exti_line)
{
    EXTI->FTSR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  读取EXTI通道上升沿触发状态
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1 
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t 上升沿触发状态
*             @arg 非0  表示已使能
*             @arg 0 表示已禁止
*/
__STATIC_INLINE uint32_t std_exti_get_rising_trigger_enable(uint32_t exti_line)
{
    return (EXTI->RTSR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  读取EXTI通道下降沿触发状态
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
* @retval uint32_t 下降沿触发状态
*             @arg 非0  表示已使能
*             @arg 0 表示已禁止
*/
__STATIC_INLINE uint32_t std_exti_get_falling_trigger_enable(uint32_t exti_line)
{
    return (EXTI->FTSR & (0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  使能EXTI通道唤醒中断
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval 无
*/
__STATIC_INLINE void std_exti_interrupt_enable(uint32_t exti_line)
{
    EXTI->IMR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  禁止EXTI通道唤醒中断
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval 无
*/
__STATIC_INLINE void std_exti_interrupt_disable(uint32_t exti_line)
{
    EXTI->IMR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  使能EXTI通道唤醒事件
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval 无
*/
__STATIC_INLINE void std_exti_event_enable(uint32_t exti_line)
{
    EXTI->EMR |= (0x01U << (exti_line & EXTI_LINE_MASK));
}

/**
* @brief  禁止EXTI通道唤醒事件
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_COMP1
*             @arg EXTI_LINE_COMP2
*             @arg EXTI_LINE_LPTIM1
* @retval 无
*/
__STATIC_INLINE void std_exti_event_disable(uint32_t exti_line)
{
    EXTI->EMR &= (~(0x01U << (exti_line & EXTI_LINE_MASK)));
}

/**
* @brief  设置EXTI通道所对应GPIO端口
* @param  gpio_id  GPIO端口ID
*             @arg EXTI_GPIOA
*             @arg EXTI_GPIOB
*             @arg EXTI_GPIOC
* @param  exti_line  EXTI通道ID
*             @arg EXTI_LINE_GPIO_PIN0
*             @arg EXTI_LINE_GPIO_PIN1
*             @arg ...
*             @arg EXTI_LINE_GPIO_PIN7
* @retval 无
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
