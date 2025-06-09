/************************************************************************************************/
/**
* @file               common.h
* @author             MCU Ecosystem Development Team
* @brief              COMMON头文件。
*                           
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

// common.h 文件添加内容
#ifndef __COMMON_H__
#define __COMMON_H__

#include "ciu32f003_std_rcc.h"
#include "ciu32f003_std_gpio.h"
#include "ciu32f003_std_flash.h"
#include "ciu32f003_std_adc.h"  // 添加ADC头文件
#include <stdint.h>

/* LED模式定义 */
typedef enum {
    MODE_0_DIM,                   // 模式0：蓝光微亮
    MODE_0_FULL,                  // 模式0：蓝光全亮
    MODE_0_COMBINED_DIM,          // 模式0：红+蓝+红外组合光微亮
    MODE_0_COMBINED_FULL,         // 模式0：红+蓝+红外组合光全亮
    
    MODE_1_DIM,                   // 模式1：红+蓝+红外微亮
    MODE_1_FULL,                  // 模式1：红+蓝+红外全亮
    
    MODE_2_DIM,                   // 模式2：红+黄+红外微亮
    MODE_2_FULL                   // 模式2：红+黄+红外全亮
} LED_Mode_t;

/* LED IO相关定义 */
#define LED_RED_PORT        GPIOA
#define LED_RED_PIN         GPIO_PIN_5
#define LED_YELLOW_PORT     GPIOA
#define LED_YELLOW_PIN      GPIO_PIN_4
#define LED_BLUE_PORT       GPIOA
#define LED_BLUE_PIN        GPIO_PIN_3
#define LED_IR_PORT         GPIOA
#define LED_IR_PIN          GPIO_PIN_7
#define LED_POWER_EN_PORT   GPIOB
#define LED_POWER_EN_PIN    GPIO_PIN_7

/* 按键定义 */
#define KEY_POWER_MODE_PORT GPIOB
#define KEY_POWER_MODE_PIN  GPIO_PIN_0

/* 触摸IC相关定义 */
#define TCH_VDD_IO_PORT       GPIOB
#define TCH_VDD_IO_PIN        GPIO_PIN_4    // PB4 触摸IC供电脚
#define TCH_DETECT_PORT       GPIOB
#define TCH_DETECT_PIN        GPIO_PIN_5    // PB5 触摸检测脚
#define MCU_RESET_PORT        GPIOB
#define MCU_RESET_PIN         GPIO_PIN_5    // PB5 复位控制脚（与触摸检测复用）

#define TIM_PERIOD_VALUE        (0x00FFU)
#define TIM_PRESCALER_VALUE     (0x05U)
/* PWM占空比定义 */
#define PWM_DIM             64              // 微亮占空比（约25%）
#define PWM_FULL            255             // 全亮占空比（100%）

/* TIM3通道脉冲值（统一为相同占空比） */
#define TIM_PULSE1_VALUE    PWM_DIM         // 所有通道使用相同占空比
#define TIM_PULSE2_VALUE    PWM_DIM
#define TIM_PULSE3_VALUE    PWM_DIM
#define TIM_PULSE4_VALUE    PWM_DIM

#define TIM_ARR_VALUE           999 

// 测试模式（秒级）
#define MODE0_PHASE1_TIME_MS   (15 * 1000)   // 蓝光 15 秒
#define MODE0_PHASE2_TIME_MS   (5 * 1000)    // 组合光 5 秒
#define MODE1_2_TOTAL_TIME_MS  (20 * 1000)   // 模式1/2全亮 20 秒
#define MAX_RUN_TIME_MS        (30 * 1000)   // 最长运行时间 30 秒

// 正式模式（分钟级，取消注释即可切换）
// #define MODE0_PHASE1_TIME_MS   (15 * 60 * 1000)   // 蓝光 15 分钟
// #define MODE0_PHASE2_TIME_MS   (5 * 60 * 1000)    // 组合光 5 分钟
// #define MODE1_2_TOTAL_TIME_MS  (20 * 60 * 1000)   // 模式1/2全亮 20 分钟
// #define MAX_RUN_TIME_MS        (30 * 60 * 1000)   // 最长运行时间 30 分钟

/* ADC相关定义 */
#define ADC_AWDG_HIGH_THRESHOLD       (0xFFFU * 7 /8)   /* 模拟看门狗监控电压高阈值 */
#define ADC_AWDG_LOW_THRESHOLD        (0xFFFU * 3 /8)   /* 模拟看门狗监控电压低阈值 */
#define VREF_ADC_VDDA_VOLTAGE         (3300U)           /* ADC参考电压为VDDA：3300mV */
#define LOW_BATTERY_THRESHOLD         (3200U)           /* 低电量阈值：3.2V */
#define ADC_LOW_THRESHOLD             (3949U)           /* 3.2V对应的ADC值 */
#define ADC_CONVER_SCALE              (4095U)           /* 12位ADC满量程值 */

/* 低电量处理相关定义 */
#define LED_BLINK_INTERVAL            (500U)            /* 500ms 闪烁间隔 */
#define LOW_BATTERY_WARNING_TIME      (5000U)           /* 5秒警告时间 */
#define POWER_OFF_DELAY               (1000U)           /* 关机前延迟 */

/* 全局变量声明 */
extern volatile uint32_t system_tick;      // 系统时钟计数器
extern uint32_t mode_start_time;            // 模式开始时间
extern uint32_t touch_start_time;           // 触摸开始时间
extern uint32_t power_on_time;              // 开机时间
extern uint8_t touch_detected;              // 触摸状态标志
extern uint8_t touch_ever_detected;         // 是否曾经检测到触摸
extern uint8_t powered_on;                  // 电源状态
extern LED_Mode_t current_mode;             // 当前LED模式
extern __IO uint32_t g_voltage;             // 电池电压值
extern __IO uint8_t g_interrupt_status;     // ADC中断状态标志
extern __IO uint8_t g_low_battery_detected; // 低电量检测标志
extern __IO uint32_t g_low_battery_start_time; // 低电量开始时间

/* 函数声明 */
void system_clock_config(void);
void gpio_init(void);
void delay_ms(uint32_t ms);
void set_all_leds_brightness(uint16_t brightness);
void set_led_mode(LED_Mode_t mode);
void set_led_brightness(uint8_t channel, uint16_t brightness);
void tim3_init(void);
void tim1_init(void);
void nvic_init(void);
void power_off(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void adc_init(void);                         // ADC初始化
void bsp_adc_software_calibrate(void);       // ADC软件校准
void handle_low_battery(void);               // 低电量处理
void power_off_system(void);                 // 系统关机

#endif
