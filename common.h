/************************************************************************************************/
/**
* @file               common.h
* @author             MCU Ecosystem Development Team
* @brief              COMMONͷ�ļ���
*                           
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

// common.h �ļ��������
#ifndef __COMMON_H__
#define __COMMON_H__

#include "ciu32f003_std_rcc.h"
#include "ciu32f003_std_gpio.h"
#include "ciu32f003_std_flash.h"
#include "ciu32f003_std_adc.h"  // ���ADCͷ�ļ�
#include <stdint.h>

/* LEDģʽ���� */
typedef enum {
    MODE_0_DIM,                   // ģʽ0������΢��
    MODE_0_FULL,                  // ģʽ0������ȫ��
    MODE_0_COMBINED_DIM,          // ģʽ0����+��+������Ϲ�΢��
    MODE_0_COMBINED_FULL,         // ģʽ0����+��+������Ϲ�ȫ��
    
    MODE_1_DIM,                   // ģʽ1����+��+����΢��
    MODE_1_FULL,                  // ģʽ1����+��+����ȫ��
    
    MODE_2_DIM,                   // ģʽ2����+��+����΢��
    MODE_2_FULL                   // ģʽ2����+��+����ȫ��
} LED_Mode_t;

/* LED IO��ض��� */
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

/* �������� */
#define KEY_POWER_MODE_PORT GPIOB
#define KEY_POWER_MODE_PIN  GPIO_PIN_0

/* ����IC��ض��� */
#define TCH_VDD_IO_PORT       GPIOB
#define TCH_VDD_IO_PIN        GPIO_PIN_4    // PB4 ����IC�����
#define TCH_DETECT_PORT       GPIOB
#define TCH_DETECT_PIN        GPIO_PIN_5    // PB5 ��������
#define MCU_RESET_PORT        GPIOB
#define MCU_RESET_PIN         GPIO_PIN_5    // PB5 ��λ���ƽţ��봥����⸴�ã�

#define TIM_PERIOD_VALUE        (0x00FFU)
#define TIM_PRESCALER_VALUE     (0x05U)
/* PWMռ�ձȶ��� */
#define PWM_DIM             64              // ΢��ռ�ձȣ�Լ25%��
#define PWM_FULL            255             // ȫ��ռ�ձȣ�100%��

/* TIM3ͨ������ֵ��ͳһΪ��ͬռ�ձȣ� */
#define TIM_PULSE1_VALUE    PWM_DIM         // ����ͨ��ʹ����ͬռ�ձ�
#define TIM_PULSE2_VALUE    PWM_DIM
#define TIM_PULSE3_VALUE    PWM_DIM
#define TIM_PULSE4_VALUE    PWM_DIM

#define TIM_ARR_VALUE           999 

// ����ģʽ���뼶��
#define MODE0_PHASE1_TIME_MS   (15 * 1000)   // ���� 15 ��
#define MODE0_PHASE2_TIME_MS   (5 * 1000)    // ��Ϲ� 5 ��
#define MODE1_2_TOTAL_TIME_MS  (20 * 1000)   // ģʽ1/2ȫ�� 20 ��
#define MAX_RUN_TIME_MS        (30 * 1000)   // �����ʱ�� 30 ��

// ��ʽģʽ�����Ӽ���ȡ��ע�ͼ����л���
// #define MODE0_PHASE1_TIME_MS   (15 * 60 * 1000)   // ���� 15 ����
// #define MODE0_PHASE2_TIME_MS   (5 * 60 * 1000)    // ��Ϲ� 5 ����
// #define MODE1_2_TOTAL_TIME_MS  (20 * 60 * 1000)   // ģʽ1/2ȫ�� 20 ����
// #define MAX_RUN_TIME_MS        (30 * 60 * 1000)   // �����ʱ�� 30 ����

/* ADC��ض��� */
#define ADC_AWDG_HIGH_THRESHOLD       (0xFFFU * 7 /8)   /* ģ�⿴�Ź���ص�ѹ����ֵ */
#define ADC_AWDG_LOW_THRESHOLD        (0xFFFU * 3 /8)   /* ģ�⿴�Ź���ص�ѹ����ֵ */
#define VREF_ADC_VDDA_VOLTAGE         (3300U)           /* ADC�ο���ѹΪVDDA��3300mV */
#define LOW_BATTERY_THRESHOLD         (3200U)           /* �͵�����ֵ��3.2V */
#define ADC_LOW_THRESHOLD             (3949U)           /* 3.2V��Ӧ��ADCֵ */
#define ADC_CONVER_SCALE              (4095U)           /* 12λADC������ֵ */

/* �͵���������ض��� */
#define LED_BLINK_INTERVAL            (500U)            /* 500ms ��˸��� */
#define LOW_BATTERY_WARNING_TIME      (5000U)           /* 5�뾯��ʱ�� */
#define POWER_OFF_DELAY               (1000U)           /* �ػ�ǰ�ӳ� */

/* ȫ�ֱ������� */
extern volatile uint32_t system_tick;      // ϵͳʱ�Ӽ�����
extern uint32_t mode_start_time;            // ģʽ��ʼʱ��
extern uint32_t touch_start_time;           // ������ʼʱ��
extern uint32_t power_on_time;              // ����ʱ��
extern uint8_t touch_detected;              // ����״̬��־
extern uint8_t touch_ever_detected;         // �Ƿ�������⵽����
extern uint8_t powered_on;                  // ��Դ״̬
extern LED_Mode_t current_mode;             // ��ǰLEDģʽ
extern __IO uint32_t g_voltage;             // ��ص�ѹֵ
extern __IO uint8_t g_interrupt_status;     // ADC�ж�״̬��־
extern __IO uint8_t g_low_battery_detected; // �͵�������־
extern __IO uint32_t g_low_battery_start_time; // �͵�����ʼʱ��

/* �������� */
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
void adc_init(void);                         // ADC��ʼ��
void bsp_adc_software_calibrate(void);       // ADC���У׼
void handle_low_battery(void);               // �͵�������
void power_off_system(void);                 // ϵͳ�ػ�

#endif
