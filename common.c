/************************************************************************************************/
/**
* @file               common.c
* @author             MCU Ecosystem Development Team
* @brief              ͨ�ú�����������ص�����ʵ�ֺ�����
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*------------------------------------------includes--------------------------------------------*/


// common.c �ļ��������
#include "common.h"

volatile uint32_t system_tick = 0;          // ϵͳʱ�Ӽ�������1ms������
uint32_t mode_start_time = 0;               // ��ǰģʽ��ʼʱ��
uint32_t touch_start_time = 0;              // ������ʼʱ��
uint32_t power_on_time = 0;                 // ����ʱ��
uint8_t touch_detected = 0;                 // ����״̬��־
uint8_t powered_on = 0;                     // ��Դ״̬
uint8_t touch_ever_detected = 0;            // �Ƿ�������⵽����
LED_Mode_t current_mode = MODE_0_DIM;       // ��ǰLEDģʽ
__IO uint32_t g_voltage = 0;                // ��ص�ѹֵ
__IO uint8_t g_interrupt_status = 0;        // ADC�ж�״̬��־
__IO uint8_t g_low_battery_detected = 0;    // �͵�������־
__IO uint32_t g_low_battery_start_time = 0; // �͵�����ʼʱ��

/* У׼ϵ������ֵ */
#define CALFACT_MAX                         (31)
#define CALFACT_MIN                         (-31)

/* ADCУ׼ϵ���ķ���λ */
#define ADC_CALFACT_SYMBOL                  (ADC_CALFACT_CALFACT_5)

/* ADC����ֵ */
#define ADC_COMPENSATION_VALUE              (*(int32_t *)(0x1FFF03CC))  

void delay_ms(uint32_t ms)
{
    uint32_t start = system_tick;
    while (system_tick - start < ms); // ����system_tick�ľ�ȷ��ʱ
}

void system_clock_config(void)
{
    std_flash_set_latency(FLASH_LATENCY_1CLK);
    std_rcc_rch_enable();
    while(std_rcc_get_rch_ready() != RCC_CSR1_RCHRDY);
    std_rcc_set_sysclk_source(RCC_SYSCLK_SRC_RCH);
    while(std_rcc_get_sysclk_source() != RCC_SYSCLK_SRC_STATUS_RCH);
    std_rcc_set_ahbdiv(RCC_HCLK_DIV1);  // AHB = 48MHz
    std_rcc_set_apbdiv(RCC_PCLK_DIV1);  // APB = 48MHz
    SystemCoreClock = 48000000;  // ����Ĭ�ϵ� RCH_VALUE/6
}

void gpio_init(void)
{
    std_gpio_init_t gpio_config = {0};

    /* ʹ��GPIOʱ�� */
    std_rcc_gpio_clk_enable(RCC_PERIPH_CLK_GPIOA | RCC_PERIPH_CLK_GPIOB);

    /* 1. ������������LED����ΪTIM3��PWM��� */
    gpio_config.mode = GPIO_MODE_ALTERNATE;      // ���ù���ģʽ
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.output_type = GPIO_OUTPUT_PUSHPULL;
    gpio_config.alternate = GPIO_AF3_TIM3;       // TIM3���ù���
    
    /* һ��������PA3(CH3)��PA4(CH2)��PA5(CH1)��PA7(CH4) */
    gpio_config.pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    std_gpio_init(GPIOA, &gpio_config);

    /* 2. ���õ�Դʹ������Ϊ��ͨ��� */
    gpio_config.mode = GPIO_MODE_OUTPUT;        // ��ͨ���ģʽ
    gpio_config.alternate = 0;           // �޸��ù���
    gpio_config.pin = LED_POWER_EN_PIN;
    std_gpio_init(LED_POWER_EN_PORT, &gpio_config);
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // ��ʼ�ص�

    /* 3. ���ð������ţ��������룩 */
    gpio_config.mode = GPIO_MODE_INPUT;
    gpio_config.pull = GPIO_PULLUP ;
    gpio_config.pin = KEY_POWER_MODE_PIN;
    gpio_config.alternate = 0;           // ȷ���޸��ù���
    std_gpio_init(KEY_POWER_MODE_PORT, &gpio_config);

    /* 4. ���ô���IC������� */
    // ����IC����ţ����������
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.pin = TCH_VDD_IO_PIN;
    gpio_config.output_type = GPIO_OUTPUT_PUSHPULL;  // �����������
    std_gpio_init(TCH_VDD_IO_PORT, &gpio_config);
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);  // ��ʼ�ϵ�

    // �������ţ��������룩
    gpio_config.mode = GPIO_MODE_INPUT;
    gpio_config.pull = GPIO_PULLDOWN;
    gpio_config.pin = TCH_DETECT_PIN;
    std_gpio_init(TCH_DETECT_PORT, &gpio_config);
    
    /* 5. ����ADC�������ţ�PB0��ΪADC_IN7�� */
    gpio_config.mode = GPIO_MODE_ANALOG;
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.pin = GPIO_PIN_0;
    std_gpio_init(GPIOB, &gpio_config);
}

void tim3_init(void) 
{
    std_tim_basic_init_t basic_init_struct = {0};
    std_tim_output_compare_init_t oc_config_struct = {0};
    
    /* TIM3ʱ��ʹ�� */
    std_rcc_apb1_clk_enable(RCC_PERIPH_CLK_TIM3);
    
    /* ����TIM3���������� */
    basic_init_struct.prescaler = TIM_PRESCALER_VALUE;
    basic_init_struct.period = TIM_PERIOD_VALUE;
    basic_init_struct.clock_div = TIM_CLOCK_DTS_DIV1;
    std_tim_init(TIM3, &basic_init_struct);
    
    /* ����TIM3Ϊ����ģʽ1 */
    std_tim_work_mode1_enable(TIM3);
    
    /* ��������ͨ�����ģʽΪPWM1ģʽ */
    oc_config_struct.output_compare_mode = TIM_OUTPUT_MODE_PWM1;
    oc_config_struct.output_pol = TIM_OUTPUT_POL_HIGH;
    oc_config_struct.output_state = TIM_OUTPUT_ENABLE;
    
    /* ʹ��ѭ����������ͨ����ȷ��ռ�ձ�һ�� */
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        oc_config_struct.pulse = PWM_DIM;  // ͳһʹ��PWM_DIM��Ϊ��ʼռ�ձ�
        std_tim_output_compare_init(TIM3, &oc_config_struct, ch);
    }
    std_tim_enable(TIM3);
}

// ��������LED����
void set_all_leds_brightness(uint16_t brightness) 
{
    for (uint8_t ch = 0; ch < 4; ch++) 
    {
        std_tim_set_ccx_value(TIM3, ch, brightness);
    }
}

// ���õ���LED����
void set_led_brightness(uint8_t channel, uint16_t brightness)
{
    if (channel < 4)
    {
        std_tim_set_ccx_value(TIM3, channel, brightness);
    }
}

void set_led_mode(LED_Mode_t mode)
{
    // �ȹر�����LED
    set_all_leds_brightness(0);
    
    switch (mode) 
    {
        case MODE_0_DIM:
            set_led_brightness(2, PWM_DIM);  // ��ɫ΢��
            break;
            
        case MODE_0_FULL:
            set_led_brightness(2, PWM_FULL);  // ��ɫȫ��
            break;
            
        case MODE_0_COMBINED_DIM:
            set_led_brightness(0, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(2, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(3, PWM_DIM);  // ����΢��
            break;
            
        case MODE_0_COMBINED_FULL:
            set_led_brightness(0, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(2, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(3, PWM_FULL);  // ����ȫ��
            break;
            
        case MODE_1_DIM:
            set_led_brightness(0, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(2, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(3, PWM_DIM);  // ����΢��
            break;
            
        case MODE_1_FULL:
            set_led_brightness(0, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(2, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(3, PWM_FULL);  // ����ȫ��
            break;
            
        case MODE_2_DIM:
            set_led_brightness(0, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(1, PWM_DIM);  // ��ɫ΢��
            set_led_brightness(3, PWM_DIM);  // ����΢��
            break;
            
        case MODE_2_FULL:
            set_led_brightness(0, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(1, PWM_FULL);  // ��ɫȫ��
            set_led_brightness(3, PWM_FULL);  // ����ȫ��
            break;
            
        default:
            break;
     }
} 

uint32_t get_TIM3_CCR3(void) 
{
    return std_tim_get_ccx_value(TIM3, TIM_CHANNEL_3);
}

void tim1_init(void) {
    std_tim_basic_init_t basic_init_struct = {0};
    uint32_t psc_value;
    
    std_rcc_apb2_clk_enable(RCC_PERIPH_CLK_TIM1);
    
    // ��̬����Ԥ��Ƶֵ��ȷ��1ms�жϣ�
    psc_value = (SystemCoreClock / 1000 / (TIM_ARR_VALUE + 1)) - 1;
    
    basic_init_struct.prescaler = psc_value;
    basic_init_struct.counter_mode = TIM_COUNTER_MODE_UP;
    basic_init_struct.period = TIM_ARR_VALUE;
    basic_init_struct.clock_div = TIM_CLOCK_DTS_DIV1;
    basic_init_struct.repeat_counter = 0x00;
    std_tim_init(TIM1, &basic_init_struct);    
    
    std_tim_clear_flag(TIM1, TIM_FLAG_UPDATE);
    std_tim_interrupt_enable(TIM1, TIM_INTERRUPT_UPDATE);
    std_tim_enable(TIM1);
}

void nvic_init(void)
{
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, NVIC_PRIO_0);
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    
    /* ʹ��ADC�ж� */
    NVIC_SetPriority(ADC_COMP_IRQn, NVIC_PRIO_1);
    NVIC_EnableIRQ(ADC_COMP_IRQn);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (std_tim_get_flag(TIM1, TIM_FLAG_UPDATE) != RESET)
    {
        std_tim_clear_flag(TIM1, TIM_FLAG_UPDATE);
        system_tick++;  // ÿ1ms����
      }
}

/**
* @brief  ADC�жϷ�����
* @retval ��
*/
void ADC_COMP_IRQHandler(void)
{
    if(std_adc_get_flag(ADC_FLAG_AWDG))
    {
        /* ���AWDG��־ */
        std_adc_clear_flag(ADC_FLAG_AWDG);
        g_interrupt_status = 1U;
    }
}

/**
* @brief  ����ADCУ׼ϵ���ľ���
* @retval ��
*/
void bsp_adc_software_calibrate(void)
{
    int32_t get_calfact = 0;
    
    /* ʹ��У׼ */
    std_adc_calibration_enable();
    
    /* �ȴ�У׼��� */
    while(std_adc_get_flag(ADC_FLAG_EOCAL) == 0U);
    
    /* ���ADCת��״̬��ȷ��֮ǰ״̬��Ӱ��ת�� */
    std_adc_clear_flag(ADC_FLAG_ALL);
    
    get_calfact = std_adc_get_calibration_factor();
    
    /* �ж�У׼ϵ������λ */
    if(get_calfact & ADC_CALFACT_SYMBOL)
    {
        /* У׼ϵ���Ǹ�ֵ��ת����32λ�з��Ÿ������������ */
        get_calfact = get_calfact | 0xFFFFFFE0;
    }
    
    /* У׼ϵ����ȥADC����ֵ��ȡ�µ�У׼ϵ�� */
    get_calfact = get_calfact - ADC_COMPENSATION_VALUE;
    
    /* �ж�У׼ϵ���Ƿ��� */
    if(get_calfact > CALFACT_MAX)
    {
        get_calfact = CALFACT_MAX;
    }
    else if(get_calfact < CALFACT_MIN)
    {
        get_calfact = CALFACT_MIN;
    }
    std_adc_calibration_factor_config(get_calfact);
}

/**
* @brief  ADC��ʼ�����ú���
* @retval ��
*/
void adc_init(void)
{
    /* ʹ��ADCʱ�� */
    std_rcc_apb2_clk_enable(RCC_PERIPH_CLK_ADC);
    
    /* ADC_CKʱ��ΪPCLK��3��Ƶ */
    std_adc_clock_config(ADC_CK_DIV3);
    
    /* �������ADC */
    std_adc_trig_sw();
    
    /* ����ת��ģʽ */
    std_adc_conversion_mode_config(ADC_SINGLE_CONVER_MODE);
    
    /* ����ʱ�����ã�119������*/
    std_adc_sampt_time_config(ADC_SAMPTIME_119CYCLES);

    /* ѡ��ͨ��7 */
    std_adc_fix_sequence_channel_enable(ADC_CHANNEL_7);
    
    /* ���Ź����ͨ��ѡ��ͨ��7 */
    std_adc_analog_watchdog_monit_channel(ADC_AWDG_CHANNEL_7);
    
    /* ���ÿ��Ź������ֵ */
    std_adc_analog_watchdog_thresholds_config(ADC_AWDG_HIGH_THRESHOLD, ADC_AWDG_LOW_THRESHOLD);
    
    /* ����waitģʽ����������δ��ʱ��ȡ��ת����� */
    std_adc_wait_mode_enable();

    /* ʹ��ADC */
    std_adc_enable();

    /* �ȴ�ADCʹ��״̬�ȶ� */
    std_delayus(ADC_EN_DELAY);

    /* ���Ź��ж�ʹ�� */
    std_adc_interrupt_enable(ADC_INTERRUPT_AWDG);
}

/**
* @brief  �͵��������� - ����LED��˸�����չػ�
* @retval ��
*/
void handle_low_battery(void)
{
    static uint32_t last_blink_time = 0;
    static uint8_t led_state = 0;
    uint32_t current_time = system_tick;
    
    // ��¼�͵�����ʼʱ��
    if (g_low_battery_detected == 0) {
        g_low_battery_detected = 1;
        g_low_battery_start_time = current_time;
        last_blink_time = current_time;
        led_state = 0;
        
        // ������ɫLED��Ϊ��ʼ��ʾ
        set_led_brightness(0, PWM_FULL);  // ��ɫȫ��
        
        // ������������ʾ����ʾ�͵�����Ϣ�Ĵ���
        // lcd_display_low_battery_warning();
    }
    
    // 5����˸����
    if (current_time - g_low_battery_start_time < LOW_BATTERY_WARNING_TIME) {
        // ����LED��˸
        if (current_time - last_blink_time >= LED_BLINK_INTERVAL) {
            last_blink_time = current_time;
            led_state = !led_state;
            
            if (led_state) {
                set_led_brightness(0, PWM_FULL);  // ��ɫLED��
            } else {
                set_led_brightness(0, 0);          // ��ɫLED��
            }
        }
    } else {
        // 5���ػ�
        set_led_brightness(0, 0);  // ȷ��LEDϨ��
        
        // ִ�йػ�ǰ��׼������
        // shutdown_preparation();
        
        // ��ʱȷ���������
        delay_ms(POWER_OFF_DELAY);
        
        // ִ�йػ� - ����Ӳ�����ʵ��
        power_off_system();
    }
}

/**
* @brief  ϵͳ�ػ����� - ����ʵ��Ӳ�����ʵ��
* @retval ��
*/
void power_off_system(void)
{
    // ʾ��1�����ʹ���ⲿ��Դ��������
    // std_gpio_reset_pin(POWER_CONTROL_PORT, POWER_CONTROL_PIN);
    
    // ʾ��2���������˯��ģʽ
    // std_pwr_enter_stop_mode(PWR_LOWPOWERREGULATOR_ON);
    
    // ʾ��3�������λ
    // std_rcc_software_reset();
    
    // �������Ӳ�����ѡ����ʵĹػ���ʽ
    // ����ֻ��ʾ������Ҫ����ʵ�����ʵ��
    
    // ʵ�ʹػ�����
    powered_on = 0;
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // �ر�LED��Դ
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // �رմ���IC��Դ
    set_all_leds_brightness(0);  // �ر�����LED
    
    // ����͹���ģʽ����ѡ��
    // std_pwr_enter_stop_mode(PWR_LOWPOWERREGULATOR_ON);
}

void power_off(void)
{
    powered_on = 0;
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);
    set_all_leds_brightness(0);
}

