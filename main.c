/************************************************************************************************/
/**
* @file               main.c
* @author             MCU Ecosystem Development Team
* @brief              ��ʾ��չʾGPIO���÷���ÿ300����LED��״̬��תһ�Ρ�
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*------------------------------------------includes--------------------------------------------*/

// main.c �ļ��޸�����
#include "common.h"

int main(void)
{
    uint8_t key_state = 1;     // ����״̬
    
    // ϵͳ��ʼ��
    system_clock_config();     // ����ϵͳʱ��
    gpio_init();               // ��ʼ��GPIO
    tim3_init();               // ��ʼ��TIM3�����ں���������TIM3��
    tim1_init();               // ��ʼ��TIM1��ϵͳʱ�ӣ�
    nvic_init();               // ��ʼ���ж� 
    adc_init();                // ��ʼ��ADC
    bsp_adc_software_calibrate(); // ADC���У׼
    
    // ����ģ�⿴�Ź���ֵΪ3.2V��Ӧ��ADCֵ
    std_adc_analog_watchdog_thresholds_config(0xFFFU, ADC_LOW_THRESHOLD);
    
    // ��ʼ״̬������LED�رգ���Դ�ر�
    set_all_leds_brightness(0);
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // �ر�LED��Դ
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // �رմ���IC��Դ
    
    while (1)
    {
        /* ����ADCת�� */
        std_adc_start_conversion();
        
        /* �ȴ�ADCͨ��ת����� */
        while(std_adc_get_flag(ADC_FLAG_EOC) == 0);
        
        /* ���EOC��־ */
        std_adc_clear_flag(ADC_FLAG_EOC);
        
        /* ��ȡ����ֵ��ת��Ϊ��ѹֵ����λmV*/
        g_voltage = std_adc_get_conversion_value() * VREF_ADC_VDDA_VOLTAGE / ADC_CONVER_SCALE;
        
        // ��ȡ����״̬
        uint8_t key_now = std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN);
        
        // ��ⰴ���½��أ����£�
        if (key_state == 1 && key_now == 0) 
        {
            delay_ms(10);  // ������ʱ
            
            // ȷ�ϰ���ȷʵ����
            if (std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN) == 0) 
            {
                uint32_t hold_time = 0;
                
                // ��ⳤ��ʱ��
                while (std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN) == 0) {
                    delay_ms(10);
                    hold_time += 10;
                    if (hold_time >= 1000) break;  // ��������1��
                }
                
                if (hold_time >= 1000) {  // ���������ػ�
                    if (!powered_on) {
                        // ��������
                        powered_on = 1;
                        std_gpio_reset_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // ��LED��Դ
                        std_gpio_set_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);        // �򿪴���IC��Դ
                        
                        current_mode = MODE_0_DIM;  // Ĭ��ģʽ������΢��
                        set_led_mode(current_mode);
                        
                        // ���ü�ʱ��
                        mode_start_time = system_tick;
                        touch_detected = 0;
                        touch_ever_detected = 0;
                        power_on_time = system_tick;  // ��¼����ʱ��
                        
                        // ���õ͵�����־
                        g_low_battery_detected = 0;
                        
                    } else {
                        // �ػ�����
                        powered_on = 0;
                        std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // �ر�LED��Դ
                        std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // �رմ���IC��Դ
                        set_all_leds_brightness(0);  // �ر�����LED
                        
                        // ���õ͵�����־
                        g_low_battery_detected = 0;
                    }
                } else 
                {  // �̰����л�LEDģʽ
                    if (powered_on) 
                    {
                        // ѭ���л�ģʽ
                        switch (current_mode) 
                        {
                            case MODE_0_DIM:
                            case MODE_0_FULL:
                            case MODE_0_COMBINED_DIM:
                            case MODE_0_COMBINED_FULL:
                                current_mode = MODE_1_DIM;  // �л���ģʽ1΢��
                                break;
                                
                            case MODE_1_DIM:
                            case MODE_1_FULL:
                                current_mode = MODE_2_DIM;  // �л���ģʽ2΢��
                                break;
                                
                            case MODE_2_DIM:
                            case MODE_2_FULL:
                                current_mode = MODE_0_DIM;  // �ص�ģʽ0΢��
                                break;
                                
                            default:
                                current_mode = MODE_0_DIM;
                                break;
                        }
                        
                        set_led_mode(current_mode);
                        
                        // ���ü�ʱ��
                        mode_start_time = system_tick;
                        touch_detected = 0;
                        touch_ever_detected = 0;
                    }
                }
            }
        }
        key_state = key_now;  // ���°���״̬
        
        // ������⣨���ڿ���״̬�£�
        if (powered_on) {
            uint8_t touch_status = std_gpio_get_input_pin(TCH_DETECT_PORT, TCH_DETECT_PIN);
            
            // ����״̬�仯����
            if (touch_status && !touch_detected) {
                // �״μ�⵽����
                touch_detected = 1;
                touch_start_time = system_tick;
                
                // ��΢���л���ȫ��
                switch (current_mode) {
                    case MODE_0_DIM:
                        current_mode = MODE_0_FULL;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_0_COMBINED_DIM:
                        current_mode = MODE_0_COMBINED_FULL;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_1_DIM:
                        current_mode = MODE_1_FULL;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_2_DIM:
                        current_mode = MODE_2_FULL;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    default:
                        // �Ѿ���ȫ��״̬����������
                        break;
                }
                
                set_led_mode(current_mode);
            } 
            // �����ͷŴ���
            else if (!touch_status && touch_detected) {
                touch_detected = 0;
                
                // ��ȫ���л���΢��
                switch (current_mode) {
                    case MODE_0_FULL:
                        current_mode = MODE_0_DIM;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_0_COMBINED_FULL:
                        current_mode = MODE_0_COMBINED_DIM;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_1_FULL:
                        current_mode = MODE_1_DIM;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    case MODE_2_FULL:
                        current_mode = MODE_2_DIM;
                        mode_start_time = system_tick;  // ���ü�ʱ��
                        break;
                    default:
                        // �Ѿ���΢��״̬����������
                        break;
                }
                
                set_led_mode(current_mode);
            }
        }
        
        // ��ʱ�����߼�
        if (powered_on) {
            uint32_t elapsed_time = system_tick - mode_start_time;
            uint32_t total_run_time = system_tick - power_on_time;
            
            // ������ʱ�䳬ʱ��⣨����ģʽͨ�ã�
            if (total_run_time >= MAX_RUN_TIME_MS) {
                power_off();
                continue;
            }
            
            // ģʽ0�����ⶨʱ�߼�
            if (current_mode == MODE_0_FULL) {
                // ���ڴ���״̬�¼�ʱ
                if (touch_detected && elapsed_time >= MODE0_PHASE1_TIME_MS) {
                    // 15����л�����Ϲ�ȫ��
                    current_mode = MODE_0_COMBINED_FULL;
                    set_led_mode(current_mode);
                }
            }
            else if (current_mode == MODE_0_COMBINED_FULL) {
                // ��Ϲ�׶μ�ʱ�������Ƿ�����
                if (elapsed_time >= (MODE0_PHASE1_TIME_MS + MODE0_PHASE2_TIME_MS)) {
                    power_off();
                }
            }
            
            // ģʽ1/2��ʱ���
            else if ((current_mode == MODE_1_FULL || current_mode == MODE_2_FULL) && touch_detected) {
                if (elapsed_time >= MODE1_2_TOTAL_TIME_MS) {
                    power_off();
                }
            }
        }
        
        // ���͵�����־
        if(g_interrupt_status == 0x01U)
        {
            g_interrupt_status = 0;
            
            // ��⵽��ѹ������ֵ����ʼ�͵�������
            if (!g_low_battery_detected && powered_on) {
                handle_low_battery();
            }
        }
        
        // ����Ѿ����ڵ͵�������״̬����������
        if (g_low_battery_detected && powered_on) {
            handle_low_battery();
        }
        
        // ��ʱ������������
        delay_ms(10);
    }
}
