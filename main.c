/************************************************************************************************/
/**
* @file               main.c
* @author             MCU Ecosystem Development Team
* @brief              该示例展示GPIO的用法。每300毫秒LED的状态翻转一次。
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*------------------------------------------includes--------------------------------------------*/

// main.c 文件修改内容
#include "common.h"

int main(void)
{
    uint8_t key_state = 1;     // 按键状态
    
    // 系统初始化
    system_clock_config();     // 配置系统时钟
    gpio_init();               // 初始化GPIO
    tim3_init();               // 初始化TIM3（已在函数内启用TIM3）
    tim1_init();               // 初始化TIM1（系统时钟）
    nvic_init();               // 初始化中断 
    adc_init();                // 初始化ADC
    bsp_adc_software_calibrate(); // ADC软件校准
    
    // 更新模拟看门狗阈值为3.2V对应的ADC值
    std_adc_analog_watchdog_thresholds_config(0xFFFU, ADC_LOW_THRESHOLD);
    
    // 初始状态：所有LED关闭，电源关闭
    set_all_leds_brightness(0);
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // 关闭LED电源
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // 关闭触摸IC电源
    
    while (1)
    {
        /* 启动ADC转换 */
        std_adc_start_conversion();
        
        /* 等待ADC通道转换完成 */
        while(std_adc_get_flag(ADC_FLAG_EOC) == 0);
        
        /* 清除EOC标志 */
        std_adc_clear_flag(ADC_FLAG_EOC);
        
        /* 获取采样值并转换为电压值，单位mV*/
        g_voltage = std_adc_get_conversion_value() * VREF_ADC_VDDA_VOLTAGE / ADC_CONVER_SCALE;
        
        // 读取按键状态
        uint8_t key_now = std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN);
        
        // 检测按键下降沿（按下）
        if (key_state == 1 && key_now == 0) 
        {
            delay_ms(10);  // 防抖延时
            
            // 确认按键确实按下
            if (std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN) == 0) 
            {
                uint32_t hold_time = 0;
                
                // 检测长按时间
                while (std_gpio_get_input_pin(KEY_POWER_MODE_PORT, KEY_POWER_MODE_PIN) == 0) {
                    delay_ms(10);
                    hold_time += 10;
                    if (hold_time >= 1000) break;  // 长按超过1秒
                }
                
                if (hold_time >= 1000) {  // 长按：开关机
                    if (!powered_on) {
                        // 开机操作
                        powered_on = 1;
                        std_gpio_reset_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // 打开LED电源
                        std_gpio_set_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);        // 打开触摸IC电源
                        
                        current_mode = MODE_0_DIM;  // 默认模式：蓝光微亮
                        set_led_mode(current_mode);
                        
                        // 重置计时器
                        mode_start_time = system_tick;
                        touch_detected = 0;
                        touch_ever_detected = 0;
                        power_on_time = system_tick;  // 记录开机时间
                        
                        // 重置低电量标志
                        g_low_battery_detected = 0;
                        
                    } else {
                        // 关机操作
                        powered_on = 0;
                        std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // 关闭LED电源
                        std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // 关闭触摸IC电源
                        set_all_leds_brightness(0);  // 关闭所有LED
                        
                        // 重置低电量标志
                        g_low_battery_detected = 0;
                    }
                } else 
                {  // 短按：切换LED模式
                    if (powered_on) 
                    {
                        // 循环切换模式
                        switch (current_mode) 
                        {
                            case MODE_0_DIM:
                            case MODE_0_FULL:
                            case MODE_0_COMBINED_DIM:
                            case MODE_0_COMBINED_FULL:
                                current_mode = MODE_1_DIM;  // 切换到模式1微亮
                                break;
                                
                            case MODE_1_DIM:
                            case MODE_1_FULL:
                                current_mode = MODE_2_DIM;  // 切换到模式2微亮
                                break;
                                
                            case MODE_2_DIM:
                            case MODE_2_FULL:
                                current_mode = MODE_0_DIM;  // 回到模式0微亮
                                break;
                                
                            default:
                                current_mode = MODE_0_DIM;
                                break;
                        }
                        
                        set_led_mode(current_mode);
                        
                        // 重置计时器
                        mode_start_time = system_tick;
                        touch_detected = 0;
                        touch_ever_detected = 0;
                    }
                }
            }
        }
        key_state = key_now;  // 更新按键状态
        
        // 触摸检测（仅在开机状态下）
        if (powered_on) {
            uint8_t touch_status = std_gpio_get_input_pin(TCH_DETECT_PORT, TCH_DETECT_PIN);
            
            // 触摸状态变化处理
            if (touch_status && !touch_detected) {
                // 首次检测到触摸
                touch_detected = 1;
                touch_start_time = system_tick;
                
                // 从微亮切换到全亮
                switch (current_mode) {
                    case MODE_0_DIM:
                        current_mode = MODE_0_FULL;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_0_COMBINED_DIM:
                        current_mode = MODE_0_COMBINED_FULL;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_1_DIM:
                        current_mode = MODE_1_FULL;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_2_DIM:
                        current_mode = MODE_2_FULL;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    default:
                        // 已经是全亮状态，不做处理
                        break;
                }
                
                set_led_mode(current_mode);
            } 
            // 触摸释放处理
            else if (!touch_status && touch_detected) {
                touch_detected = 0;
                
                // 从全亮切换到微亮
                switch (current_mode) {
                    case MODE_0_FULL:
                        current_mode = MODE_0_DIM;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_0_COMBINED_FULL:
                        current_mode = MODE_0_COMBINED_DIM;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_1_FULL:
                        current_mode = MODE_1_DIM;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    case MODE_2_FULL:
                        current_mode = MODE_2_DIM;
                        mode_start_time = system_tick;  // 重置计时器
                        break;
                    default:
                        // 已经是微亮状态，不做处理
                        break;
                }
                
                set_led_mode(current_mode);
            }
        }
        
        // 定时控制逻辑
        if (powered_on) {
            uint32_t elapsed_time = system_tick - mode_start_time;
            uint32_t total_run_time = system_tick - power_on_time;
            
            // 总运行时间超时检测（所有模式通用）
            if (total_run_time >= MAX_RUN_TIME_MS) {
                power_off();
                continue;
            }
            
            // 模式0的特殊定时逻辑
            if (current_mode == MODE_0_FULL) {
                // 仅在触摸状态下计时
                if (touch_detected && elapsed_time >= MODE0_PHASE1_TIME_MS) {
                    // 15秒后切换到组合光全亮
                    current_mode = MODE_0_COMBINED_FULL;
                    set_led_mode(current_mode);
                }
            }
            else if (current_mode == MODE_0_COMBINED_FULL) {
                // 组合光阶段计时（无论是否触摸）
                if (elapsed_time >= (MODE0_PHASE1_TIME_MS + MODE0_PHASE2_TIME_MS)) {
                    power_off();
                }
            }
            
            // 模式1/2超时检测
            else if ((current_mode == MODE_1_FULL || current_mode == MODE_2_FULL) && touch_detected) {
                if (elapsed_time >= MODE1_2_TOTAL_TIME_MS) {
                    power_off();
                }
            }
        }
        
        // 检查低电量标志
        if(g_interrupt_status == 0x01U)
        {
            g_interrupt_status = 0;
            
            // 检测到电压低于阈值，开始低电量处理
            if (!g_low_battery_detected && powered_on) {
                handle_low_battery();
            }
        }
        
        // 如果已经处于低电量处理状态，继续处理
        if (g_low_battery_detected && powered_on) {
            handle_low_battery();
        }
        
        // 延时处理其他任务
        delay_ms(10);
    }
}
