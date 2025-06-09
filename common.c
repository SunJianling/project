/************************************************************************************************/
/**
* @file               common.c
* @author             MCU Ecosystem Development Team
* @brief              通用函数或本外设相关的配置实现函数。
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/*------------------------------------------includes--------------------------------------------*/


// common.c 文件添加内容
#include "common.h"

volatile uint32_t system_tick = 0;          // 系统时钟计数器（1ms递增）
uint32_t mode_start_time = 0;               // 当前模式开始时间
uint32_t touch_start_time = 0;              // 触摸开始时间
uint32_t power_on_time = 0;                 // 开机时间
uint8_t touch_detected = 0;                 // 触摸状态标志
uint8_t powered_on = 0;                     // 电源状态
uint8_t touch_ever_detected = 0;            // 是否曾经检测到触摸
LED_Mode_t current_mode = MODE_0_DIM;       // 当前LED模式
__IO uint32_t g_voltage = 0;                // 电池电压值
__IO uint8_t g_interrupt_status = 0;        // ADC中断状态标志
__IO uint8_t g_low_battery_detected = 0;    // 低电量检测标志
__IO uint32_t g_low_battery_start_time = 0; // 低电量开始时间

/* 校准系数的最值 */
#define CALFACT_MAX                         (31)
#define CALFACT_MIN                         (-31)

/* ADC校准系数的符号位 */
#define ADC_CALFACT_SYMBOL                  (ADC_CALFACT_CALFACT_5)

/* ADC补偿值 */
#define ADC_COMPENSATION_VALUE              (*(int32_t *)(0x1FFF03CC))  

void delay_ms(uint32_t ms)
{
    uint32_t start = system_tick;
    while (system_tick - start < ms); // 基于system_tick的精确延时
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
    SystemCoreClock = 48000000;  // 覆盖默认的 RCH_VALUE/6
}

void gpio_init(void)
{
    std_gpio_init_t gpio_config = {0};

    /* 使能GPIO时钟 */
    std_rcc_gpio_clk_enable(RCC_PERIPH_CLK_GPIOA | RCC_PERIPH_CLK_GPIOB);

    /* 1. 批量配置所有LED引脚为TIM3的PWM输出 */
    gpio_config.mode = GPIO_MODE_ALTERNATE;      // 复用功能模式
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.output_type = GPIO_OUTPUT_PUSHPULL;
    gpio_config.alternate = GPIO_AF3_TIM3;       // TIM3复用功能
    
    /* 一次性配置PA3(CH3)、PA4(CH2)、PA5(CH1)、PA7(CH4) */
    gpio_config.pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
    std_gpio_init(GPIOA, &gpio_config);

    /* 2. 配置电源使能引脚为普通输出 */
    gpio_config.mode = GPIO_MODE_OUTPUT;        // 普通输出模式
    gpio_config.alternate = 0;           // 无复用功能
    gpio_config.pin = LED_POWER_EN_PIN;
    std_gpio_init(LED_POWER_EN_PORT, &gpio_config);
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // 初始关电

    /* 3. 配置按键引脚（下拉输入） */
    gpio_config.mode = GPIO_MODE_INPUT;
    gpio_config.pull = GPIO_PULLUP ;
    gpio_config.pin = KEY_POWER_MODE_PIN;
    gpio_config.alternate = 0;           // 确保无复用功能
    std_gpio_init(KEY_POWER_MODE_PORT, &gpio_config);

    /* 4. 配置触摸IC相关引脚 */
    // 触摸IC供电脚（推挽输出）
    gpio_config.mode = GPIO_MODE_OUTPUT;
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.pin = TCH_VDD_IO_PIN;
    gpio_config.output_type = GPIO_OUTPUT_PUSHPULL;  // 设置推挽输出
    std_gpio_init(TCH_VDD_IO_PORT, &gpio_config);
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);  // 初始断电

    // 触摸检测脚（下拉输入）
    gpio_config.mode = GPIO_MODE_INPUT;
    gpio_config.pull = GPIO_PULLDOWN;
    gpio_config.pin = TCH_DETECT_PIN;
    std_gpio_init(TCH_DETECT_PORT, &gpio_config);
    
    /* 5. 配置ADC输入引脚（PB0作为ADC_IN7） */
    gpio_config.mode = GPIO_MODE_ANALOG;
    gpio_config.pull = GPIO_NOPULL;
    gpio_config.pin = GPIO_PIN_0;
    std_gpio_init(GPIOB, &gpio_config);
}

void tim3_init(void) 
{
    std_tim_basic_init_t basic_init_struct = {0};
    std_tim_output_compare_init_t oc_config_struct = {0};
    
    /* TIM3时钟使能 */
    std_rcc_apb1_clk_enable(RCC_PERIPH_CLK_TIM3);
    
    /* 配置TIM3计数器参数 */
    basic_init_struct.prescaler = TIM_PRESCALER_VALUE;
    basic_init_struct.period = TIM_PERIOD_VALUE;
    basic_init_struct.clock_div = TIM_CLOCK_DTS_DIV1;
    std_tim_init(TIM3, &basic_init_struct);
    
    /* 配置TIM3为工作模式1 */
    std_tim_work_mode1_enable(TIM3);
    
    /* 配置所有通道输出模式为PWM1模式 */
    oc_config_struct.output_compare_mode = TIM_OUTPUT_MODE_PWM1;
    oc_config_struct.output_pol = TIM_OUTPUT_POL_HIGH;
    oc_config_struct.output_state = TIM_OUTPUT_ENABLE;
    
    /* 使用循环配置所有通道，确保占空比一致 */
    for (uint8_t ch = 0; ch < 4; ch++)
    {
        oc_config_struct.pulse = PWM_DIM;  // 统一使用PWM_DIM作为初始占空比
        std_tim_output_compare_init(TIM3, &oc_config_struct, ch);
    }
    std_tim_enable(TIM3);
}

// 设置所有LED亮度
void set_all_leds_brightness(uint16_t brightness) 
{
    for (uint8_t ch = 0; ch < 4; ch++) 
    {
        std_tim_set_ccx_value(TIM3, ch, brightness);
    }
}

// 设置单个LED亮度
void set_led_brightness(uint8_t channel, uint16_t brightness)
{
    if (channel < 4)
    {
        std_tim_set_ccx_value(TIM3, channel, brightness);
    }
}

void set_led_mode(LED_Mode_t mode)
{
    // 先关闭所有LED
    set_all_leds_brightness(0);
    
    switch (mode) 
    {
        case MODE_0_DIM:
            set_led_brightness(2, PWM_DIM);  // 蓝色微亮
            break;
            
        case MODE_0_FULL:
            set_led_brightness(2, PWM_FULL);  // 蓝色全亮
            break;
            
        case MODE_0_COMBINED_DIM:
            set_led_brightness(0, PWM_DIM);  // 红色微亮
            set_led_brightness(2, PWM_DIM);  // 蓝色微亮
            set_led_brightness(3, PWM_DIM);  // 红外微亮
            break;
            
        case MODE_0_COMBINED_FULL:
            set_led_brightness(0, PWM_FULL);  // 红色全亮
            set_led_brightness(2, PWM_FULL);  // 蓝色全亮
            set_led_brightness(3, PWM_FULL);  // 红外全亮
            break;
            
        case MODE_1_DIM:
            set_led_brightness(0, PWM_DIM);  // 红色微亮
            set_led_brightness(2, PWM_DIM);  // 蓝色微亮
            set_led_brightness(3, PWM_DIM);  // 红外微亮
            break;
            
        case MODE_1_FULL:
            set_led_brightness(0, PWM_FULL);  // 红色全亮
            set_led_brightness(2, PWM_FULL);  // 蓝色全亮
            set_led_brightness(3, PWM_FULL);  // 红外全亮
            break;
            
        case MODE_2_DIM:
            set_led_brightness(0, PWM_DIM);  // 红色微亮
            set_led_brightness(1, PWM_DIM);  // 黄色微亮
            set_led_brightness(3, PWM_DIM);  // 红外微亮
            break;
            
        case MODE_2_FULL:
            set_led_brightness(0, PWM_FULL);  // 红色全亮
            set_led_brightness(1, PWM_FULL);  // 黄色全亮
            set_led_brightness(3, PWM_FULL);  // 红外全亮
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
    
    // 动态计算预分频值（确保1ms中断）
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
    
    /* 使能ADC中断 */
    NVIC_SetPriority(ADC_COMP_IRQn, NVIC_PRIO_1);
    NVIC_EnableIRQ(ADC_COMP_IRQn);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if (std_tim_get_flag(TIM1, TIM_FLAG_UPDATE) != RESET)
    {
        std_tim_clear_flag(TIM1, TIM_FLAG_UPDATE);
        system_tick++;  // 每1ms递增
      }
}

/**
* @brief  ADC中断服务函数
* @retval 无
*/
void ADC_COMP_IRQHandler(void)
{
    if(std_adc_get_flag(ADC_FLAG_AWDG))
    {
        /* 清除AWDG标志 */
        std_adc_clear_flag(ADC_FLAG_AWDG);
        g_interrupt_status = 1U;
    }
}

/**
* @brief  提升ADC校准系数的精度
* @retval 无
*/
void bsp_adc_software_calibrate(void)
{
    int32_t get_calfact = 0;
    
    /* 使能校准 */
    std_adc_calibration_enable();
    
    /* 等待校准完成 */
    while(std_adc_get_flag(ADC_FLAG_EOCAL) == 0U);
    
    /* 清除ADC转换状态，确保之前状态不影响转换 */
    std_adc_clear_flag(ADC_FLAG_ALL);
    
    get_calfact = std_adc_get_calibration_factor();
    
    /* 判断校准系数符号位 */
    if(get_calfact & ADC_CALFACT_SYMBOL)
    {
        /* 校准系数是负值，转换成32位有符号负数，方便计算 */
        get_calfact = get_calfact | 0xFFFFFFE0;
    }
    
    /* 校准系数减去ADC补偿值获取新的校准系数 */
    get_calfact = get_calfact - ADC_COMPENSATION_VALUE;
    
    /* 判断校准系数是否超限 */
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
* @brief  ADC初始化配置函数
* @retval 无
*/
void adc_init(void)
{
    /* 使能ADC时钟 */
    std_rcc_apb2_clk_enable(RCC_PERIPH_CLK_ADC);
    
    /* ADC_CK时钟为PCLK的3分频 */
    std_adc_clock_config(ADC_CK_DIV3);
    
    /* 软件触发ADC */
    std_adc_trig_sw();
    
    /* 单次转换模式 */
    std_adc_conversion_mode_config(ADC_SINGLE_CONVER_MODE);
    
    /* 采样时间配置，119个周期*/
    std_adc_sampt_time_config(ADC_SAMPTIME_119CYCLES);

    /* 选择通道7 */
    std_adc_fix_sequence_channel_enable(ADC_CHANNEL_7);
    
    /* 看门狗监控通道选择通道7 */
    std_adc_analog_watchdog_monit_channel(ADC_AWDG_CHANNEL_7);
    
    /* 配置看门狗监控阈值 */
    std_adc_analog_watchdog_thresholds_config(ADC_AWDG_HIGH_THRESHOLD, ADC_AWDG_LOW_THRESHOLD);
    
    /* 配置wait模式，避免数据未及时读取，转换溢出 */
    std_adc_wait_mode_enable();

    /* 使能ADC */
    std_adc_enable();

    /* 等待ADC使能状态稳定 */
    std_delayus(ADC_EN_DELAY);

    /* 看门狗中断使能 */
    std_adc_interrupt_enable(ADC_INTERRUPT_AWDG);
}

/**
* @brief  低电量处理函数 - 控制LED闪烁并最终关机
* @retval 无
*/
void handle_low_battery(void)
{
    static uint32_t last_blink_time = 0;
    static uint8_t led_state = 0;
    uint32_t current_time = system_tick;
    
    // 记录低电量开始时间
    if (g_low_battery_detected == 0) {
        g_low_battery_detected = 1;
        g_low_battery_start_time = current_time;
        last_blink_time = current_time;
        led_state = 0;
        
        // 点亮红色LED作为初始提示
        set_led_brightness(0, PWM_FULL);  // 红色全亮
        
        // 这里可以添加显示屏显示低电量信息的代码
        // lcd_display_low_battery_warning();
    }
    
    // 5秒闪烁警告
    if (current_time - g_low_battery_start_time < LOW_BATTERY_WARNING_TIME) {
        // 控制LED闪烁
        if (current_time - last_blink_time >= LED_BLINK_INTERVAL) {
            last_blink_time = current_time;
            led_state = !led_state;
            
            if (led_state) {
                set_led_brightness(0, PWM_FULL);  // 红色LED亮
            } else {
                set_led_brightness(0, 0);          // 红色LED灭
            }
        }
    } else {
        // 5秒后关机
        set_led_brightness(0, 0);  // 确保LED熄灭
        
        // 执行关机前的准备工作
        // shutdown_preparation();
        
        // 延时确保操作完成
        delay_ms(POWER_OFF_DELAY);
        
        // 执行关机 - 根据硬件设计实现
        power_off_system();
    }
}

/**
* @brief  系统关机函数 - 根据实际硬件设计实现
* @retval 无
*/
void power_off_system(void)
{
    // 示例1：如果使用外部电源控制引脚
    // std_gpio_reset_pin(POWER_CONTROL_PORT, POWER_CONTROL_PIN);
    
    // 示例2：进入深度睡眠模式
    // std_pwr_enter_stop_mode(PWR_LOWPOWERREGULATOR_ON);
    
    // 示例3：软件复位
    // std_rcc_software_reset();
    
    // 根据你的硬件设计选择合适的关机方式
    // 这里只是示例，需要根据实际情况实现
    
    // 实际关机操作
    powered_on = 0;
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);  // 关闭LED电源
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);     // 关闭触摸IC电源
    set_all_leds_brightness(0);  // 关闭所有LED
    
    // 进入低功耗模式（可选）
    // std_pwr_enter_stop_mode(PWR_LOWPOWERREGULATOR_ON);
}

void power_off(void)
{
    powered_on = 0;
    std_gpio_set_pin(LED_POWER_EN_PORT, LED_POWER_EN_PIN);
    std_gpio_reset_pin(TCH_VDD_IO_PORT, TCH_VDD_IO_PIN);
    set_all_leds_brightness(0);
}

