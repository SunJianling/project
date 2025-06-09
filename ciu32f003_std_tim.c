/************************************************************************************************/
/**
* @file               ciu32f003_std_tim.c
* @author             MCU Ecosystem Development Team
* @brief              TIM STD��������
*                     ʵ��TIM�������������벶������Ƚϵȹ��ܳ�ʼ��API��
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
* @addtogroup TIM
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_TIM_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/
/************************************************************************************************/
/**
* @addtogroup TIM_External_Functions 
* @{
*
*/
/************************************************************************************************/ 
/**
* @brief  TIM��ʼ��
* @param  timx TIM����
* @param  tim_init_param TIM��ʼ���ṹ��
* @note   ����DIRλ�����Ķ���ģʽ��Ϊֻ���������Ķ���ģʽ�л������ض��������ģʽ(����)��
*         ��Ҫֹͣ�������޸ģ��Ա�����ɼ����쳣��
* @retval ��
*/
void std_tim_init(TIM_t *timx, std_tim_basic_init_t *tim_init_param)
{   
    if(TIM1 == timx)
    {
        /* ���л�������ģʽ�������޸�DIR����λ */   
        timx->CR1 &= (~TIM_CR1_CMS);
        
        /* ѡ�������ģʽ��ʱ�ӷ�Ƶ���� */    
        MODIFY_REG(timx->CR1,
                  ((TIM_CR1_DIR | TIM_CR1_CMS) | TIM_CR1_CLK_DIV),
                  (tim_init_param->counter_mode | tim_init_param->clock_div));
            
        /* �����ظ��������� */
        std_tim_set_repcounter(timx, tim_init_param->repeat_counter);
    }
    else if(TIM3 == timx)
    {
        /* ����ʱ�ӷ�Ƶ���� */    
        MODIFY_REG(timx->CR1, TIM_CR1_CLK_DIV, tim_init_param->clock_div);
    }
    
    /* �����Զ�����ֵ */
    std_tim_set_autoreload(timx, tim_init_param->period);       
    
    /* ����Ԥ��Ƶֵ */
    std_tim_set_psc(timx, tim_init_param->prescaler);
       
    /* ����һ�������¼������¼���ԤԤ��Ƶֵ */
    /* ���֧��RCRģʽ��������¼�Ҳ�����¼����ظ�������ֵ */
    std_tim_set_sw_trig_event(timx, TIM_EVENT_SRC_UPDATE);
}

/**
* @brief  TIMȥ��ʼ��
* @param  timx TIM����
* @retval ��
*/
void std_tim_deinit(TIM_t *timx)    
{       
    /* ��λ���� */
    if(TIM1 == timx)
    {
        std_rcc_apb2_reset(RCC_PERIPH_RESET_TIM1);
    }
    else if(TIM3 == timx)
    {
        std_rcc_apb1_reset(RCC_PERIPH_RESET_TIM3);
    }
}

/**
* @brief  ����std_tim_basic_init_t�ṹ��ΪĬ��ֵ
* @param  tim_init_struct TIM��ʼ���ṹ��
* @retval ��
*/
void std_tim_struct_init(std_tim_basic_init_t *tim_init_struct)
{
    tim_init_struct->prescaler             = 0x0000U;
    tim_init_struct->counter_mode          = TIM_COUNTER_MODE_UP;
    tim_init_struct->period                = 0xFFFFU;
    tim_init_struct->clock_div             = TIM_CLOCK_DTS_DIV1;
    tim_init_struct->repeat_counter        = 0x0000U;
}


/**
* @brief  ����TIM���벶��ͨ��
* @param  timx TIM����
* @param  input_config TIM ���벶�����ýṹ��
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
* @retval ��
*/
void std_tim_input_capture_init(TIM_t *timx, std_tim_input_capture_init_t *input_config, uint32_t channel_id)
{         
    uint32_t tmp_value = ((channel_id & 0x01) == 0)?0U:8U;
    uint32_t shift_value = ((channel_id & 0x02) == 0)?0U:4U;        
    __IO uint32_t *preg = (__IO uint32_t *)((uint32_t)(&timx->CCM1) + shift_value);
    
    /* ��ֹCCxEλ */
    timx->CCEN &= (~(TIM_CCEN_CC1E << (channel_id << 2)));

    /* ѡ������Դ���˲�������Ԥ��Ƶ���� */
    MODIFY_REG(*preg,
              ((TIM_CCM1_CC1S | TIM_CCM1_IC1F |TIM_CCM1_IC1PSC ) << tmp_value),
              ((input_config->input_capture_sel | (input_config->input_capture_filter << 4U) | input_config->input_capture_prescaler) << tmp_value));
    
    /* ѡ���� */
    std_tim_set_input_pol(timx, channel_id, input_config->input_capture_pol);
}


/**
* @brief  ����std_tim_input_capture_init_t�ṹ��ΪĬ��ֵ
* @param  input_init_struct TIM���벶��ṹ��
* @retval ��
*/
void std_tim_input_capture_struct_init(std_tim_input_capture_init_t *input_init_struct)
{
    input_init_struct->input_capture_pol       = TIM_INPUT_POL_RISING;
    input_init_struct->input_capture_sel       = TIM_INPUT_CAPTURE_SEL_DIRECTTI;
    input_init_struct->input_capture_prescaler = TIM_INPUT_CAPTURE_PSC_DIV1;
    input_init_struct->input_capture_filter    = 0x00U;
}


/**
* @brief  ����TIM�Ƚ��������
* @param  timx TIM����
* @param  output_config TIM ����Ƚ����ýṹ��
* @param  channel_id TIM ͨ������
*             @arg TIM_CHANNEL_1
*             @arg TIM_CHANNEL_2
*             @arg TIM_CHANNEL_3
*             @arg TIM_CHANNEL_4
* @retval ��
*/
void std_tim_output_compare_init(TIM_t *timx, std_tim_output_compare_init_t *output_config, uint32_t channel_id)
{   
    uint32_t channel_oisx = (channel_id << 1);
    uint32_t channel_ccxe = (channel_id << 2);
    
    /* ��ֹCCxE��CCxNEλ */
    timx->CCEN &= (~((TIM_CCEN_CC1E | TIM_CCEN_CC1NE) << channel_ccxe));
    
    /* ѡ������Ƚ�ģʽ */
    std_tim_set_ocmode(timx, channel_id, output_config->output_compare_mode);   
    
    /* ��������Ƚϼ��ԡ����ʹ��λ */
    MODIFY_REG(timx->CCEN, 
              ((TIM_CCEN_CC1P | TIM_CCEN_CC1E) << channel_ccxe), 
              (output_config->output_pol << channel_ccxe) | (output_config->output_state << channel_ccxe));
    
    /* ���ñȽ�ƥ��ֵ */
    std_tim_set_ccx_value(timx, channel_id, output_config->pulse);
        
    if (timx == TIM1)
    {
        /* ����ͨ���ͻ������ͨ���Ŀ���״̬ */
        MODIFY_REG(timx->CR2,
                  ((TIM_CR2_OIS1 | TIM_CR2_OIS1N) << channel_oisx),
                  ((output_config->output_idle_state | output_config->output_negtive_idle_state) << channel_oisx));
            
        /* ���û���ͨ������Ƚϼ��ԡ����ʹ��λ */
        MODIFY_REG(timx->CCEN, 
                  ((TIM_CCEN_CC1NP | TIM_CCEN_CC1NE) << channel_ccxe), 
                  (output_config->output_negtive_pol << channel_ccxe) | (output_config->output_negtive_state << channel_ccxe));        
    }
}


/**
* @brief  ����std_tim_output_compare_init_t�ṹ��ΪĬ��ֵ
* @param  output_init_struct TIM����ṹ��
* @retval ��
*/
void std_tim_output_compare_struct_init(std_tim_output_compare_init_t *output_init_struct)
{
    output_init_struct->output_compare_mode         = TIM_OUTPUT_MODE_FROZEN;
    output_init_struct->pulse                       = 0x0000U;
    output_init_struct->output_state                = TIM_OUTPUT_DISABLE;
    output_init_struct->output_negtive_state        = TIM_OUTPUT_NEGTIVE_DISABLE;
    output_init_struct->output_pol                  = TIM_OUTPUT_POL_HIGH;
    output_init_struct->output_negtive_pol          = TIM_OUTPUT_NEGTIVE_POL_HIGH;
    output_init_struct->output_idle_state           = TIM_OUTPUT_IDLE_RESET;
    output_init_struct->output_negtive_idle_state   = TIM_OUTPUT_NEGTIVE_IDLE_RESET;
}


/**
* @brief  ��·������ʼ��
* @param  timx TIM����
* @param  bdt_init_param TIM��·�����ṹ��
* @note   ��������ϵͳ��·���������²���������ã�
*             1��ʹ����Ӧ��ϵͳ��·Դ��
*             2������std_tim_brk_source_enable()��������ֹͨ����·ʹ�ܣ�
*             3������std_tim_bken_enable()������ʹ��BKEN��
* @retval ��
*/
void std_tim_bdt_init(TIM_t* timx, std_tim_break_init_t *bdt_init_param)
{   
    /* ���ö�·�������� */
    MODIFY_REG(timx->BDT, 
              (TIM_BDT_DTG | TIM_BDT_OSSI | TIM_BDT_OSSR),
              (bdt_init_param->dead_time | bdt_init_param->off_state_idle_mode | bdt_init_param->off_state_run_mode));    
    
    /* �����������𣬲�ʹ�ܶ�· */
    MODIFY_REG(timx->BDT, 
              (TIM_BDT_BKEN | TIM_BDT_LOCK),
              (bdt_init_param->break_state | bdt_init_param->lock_level));   
}


/**
* @brief  ����std_tim_break_init_t�ṹ��ΪĬ��ֵ
* @param  bdt_init_struct TIM��·�����ṹ��
* @retval ��
*/
void std_tim_bdt_struct_init(std_tim_break_init_t *bdt_init_struct)
{
    bdt_init_struct->off_state_run_mode    = TIM_OSSR_DISABLE;
    bdt_init_struct->off_state_idle_mode   = TIM_OSSI_DISABLE;
    bdt_init_struct->lock_level            = TIM_LOCK_LEVEL_OFF;
    bdt_init_struct->dead_time             = 0x00U;
    bdt_init_struct->break_state           = TIM_BREAK_DISABLE;
}



/** 
* @} 
*/

#endif /* STD_TIM_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/
