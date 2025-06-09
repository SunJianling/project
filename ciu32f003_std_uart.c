/************************************************************************************************/
/**
* @file               ciu32f003_std_uart.c
* @author             MCU Ecosystem Development Team
* @brief              UART STD��������
*                     ʵ��UART��ʼ����API��
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
* @addtogroup  UART
* @{
*
*/
/************************************************************************************************/

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_UART_PERIPHERAL_USED
/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup UART_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  UART��ʼ��
* @param  uartx UART����
* @param  uart_init_param UART��ʼ���ṹ��
* @retval  ��
*/
void std_uart_init(UART_t *uartx,std_uart_init_t *uart_init_param)
{
    uint32_t pclk;
   
    /* ���� UART �ֳ� ���շ�ģʽ��У��*/
    MODIFY_REG(uartx->CR1,
              (UART_CR1_WL|UART_CR1_TE|UART_CR1_RE|UART_CR1_PTS|UART_CR1_PEN),
              (uart_init_param->wordlength|uart_init_param->direction|uart_init_param->parity));
     
    /* ����UARTֹͣλ */
    std_uart_set_stopbits(uartx,uart_init_param->stopbits);
           
     /* ��ȡUARTʱ��Ƶ�� */
    pclk = std_rcc_get_pclkfreq();
 
    /* BRRȡֵ��Χ[0x10,0xFFFF] */    
    if(uart_init_param->baudrate != 0)
    {
        uartx->BRR = (pclk + (uart_init_param->baudrate>>1))/uart_init_param->baudrate;
    }
}

/**
* @brief  UARTȥ��ʼ��
* @param  uartx UART����
* @retval ��
*/
void std_uart_deinit(UART_t *uartx)
{
    /* UART ��RCCʱ�Ӹ�λ */
    if(uartx == UART1)
    {
        std_rcc_apb2_reset(RCC_PERIPH_RESET_UART1);
    }
    else if(uartx == UART2)
    {
        std_rcc_apb1_reset(RCC_PERIPH_RESET_UART2);
    }
}

/**
* @brief  UART�ṹ���ʼ��
* @param  uart_init_struct UART��ʼ���ṹ��
* @retval ��
*/
void std_uart_struct_init(std_uart_init_t *uart_init_struct)
{ 
    uart_init_struct->baudrate = 115200;
    uart_init_struct->wordlength = UART_WORDLENGTH_8BITS;
    uart_init_struct->stopbits = UART_STOPBITS_1;
    uart_init_struct->direction = UART_DIRECTION_SEND_RECEIVE;
    uart_init_struct->parity = UART_PARITY_NONE;
}

/** 
* @} 
*/

#endif /* STD_UART_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/

