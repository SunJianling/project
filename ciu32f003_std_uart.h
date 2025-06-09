/************************************************************************************************/
/**
* @file               ciu32f003_std_uart.h
* @author             MCU Ecosystem Development Team
* @brief              UART STD������ͷ�ļ���
*                     �ṩUART��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_UART_H
#define CIU32F003_STD_UART_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup UART UART
* @brief ͨ���첽�շ�����STD������
* @{
*/
/************************************************************************************************/

#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------includes--------------------------------------------------*/
#include "ciu32f003_std_common.h"

/*------------------------------------type define-----------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup UART_Types UART Types 
* @brief  UART�������Ͷ���
* @{
*
*/
/************************************************************************************************/

/**
* @brief  UART��ʼ�����ýṹ�嶨��
*/
typedef struct
{
    uint32_t direction;                                      /**< UART ����ģʽ(���䷽��)
                                                                      @arg UART_DIRECTION_SEND ...                     */
    uint32_t baudrate;                                       /**< UART ������                                          */
    
    uint32_t wordlength;                                     /**< UART ����֡�ֳ�
                                                                      @arg UART_WORDLENGTH_8BITS ...                   */
    uint32_t stopbits;                                       /**< UART ����ֹ֡ͣλ����
                                                                      @arg UART_STOPBITS_1 ...                         */
    uint32_t parity;                                         /**< UART ����֡��żУ��
                                                                      @arg UART_PARITY_NONE ...                        */
}std_uart_init_t;

/**
* @}
*/

/*--------------------------------------------define--------------------------------------------*/
/************************************************************************************************/
/**
* @defgroup UART_Constants UART Constants
* @brief    UART�������弰�궨��
* @{
*
*/
/************************************************************************************************/

/* UART ����֡���� */
#define  UART_WORDLENGTH_8BITS                 UART_CR1_WL8BITS                     /**< 8-bits �ֳ� */
#define  UART_WORDLENGTH_9BITS                 UART_CR1_WL9BITS                     /**< 9-bits �ֳ� */

/* UART ����֡��żУ�� */
#define  UART_PARITY_NONE                      (0x00000000U)                         /**< ��У�� */
#define  UART_PARITY_EVEN                       UART_CR1_PEN                         /**< żУ�� */
#define  UART_PARITY_ODD                       (UART_CR1_PEN | UART_CR1_PTS)         /**< ��У�� */

/* UART����ģʽ */
#define  UART_DIRECTION_NONE                   (0x00000000U)                         /**< δʹ�ܷ������ͽ�����          */
#define  UART_DIRECTION_SEND                   UART_CR1_TE                           /**< ������ģʽ(��ʹ�ܽ�����)      */
#define  UART_DIRECTION_RECEIVE                UART_CR1_RE                           /**< ������ģʽ(��ʹ�ܷ�����)      */
#define  UART_DIRECTION_SEND_RECEIVE           (UART_CR1_TE |UART_CR1_RE)            /**< ����/����(ʹ�ܷ������ͽ�����) */

/* UART ����֡��С�� */
#define  UART_DATA_ORDER_LSBFIRST              (0x00000000U)                         /**< ����Чλ���� */
#define  UART_DATA_ORDER_MSBFIRST              UART_CR2_MSBFIRST                     /**< ����Чλ���� */

/* UART ����ֹ֡ͣλ */
#define  UART_STOPBITS_1                       UART_CR2_STOPBIT_1                    /**< 1bitֹͣλ    */
#define  UART_STOPBITS_2                       UART_CR2_STOPBIT_2                    /**< 2bitsֹͣλ   */

/* UART ����֡�������� */
#define  UART_SAMPLE_THREE_BIT                 (0x00000000U)                         /**< Three-bit������֧��������� */
#define  UART_SAMPLE_ONE_BIT                   UART_CR3_OBS                          /**< One-bit��������֧��������� */

/*UART�ж�ʹ��λ */
#define  UART_CR1_INTERRUPT_RXNE               UART_CR1_RXNEIE                       /**< UART �������ݼĴ����ǿ��ж�ʹ��                   */
#define  UART_CR1_INTERRUPT_TC                 UART_CR1_TCIE                         /**< UART ��������ж�ʹ��                             */
#define  UART_CR1_INTERRUPT_TXE                UART_CR1_TXEIE                        /**< UART �������ݼĴ������ж�ʹ��                     */
#define  UART_CR1_INTERRUPT_PE                 UART_CR1_PEIE                         /**< UART ��żУ������ж�ʹ��                         */

/* UART�жϱ�־���λ */
#define  UART_CLEAR_PE                         UART_ICR_PECF                         /**< ��żУ������־��� */
#define  UART_CLEAR_FE                         UART_ICR_FECF                         /**< ֡�����־���       */
#define  UART_CLEAR_NOISE                      UART_ICR_NOISECF                      /**< ���������־���     */
#define  UART_CLEAR_ORE                        UART_ICR_ORECF                        /**< ������������־��� */
#define  UART_CLEAR_TC                         UART_ICR_TCCF                         /**< ������ɱ�־���     */

/* UART ״̬��־ */
#define  UART_FLAG_RECEIVE_BUSY                UART_ISR_BUSY                         /**< UART ����æ(���չܽ��������ݴ���)          */
#define  UART_FLAG_TXE                         UART_ISR_TXE                          /**< UART �������ݼĴ����ձ�־                  */
#define  UART_FLAG_TC                          UART_ISR_TC                           /**< UART ������ɱ�־                          */
#define  UART_FLAG_RXNE                        UART_ISR_RXNE                         /**< UART �������ݼĴ����ǿձ�־                */
#define  UART_FLAG_ORE                         UART_ISR_ORE                          /**< UART ������������־                      */
#define  UART_FLAG_NOISE                       UART_ISR_NOISE                        /**< UART ���������־                          */
#define  UART_FLAG_FE                          UART_ISR_FE                           /**< UART ֡�����־                            */
#define  UART_FLAG_PE                          UART_ISR_PE                           /**< UART ��żУ������־                      */
#define  UART_FLAG_ERR                         (UART_FLAG_ORE | UART_FLAG_NOISE \
                                                              | UART_FLAG_FE)        /**< UART �����־�����������������֡����*/       

/**
* @}
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup UART_External_Functions UART External Functions
* @brief    UART���⺯��
* @{
*
*/
/************************************************************************************************/

/** 
* @brief  ʹ��UART
* @param  uartx UART���� 
* @retval ��
*/
__STATIC_INLINE void std_uart_enable(UART_t *uartx)
{
    uartx->CR1 |= (UART_CR1_UE);    
}

/** 
* @brief  ��ֹUART
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_disable(UART_t *uartx)
{
    uartx->CR1 &= (~UART_CR1_UE);
}

/** 
* @brief  ����UART�ַ�����
* @param  uartx UART����
* @param  word_length UART �ַ�����
*             @arg UART_WORDLENGTH_8BITS
*             @arg UART_WORDLENGTH_9BITS 
* @retval ��
*/
__STATIC_INLINE void std_uart_set_word_length(UART_t *uartx, uint32_t word_length)
{
    MODIFY_REG(uartx->CR1, UART_CR1_WL, word_length);
}

/** 
* @brief  ��ȡUART�ַ�����
* @param  uartx UART����
* @retval uint32_t UART �ַ�����
*             @arg UART_WORDLENGTH_8BITS
*             @arg UART_WORDLENGTH_9BITS 
*/
__STATIC_INLINE uint32_t std_uart_get_word_length(UART_t *uartx)
{
    return (uartx->CR1 & UART_CR1_WL);
}

/** 
* @brief  ����UART��żУ��
* @param  uartx UART����
* @param  parity UART��żУ����
*             @arg UART_PARITY_NONE
*             @arg UART_PARITY_EVEN
*             @arg UART_PARITY_ODD 
* @retval ��
*/
__STATIC_INLINE void std_uart_set_parity(UART_t *uartx,uint32_t parity)
{
    MODIFY_REG(uartx->CR1,(UART_CR1_PTS | UART_CR1_PEN), parity);
} 

/** 
* @brief  ��ȡUART��żУ��
* @param  uartx UART����
* @retval uint32_t UART��żУ����
*             @arg UART_PARITY_NONE
*             @arg UART_PARITY_EVEN
*             @arg UART_PARITY_ODD 
*/
__STATIC_INLINE uint32_t std_uart_get_parity(UART_t *uartx )
{
    return(uartx->CR1 & (UART_CR1_PTS | UART_CR1_PEN));
} 

/** 
* @brief  ����UARTֹͣλ
* @param  uartx UART����
* @param  stopbits UARTֹͣλλ�� 
*             @arg UART_STOPBITS_1     
*             @arg UART_STOPBITS_2    
* @retval ��
*/
__STATIC_INLINE void std_uart_set_stopbits(UART_t *uartx, uint32_t stopbits)
{
    MODIFY_REG(uartx->CR2, UART_CR2_STOPBIT, stopbits);
}

/** 
* @brief  ��ȡUARTֹͣλ
* @param  uartx UART����
* @retval uint32_t UARTֹͣλλ�� 
*             @arg UART_STOPBITS_1    
*             @arg UART_STOPBITS_2    
*/
__STATIC_INLINE uint32_t std_uart_get_stopbits(UART_t *uartx)
{
    return(uartx->CR2 & UART_CR2_STOPBIT);
}

/** 
* @brief  ����UART�ַ���С��
* @param  uartx UART����
* @param  data_order UART �ַ�֡��ʽ��С��
*             @arg UART_DATA_ORDER_LSBFIRST
*             @arg UART_DATA_ORDER_MSBFIRST
* @retval ��
*/
__STATIC_INLINE void std_uart_set_data_order(UART_t *uartx, uint32_t data_order)
{
    MODIFY_REG(uartx->CR2, UART_CR2_MSBFIRST, data_order);
}

/** 
* @brief  ��ȡUART�ַ���С��
* @param  uartx UART����
* @retval uint32_t UART �ַ�֡��ʽ��С��
*             @arg UART_DATA_ORDER_LSBFIRST
*             @arg UART_DATA_ORDER_MSBFIRST
*/
__STATIC_INLINE uint32_t std_uart_get_data_order(UART_t *uartx)
{
    return(uartx->CR2 & UART_CR2_MSBFIRST);
}

/** 
* @brief  ����UART��������
* @param  uartx UART����
* @param  sample_method UART��������
*             @arg UART_SAMPLE_THREE_BIT
*             @arg UART_SAMPLE_ONE_BIT
* @retval ��
*/
__STATIC_INLINE void std_uart_set_sample_method(UART_t *uartx,uint32_t sample_method)
{
    MODIFY_REG(uartx->CR3,UART_CR3_OBS, sample_method);
}

/** 
* @brief  ��ȡUART��������
* @param  uartx UART����
* @retval uint32_t UART��������
*             @arg UART_SAMPLE_THREE_BIT
*             @arg UART_SAMPLE_ONE_BIT
*/
__STATIC_INLINE uint32_t std_uart_get_sample_method(UART_t *uartx)
{
    return (uartx->CR3 & UART_CR3_OBS);
}

/** 
* @brief  ����UART���䷽��
* @param  uartx UART����
* @param  direction UART���䷽��
*             @arg UART_DIRECTION_NONE
*             @arg UART_DIRECTION_SEND                                                                                        
*             @arg UART_DIRECTION_RECEIVE                                                      
*             @arg UART_DIRECTION_SEND_RECEIVE                                                                                                               
* @retval ��
*/
__STATIC_INLINE void std_uart_set_transfer_direction(UART_t *uartx, uint32_t direction)
{
    MODIFY_REG(uartx->CR1, UART_CR1_TE|UART_CR1_RE, direction);
}

/** 
* @brief  ��ȡ UART ���䷽�����
* @param  uartx UART����
* @retval uint32_t ���䷽��
*             @arg UART_DIRECTION_NONE  
*             @arg UART_DIRECTION_SEND                                                                                        
*             @arg UART_DIRECTION_RECEIVE                                                      
*             @arg UART_DIRECTION_SEND_RECEIVE                                                                                                               
*/
__STATIC_INLINE uint32_t std_uart_get_transfer_direction(UART_t *uartx)
{
    return(uartx->CR1 & (UART_CR1_TE|UART_CR1_RE));
}

/** 
* @brief  ʹ��UART���Ž���
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_pin_swap_enable(UART_t *uartx)
{
    uartx->CR2 |= (UART_CR2_SWAP);
} 

/** 
* @brief  ��ֹUART���Ž���
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_pin_swap_disable(UART_t *uartx)
{
    uartx->CR2 &= (~UART_CR2_SWAP);
} 

/** 
* @brief  ʹ��UART RX���ŵ�ƽ����
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_rx_level_invert_enable(UART_t *uartx)
{
    uartx->CR2 |= (UART_CR2_RXIVC);
} 

/** 
* @brief  ��ֹUART RX���ŵ�ƽ����
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_rx_level_invert_disable(UART_t *uartx)
{
    uartx->CR2 &= (~UART_CR2_RXIVC);
} 

/** 
* @brief  ʹ��UART TX���ŵ�ƽ����
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_tx_level_invert_enable(UART_t *uartx)
{
    uartx->CR2 |= (UART_CR2_TXIVC);
} 

/** 
* @brief  ��ֹUART TX���ŵ�ƽ����
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_tx_level_invert_disable(UART_t *uartx)
{
    uartx->CR2 &= (~UART_CR2_TXIVC);
} 

/** 
* @brief  ʹ��UART���ݼ��Է���
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_data_invert_enable(UART_t *uartx)
{
    uartx->CR2 |= (UART_CR2_DATAIVC);
} 

/** 
* @brief  ��ֹUART���ݼ��Է���
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_data_invert_disable(UART_t *uartx)
{
    uartx->CR2 &= (~UART_CR2_DATAIVC);
}

/** 
* @brief  ʹ��UART���߰�˫��ģʽ
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_half_duplex_enable(UART_t *uartx)
{
    uartx->CR3 |= (UART_CR3_HDEN);
} 

/** 
* @brief  ��ֹUART���߰�˫��ģʽ
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_half_duplex_disable(UART_t *uartx)
{
    uartx->CR3 &= (~UART_CR3_HDEN);
}

/** 
* @brief  ��ֹUART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_overrun_disable(UART_t *uartx)
{
    uartx->CR3 |= (UART_CR3_ORED);
}

/** 
* @brief  ʹ��UART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_overrun_enable(UART_t *uartx)
{
    uartx->CR3 &= (~UART_CR3_ORED);
}

/** 
* @brief  ʹ��UART CR1�Ĵ����п��Ƶ��ж�
* @param  uartx UART����
* @param  interrupt UART�ж�Դѡ��     
*             @arg UART_CR1_INTERRUPT_RXNE                             
*             @arg UART_CR1_INTERRUPT_TC                    
*             @arg UART_CR1_INTERRUPT_TXE                    
*             @arg UART_CR1_INTERRUPT_PE                                     
* @retval ��
*/
__STATIC_INLINE void std_uart_cr1_interrupt_enable(UART_t *uartx, uint32_t interrupt)
{
    uartx->CR1 |= (interrupt);
}

/** 
* @brief  ��ֹUART CR1�Ĵ����п��Ƶ��ж�
* @param  uartx UART����
* @param  interrupt UART�ж�Դѡ��
*             @arg UART_CR1_INTERRUPT_RXNE                                 
*             @arg UART_CR1_INTERRUPT_TC                    
*             @arg UART_CR1_INTERRUPT_TXE                    
*             @arg UART_CR1_INTERRUPT_PE                                     
* @retval ��
*/
__STATIC_INLINE void std_uart_cr1_interrupt_disable(UART_t *uartx, uint32_t interrupt)
{
    uartx->CR1 &= (~interrupt);
}

/** 
* @brief  ��ȡUART CR1�Ĵ����п��Ƶ��ж�ʹ��
* @param  uartx UART����
* @param  interrupt UART�ж�Դѡ��
*             @arg UART_CR1_INTERRUPT_RXNE                                
*             @arg UART_CR1_INTERRUPT_TC                    
*             @arg UART_CR1_INTERRUPT_TXE                    
*             @arg UART_CR1_INTERRUPT_PE                                     
* @retval uint32_t ����ѡ���UART�ж�Դʹ��״̬
*             @arg ��0:  ��ʾѡ���ж�Դʹ��
*             @arg 0:    ��ʾѡ���ж�Դδʹ��
*/
__STATIC_INLINE uint32_t std_uart_get_cr1_interrupt_enable(UART_t *uartx, uint32_t interrupt)
{
    return (uartx->CR1 & interrupt);
}

/** 
* @brief  ʹ��UART CR3�Ĵ����п��Ƶ� ERR �ж�
* @param  uartx UART����                                            
* @retval ��
*/
__STATIC_INLINE void std_uart_cr3_interrupt_err_enable(UART_t *uartx)
{
    uartx->CR3 |= (UART_CR3_EIE);
}

/** 
* @brief  ��ֹUART CR3�Ĵ����п��Ƶ� ERR �ж�
* @param  uartx UART����                                         
* @retval ��
*/
__STATIC_INLINE void std_uart_cr3_interrupt_err_disable(UART_t *uartx)
{
    uartx->CR3 &= (~UART_CR3_EIE);
}

/** 
* @brief  ��ȡUART CR3�Ĵ������ж�ERR �ж�
* @param  uartx UART����
* @retval uint32_t ����ѡ���UART�ж�Դʹ��״̬
*             @arg ��0��  ��ʾѡ���ж�ԴEIE ʹ��
*             @arg 0��    ��ʾѡ���ж�ԴEIE δʹ��
*/
__STATIC_INLINE uint32_t std_uart_get_cr3_interrupt_err_enable(UART_t *uartx)
{
     return (uartx->CR3 & UART_CR3_EIE);
}

/** 
* @brief  ��ȡUART��־
* @param  uartx UART����
* @param  flag UART��־
*             @arg UART_FLAG_RECEIVE_BUSY     
*             @arg UART_FLAG_TXE              
*             @arg UART_FLAG_TC               
*             @arg UART_FLAG_RXNE             
*             @arg UART_FLAG_ORE
*             @arg UART_FLAG_NOISE              
*             @arg UART_FLAG_FE               
*             @arg UART_FLAG_PE               
* @retval uint32_t UART��־����״̬
*             @arg ��0��  ״̬����
*             @arg 0��    ״̬δ����
*/
__STATIC_INLINE uint32_t std_uart_get_flag(UART_t *uartx,uint32_t flag)
{
    return (uartx->ISR & flag);
}

/** 
* @brief  ���UART��־ 
* @param  uartx UART����
* @param  clear_flag UART��־��Ϣ
*             @arg UART_CLEAR_PE     
*             @arg UART_CLEAR_FE              
*             @arg UART_CLEAR_NOISE               
*             @arg UART_CLEAR_ORE             
*             @arg UART_CLEAR_TC                           
* @retval ��      
*/
__STATIC_INLINE void std_uart_clear_flag(UART_t *uartx, uint32_t clear_flag)
{
    uartx->ICR = (clear_flag);
}

/** 
* @brief  ʹ��UART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_rx_enable(UART_t *uartx)
{
    uartx->CR1 |= (UART_CR1_RE);
} 

/** 
* @brief  ��ֹUART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_rx_disable(UART_t *uartx)
{
    uartx->CR1 &= (~UART_CR1_RE);
} 

/** 
* @brief  ʹ��UART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_tx_enable(UART_t *uartx)
{
    uartx->CR1 |= (UART_CR1_TE);
} 

/** 
* @brief  ��ֹUART������
* @param  uartx UART����
* @retval ��
*/
__STATIC_INLINE void std_uart_tx_disable(UART_t *uartx)
{
    uartx->CR1 &= (~UART_CR1_TE);
} 

/** 
* @brief  ��ȡUART��������
* @param  uartx UART����
* @retval uint32_t UART���յ�������
*/
__STATIC_INLINE uint32_t std_uart_rx_read_data(UART_t *uartx)
{
    return (uartx->RDR);
}

/** 
* @brief  д��UART��������
* @param  uartx UART����
* @param  data_value ��������
* @retval ��
*/
__STATIC_INLINE void std_uart_tx_write_data(UART_t *uartx, uint32_t data_value)
{
    uartx->TDR = data_value;
}

/** 
* @brief  д��UART �����ʷ�Ƶ�Ĵ���
* @param  uartx UART����
* @param  brr_value �����ʷ�Ƶֵ��Χ�� 0x10 ~ 0xFFFF
* @retval ��
*/
__STATIC_INLINE void std_uart_set_brr_value(UART_t *uartx, uint32_t brr_value)
{
    uartx->BRR = brr_value;
}

/** 
* @brief  ��ȡUART�����ʷ�Ƶ����
* @param  uartx UART����
* @retval uint32_t UART�����ʷ�Ƶ������ֵ
*/
__STATIC_INLINE uint32_t std_uart_get_brr_value(UART_t *uartx)
{
    return (uartx->BRR);
}

void std_uart_init(UART_t *uartx,std_uart_init_t *UART_init_param);
void std_uart_deinit(UART_t *uartx);
void std_uart_struct_init(std_uart_init_t *UART_init_struct);

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

#endif /* CIU32F003_STD_UART_H */
