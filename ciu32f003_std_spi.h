/************************************************************************************************/
/**
* @file               ciu32f003_std_spi.h
* @author             MCU Ecosystem Development Team
* @brief              SPI STD������ͷ�ļ���
*                     �ṩSPI��ص�STD������������������������Լ������Ķ��塣
*
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_SPI_H
#define CIU32F003_STD_SPI_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup SPI SPI
* @brief ��������ӿڵ�STD������ 
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
* @defgroup SPI_Types SPI Types
* @brief  SPI�������Ͷ���
* @{
*
*/
/************************************************************************************************/

/**
* @brief  SPI��ʼ�����ýṹ�嶨��
*/
typedef struct
{
    uint32_t mode;                     /**< SPI����ģʽ
                                                @arg SPI_MODE_SLAVE
                                                @arg SPI_MODE_MASTER                            */

    uint32_t baud_rate_prescaler;      /**< SPI�������ò���                                    
                                                @arg  SPI_BAUD_PCLKDIV_2          
                                                @arg  SPI_BAUD_PCLKDIV_4 ...      
                                                @note �������ʱ�����ã��ӻ�����Ҫ����          */

    uint32_t clk_polarity;             /**< SPIʱ�Ӽ���                                
                                                @arg SPI_POLARITY_LOW                  
                                                @arg SPI_POLARITY_HIGH                          */

    uint32_t clk_phase;                /**< SPIʱ����λ                                
                                                @arg SPI_PHASE_1EDGE                   
                                                @arg SPI_PHASE_2EDGE                            */
    
    uint32_t bitorder;                 /**< SPI���ݴ�С��
                                                @arg SPI_FIRSTBIT_MSB
                                                @arg SPI_FIRSTBIT_LSB                           */
}std_spi_init_t; 


/** 
* @} 
*/


/*--------------------------------------------define--------------------------------------------*/
/************************************************************************************************/
/** 
* @defgroup SPI_Constants SPI Constants
* @brief SPI�������弰�궨��
* @{
*/
/************************************************************************************************/
/* SPI ����ģʽ */
#define SPI_MODE_SLAVE                  (0x00000000U)                      /**< SPI�ӻ�ģʽ     */
#define SPI_MODE_MASTER                 SPI_CR1_MSTR                       /**< SPI����ģʽ     */

/* SPI �������ò��� */    
#define SPI_BAUD_PCLKDIV_2              SPI_CR1_BR_PCLK_DIV_2              /**< SPI����ΪfPLCK/2   */
#define SPI_BAUD_PCLKDIV_4              SPI_CR1_BR_PCLK_DIV_4              /**< SPI����ΪfPLCK/4   */
#define SPI_BAUD_PCLKDIV_8              SPI_CR1_BR_PCLK_DIV_8              /**< SPI����ΪfPLCK/8   */
#define SPI_BAUD_PCLKDIV_16             SPI_CR1_BR_PCLK_DIV_16             /**< SPI����ΪfPLCK/16  */
#define SPI_BAUD_PCLKDIV_32             SPI_CR1_BR_PCLK_DIV_32             /**< SPI����ΪfPLCK/32  */
#define SPI_BAUD_PCLKDIV_64             SPI_CR1_BR_PCLK_DIV_64             /**< SPI����ΪfPLCK/64  */
#define SPI_BAUD_PCLKDIV_128            SPI_CR1_BR_PCLK_DIV_128            /**< SPI����ΪfPLCK/128 */

/* SPI ʱ�Ӽ��� */
#define SPI_POLARITY_LOW                (0x00000000U)                      /**< SPIʱ�ӿ���Ϊ�� */
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL                       /**< SPIʱ�ӿ���Ϊ�� */

/* SPI ʱ����λ */
#define SPI_PHASE_1EDGE                 (0x00000000U)                      /**< SPI���ݲ����ڵ�һ��ʱ���� */
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA                       /**< SPI���ݲ����ڵڶ���ʱ���� */

/* SPI ���ݴ�С�� */
#define SPI_FIRSTBIT_MSB                (0x00000000U)                      /**< SPI�����շ�Ϊ��λ���� */
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST                   /**< SPI�����շ�Ϊ��λ���� */

/* SPI NSS���״̬*/
#define SPI_NSS_OUTPUT_LOW              (0x00000000U)                      /**< SPIƬѡ�ź�����͵�ƽ */
#define SPI_NSS_OUTPUT_HIGH             SPI_CR2_NSSO                       /**< SPIƬѡ�ź�����ߵ�ƽ */

/* SPI �ж��¼� */
#define SPI_INTERRUPT_TXFE              SPI_CR1_TXFEIE                     /**< SPI�������ݼĴ���Ϊ���ж�ʹ�� */
#define SPI_INTERRUPT_RXFNE             SPI_CR1_RXFNEIE                    /**< SPI�������ݼĴ����ǿ��ж�ʹ�� */
#define SPI_INTERRUPT_ERR               SPI_CR1_ERRIE                      /**< SPI�����ж�ʹ��               */

/* SPI Ӳ��״̬��Ϣ */
#define SPI_FLAG_TXFE                   SPI_ISR_TXFE                       /**< SPI�������ݼĴ����ձ�־   */
#define SPI_FLAG_RXFNE                  SPI_ISR_RXFNE                      /**< SPI�������ݼĴ����ǿձ�־ */
#define SPI_FLAG_BUSY                   SPI_ISR_BUSY                       /**< SPI���ߴ���״̬��־       */
#define SPI_FLAG_OVR                    SPI_ISR_OVR                        /**< SPI�����������־         */
#define SPI_FLAG_MMF                    SPI_ISR_MMF                        /**< SPI����ģʽ��ͻ��־       */

/* SPI ״̬�����Ϣ */
#define SPI_CLEAR_FLAG_OVR              SPI_ICR_OVRCF                      /**< SPI���������־���     */
#define SPI_CLEAR_FLAG_MMF              SPI_ICR_MMFCF                      /**< SPI����ģʽ��ͻ��־��� */

/** 
* @} 
*/


/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup SPI_External_Functions SPI External Functions
* @brief    SPI���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ʹ��SPI
* @retval ��
*/
__STATIC_INLINE void std_spi_enable(void)
{
    SPI1->CR1 |= SPI_CR1_SPE;
}


/** 
* @brief  ��ֹSPI
* @retval ��
*/
__STATIC_INLINE void std_spi_disable(void)
{
    SPI1->CR1 &= (~SPI_CR1_SPE);
}


/** 
* @brief  ����SPI����ģʽ
* @param  mode SPI����ģʽ
*             @arg SPI_MODE_SLAVE
*             @arg SPI_MODE_MASTER   
* @retval ��
*/
__STATIC_INLINE void std_spi_set_mode(uint32_t mode)
{
    MODIFY_REG(SPI1->CR1, SPI_CR1_MSTR, mode);
}


/** 
* @brief  ��ȡSPI����ģʽ
* @retval uint32_t SPI����ģʽ
*             @arg SPI_MODE_SLAVE
*             @arg SPI_MODE_MASTER  
*/
__STATIC_INLINE uint32_t std_spi_get_mode(void)
{
    return (SPI1->CR1 & SPI_CR1_MSTR);
}


/** 
* @brief  ����SPI������
* @param  baud_rate SPI������
*             @arg  SPI_BAUD_PCLKDIV_2      
*             @arg  SPI_BAUD_PCLKDIV_4
*             @arg  ...  
*             @arg  SPI_BAUD_PCLKDIV_128
* @note  �������ʱ�����ã��ӻ�����Ҫ����
* @retval ��
*/
__STATIC_INLINE void std_spi_set_baud_rate(uint32_t baud_rate)
{
    MODIFY_REG(SPI1->CR1, SPI_CR1_BR, baud_rate);
}


/** 
* @brief  ��ȡSPI������
* @retval uint32_t  SPI������
*             @arg  SPI_BAUD_PCLKDIV_2
*             @arg  SPI_BAUD_PCLKDIV_4
*             @arg  ...  
*             @arg  SPI_BAUD_PCLKDIV_128
*/
__STATIC_INLINE uint32_t std_spi_get_baud_rate(void)
{
    return (SPI1->CR1 & SPI_CR1_BR);
}


/** 
* @brief  ����SPIʱ�Ӽ���
* @param  polarity SPIʱ�Ӽ���
*             @arg SPI_POLARITY_LOW
*             @arg SPI_POLARITY_HIGH
* @retval ��
*/
__STATIC_INLINE void std_spi_set_polarity(uint32_t polarity)
{
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPOL, polarity);
}


/**
* @brief  ����SPIʱ����λ 
* @param  phase SPIʱ����λ
*             @arg SPI_PHASE_1EDGE 
*             @arg SPI_PHASE_2EDGE 
* @retval ��
*/
__STATIC_INLINE void std_spi_set_phase(uint32_t phase)
{
    MODIFY_REG(SPI1->CR1, SPI_CR1_CPHA, phase);
}


/**
* @brief  ����SPI���ݴ�С��
* @param  first_bit SPI���ݴ�С��
*             @arg SPI_FIRSTBIT_MSB 
*             @arg SPI_FIRSTBIT_LSB 
* @note   ���ڽ�ֹSPI��SPEΪ0��ʱ�ſɶԴ�λִ��д����
* @retval ��
*/
__STATIC_INLINE void std_spi_set_first_bit(uint32_t first_bit)
{
    MODIFY_REG(SPI1->CR1, SPI_CR1_LSBFIRST, first_bit);
}


/**
* @brief  ʹ��NSS��Ƭѡ
* @note   ���ڴӻ�ģʽ��������Ч
* @retval ��
*/
__STATIC_INLINE void std_spi_nss_soft_chip_select_enable(void)
{
    SPI1->CR1 |= SPI_CR1_SSM;
}


/**
* @brief  ��ֹNSS��Ƭѡ
* @note   ���ڴӻ�ģʽ��������Ч
* @retval ��
*/
__STATIC_INLINE void std_spi_nss_soft_chip_select_disable(void)
{
    SPI1->CR1 &= (~SPI_CR1_SSM);
}


/**
* @brief  ʹ��SPI NSS���
* @note   ��������ģʽ��������Ч
* @retval ��
*/
__STATIC_INLINE void std_spi_nss_output_enable(void)
{
    SPI1->CR1 |= SPI_CR1_NSSOE;
}


/**
* @brief  ��ֹSPI NSS���
* @retval ��
*/
__STATIC_INLINE void std_spi_nss_output_disable(void)
{
    SPI1->CR1 &= (~SPI_CR1_NSSOE);
}


/**
* @brief  ʹ��SPI�ж�
* @param  spi_interrupt SPI�ж���Ϣ
*             @arg SPI_INTERRUPT_TXFE     
*             @arg SPI_INTERRUPT_RXFNE    
*             @arg SPI_INTERRUPT_ERR      
* @retval ��
*/
__STATIC_INLINE void std_spi_interrupt_enable(uint32_t spi_interrupt)
{
    SPI1->CR1 |= spi_interrupt;
}


/**
* @brief  ��ֹSPI�ж�
* @param  spi_interrupt SPI�ж���Ϣ
*             @arg SPI_INTERRUPT_TXFE     
*             @arg SPI_INTERRUPT_RXFNE    
*             @arg SPI_INTERRUPT_ERR      
* @retval ��
*/
__STATIC_INLINE void std_spi_interrupt_disable(uint32_t spi_interrupt)
{
    SPI1->CR1 &= (~spi_interrupt);
}


/**
* @brief  ��ȡSPI�ж�ʹ��״̬
* @param  spi_interrupt SPI�ж���Ϣ
*             @arg SPI_INTERRUPT_TXFE     
*             @arg SPI_INTERRUPT_RXFNE    
*             @arg SPI_INTERRUPT_ERR      
* @retval uint32_t �ж�ʹ��״̬
*             @arg ��0: ʹ��
*             @arg 0: ��ֹ
*/
__STATIC_INLINE uint32_t std_spi_get_interrupt_enable(uint32_t spi_interrupt)
{
    return (SPI1->CR1 & (spi_interrupt));
}


/**
* @brief  ����SPI NSS���ŵ�ƽ
* @param  nss_output NSS�����ƽ
*             @arg SPI_NSS_OUTPUT_LOW   
*             @arg SPI_NSS_OUTPUT_HIGH  
* @note   ��������ģʽ����SPEʹ�ܵ������������Ч
* @retval ��
*/
__STATIC_INLINE void std_spi_set_nss_output(uint32_t nss_output)
{
    SPI1->CR2 = nss_output;
}


/**
* @brief  ��ȡSPI״̬��־λ
* @param  flag SPI״̬��־λ
*             @arg SPI_FLAG_TXFE 
*             @arg SPI_FLAG_RXFNE
*             @arg SPI_FLAG_BUSY 
*             @arg SPI_FLAG_OVR  
*             @arg SPI_FLAG_MMF  
* @retval uint32_t SPI��־λ״̬
*             @arg ��0: ��־λ��λ
*             @arg 0:   ��־λ���
*/
__STATIC_INLINE uint32_t std_spi_get_flag(uint32_t flag)
{
    return (SPI1->ISR & (flag));
}


/**
* @brief  SPI��־����
* @param  flag SPI��־
*             @arg SPI_CLEAR_FLAG_OVR
*             @arg SPI_CLEAR_FLAG_MMF
* @retval ��
*/
__STATIC_INLINE void std_spi_clear_flag(uint32_t flag)
{
    SPI1->ICR = (flag);
}


/**
* @brief  SPI������
* @retval uint8_t: ���ض�ȡ��SPI���ݼĴ�����ȡ������
*/
__STATIC_INLINE uint8_t std_spi_read_data(void)
{
    return (SPI1->DR);
}

/**
* @brief  SPIд����
* @param  send_data: д��SPI���ݼĴ���������
* @retval ��
*/
__STATIC_INLINE void std_spi_write_data(uint8_t send_data)
{
    SPI1->DR = send_data;
}


void std_spi_init(std_spi_init_t *spi_init_param);
void std_spi_deinit(void);
void std_spi_struct_init(std_spi_init_t *spi_init_struct);


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

#endif /* CIU32F003_STD_SPI_H */
