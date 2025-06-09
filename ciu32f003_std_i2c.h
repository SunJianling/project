/************************************************************************************************/
/**
* @file               ciu32f003_std_i2c.h
* @author             MCU Ecosystem Development Team
* @brief              I2C STD������ͷ�ļ���
*                     �ṩI2C��ص�STD������������������������Լ������Ķ��塣                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_I2C_H
#define CIU32F003_STD_I2C_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup I2C I2C
* @brief I2C�ӿڵ�STD������
* @{
*/
/************************************************************************************************/


#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------includes--------------------------------------------------*/
#include "ciu32f003_std_common.h"

/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup I2C_Constants I2C Constants
* @brief    I2C�������弰�궨��
* @{
*
*/
/************************************************************************************************/
/* �����˲������� */
#define I2C_DIGITALFILTER_DISABLE       I2C_CR1_DNF_DISABLE         /**< �����˲�����ֹ             */
#define I2C_DIGITALFILTER_1CLK          I2C_CR1_DNF_1CLK            /**< �˳�С��1��I2C_KCLK������  */
#define I2C_DIGITALFILTER_2CLK          I2C_CR1_DNF_2CLK            /**< �˳�С��2��I2C_KCLK������  */
#define I2C_DIGITALFILTER_3CLK          I2C_CR1_DNF_3CLK            /**< �˳�С��3��I2C_KCLK������  */
#define I2C_DIGITALFILTER_4CLK          I2C_CR1_DNF_4CLK            /**< �˳�С��4��I2C_KCLK������  */
#define I2C_DIGITALFILTER_5CLK          I2C_CR1_DNF_5CLK            /**< �˳�С��5��I2C_KCLK������  */
#define I2C_DIGITALFILTER_6CLK          I2C_CR1_DNF_6CLK            /**< �˳�С��6��I2C_KCLK������  */
#define I2C_DIGITALFILTER_7CLK          I2C_CR1_DNF_7CLK            /**< �˳�С��7��I2C_KCLK������  */
#define I2C_DIGITALFILTER_8CLK          I2C_CR1_DNF_8CLK            /**< �˳�С��8��I2C_KCLK������  */
#define I2C_DIGITALFILTER_9CLK          I2C_CR1_DNF_9CLK            /**< �˳�С��9��I2C_KCLK������  */
#define I2C_DIGITALFILTER_10CLK         I2C_CR1_DNF_10CLK           /**< �˳�С��10��I2C_KCLK������ */
#define I2C_DIGITALFILTER_11CLK         I2C_CR1_DNF_11CLK           /**< �˳�С��11��I2C_KCLK������ */
#define I2C_DIGITALFILTER_12CLK         I2C_CR1_DNF_12CLK           /**< �˳�С��12��I2C_KCLK������ */
#define I2C_DIGITALFILTER_13CLK         I2C_CR1_DNF_13CLK           /**< �˳�С��13��I2C_KCLK������ */
#define I2C_DIGITALFILTER_14CLK         I2C_CR1_DNF_14CLK           /**< �˳�С��14��I2C_KCLK������ */
#define I2C_DIGITALFILTER_15CLK         I2C_CR1_DNF_15CLK           /**< �˳�С��15��I2C_KCLK������ */

/* I2C �ж�Դ���� */
#define I2C_INTERRUPT_ERR               I2C_CR1_ERRIE               /**< �����ж�        */
#define I2C_INTERRUPT_BUF               I2C_CR1_BUFIE               /**< �������ж�      */
#define I2C_INTERRUPT_EVT               I2C_CR1_EVTIE               /**< �¼��ж�        */

/* I2C ״̬��־λ */
#define I2C_FLAG_TXE                    I2C_ISR_TXE                 /**< �������ݼĴ���Ϊ�ձ�־    */
#define I2C_FLAG_TXIS                   I2C_ISR_TXIS                /**< �����ж�״̬��־          */
#define I2C_FLAG_RXNE                   I2C_ISR_RXNE                /**< �������ݼĴ����ǿձ�־    */
#define I2C_FLAG_ADDR                   I2C_ISR_ADDR                /**< ��ַƥ���־              */
#define I2C_FLAG_NACK                   I2C_ISR_NACKF               /**< ����NACK��־              */
#define I2C_FLAG_STOP                   I2C_ISR_STOPF               /**< ֹͣλ����־            */
#define I2C_FLAG_BERR                   I2C_ISR_BERR                /**< ���ߴ����־              */
#define I2C_FLAG_OVR                    I2C_ISR_OVR                 /**< �����־                  */
#define I2C_FLAG_BUSY                   I2C_ISR_BUSY                /**< ���߱�ռ�ñ�־            */
#define I2C_FLAG_DIR                    I2C_ISR_DIR                 /**< ���ݴ��䷽���־          */

/* I2C ���״̬λ */
#define I2C_CLEAR_ADDR                  I2C_ICR_ADDRCF              /**< �����ַƥ���־          */
#define I2C_CLEAR_NACK                  I2C_ICR_NACKCF              /**< ���NACK��־              */
#define I2C_CLEAR_STOP                  I2C_ICR_STOPCF              /**< ���ֹͣλ����־        */
#define I2C_CLEAR_BERR                  I2C_ICR_BERRCF              /**< ������ߴ����־          */
#define I2C_CLEAR_OVR                   I2C_ICR_OVRCF               /**< ��������־              */

/* I2C���䷽�� */
#define I2C_DIR_RX                      (0x00000000U)               /**< I2C��������               */
#define I2C_DIR_TX                      I2C_ISR_DIR                 /**< I2C��������               */

/* I2C ���ݴ��䷽������ */
#define I2C_REQUEST_WRITE               (0x00000000U)               /**< д����     */
#define I2C_REQUEST_READ                I2C_CR2_RD_WRN              /**< ������     */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup I2C_External_Functions I2C External Functions
* @brief    I2C���⺯��
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  ʹ��I2C�ӿ�
* @retval ��
*/
__STATIC_INLINE void std_i2c_enable(void)
{
    I2C1->CR1 |= I2C_CR1_PE;
}


/** 
* @brief  ��ֹI2C�ӿ�
* @retval ��
*/
__STATIC_INLINE void std_i2c_disable(void)
{
    I2C1->CR1 &= (~I2C_CR1_PE);
}


/** 
* @brief  ���������˲���
* @param  digital_filter �����˲������ò���
*             @arg I2C_DIGITALFILTER_DISABLE: ��ֹ�����˲���
*             @arg I2C_DIGITALFILTER_1CLK:    �˲�����С��1 * I2C_KCLK
*             @arg I2C_DIGITALFILTER_2CLK:    �˲�����С��2 * I2C_KCLK
*             @arg ...
*             @arg I2C_DIGITALFILTER_15CLK:   �˲�����С��15 * I2C_KCLK
* @note   �˲�������ֻ����I2C��ֹ��״̬����������Ч
* @retval ��
*/
__STATIC_INLINE void std_i2c_digital_filter_config(uint32_t digital_filter)
{
    MODIFY_REG(I2C1->CR1, I2C_CR1_DNF, digital_filter);
}


/** 
* @brief  I2C��ģʽʱ���ӳ����ܿ���
* @retval ��
*/
__STATIC_INLINE void std_i2c_clock_stretch_enable(void)
{
    I2C1->CR1 &= (~I2C_CR1_NOSTRETCH);
}


/** 
* @brief  I2C��ģʽʱ���ӳ����ܽ�ֹ
* @retval ��
*/
__STATIC_INLINE void std_i2c_clock_stretch_disable(void)
{
    I2C1->CR1 |= I2C_CR1_NOSTRETCH;
}


/** 
* @brief  I2C�㲥��ַӦ��ʹ��
* @retval ��
*/
__STATIC_INLINE void std_i2c_general_call_address_enable(void)
{
    I2C1->CR1 |= I2C_CR1_GCEN;
}


/** 
* @brief  I2C�㲥��ַӦ���ֹ
* @retval ��
*/
__STATIC_INLINE void std_i2c_general_call_address_disable(void)
{
    I2C1->CR1 &= (~I2C_CR1_GCEN);
}


/** 
* @brief  ����I2C��ģʽ��ַ1
* @param  dev_address I2C�豸��ַ1���ñ����ķ�Χ��0x0~0x7F֮�䣩
* @retval ��
*/
__STATIC_INLINE void std_i2c_device_address1_config(uint32_t dev_address)
{
    I2C1->ADDR1 = dev_address;
}


/** 
* @brief  ʹ��I2C�ж�
* @param  interrupt ʹ��I2C�ж�Դѡ��
*             @arg I2C_INTERRUPT_ERR:  I2C�����ж�
*             @arg I2C_INTERRUPT_BUF:  I2C�������ж�
*             @arg I2C_INTERRUPT_EVT:  I2C�¼��ж�
* @retval ��
*/
__STATIC_INLINE void std_i2c_interrupt_enable(uint32_t interrupt)
{
    I2C1->CR1 |= interrupt;
}


/** 
* @brief  ��ֹI2C�ж�
* @param  interrupt I2C�ж�ѡ��
*             @arg I2C_INTERRUPT_ERR:  I2C�����ж�
*             @arg I2C_INTERRUPT_BUF:  I2C�������ж�
*             @arg I2C_INTERRUPT_EVT:  I2C�¼��ж�
* @retval ��
*/
__STATIC_INLINE void std_i2c_interrupt_disable(uint32_t interrupt)
{
    I2C1->CR1 &= (~interrupt);
}


/** 
* @brief  ��ȡI2C�ж�ʹ��״̬
* @param  interrupt I2C�ж�ѡ��
*             @arg I2C_INTERRUPT_ERR:  I2C�����ж�
*             @arg I2C_INTERRUPT_BUF:  I2C�������ж�
*             @arg I2C_INTERRUPT_EVT:  I2C�¼��ж�
* @retval uint32_t ����ѡ���I2C�ж�ʹ��״̬
*             @arg ��0: ʹ��
*             @arg 0:   ��ֹ
*/
__STATIC_INLINE uint32_t std_i2c_get_interrupt_enable(uint32_t interrupt)
{
    return (I2C1->CR1 & interrupt);
}


/** 
* @brief  ��ȡI2C״̬��־λ�����ݴ��䷽��
* @param  flag I2C״̬��־λѡ��
*             @arg I2C_FLAG_TXE:      I2C�������ݼĴ���Ϊ��
*             @arg I2C_FLAG_TXIS:     I2C�����ж�״̬
*             @arg I2C_FLAG_RXNE:     I2C�������ݼĴ����ǿ�
*             @arg ...
*             @arg I2C_FLAG_DIR:      I2C���ݴ��䷽���־
* @retval uint32_t ����ѡ���I2C״̬��־λ״̬
*             @arg ��0: ��־λ��λ
*             @arg 0:   ��־λ���
*/
__STATIC_INLINE uint32_t std_i2c_get_flag(uint32_t flag)
{
    return (I2C1->ISR & flag);
}


/** 
* @brief  ���I2C״̬��־λ
* @param  flag I2C״̬��־λѡ��
*             @arg I2C_CLEAR_ADDR:     ���ADDR��־
*             @arg I2C_CLEAR_NACK:     ���NACKF��־
*             @arg I2C_CLEAR_STOP:     ���STOPF��־
*             @arg I2C_CLEAR_BERR:     ���BERR��־
*             @arg I2C_CLEAR_OVR:      ���OVR��־
* @retval ��
*/
__STATIC_INLINE void std_i2c_clear_flag(uint32_t flag)
{
    I2C1->ICR = flag;
}


/** 
* @brief  ���ô�ģʽ�µ�ַƥ�������¸�����ʱ����NACKӦ��
* @retval ��
*/
__STATIC_INLINE void std_i2c_set_next_data_nack(void)
{
    I2C1->CR2 = I2C_CR2_NACK;
}


/** 
* @brief  �����ݼĴ���
* @retval uint8_t �������ݼĴ����е�ֵ����ֵ��ΧΪ0x00~0xFF��
*/
__STATIC_INLINE uint8_t std_i2c_receive_byte(void)
{
    return (uint8_t)(I2C1->RDR);
}


/** 
* @brief  д���ݼĴ���
* @param  send_data д��I2C TDR�Ĵ����е�ֵ���ñ�����ΧΪ0x00~0xFF��
* @retval ��
*/
__STATIC_INLINE void std_i2c_transmit_byte(uint8_t send_data)
{
    I2C1->TDR = send_data;
}


/** 
* @brief  ����������ݼĴ���
* @retval ��
*/
__STATIC_INLINE void std_i2c_clear_tx_data(void)
{
    I2C1->ISR = I2C_ISR_TXE;
}


/* I2Cȥ��ʼ������ */
void std_i2c_deinit(void);

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

#endif /* CIU32F003_STD_I2C_H */
