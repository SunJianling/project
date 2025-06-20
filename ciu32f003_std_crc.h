/************************************************************************************************/
/**
* @file               ciu32f003_std_crc.h
* @author             MCU Ecosystem Development Team
* @brief              CRC STD库驱动头文件。
*                     提供CRC相关的STD库操作函数声明、数据类型以及常量的定义。                         
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* 避免头文件重复引用 */
#ifndef CIU32F003_STD_CRC_H
#define CIU32F003_STD_CRC_H

/************************************************************************************************/
/** 
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @defgroup CRC CRC
* @brief 循环冗余校验的STD库驱动
* @{
*/
/************************************************************************************************/


#ifdef __cplusplus
 extern "C" {
#endif

/*------------------------------------------includes--------------------------------------------*/
#include "ciu32F003_std_common.h"


/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup CRC_Constants CRC Constants 
* @brief    CRC常量定义及宏定义
* @{
*
*/
/************************************************************************************************/
/* CRC多项式选择 */
#define CRC_POLY_16                           CRC_CSR_POLY_SIZE_16     /**< 16位多项式       */
#define CRC_POLY_32                           CRC_CSR_POLY_SIZE_32     /**< 32位多项式       */

/* CRC默认初始值 */
#define CRC_DEFAULT_INIT_VALUE                (0xFFFFFFFFU)            /**< CRC默认初始值    */

/** 
* @} 
*/

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup CRC_External_Functions CRC External Functions
* @brief    CRC对外函数
* @{
*
*/
/************************************************************************************************/
/** 
* @brief  配置CRC多项式
* @param  poly_size 多项式选择
*             @arg CRC_POLY_16：16位多项式
*             @arg CRC_POLY_32：32位多项式
* @retval 无
*/
__STATIC_INLINE void std_crc_set_poly_size(uint32_t poly_size)
{
    CRC->CSR = poly_size;
}

/**
* @brief  将CRC初始值未进行反转直接写入寄存器
* @param  init_value CRC的初始值
* @retval 无
*/
__STATIC_INLINE void std_crc_set_init_value(uint32_t init_value)
{
    CRC->RDR = init_value;
}

/**
* @brief  读取CRC计算结果
* @retval uint32_t CRC计算结果
*/
__STATIC_INLINE uint32_t std_crc_get_value(void)
{
    return(CRC->RDR);
}

/**
* @brief  向数据寄存器中写入输入的1字节数据
* @param  input_data 要进行CRC计算的原始数据
* @retval 无
*/
__STATIC_INLINE void std_crc_set_byte(uint8_t input_data)
{
    CRC->DR = (uint32_t)input_data;
}


void std_crc_deinit(void);
void std_crc_set_init_value_invert(uint32_t poly_sel, uint32_t init_value);

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

#endif /* CIU32F003_STD_CRC_H */
