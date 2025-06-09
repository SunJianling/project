/************************************************************************************************/
/**
* @file               ciu32f003_std_common.h
* @author             MCU Ecosystem Development Team
* @brief              STD��ͨ�õ���ض��塣  
*                     
*
**************************************************************************************************
* @attention
* Copyright (c) CEC Huada Electronic Design Co.,Ltd. All rights reserved.
*
**************************************************************************************************
*/

/* ����ͷ�ļ��ظ����� */
#ifndef CIU32F003_STD_COMMON_H
#define CIU32F003_STD_COMMON_H

/************************************************************************************************/
/**
* @addtogroup CIU32F003_STD_Driver
* @{
*/

/**
* @addtogroup STD
* @{
*
*/
/************************************************************************************************/ 


#ifdef __cplusplus
extern "C" {
#endif


       
/*-----------------------------------------type define------------------------------------------*/

/************************************************************************************************/
/**
* @defgroup STD_Types STD Types
* @brief STD��ͨ���������Ͷ���
* @{
*
*/
/************************************************************************************************/ 
/**
* @brief  bitλ״̬����
*/
typedef enum
{
    RESET = 0,
    SET = !RESET
}bit_status_t;


/**
* @brief  STD��API����ֵ���Ͷ���
*/
typedef enum
{
    STD_OK                   = 0x00U,
    STD_ERR                  = 0x01U,
    STD_ERR_PARAM            = 0x02U,
    STD_ERR_BUSY             = 0x03U,
    STD_ERR_TIMEOUT          = 0x04U
} std_status_t;


/**
* @}
*/
  

/*--------------------------------------------define--------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup STD_Constants 
* @{
*
*/
/************************************************************************************************/ 

/* ������뾯�� */
#define UNUSED(X) (void)X      


/* ���������컯���� */
#if  defined ( __GNUC__ )
    #ifndef __weak
        #define __weak   __attribute__((weak))
    #endif /* __weak */
    
    #ifndef __packed
        #define __packed __attribute__((__packed__))
    #endif /* __packed */
#endif /* __GNUC__ */


/* bit operations */
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

/** 
* @} 
*/
                                    
                                    
/*------------------------------------------includes--------------------------------------------*/
#include <stddef.h>
#include <stdbool.h>
#include "ciu32f003.h"
#include "ciu32f003_std.h"


/*-------------------------------------------functions------------------------------------------*/

                                    
#ifdef __cplusplus
}
#endif


/**
* @} 
*/  

/**
* @}
*/


#endif /* CIU32F003_STD_COMMON_H */
