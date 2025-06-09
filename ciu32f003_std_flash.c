/************************************************************************************************/
/**
* @file               ciu32f003_std_flash.c
* @author             MCU Ecosystem Development Team
* @brief              FLASH STD��������
*                     ʵ��FLASH���������API��
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
* @addtogroup FLASH 
* @{
*
*/
/************************************************************************************************/


/*------------------------------------------includes--------------------------------------------*/
#include "ciu32f003_std.h"

#ifdef STD_FLASH_PERIPHERAL_USED

/*-------------------------------------------functions------------------------------------------*/

/************************************************************************************************/
/**
* @addtogroup FLASH_External_Functions 
* @{
*
*/
/************************************************************************************************/ 

/**
* @brief  Flash������Option byte������
* @param  mode ����ģʽ
*             @arg FLASH_MODE_PAGE_ERASE
*             @arg FLASH_MODE_MASS_ERASE
* @param  address �������ʵ�ַ
* @note   user flash������ʱ�����ȵ�std_flash_unlock()������flash
* @note   Option Byte������ʱ�����ȵ���std_flash_opt_unlock()������ѡ���ֽ�
* @retval std_status_t APIִ�н��
*/
std_status_t std_flash_erase(uint32_t mode, uint32_t address)
{
    std_status_t status = STD_OK;
    
    /* ���ò���ģʽ */
    std_flash_set_operate_mode(mode);
    
    /* ִ�в��� */
    *(uint32_t *)address = 0xFFFFFFFF;
    
    /* �ȴ�������ɣ���ѯ�쳣��־λ */
    while (std_flash_get_flag(FLASH_FLAG_BSY));
    if ((FLASH->SR & FLASH_FLAG_WRPERR) != 0x00000000U)
    {
        status = STD_ERR;
    }
    
    /* ������б�־ */
    std_flash_clear_flag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
    
    /* �˳�����ģʽ */
    std_flash_set_operate_mode(FLASH_MODE_IDLE);
    
    return (status);
}


/**
* @brief  User Flash����Option Byte���ֱ��
* @param  address   ��̵�ַ
* @param  prog_data ������ݣ�4�ֽڣ�
* @note   user flash�����ʱ�����ȵ�std_flash_unlock()������flash
* @note   Option Byte���ֱ��ʱ�����ȵ���std_flash_opt_unlock()������ѡ���ֽ�
* @retval std_status_t APIִ�н��
*/
std_status_t std_flash_word_program(uint32_t address, uint32_t prog_data)
{
    std_status_t status = STD_OK;
    
    /* ������ģʽ */
    std_flash_set_operate_mode(FLASH_MODE_PROGRAM);

    /* ��Ŀ���ַд������ */
    *(uint32_t *)address = prog_data;
    
    /* �ȴ������ɣ���ѯ�쳣��־λ */
    while (std_flash_get_flag(FLASH_FLAG_BSY));
    if ((FLASH->SR & FLASH_FLAG_WRPERR) != 0x00000000U)
    {
        status = STD_ERR;
    }
    
    if (status == STD_OK)
    {
        /* ����������Ƿ���ȷ */
        if(*((__IO uint32_t *)address) != prog_data)
        {
            status = STD_ERR;
        }
    }
    
    /* ������б�־ */
    std_flash_clear_flag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
    
    /* �˳����ģʽ */
    std_flash_set_operate_mode(FLASH_MODE_IDLE);
    
    return (status);
}


/** 
* @} 
*/

#endif /* STD_FLASH_PERIPHERAL_USED */

/** 
* @} 
*/

/** 
* @} 
*/
