/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Custom Function Prototypes */
//1. Bootloader Functions
void bootloader_uart_read_data(void);
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len);
void bootloader_jump_to_user_app(void);

//2. Helper Functions
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len);
void bootloader_send_nack(void);

uint8_t bootloader_verify_crc (uint8_t *pData, uint32_t len,uint32_t crc_host);

uint8_t get_bootloader_version(void);
uint16_t get_mcu_chip_id(void);
uint8_t get_flash_rdp_level(void);
uint8_t verify_address(uint32_t go_address);
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector);
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len);
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable);
uint16_t read_OB_rw_protection_status(void);

//3. Bootloader Handle Command Functions
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);


/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

#define BL_VERSION 0x10

//CRC
#define VERIFY_CRC_FAIL    1
#define VERIFY_CRC_SUCCESS 0

//ACK & NACK
#define BL_ACK   0XA5
#define BL_NACK  0X7F

#define ADDR_VALID 0x00
#define ADDR_INVALID 0x01

#define INVALID_SECTOR 0x04

//RAM & FLASH
#define SRAM1_SIZE             112*1024                   //112KB
#define SRAM1_END              (SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE             16*1024                    //16KB
#define SRAM2_END              (SRAM2_BASE + SRAM2_SIZE)
#define FLASH_SIZE             512*1024                   //512KB
#define BKPSRAM_SIZE           4*1024                     //4KB
#define BKPSRAM_END            (BKPSRAM_BASE + BKPSRAM_SIZE)

//Bootloader Command definition
#define BL_GET_VER				      0x51
#define BL_GET_HELP				      0x52
#define BL_GET_CID				      0x53
#define BL_GET_RDP_STATUS		    0x54
#define BL_GO_TO_ADDR			      0x55
#define BL_FLASH_ERASE          0x56
#define BL_MEM_WRITE			      0x57
#define BL_EN_RW_PROTECT		    0x58
#define BL_MEM_READ				      0x59
#define BL_READ_SECTOR_P_STATUS	0x5A

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */