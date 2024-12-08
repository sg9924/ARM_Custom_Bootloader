/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f407xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<stdarg.h>
#include<string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//enable this macro for debug messages over UART3
#define DEBUG_MSG_EN
#define DEBUG_UART &huart3  //Debug UART
#define VCOM_UART  &huart2  //Virtual COM UART

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t supported_commands[] = {
                               BL_GET_VER ,
                               BL_GET_HELP}

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
static void print_msg(char *format,...);
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
	
  /* USER CODE BEGIN 2 */
	
	/* If button is pressed -> go to bootloader, if not pressed -> go to user application */
  if(HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) == GPIO_PIN_RESET)
  {
	  print_msg("[Debug]: User Button is pressed -> Going into Bootloader....\n\r");

	  //continue in bootloader mode
	  bootloader_uart_read_data();
  }
  else
  {
	  print_msg("[Debug]: User Button is not pressed -> Entering User Application....\n\r");
	  
		//Jump to User Application
		bootloader_jump_to_user_app();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		uint32_t cur_tick = HAL_GetTick();
		print_msg("Current Tick Value = %d\r\n", cur_tick);
		while(HAL_GetTick()<= (cur_tick+500)); //delay using systick
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if(HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);


  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin*/
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


/* Custom Function Definitions Start */

//1. custom print function to print debug messages over UART3
static void print_msg(char *format,...)
 {
#ifdef DEBUG_MSG_EN
	char msg[100];

	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(msg, format,args);
	HAL_UART_Transmit(DEBUG_UART,(uint8_t *)msg, strlen(msg),HAL_MAX_DELAY);
	va_end(args);
#endif
 }

 
//2. Bootloader Command Handler
//Read and decode commands from Host
 void  bootloader_uart_read_data(void)
{
  uint8_t rcv_len=0;

	while(1)
	{
		memset(bl_rx_buffer,0,200);
		//read first byte from Host - Length of the packet
    HAL_UART_Receive(VCOM_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];

    //read second byte of the packet - Command Code
		HAL_UART_Receive(VCOM_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);

		switch(bl_rx_buffer[1])
		{
			case BL_GET_VER:
				bootloader_handle_getver_cmd(bl_rx_buffer);
				break;
      case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
      case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
      case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
			
      default:
        print_msg("[Debug]: Invalid command code received from host...\n");
        break;
		}
	}
}


//3. Bootloader read data
void bootloader_uart_write_data(uint8_t *pBuffer,uint32_t len)
{
    /*you can replace the below ST's USART driver API call with your MCUs driver API call */
	HAL_UART_Transmit(VCOM_UART,pBuffer,len,HAL_MAX_DELAY);

}

//4. Jump to User Application
void bootloader_jump_to_user_app(void)
{
	//function pointer to hold reset handler of User Application
  void (*app_reset_handler)(void);
	
	print_msg("[Debug]: bootloader_jump_to_user_app\n");
	
	//1. Configure MSP by reading value from the base address of Flash sector 2
  uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
  print_msg("[Debug]: MSP value: %#x\n",msp_value);
	
	//2. set MSP via CMSIS function
  __set_MSP(msp_value);
	
	//3. Fetch reset handler address from the 2nd address of Flash Sector 2
	uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS+4);
	
	//4. assign reset handler address to function pointer we defined
	app_reset_handler = (void*) resethandler_address;
	print_msg("[Debug]: User App Reset Handler Address: %#x\n",app_reset_handler);
	
	//5. call the user application reset handler
	app_reset_handler();
}
	
	
	
//Helper functions start
//1. Verify CRC
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    uint32_t uwCRCValue=0xFF;

		//accumulate each CRC byte
    for(uint32_t i=0; i<len; i++)
		{
        uint32_t i_data = pData[i];
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
		}

	 //Reset CRC Calculation Unit (HAL function)
  __HAL_CRC_DR_RESET(&hcrc);

	//check if the hist CRc and the CRC of the data received are same or not
	if(uwCRCValue == crc_host)
		return VERIFY_CRC_SUCCESS;

	return VERIFY_CRC_FAIL;
}

//2. Sending ACK
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
	//sending 2 bytes -> ACK + length to follow
	uint8_t ack_buf[2];
	ack_buf[0] = BL_ACK;
	ack_buf[1] = follow_len;
	HAL_UART_Transmit(VCOM_UART,ack_buf,2,HAL_MAX_DELAY);
}


//3. Sending NACK
void bootloader_send_nack(void)
{
	uint8_t nack = BL_NACK;
	HAL_UART_Transmit(VCOM_UART,&nack,1,HAL_MAX_DELAY);
}


//4. get bootloader version value
uint8_t get_bootloader_version(void)
{
  return (uint8_t)BL_VERSION;
}


//5. get MCU Chip ID
uint16_t get_mcu_chip_id(void)
{
	uint16_t cid;
	cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;
	return  cid;
}


//6. get flash RDP status
uint8_t get_flash_rdp_level(void)
{

	uint8_t rdp_status=0;
  #if 0
	  FLASH_OBProgramInitTypeDef  ob_handle;
	  HAL_FLASHEx_OBGetConfig(&ob_handle);
	  rdp_status = (uint8_t)ob_handle.RDPLevel;
  #else
	 volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
	 rdp_status =  (uint8_t)(*pOB_addr >> 8);
  #endif

	return rdp_status;
}


//7. execute flash erase
uint8_t execute_flash_erase(uint8_t sector_number , uint8_t number_of_sector)
{
  //8 sectors in stm32f407 -> sector[0 to 7]
	//sector_number = 0xFF -> Mass Erase!

	FLASH_EraseInitTypeDef flashErase_handle;
	uint32_t sectorError;
	HAL_StatusTypeDef status;


	if(number_of_sector > 8)
		return INVALID_SECTOR;

	if((sector_number == 0xFF) || (sector_number <= 7))
	{
		if(sector_number == (uint8_t) 0xFF)
		{
			flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;  //setting up erase type in HAL Flash Handle
		}
    else
		{
			uint8_t remanining_sector = 8 - sector_number;
      if(number_of_sector > remanining_sector)
        number_of_sector = remanining_sector;

			flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
			flashErase_handle.Sector = sector_number; //initial sector
			flashErase_handle.NbSectors = number_of_sector;
		}

		flashErase_handle.Banks = FLASH_BANK_1;

		HAL_FLASH_Unlock();
		flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;  //Flash Voltage Range for stm32f407
		status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
		HAL_FLASH_Lock();

		return status;
	}
	return INVALID_SECTOR;
}


//8. verify address
uint8_t verify_address(uint32_t go_address)
{
	//we allow jumping to system, sram1, srma2, backup sram, external memory but not peripheral emmory

	if(go_address >= SRAM1_BASE && go_address <= SRAM1_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= SRAM2_BASE && go_address <= SRAM2_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= FLASH_BASE && go_address <= FLASH_END)
	{
		return ADDR_VALID;
	}
	else if(go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
	{
		return ADDR_VALID;
	}
	else
		return ADDR_INVALID;
}


//9. Configure Flash R/W protection
/*
Modifying user option bytes sequence:
1. Check for no ongoing Flash Memory Operation using the BSY bit in FLASH_SR register
2. Write the desired option value in FLASH_OPTCR register.
3. Set the option start bit (OPTSTRT) in FLASH_OPTCR register
4. Wait for the BSY bit to be cleared.
*/
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
  //configure the protection mode
  //protection_mode = 1 -> write protect of  user flash sectors
  //protection_mode = 2 -> read/write protect of  user flash sectors
  //For stm32f407xx, We have to modify the address 0x1FFF C008 bit 15(SPRMOD)

	 //Flash option control register (OPTCR)
  volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

	if(disable)
	{
		//disable r/w protection on all sectors

		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//clear the 31st bit (default state)
		*pOPTCR &= ~(1 << 31);
		//clear the protection: make all bits belonging to sectors as 1
		*pOPTCR |= (0xFF << 16);
		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
		return 0;
	}

	if(protection_mode == (uint8_t) 1)
  {
    //write protection on the sectors encoded in sector_details argument

		//Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//here we are setting just write protection for the sectors
		//clear the 31st bit
		*pOPTCR &= ~(1 << 31);
		//put write protection on sectors
		*pOPTCR &= ~ (sector_details << 16);
		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
	}
	else if (protection_mode == (uint8_t) 2)
  {
	  //Option byte configuration unlock
		HAL_FLASH_OB_Unlock();

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		//here wer are setting read and write protection for the sectors
		//set the 31st bit
		//please refer : Flash option control register (FLASH_OPTCR) in RM
		*pOPTCR |= (1 << 31);

		//put read and write protection on sectors
    *pOPTCR &= ~(0xff << 16);
		*pOPTCR |= (sector_details << 16);

		//Set the option start bit (OPTSTRT) in the FLASH_OPTCR register
		*pOPTCR |= ( 1 << 1);

		//wait till no active operation on flash
		while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

		HAL_FLASH_OB_Lock();
  }
  return 0;
}
//Helper functions end
	
	
	
//Bootloader Command functions start
//1. Get Bootloader Version
	void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;

      print_msg("[Debug]: bootloader_handle_getver_cmd\n");

	  //getting total length of the packet -> length to follow + 1
	  uint32_t command_packet_len = bl_rx_buffer[0]+1;

	  //Extract CRC32 sent by the Host Application
	  uint32_t host_crc = *((uint32_t*) (bl_rx_buffer + command_packet_len - 4));

		//CRC is successfull
    if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
    {
        print_msg("[Debug]: Checksum Success!!\n");
        //Sending ACK as checksum is correct
        bootloader_send_ack(bl_rx_buffer[0],1);
			
				//get bootloader version
        bl_version=get_bootloader_version();
			
        print_msg("[Debug]: BL_VER -> %d %#x\n",bl_version,bl_version);
			
				//send the bootloader version number to Host Application
        bootloader_uart_write_data(&bl_version,1);
    }
		else //CRC is a failure
    {
        print_msg("[Debug]: Checksum Fail...\n");
			
        //Sending NACK as checksum is wrong
        bootloader_send_nack();
    }
}


//2. Get Help
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    print_msg("[Debug]: bootloader_handle_gethelp_cmd\n");

	//getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

  //CRC is successfull
	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    //Sending ACK as checksum is correct
    bootloader_send_ack(pBuffer[0],sizeof(supported_commands));

    //send the supported Bootloader commands to the Host Application
    bootloader_uart_write_data(supported_commands,sizeof(supported_commands));

	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");

    //Sending NACK as checksum is wrong
    bootloader_send_nack();
	}
}


//3. Get CID
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
	uint16_t bl_cid_num = 0;
	print_msg("[Debug]: bootloader_handle_getcid_cmd\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    //Sending ACK as checksum is correct
    bootloader_send_ack(pBuffer[0],2);

    //get MCU Chip ID
    bl_cid_num = get_mcu_chip_id();

    print_msg("[Debug]: MCU ID : %d %#x !!\n",bl_cid_num, bl_cid_num);

    //send the cid number to the Host Application
    bootloader_uart_write_data((uint8_t *)&bl_cid_num,2);
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");

    //Sending NACK as checksum is wrong
    bootloader_send_nack();
	}
}


//4. Get RDP Status
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    uint8_t rdp_level = 0x00;
    print_msg("[Debug]: bootloader_handle_getrdp_cmd\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*)(bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    //Sending ACK as checksum is correct
    bootloader_send_ack(pBuffer[0],1);

    //get flash RDP level
    rdp_level = get_flash_rdp_level();

    print_msg("[Debug]: RDP level -> %d %#x\n",rdp_level,rdp_level);

    //send the flash rdp status to the Host Application
    bootloader_uart_write_data(&rdp_level,1);
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail!..\n");

    //Sending NACK as checksum is wrong
    bootloader_send_nack();
	}
}


//5. Go to Address Command
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    uint32_t go_address=0;
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    print_msg("[Debug]: bootloader_handle_go_cmd\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]:Checksum Success!!\n");
    bootloader_send_ack(pBuffer[0],1);

    //Extract the Go Address
    go_address = *((uint32_t *)&pBuffer[2]);
    print_msg("[Debug]: Go Address -> %#x\n",go_address);

    if(verify_address(go_address) == ADDR_VALID)
    {
      //tell host that address is fine
      bootloader_uart_write_data(&addr_valid,1);

      /* Not doing the below line will result in hardfault exception for ARM cortex M */
      go_address+=1; //make T bit=1

      void (*jump)(void) = (void *)go_address;
      print_msg("[Debug]: Jumping to Go Address..\n");

      //Jumping to the Go Address
      jump();
		}
    else
		{
      print_msg("[Debug]: Go Address is Invalid..\n");
      //inform Host that Go Address is invalid
      bootloader_uart_write_data(&addr_invalid,1);
		}
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");
    bootloader_send_nack();
	}
}


//6. Flash Erase Command
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
  uint8_t erase_status = 0x00;
  print_msg("[Debug]:bootloader_handle_flash_erase_cmd\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    bootloader_send_ack(pBuffer[0],1);
    print_msg("[Debug]: Initial Sector -> %d , No of Sectors -> %d\n",pBuffer[2],pBuffer[3]);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,1); //make LED pin high while erasing flash
    //execute flash erase via HAL function
    erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,0); //after erase operation is over, make LEd pin low

    print_msg("[Debug]: Flash Erase Status -> %#x\n",erase_status);

    //send flash erase status to Host
    bootloader_uart_write_data(&erase_status,1);
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");
    bootloader_send_nack();
	}
}


//7. Memory Write Command
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
	uint8_t addr_valid = ADDR_VALID;
	uint8_t write_status = 0x00;
	uint8_t chksum =0, len=0;
	len = pBuffer[0];
	uint8_t payload_len = pBuffer[6];

	uint32_t mem_address = *((uint32_t *) (&pBuffer[2]));

	chksum = pBuffer[len];

  print_msg("[Debug]:bootloader_handle_mem_write_cmd\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    bootloader_send_ack(pBuffer[0],1);
    print_msg("[Debug]: Memory Write Address -> %#x\n",mem_address);

		if(verify_address(mem_address) == ADDR_VALID)
		{
      print_msg("[Debug]: Valid Memory Write Address\n");

      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);    //make LED pin high while writing to memory
      //Execute Memory Write
      write_status = execute_mem_write(&pBuffer[7],mem_address, payload_len);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  //make LED pin low after memory write operation is done

      //Send status of memory write operation to Host
      bootloader_uart_write_data(&write_status,1);
		}
    else
		{
      print_msg("[Debug]: Invalid Memory Write Address..n");
      write_status = ADDR_INVALID;

      //Inform Host that memory write address is invalid
      bootloader_uart_write_data(&write_status,1);
		}
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");
    bootloader_send_nack();
	}
}


//8. Enable read write protection command
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
  uint8_t status = 0x00;
  print_msg("[Debug]: bootloader_handle_en_rw_protect\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if (!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    bootloader_send_ack(pBuffer[0],1);

    status = configure_flash_sector_rw_protection(pBuffer[2],pBuffer[3],0);
    print_msg("[Debug]: Flash Sector Enable R/W Protection Status -> %#x\n",status);
    //send status to the Host
    bootloader_uart_write_data(&status,1);
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: CHecksum Fail..\n");
    bootloader_send_nack();
	}
}


//9. Disable read write protection command
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
  uint8_t status = 0x00;
  print_msg("[Debug]: bootloader_handle_dis_rw_protect\n");

  //getting total length of the packet -> length to follow + 1
	uint32_t command_packet_len = bl_rx_buffer[0]+1 ;

	//Extract CRC32 sent by the Host Application
	uint32_t host_crc = *((uint32_t*) (bl_rx_buffer+command_packet_len - 4));

	if(!bootloader_verify_crc(&bl_rx_buffer[0],command_packet_len-4,host_crc))
	{
    print_msg("[Debug]: Checksum Success!!\n");
    bootloader_send_ack(pBuffer[0],1);

    status = configure_flash_sector_rw_protection(0,0,1); //R/W protection will be disabled on all sectors -> so it doesn't matter which sector number you pass as argument
    print_msg("[Debug]: Flash Sector Disable R/W Protection Status -> %#x\n",status);

    bootloader_uart_write_data(&status,1);
	}
  else //CRC is a failure
	{
    print_msg("[Debug]: Checksum Fail..\n");
    bootloader_send_nack();
	}
}
//Bootloader Command function end
/* Custom Function Definitions End */