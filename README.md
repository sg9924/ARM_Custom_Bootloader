# Custom Bootloader development for STM32F407


# Initial Steps

1. Install STM Cube MX -> used to create code in an instant using HAL libraries automatically by specifying the peripherals we need to use
2. Install Teraterm -> for reading UART messages from COM ports
3. Install Keil uVision 5
4. By default, the compiler version 5 does not come installed in uVision, so use version 6 which is pre-installed
5. Create a stm32 account as we need it to use the Cube MX software




# Bootloader Code setup

1. Enable the following peripherals -> GPIO, USART2, USART3, CRC, RCC
2. Select an appropriate workspace folder to save the project.
3. Specify the uVision workspace path for the generated code
4. Click the generate code option to generate the code
5. Install any extensions or software when prompted (user login required - STM32 account)
6. Open the generated code in uVision
7. If any other code apart from the above peripherals are present, remove those and compile the code to verify in uVision



# UART Communication Setup
1. Obtain a UART to USB converter -> https://www.amazon.in/dp/B072TGR223?ref=ppx_yo2ov_dt_b_fed_asin_title
2. For this board -> STM32F407VG the USART is not connected to the ST-LINK, so when the board is connected we can't view any messages sent via USART2
3. Hence we need a separate device which is connected to the USART GPIO pins in the board and to the host computer via the USB
4. But for Nucelo Boards, USART2 is connected to the ST-LINK which allows us to use that connection iteself to view USART messages
5. Driver Installation for the USART to USB converter -> https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers
6. After installation of the driver, the converte device shows under COM ports in Device manager when connected