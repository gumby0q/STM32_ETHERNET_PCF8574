/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* vim: set ai et ts=4 sw=4: */
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <ctype.h>
#include "socket.h"
#include "dhcp.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

//UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
//static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DHCP_SOCKET     0
#define DNS_SOCKET      1
#define HTTP_SOCKET     2
#define SOCK_UDP_CLIENT	3

#define PCF857x_ADDRESS1 0x20
#define PCF857x_ADDRESS2 0x22
#define PCF857x_ADDRESS3 0x20

#define PCF857x_I2C_ADDR1        PCF857x_ADDRESS1<<1 // 0x38<<1 = 0x70
#define PCF857x_I2C_ADDR2        PCF857x_ADDRESS2<<1 // 0x38<<1 = 0x70
#define PCF857x_I2C_ADDR3        PCF857x_ADDRESS3<<1 // 0x38<<1 = 0x70

#define DATA_BUF_SIZE   100

uint8_t gDATABUF[DATA_BUF_SIZE];
uint8_t gateWay[4] = {192, 168, 1, 1};

uint16_t expander1_state = 0;
uint16_t expander2_state = 0;
uint16_t expander3_state = 0;
uint16_t global_state = 0;
uint8_t  expander_state_buffer[2]  = {0x0,0x0};

#define json_answer	"HTTP/1.0 200 OK\r\n"\
		"Content-Type: application/json\r\n"\
		"Access-Control-Allow-Origin: *\r\n"\
						"\r\n"\
						"{\"success\": %d }\n"


//void UART_Printf(const char* fmt, ...) {
//    char buff[256];
//    va_list args;
//    va_start(args, fmt);
//    vsnprintf(buff, sizeof(buff), fmt, args);
//    HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff),
//                      HAL_MAX_DELAY);
//    va_end(args);
//}

void W5500_Select(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}
uint8_t dhcp_buffer[1];

volatile bool ip_assigned = false;

void Callback_IPAssigned(void) {
//    UART_Printf("Callback: IP assigned! Leased time: %d sec\r\n", getDHCPLeasetime());
    ip_assigned = true;
}

void Callback_IPConflict(void) {
  //  UART_Printf("Callback: IP conflict!\r\n");
}

void init() {
//    UART_Printf("\r\ninit() called!\r\n");
//    UART_Printf("Registering W5500 callbacks...\r\n");

	uint8_t tmp;
    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

//    UART_Printf("Calling wizchip_init()...\r\n");
    uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};

	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1){
		//init fail
		//printf("WIZCHIP Initialized fail.\r\n");
		while(1);
	}

	do{
		if(ctlwizchip(CW_GET_PHYLINK, (void*)&tmp) == -1){};
			//printf("Unknown PHY Link stauts.\r\n");
	}while(tmp == PHY_LINK_OFF);

	wiz_NetInfo net_info = { .mac = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef},
	                            .ip = {192, 168, 1, 154},
	                            .sn = {255, 255, 254, 0},
	                            .gw = gateWay,
	                            .dns = {0, 0, 0, 0},
	                            .dhcp = NETINFO_DHCP };

	setSHAR(net_info.mac);			//set MAC addr
	setSIPR(net_info.ip);			//set IP addr
	setGAR(net_info.gw);			//set gate way
	setSUBR(net_info.sn);			//set subnet
///////////////////////////////////////////
	return;
//    DHCP_init(DHCP_SOCKET, dhcp_buffer);
//    reg_dhcp_cbfunc(
//           Callback_IPAssigned,
//           Callback_IPAssigned,
//           Callback_IPConflict
//       );
//
//    uint32_t ctr = 10000;
//	while((!ip_assigned) && (ctr > 0)) {
//		DHCP_run();
//		ctr--;
//	}
//	if(!ip_assigned) {
////		UART_Printf("\r\nIP was not assigned :(\r\n");
//		return;
//	}
//
//	getIPfromDHCP(net_info.ip);
//	getGWfromDHCP(net_info.gw);
//	getSNfromDHCP(net_info.sn);
//
//	 wizchip_setnetinfo(&net_info);

}

void write_global_state(){
	expander_state_buffer[0] = global_state;
	expander_state_buffer[1] = global_state;
	HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR1, expander_state_buffer, 2, 50);
	HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR2, expander_state_buffer, 2, 50);
	HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR3, expander_state_buffer, 2, 50);
}


int32_t loopback_tcps_g(uint8_t sn, uint8_t* buf, uint16_t port){
	int32_t ret;
	uint16_t size = 0, sentsize=0;
	uint8_t *url;
	uint8_t pin = 0;
	uint8_t temp_pin = 0;
	uint8_t temp_addr = 0;

	switch(getSn_SR(sn)){
		case SOCK_ESTABLISHED :
			if(getSn_IR(sn) & Sn_IR_CON){
//				printf("%d:Connected\r\n",sn);
				setSn_IR(sn,Sn_IR_CON);
			}
			if((size = getSn_RX_RSR(sn)) > 0){
				if(size > DATA_BUF_SIZE) size = DATA_BUF_SIZE;
				ret = recv(sn,buf,size);
				if(ret <= 0) return ret;
				sentsize = 0;
				if(memcmp(buf, "GET ", 4)==0){
					// extract URL from request header
					url = buf + 4;
					//http data fill
					if(memcmp(url, "/toggle/", 8)==0){
						pin = atoi(url+8);
						temp_addr = 0;
						if(pin < 16){
							temp_pin = pin;
							temp_addr = PCF857x_I2C_ADDR1;
							expander1_state ^= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander1_state;
							expander_state_buffer[1] = (uint8_t)(expander1_state>>8);

						}else if(pin<32){
							temp_pin = pin-16;
							temp_addr = PCF857x_I2C_ADDR2;
							expander2_state ^= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander2_state;
							expander_state_buffer[1] = (uint8_t)(expander2_state>>8);
						}else if(pin<48){
							temp_pin = pin-32;
							temp_addr = PCF857x_I2C_ADDR3;
							expander3_state ^= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander3_state;
							expander_state_buffer[1] = (uint8_t)(expander3_state>>8);
						}else{

						}
						if(temp_addr){
							HAL_I2C_Master_Transmit(&hi2c1, temp_addr, expander_state_buffer, 2, 50);
						}
						sprintf(buf, json_answer,1);
					}
					else if(memcmp(url, "/toggle", 7)==0){
						pin = 0;
						global_state = (global_state == 0) ? 0xffff:0;
						expander1_state = global_state;
						expander2_state = global_state;
						expander3_state = global_state;
						write_global_state();
						sprintf(buf, json_answer,2);
					}else if(memcmp(url, "/off/", 5)==0){
						pin = atoi(url+5);
						temp_addr = 0;
						if(pin < 16){
							temp_pin = pin;
							temp_addr = PCF857x_I2C_ADDR1;
							expander1_state &= ~(1UL << temp_pin);
							expander_state_buffer[0] = (uint8_t)expander1_state;
							expander_state_buffer[1] = (uint8_t)(expander1_state>>8);
						}else if(pin<32){
							temp_pin = pin-16;
							temp_addr = PCF857x_I2C_ADDR2;
							expander2_state &= ~(1UL << temp_pin);
							expander_state_buffer[0] = (uint8_t)expander2_state;
							expander_state_buffer[1] = (uint8_t)(expander2_state>>8);
						}else if(pin<48){
							temp_pin = pin-32;
							temp_addr = PCF857x_I2C_ADDR3;
							expander3_state &= ~(1UL << temp_pin);
							expander_state_buffer[0] = (uint8_t)expander3_state;
							expander_state_buffer[1] = (uint8_t)(expander3_state>>8);
						}
						if(temp_addr){
							HAL_I2C_Master_Transmit(&hi2c1, temp_addr, expander_state_buffer, 2, 50);
						}
						sprintf(buf, json_answer,6);
					}else if(memcmp(url, "/off", 4)==0){
						global_state = 0;
						expander1_state = global_state;
						expander2_state = global_state;
						expander3_state = global_state;
						write_global_state();
						sprintf(buf, json_answer,4);
					}else if(memcmp(url, "/on/", 4)==0){
						pin = atoi(url+4);

						if(pin < 16){
							temp_pin = pin;
							temp_addr = PCF857x_I2C_ADDR1;
							expander1_state |= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander1_state;
							expander_state_buffer[1] = (uint8_t)(expander1_state>>8);
						}else if(pin<32){
							temp_pin = pin-16;
							temp_addr = PCF857x_I2C_ADDR2;
							expander2_state |= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander2_state;
							expander_state_buffer[1] = (uint8_t)(expander2_state>>8);
						}else if(pin<48){
							temp_pin = pin-32;
							temp_addr = PCF857x_I2C_ADDR3;
							expander3_state |= 1UL << temp_pin;
							expander_state_buffer[0] = (uint8_t)expander3_state;
							expander_state_buffer[1] = (uint8_t)(expander3_state>>8);
						}
						HAL_I2C_Master_Transmit(&hi2c1, temp_addr, expander_state_buffer, 2, 50);
						sprintf(buf, json_answer,5);
					}else if(memcmp(url, "/on", 3)==0){
						global_state = 0xffff;
						expander1_state = global_state;
						expander2_state = global_state;
						expander3_state = global_state;

						write_global_state();
						sprintf(buf, json_answer,3);


					}else{
						pin = 200;
						sprintf(buf, json_answer,7);
					}
					size=strlen(buf);

					//sending answer
					while(size != sentsize){
						ret = send(sn,buf+sentsize,size-sentsize);
						if(ret < 0){
							close(sn);
							return ret;
						}
						sentsize += ret; // Don't care SOCKERR_BUSY, because it is zero.
					}
				}
				//ending
				disconnect(sn);

			}
			break;
		case SOCK_CLOSE_WAIT :
//			printf("%d:CloseWait\r\n",sn);
			if((ret=disconnect(sn)) != SOCK_OK) return ret;
//			printf("%d:Closed\r\n",sn);
			break;
		case SOCK_INIT :
//			printf("%d:Listen, port [%d]\r\n",sn, port);
			if( (ret = listen(sn)) != SOCK_OK) return ret;
			break;
		case SOCK_CLOSED:
//			printf("%d:LBTStart\r\n",sn);
			if((ret=socket(sn,Sn_MR_TCP,port,0x00)) != sn)
			return ret;
	//		printf("%d:Opened\r\n",sn);
			break;
		default:
			break;
	}
	return 1;
}


void loop() {
}

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
  MX_I2C1_Init();
  MX_SPI1_Init();
//  MX_USART1_UART_Init();


  /* USER CODE BEGIN 2 */
  init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR1, expander_state_buffer, 2, 50);
  HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR2, expander_state_buffer, 2, 50);
  HAL_I2C_Master_Transmit(&hi2c1, PCF857x_I2C_ADDR3	, expander_state_buffer, 2, 50);

  while (1)
  {
	loopback_tcps_g(HTTP_SOCKET,gDATABUF, 80);
//    counter++;
//	close(SOCK_UDP_CLIENT);
//	socket(SOCK_UDP_CLIENT, Sn_MR_UDP, 3102, SF_IO_NONBLOCK);
//	sprintf(gDATABUF, UDP_info_answer, 1);
//	sendto(SOCK_UDP_CLIENT, gDATABUF, strlen(gDATABUF), gateWay, 3015);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_USART1_UART_Init(void)
//{
//
//  /* USER CODE BEGIN USART1_Init 0 */
//
//  /* USER CODE END USART1_Init 0 */
//
//  /* USER CODE BEGIN USART1_Init 1 */
//
//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
//  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */
//
//  /* USER CODE END USART1_Init 2 */
//
//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS0_Pin */
  GPIO_InitStruct.Pin = SPI1_CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS0_GPIO_Port, &GPIO_InitStruct);

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
