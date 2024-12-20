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
#include "stdio.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CO2_SENSOR_ADDRESS 0x28<<1
#define CO2_SENSOR_STATUS 0x01
#define CO2_MSB_RATE 0x02
#define CO2_LSB_RATE 0x03
#define CO2_CONFIG 0x04
#define CO2_MSB 0x05
#define CO2_LSB 0x06

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void PERIOD_INIT(void);
void MODE_CONFIG(void);
void SENSOR_STATUS_CHECK(void);
uint16_t get_CO2(void);
uint8_t get_LSB_CO2(void);

void send_data_USART(char* data);
void I2C_Scan(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  PERIOD_INIT();
  MODE_CONFIG();
  SENSOR_STATUS_CHECK();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //I2C_Scan();
	 // uint8_t MSB = get_MSB_CO2();
	  //uint8_t LSB = get_LSB_CO2();
	  uint16_t CO2 = get_CO2();

	  char buffer[20];
	  sprintf(buffer, "CO2 reading: %d\n",CO2);
	  send_data_USART(buffer);

	 // char buffer_2[30];
	  //sprintf(buffer_2,"MSB:0x%02X      LSB:0x%02X\n", MSB, LSB);
	  //send_data_USART(buffer_2);

	  LL_mDelay(10000);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_16, 336, LL_RCC_PLLP_DIV_4);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(84000000);
  LL_SetSystemCoreClock(84000000);
  LL_RCC_SetTIMPrescaler(LL_RCC_TIM_PRESCALER_TWICE);
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

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**I2C1 GPIO Configuration
  PB8   ------> I2C1_SCL
  PB9   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C1);
  LL_I2C_DisableGeneralCall(I2C1);
  LL_I2C_EnableClockStretching(I2C1);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C1, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C1, 0);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_ResetOutputPin(LD2_GPIO_Port, LD2_Pin);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_13;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_FALLING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinPull(B1_GPIO_Port, B1_Pin, LL_GPIO_PULL_NO);

  /**/
  LL_GPIO_SetPinMode(B1_GPIO_Port, B1_Pin, LL_GPIO_MODE_INPUT);

  /**/
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
void send_data_USART(char* data){
	while (*data) {

	       while (!LL_USART_IsActiveFlag_TXE(USART2));  // Wait for TX buffer to be empty

	       LL_USART_TransmitData8(USART2, *data++);  // Send the next character

	   }

	   while (!LL_USART_IsActiveFlag_TC(USART2));  // Wait for transmission to complete

}
/* USER CODE BEGIN 4 */
void SENSOR_STATUS_CHECK(){
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));  // Wait for start condition

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_SENSOR_STATUS);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));

	uint8_t status = LL_I2C_ReceiveData8(I2C1);
	LL_I2C_GenerateStopCondition(I2C1);

	char buffer_3[30];
	sprintf(buffer_3, "status:%d\n", status);
	send_data_USART(buffer_3);

	/*for(i=7, i>=0, i--){
		printf()
	}*/

}
void PERIOD_INIT(){
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_MSB_RATE);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x00);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_LSB_RATE);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x0A);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStopCondition(I2C1);
}

void MODE_CONFIG(){
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_CONFIG);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x06);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStopCondition(I2C1);

}

uint16_t get_CO2(){
	uint8_t MSB = 0;
	uint8_t LSB =0;
	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_MSB);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));

	MSB = LL_I2C_ReceiveData8(I2C1);

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS));
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, CO2_LSB);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1);
	while (!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, (CO2_SENSOR_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));

	LSB = LL_I2C_ReceiveData8(I2C1);

	LL_I2C_GenerateStopCondition(I2C1);
	char buffer41[30];
	sprintf(buffer41, "\nMSB:0x%02X      LSB: 0x%02X\n", MSB, LSB);
	send_data_USART(buffer41);


	uint16_t CO2 = (uint16_t)((MSB<<8)|LSB);
	return CO2;

}


void I2C_Scan(void)
{
    char buffer4[50];
    uint8_t address;
    uint8_t devices_found = 0;

    for (address = 1; address < 128; address++)  // I2C addresses range from 0x01 to 0x7F
    {
        LL_I2C_GenerateStartCondition(I2C1);
        while (!LL_I2C_IsActiveFlag_SB(I2C1));  // Wait for start condition

        LL_I2C_TransmitData8(I2C1, (address << 1));  // Transmit the address in write mode
        LL_mDelay(1);

        if (LL_I2C_IsActiveFlag_ADDR(I2C1))  // If the address is acknowledged
        {
            LL_I2C_ClearFlag_ADDR(I2C1);
            sprintf(buffer4, "Device found at 0x%02X\n", address);
            send_data_USART(buffer4);
            devices_found++;
        }

        LL_I2C_GenerateStopCondition(I2C1);  // Stop condition
        LL_mDelay(10);  // Short delay to avoid bus conflicts
    }

    if (devices_found == 0)
    {
        send_data_USART("No I2C devices found.\n");
    }
    else
    {
        sprintf(buffer4, "Total devices found: %d\n", devices_found);
        send_data_USART(buffer4);
    }
}
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
