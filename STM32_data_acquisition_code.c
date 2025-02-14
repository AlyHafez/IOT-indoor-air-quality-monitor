/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

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

#define PM_25_ADDRESS 0x33<<1
#define PM_25_SENSOR_STATUS 0X26
#define PM_25_LL 0x04
#define PM_25_LH 0x05
#define PM_25_HL 0x06
#define PM_25_HH 0x07

#define resistor 100000
#define sensitivity 50e-9
#define ADC_CHANNEL LL_ADC_CHANNEL_0


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
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void PERIOD_INIT(void);
void MODE_CONFIG(void);
void SENSOR_STATUS_CHECK(void);
uint16_t get_CO2(void);
void send_data_USART(char* data);
void I2C_Scan(void);
uint16_t get_PM25(void);
float get_ADC_voltage();
int CalculatePPM(float voltage);
uint16_t calculateChecksum(uint16_t *data);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  PERIOD_INIT();
  MODE_CONFIG();
  SENSOR_STATUS_CHECK();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  uint16_t CO2 = get_CO2();




	  uint16_t PM25 = get_PM25();

	  float Voltage = get_ADC_voltage();
	  int CO_value = CalculatePPM(Voltage);
	  uint16_t data[3]={CO2, CO_value,PM25};

	  uint16_t checksum = calculateChecksum(data);


	  char buffer[150];
	  sprintf(buffer, "CO2: %d ppm\n CO:%d ppm\n PM2.5:%d ug/m^3\n Checksum: %d\n",data[0], data[1],data[2],checksum);
	  send_data_USART(buffer);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0-WKUP   ------> ADC1_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetFlagEndOfConversion(ADC1, LL_ADC_REG_FLAG_EOC_UNITARY_CONV);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_SYNC_PCLK_DIV4;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_0);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_0, LL_ADC_SAMPLINGTIME_3CYCLES);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  PB6   ------> I2C1_SCL
  PB7   ------> I2C1_SDA
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_6|LL_GPIO_PIN_7;
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  LL_I2C_InitTypeDef I2C_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**I2C3 GPIO Configuration
  PC9   ------> I2C3_SDA
  PA8   ------> I2C3_SCL
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */

  /** I2C Initialization
  */
  LL_I2C_DisableOwnAddress2(I2C3);
  LL_I2C_DisableGeneralCall(I2C3);
  LL_I2C_EnableClockStretching(I2C3);
  I2C_InitStruct.PeripheralMode = LL_I2C_MODE_I2C;
  I2C_InitStruct.ClockSpeed = 100000;
  I2C_InitStruct.DutyCycle = LL_I2C_DUTYCYCLE_2;
  I2C_InitStruct.OwnAddress1 = 0;
  I2C_InitStruct.TypeAcknowledge = LL_I2C_ACK;
  I2C_InitStruct.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  LL_I2C_Init(I2C3, &I2C_InitStruct);
  LL_I2C_SetOwnAddress2(I2C3, 0);
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/* USER CODE BEGIN 4 */
void send_data_USART(char* data){
	while (*data) {

	       while (!LL_USART_IsActiveFlag_TXE(USART1));  // Wait for TX buffer to be empty

	       LL_USART_TransmitData8(USART1, *data++);  // Send the next character

	   }

	   while (!LL_USART_IsActiveFlag_TC(USART1));  // Wait for transmission to complete

}

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



	uint16_t CO2 = (uint16_t)((MSB<<8)|LSB);
	return CO2;

}

uint16_t get_PM25(){
	uint8_t LL = 0;
	uint8_t LH = 0;
	uint8_t HL = 0;
	uint8_t HH = 0;

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS));//write to pm2.5 address
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_I2C_TransmitData8(I2C3, (PM_25_LL));
	while(!LL_I2C_IsActiveFlag_TXE(I2C3));

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_mDelay(1);

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C3));

	LL = LL_I2C_ReceiveData8(I2C3);

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS));//write to pm2.5 address
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_I2C_TransmitData8(I2C3, (PM_25_LH));
	while(!LL_I2C_IsActiveFlag_TXE(I2C3));

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_mDelay(1);

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C3));

	LH = LL_I2C_ReceiveData8(I2C3);


	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS));//write to pm2.5 address
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_I2C_TransmitData8(I2C3, (PM_25_HL));
	while(!LL_I2C_IsActiveFlag_TXE(I2C3));

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_mDelay(1);

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C3));

	HL = LL_I2C_ReceiveData8(I2C3);

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS));//write to pm2.5 address
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_I2C_TransmitData8(I2C3, (PM_25_HH));
	while(!LL_I2C_IsActiveFlag_TXE(I2C3));

	LL_I2C_GenerateStartCondition(I2C3);
	while (!LL_I2C_IsActiveFlag_SB(I2C3));

	LL_I2C_TransmitData8(I2C3, (PM_25_ADDRESS)|0x01);
	while(!LL_I2C_IsActiveFlag_ADDR(I2C3));
	LL_I2C_ClearFlag_ADDR(I2C3);

	LL_mDelay(1);

	LL_I2C_AcknowledgeNextData(I2C3, LL_I2C_NACK);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C3));

	HH = LL_I2C_ReceiveData8(I2C3);
	LL_I2C_GenerateStopCondition(I2C3);

	uint32_t conversion = (HH<<24)|(HL<<16)|(LH<<8)|(LL);
	uint16_t PM_25= conversion&0xFFFF;

	return PM_25;

}

float get_ADC_voltage(){
	 LL_ADC_REG_StartConversionSWStart(ADC1);
    // Wait until conversion is complete
	 while (!LL_ADC_IsActiveFlag_EOCS(ADC1));
	 LL_ADC_ClearFlag_EOCS(ADC1);
    // Read ADC value (12-bit)
    uint16_t adc_value = LL_ADC_REG_ReadConversionData12(ADC1);

    // Convert to voltage (0 - 3.3V)
    return (adc_value / 4095.0f) * 3.3f;
}

int CalculatePPM(float voltage){
	float current = voltage/resistor;
	return (int)(current/sensitivity);
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

            devices_found++;
        }

        LL_I2C_GenerateStopCondition(I2C1);  // Stop condition
        LL_mDelay(10);  // Short delay to avoid bus conflicts
    }

    if (devices_found == 0)
    {

    }
    else
    {
        sprintf(buffer4, "Total devices found: %d\n", devices_found);

    }
}
uint16_t calculateChecksum(uint16_t *data){

	uint32_t sum = 0;

	for (size_t i=0;i<3;i++){
		sum+=data[i];

		if(sum>=0x10000){
			sum = (sum&0xFFFF) + 1;
		}
	}
	return ~((uint16_t) sum);
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
