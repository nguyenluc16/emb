/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define DS1307_I2C_ADDR     0x68
#define DS1307_REG_SECOND     0x00
#define DS1307_REG_MINUTE     0x01
#define DS1307_REG_HOUR      0x02
#define DS1307_REG_DOW        0x03
#define DS1307_REG_DATE       0x04
#define DS1307_REG_MONTH      0x05
#define DS1307_REG_YEAR       0x06
#define DS1307_REG_CONTROL     0x07
#define DS1307_REG_UTC_HR    0x08
#define DS1307_REG_UTC_MIN    0x09
#define DS1307_REG_CENT        0x10
#define DS1307_TIMEOUT        1000
typedef struct {
    uint8_t    sec;
    uint8_t min;
    uint8_t hour;
    uint8_t dow;
    uint8_t date;
    uint8_t month;
    uint16_t year;
}DS1307_STRUCT;
DS1307_STRUCT    ds1307;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for nhietDoTask */
osThreadId_t nhietDoTaskHandle;
const osThreadAttr_t nhietDoTask_attributes = {
  .name = "nhietDoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for thoiGianTask */
osThreadId_t thoiGianTaskHandle;
const osThreadAttr_t thoiGianTask_attributes = {
  .name = "thoiGianTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for hienThiTask */
osThreadId_t hienThiTaskHandle;
const osThreadAttr_t hienThiTask_attributes = {
  .name = "hienThiTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void nhietDoFunction(void *argument);
void thoiGianFunction(void *argument);
void hienThiFunction(void *argument);

/* USER CODE BEGIN PFP */

uint8_t DS1307_DecodeBCD(uint8_t bin);
uint8_t DS1307_EncodeBCD(uint8_t dec);
void DS1307_SetClockHalt(uint8_t halt);
void DS1307_SetRegByte(uint8_t regAddr, uint8_t val);
void DS1307_SetTimeZone(int8_t hr, uint8_t min);
uint8_t DS1307_GetClockHalt(void);
uint8_t DS1307_GetRegByte(uint8_t regAddr);
void DS1307_config();
void DS1307_gettime();
void DS1307_settime(uint8_t sec,uint8_t min,uint8_t hour_24mode,uint8_t dayOfWeek,uint8_t date,uint8_t month, uint16_t year);

typedef struct{
	uint8_t IO_id;
	uint8_t IO_data[8];
	uint16_t IO_data_year;
}myQueueData_t;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CONVERT_NUMBER_FOUR(uint16_t tick, uint8_t* k)
{
	*(k) = (uint8_t)((tick%10000)/1000) | 0x30;
	*(k+1) = (uint8_t)((tick%1000)/100) | 0x30;
	*(k+2) = (uint8_t)((tick%100)/10) | 0x30;
	*(k+3) = (uint8_t)(tick%10) | 0x30;
}

void CONVERT_NUMBER_TWO(uint8_t tick, uint8_t* k)
{
	*k = (uint8_t)((tick%100)/10) | 0x30;
	*(k+1) = (uint8_t)(tick%10) | 0x30;
}

//bien luu moc thoi gian
uint16_t k1, k2;

//bien cau truc nhiet do do am
DHT_Name DHT1;
uint8_t nhietDo[11] = " Temp: 12\n";
uint8_t doAm[10] = "Humi: 34\n";

// bien thoi gian
//DS1307_STRUCT ds1307;
uint8_t giay[10] = "giay: 56\n";
uint8_t phut[10] = "phut: 56\n";
uint8_t gio[10] = "gio : 56\n";
uint8_t thu[10] = "thu : 56\n";
uint8_t ngay[10] = "ngay: 56\n";
uint8_t thang[10] = "thg : 56\n";
uint8_t nam[12] = "nam : 5678\n";

//task1-temperature humility
void TASK1()
{
	DHT_ReadTempHum(&DHT1);
	// nhietDo = (uint8_t)DHT1.Temp;
	// doAm = (uint8_t)DHT1.Humi;
	//CONVERT_NUMBER_TWO((uint8_t)DHT1.Temp, &nhietDo[7]);
	//CONVERT_NUMBER_TWO((uint8_t)DHT1.Humi, &doAm[6]);
	
}
// TASK 2 get times
void TASK2()
{
	DS1307_gettime();
	/*
	CONVERT_NUMBER_TWO(ds1307.sec, &giay[6]);
	CONVERT_NUMBER_TWO(ds1307.min, &phut[6]);
	CONVERT_NUMBER_TWO(ds1307.hour, &gio[6]);
	CONVERT_NUMBER_TWO(ds1307.dow, &thu[6]);
	CONVERT_NUMBER_TWO(ds1307.date, &ngay[6]);
	CONVERT_NUMBER_TWO(ds1307.month, &thang[6]);
	CONVERT_NUMBER_FOUR(ds1307.year, &nam[6]);
	*/
}

// task3-display on LCD
void TASK3()
{
	// lcd_init();
	lcd_clear_display();
	lcd_goto_XY(1,0);
	lcd_send_string((char*)nhietDo);
	lcd_goto_XY(2,0);
	lcd_send_string((char*)doAm);
}

//task 4 transmiter uart
void TASK4()
{
	HAL_UART_Transmit(&huart1, giay, sizeof(giay), 100);
	HAL_UART_Transmit(&huart1, phut, sizeof(phut), 100);
	HAL_UART_Transmit(&huart1, gio, sizeof(gio), 100);
	HAL_UART_Transmit(&huart1, thu, sizeof(thu), 100);
	HAL_UART_Transmit(&huart1, ngay, sizeof(ngay), 100);
	HAL_UART_Transmit(&huart1, thang, sizeof(thang), 100);
	HAL_UART_Transmit(&huart1, nam, sizeof(nam), 100);
	HAL_UART_Transmit(&huart1, nhietDo, sizeof(nhietDo), 100);
	HAL_UART_Transmit(&huart1, doAm, sizeof(doAm), 100);
}



// void (*TASK_POINT[3])(void)={TASK1, TASK2, TASK3};
// uint8_t i = 0;
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
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	lcd_init();
	DHT_Init(&DHT1, DHT11, &htim3, DHT11_GPIO_Port, DHT11_Pin);
	DS1307_config();
  DS1307_settime(30, 59, 6, 5, 24, 2, 2022);
	//TASK2();
	//TASK1();
	//1 = uwTick;
	 //TASK1();
	 //k1 = uwTick;
	 //TASK3();
	//TASK2();
	//TASK4();
	
	//k2 = uwTick;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (16, sizeof(myQueueData_t), &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of nhietDoTask */
  nhietDoTaskHandle = osThreadNew(nhietDoFunction, NULL, &nhietDoTask_attributes);

  /* creation of thoiGianTask */
  thoiGianTaskHandle = osThreadNew(thoiGianFunction, NULL, &thoiGianTask_attributes);

  /* creation of hienThiTask */
  hienThiTaskHandle = osThreadNew(hienThiFunction, NULL, &hienThiTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		HAL_Delay(1000);
		
		/*
		if(uwTick%10000==100)
		{
			for(i=0; i<3; i++)
			{
				TASK_POINT[i]();
			}
		}
		*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void DS1307_gettime(){
    uint16_t cen;
    ds1307.sec=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_SECOND) & 0x7f);
    ds1307.min=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MINUTE));
    ds1307.hour=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_HOUR) & 0x3f);
    ds1307.dow=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_DOW));
    ds1307.date=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_DATE));
    ds1307.month=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_MONTH));
    cen = DS1307_GetRegByte(DS1307_REG_CENT) * 100;
    ds1307.year=DS1307_DecodeBCD(DS1307_GetRegByte(DS1307_REG_YEAR)) + cen;
}
void DS1307_SetRegByte(uint8_t regAddr, uint8_t val) {
    uint8_t bytes[2] = { regAddr, val };
    HAL_I2C_Master_Transmit(&hi2c1, DS1307_I2C_ADDR << 1, bytes, 2, DS1307_TIMEOUT);
}
uint8_t DS1307_GetClockHalt(void) {
    return (DS1307_GetRegByte(DS1307_REG_SECOND) & 0x80) >> 7;
}
void DS1307_settime(uint8_t sec,uint8_t min,uint8_t hour_24mode,uint8_t dayOfWeek,uint8_t date,uint8_t month, uint16_t year){
    DS1307_SetRegByte(DS1307_REG_SECOND, DS1307_EncodeBCD(sec | DS1307_GetClockHalt()));
    DS1307_SetRegByte(DS1307_REG_MINUTE, DS1307_EncodeBCD(min));
    DS1307_SetRegByte(DS1307_REG_HOUR, DS1307_EncodeBCD(hour_24mode & 0x3f));//hour_24mode Hour in 24h format, 0 to 23.
    DS1307_SetRegByte(DS1307_REG_DOW, DS1307_EncodeBCD(dayOfWeek));//dayOfWeek Days since last Sunday, 0 to 6.
    DS1307_SetRegByte(DS1307_REG_DATE, DS1307_EncodeBCD(date));//date Day of month, 1 to 31.
    DS1307_SetRegByte(DS1307_REG_MONTH, DS1307_EncodeBCD(month));//month Month, 1 to 12.
    DS1307_SetRegByte(DS1307_REG_CENT, year / 100);
    DS1307_SetRegByte(DS1307_REG_YEAR, DS1307_EncodeBCD(year % 100));//2000 to 2099.
}
uint8_t DS1307_GetRegByte(uint8_t regAddr) {
    uint8_t val;
    HAL_I2C_Master_Transmit(&hi2c2, DS1307_I2C_ADDR << 1, &regAddr, 1, DS1307_TIMEOUT);
    HAL_I2C_Master_Receive(&hi2c2, DS1307_I2C_ADDR << 1, &val, 1, DS1307_TIMEOUT);
    return val;
}
void DS1307_SetClockHalt(uint8_t halt) {
    uint8_t ch = (halt ? 1 << 7 : 0);
    DS1307_SetRegByte(DS1307_REG_SECOND, ch | (DS1307_GetRegByte(DS1307_REG_SECOND) & 0x7f));
}
/**
 * @brief Sets UTC offset.
 * @note  UTC offset is not updated automatically.
 * @param hr UTC hour offset, -12 to 12.
 * @param min UTC minute offset, 0 to 59.
 */
void DS1307_SetTimeZone(int8_t hr, uint8_t min) {
    DS1307_SetRegByte(DS1307_REG_UTC_HR, hr);
    DS1307_SetRegByte(DS1307_REG_UTC_MIN, min);
}
void DS1307_config(){
    DS1307_SetClockHalt(0);
    DS1307_SetTimeZone(+8, 00);
}
uint8_t DS1307_DecodeBCD(uint8_t bin) {
    return (((bin & 0xf0) >> 4) * 10) + (bin & 0x0f);
}
uint8_t DS1307_EncodeBCD(uint8_t dec) {
    return (dec % 10 + ((dec / 10) << 4));
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_nhietDoFunction */
/**
  * @brief  Function implementing the nhietDoTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_nhietDoFunction */
void nhietDoFunction(void *argument)
{
  /* USER CODE BEGIN 5 */
	myQueueData_t msg;
	msg.IO_id = 1;
	
  /* Infinite loop */
  for(;;)
  {
		//taskENTER_CRITICAL();
		DHT_ReadTempHum(&DHT1);
		//taskEXIT_CRITICAL();
		msg.IO_data[0] = DHT1.Temp;
		msg.IO_data[1] = DHT1.Humi;
		osMessageQueuePut(myQueue01Handle, &msg, 0, 0);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_thoiGianFunction */
/**
* @brief Function implementing the thoiGianTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_thoiGianFunction */
void thoiGianFunction(void *argument)
{
  /* USER CODE BEGIN thoiGianFunction */
	myQueueData_t msg;
	msg.IO_id = 2;
	
  /* Infinite loop */
  for(;;)
  {
		osMutexAcquire(myMutex01Handle, osWaitForever);
		DS1307_gettime();
		osMutexRelease(myMutex01Handle);

		msg.IO_data[2] = ds1307.sec;
		msg.IO_data[3] = ds1307.min;
		msg.IO_data[4] = ds1307.hour;
		msg.IO_data[5] = ds1307.dow;
		msg.IO_data[6] = ds1307.date;
		msg.IO_data[7] = ds1307.month;
		msg.IO_data_year = ds1307.year;

		osMessageQueuePut(myQueue01Handle, &msg, 0, 0);

    osDelay(1000);
  }
  /* USER CODE END thoiGianFunction */
}

/* USER CODE BEGIN Header_hienThiFunction */
/**
* @brief Function implementing the hienThiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hienThiFunction */
void hienThiFunction(void *argument)
{
  /* USER CODE BEGIN hienThiFunction */
	myQueueData_t msg;
  /* Infinite loop */
  for(;;)
  {
		osMessageQueueGet(myQueue01Handle, &msg, 0, osWaitForever);
		if(msg.IO_id==1)
		{
			
			CONVERT_NUMBER_TWO(msg.IO_data[0], &nhietDo[7]);
			CONVERT_NUMBER_TWO(msg.IO_data[1], &doAm[6]);
			osMutexAcquire(myMutex01Handle, osWaitForever);
			lcd_clear_display();
			lcd_goto_XY(1,0);
			lcd_send_string((char*)nhietDo);
			lcd_goto_XY(2,0);
			lcd_send_string((char*)doAm);
			osMutexRelease(myMutex01Handle);

		}
		else
		{
			CONVERT_NUMBER_TWO(msg.IO_data[2], &giay[6]);
			CONVERT_NUMBER_TWO(msg.IO_data[3], &phut[6]);
			CONVERT_NUMBER_TWO(msg.IO_data[4], &gio[6]);
			CONVERT_NUMBER_TWO(msg.IO_data[5], &thu[6]);
			CONVERT_NUMBER_TWO(msg.IO_data[6], &ngay[6]);
			CONVERT_NUMBER_TWO(msg.IO_data[7], &thang[6]);
			CONVERT_NUMBER_FOUR(msg.IO_data_year, &nam[6]);
			
			HAL_UART_Transmit(&huart1, giay, sizeof(giay), 100);
			HAL_UART_Transmit(&huart1, phut, sizeof(phut), 100);
			HAL_UART_Transmit(&huart1, gio, sizeof(gio), 100);
			HAL_UART_Transmit(&huart1, thu, sizeof(thu), 100);
			HAL_UART_Transmit(&huart1, ngay, sizeof(ngay), 100);
			HAL_UART_Transmit(&huart1, thang, sizeof(thang), 100);
			HAL_UART_Transmit(&huart1, nam, sizeof(nam), 100);
			HAL_UART_Transmit(&huart1, nhietDo, sizeof(nhietDo), 100);
			HAL_UART_Transmit(&huart1, doAm, sizeof(doAm), 100);
		}
    //osDelay(1);
  }
  /* USER CODE END hienThiFunction */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
