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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SPI_TO_USART_QUEUE_LENGTH	10
#define USART_TO_SPI_QUEUE_LENGTH	10
#define BINARY_SEMAPHORE_LENGTH	1
#define SPI_POLLING_DELAY  100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId defaultTaskHandle;
osThreadId spi_taskHandle;
osThreadId usart_taskHandle;
osMessageQId usart_to_spiHandle;
osMessageQId spi_to_usartHandle;
osSemaphoreId spi_txrx_cplt_Handle;
osStaticSemaphoreDef_t spi_txrx_cplt_ControlBlock;
osSemaphoreId usart_tx_eventHandle;
osStaticSemaphoreDef_t usart_txc_eventControlBlock;
osSemaphoreId usart_rx_eventHandle;
osStaticSemaphoreDef_t usart_rx_eventControlBlock;
osSemaphoreId spi_rx_cpltHandle;
osStaticSemaphoreDef_t spi_rx_cpltControlBlock;
/* USER CODE BEGIN PV */
QueueSetHandle_t xQueueSet;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void spi_handle_function(void const * argument);
void usart_handle_function(void const * argument);

/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUNDLE_SIZE 64u
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of spi_txrx_cplt_ */
  osSemaphoreStaticDef(spi_txrx_cplt_, &spi_txrx_cplt_ControlBlock);
  spi_txrx_cplt_Handle = osSemaphoreCreate(osSemaphore(spi_txrx_cplt_), 1);

  /* definition and creation of usart_tx_event */
  osSemaphoreStaticDef(usart_tx_event, &usart_txc_eventControlBlock);
  usart_tx_eventHandle = osSemaphoreCreate(osSemaphore(usart_tx_event), 1);

  /* definition and creation of usart_rx_event */
  osSemaphoreStaticDef(usart_rx_event, &usart_rx_eventControlBlock);
  usart_rx_eventHandle = osSemaphoreCreate(osSemaphore(usart_rx_event), 1);

  /* definition and creation of spi_rx_cplt */
  osSemaphoreStaticDef(spi_rx_cplt, &spi_rx_cpltControlBlock);
  spi_rx_cpltHandle = osSemaphoreCreate(osSemaphore(spi_rx_cplt), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of usart_to_spi */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* Create the queue(s) */
  /* definition and creation of usart_to_spi */
  osMessageQDef(usart_to_spi, USART_TO_SPI_QUEUE_LENGTH, uint8_t*);
  usart_to_spiHandle = osMessageCreate(osMessageQ(usart_to_spi), NULL);

  /* definition and creation of spi_to_usart */
  osMessageQDef(spi_to_usart,SPI_TO_USART_QUEUE_LENGTH , uint8_t*);
  spi_to_usartHandle = osMessageCreate(osMessageQ(spi_to_usart), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of spi_task */
  osThreadDef(spi_task, spi_handle_function, osPriorityHigh, 0, 512);
  spi_taskHandle = osThreadCreate(osThread(spi_task), NULL);

  /* definition and creation of usart_task */
  osThreadDef(usart_task, usart_handle_function, osPriorityNormal, 0, 512);
  usart_taskHandle = osThreadCreate(osThread(usart_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
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
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  // определить логгирование
  return ch;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi == &hspi1)
  {
	  osSemaphoreRelease(spi_txrx_cplt_Handle);
  }
}


void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi == &hspi1)
	{
		osSemaphoreRelease(spi_txrx_cplt_Handle);
		puts("Error spi transmission");
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart1)
	{
		osSemaphoreRelease(usart_rx_eventHandle);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		osSemaphoreRelease(usart_tx_eventHandle);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		osSemaphoreRelease(usart_rx_eventHandle);
		puts("Error usart transmission");
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_spi_handle_function */
/**
* @brief Function implementing the spi_handle_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_spi_handle_function */
void spi_handle_function(void const * argument)
{
  /* USER CODE BEGIN spi_handle_function */
  uint8_t * byte;								// указатель на текущий элемент приёмного буфера
  uint8_t  *rx_buffer = malloc(BUNDLE_SIZE);	// указатель на временно аллоцирумую память приёмного буфера
  uint8_t  rx_buffer_itt = 0;					// итератор приёмного буфера

  uint8_t * tx_buffer;							// указатель на буфер передачи

  bool transmission = false;					// флаг о необходимости помещения временного буфера в очередь

  /*!
   *  Составление сета из блокировки на очереди и семафоре/мютексе конца передачи по SPI
   */
  QueueSetHandle_t xQueueSet;
  xQueueSet = xQueueCreateSet( USART_TO_SPI_QUEUE_LENGTH + BINARY_SEMAPHORE_LENGTH);
  xQueueAddToSet(usart_to_spiHandle, xQueueSet);
  xQueueAddToSet(spi_txrx_cplt_Handle, xQueueSet);
  QueueSetMemberHandle_t xActivatedMember;
  for(;;)
  {
	  xActivatedMember = xQueueSelectFromSet( xQueueSet, SPI_POLLING_DELAY);

	  if ( xActivatedMember == usart_to_spiHandle )
	  {
		  /*!
		   *  проверка типа события. Либо в очереди появился новый элемент и мы должны его передать в SPI Slave.
		   *  Либо сработал семафор конца передачи по DMA или ISR
		   */
		  osEvent event = osMessageGet(usart_to_spiHandle, 10);
		  if (osEventMessage == event.status)
		  {
			  tx_buffer = event.value.p;
			  HAL_SPI_TransmitReceive_DMA(&hspi1, &tx_buffer[1], &tx_buffer[1], tx_buffer[0]);
		  }

	  } else if (xActivatedMember == spi_txrx_cplt_Handle){
		  /*!
		   *  Если  tx_buffer не NULL, значит семафор сработал на конец передачи по DMA , а не по ISR.
		   *  Проверяем пришли ли какие либо данные за время передачи.
		   *  При необходимости переносим данные в  приёмный буфер вместе с символом конца строки.
		   *  Если буфер закончился без символа конца строки, то просто добираем фрейм через ISR,
		   *  либо добавляем в очередь при переполнении буфера.
		   *  Подразумеваем , что за один обмен Spi Slave передаст не более одного фрейма.
		   */
		  if (tx_buffer)
		  {
			  uint16_t itt = 0;
			  while(++itt < tx_buffer[0])
			  {
				  if(tx_buffer[itt])
				  {
					  rx_buffer[++rx_buffer_itt] = tx_buffer[itt];
				  }
				  else
				  {
					  rx_buffer[++rx_buffer_itt] = 0;
					  transmission = true;
				  }
			  }
			  if(BUNDLE_SIZE < itt) transmission = true;
			  free(tx_buffer);
			  tx_buffer = NULL;
		  }
		  else
		  {
			  if (!byte)
			  {
				  if (rx_buffer_itt)
				  {
					 transmission = true;
					 rx_buffer[0]=rx_buffer_itt - 1;
					 rx_buffer_itt = 0;
				  }
			  }
			  else
			  {
				  if(BUNDLE_SIZE < rx_buffer_itt) transmission = true;
			  }

			  /*!
			   *  взводим новый сеанс приемопередачи по прерыванию если фрейм ещё не получен.
			   */
			  if(rx_buffer_itt)
			  {
				  byte = &rx_buffer[++rx_buffer_itt];
				  HAL_SPI_TransmitReceive_IT(&hspi1,byte, byte, 1);
			  }
		  }

		  /*!
		   *  В случае переполнения аллоцированного буфера или события конца строки - добавляем буфер в очередь
		   */
		  if (transmission)
		  {
			  if (osOK != osMessagePut(spi_to_usartHandle, *(uint32_t*)rx_buffer, 100 / portTICK_PERIOD_MS))
			  {
				  puts("Error  spi_to_usart QUEUE inserting");
			  }

			  rx_buffer = malloc(BUNDLE_SIZE); // аллоцируем новый приемный буфер
		  }

      } else
      {
		  /*!
		   *  взводим новый сеанс приемопередачи по прерыванию по таймауту,
		   *  который возникнет если не будет новой посылки по UART
		   *  Данный метод нужен изза того, что устройство SPI slave можете передавать асинхронно.
		   *  Время реакции на событие SPI Slave равно времени таймаута + накладные издержки времени.
		   */
    	  byte = &rx_buffer[++rx_buffer_itt];
    	  HAL_SPI_TransmitReceive_IT(&hspi1,byte, byte, 1);
      }
  }
  /* USER CODE END spi_handle_function */
}

/* USER CODE BEGIN Header_usart_handle_function */
/**
* @brief Function implementing the usart_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usart_handle_function */
void usart_handle_function(void const * argument)
{
  /* USER CODE BEGIN usart_handle_function */
  uint8_t * tx_buffer;						   // указатель на буфер передачи

  uint8_t  *rx_buffer = malloc(BUNDLE_SIZE);   // указатель на временно аллоцирумую память приёмного буфера
  uint8_t  buffer_itt = 0;                     // итератор приёмного буфера

  uint8_t byte;								   // буферная переменная для приёма по ISR

  /*!
   *  Составление сета из блокировки на очереди и семафорах /мютексах конца передачи и приёма
   */
  QueueSetHandle_t xQueueSet;
  xQueueSet = xQueueCreateSet( SPI_TO_USART_QUEUE_LENGTH + BINARY_SEMAPHORE_LENGTH + BINARY_SEMAPHORE_LENGTH );
  xQueueAddToSet(spi_to_usartHandle, xQueueSet);
  xQueueAddToSet(usart_rx_eventHandle, xQueueSet);
  xQueueAddToSet(usart_tx_eventHandle, xQueueSet);
  QueueSetMemberHandle_t xActivatedMember;
  bool transmission = false;

  /*!
   *  взводим прерывание приёма по ISR для одной переменной т.к. размер фрейма динамический.
   */
  HAL_UART_Receive_IT(&huart1, &byte, 1);
  /* Infinite loop */
  for(;;)
  {
	  xActivatedMember = xQueueSelectFromSet( xQueueSet, 100 / portTICK_PERIOD_MS);
	  if (xActivatedMember == usart_tx_eventHandle ) {

	  		  /*!
	  		   *  по концу передачи разблокируемся и освобождаем буфер.
	  		   *  важно проверять это событие до события очереди чтобы избежать OVERRUN и утечки памяти.
	  		   */
	  		  free(tx_buffer);
	  		  tx_buffer = NULL;
	  } else if ( xActivatedMember == spi_to_usartHandle ) {
		  /*!
		   *   разблокированы по событию очереди.
		   *   отправяем фрейм в DMA.
		   *   в первом байте буфера хранится размер данных буфера или всего фрейма ,
		   *   если он меньше размера буфера
		   */
		  osEvent event = osMessageGet(spi_to_usartHandle, 10);
		  if (osEventMessage == event.status)
		  {
			  tx_buffer = event.value.p;
			  HAL_UART_Transmit_DMA(&huart1,&tx_buffer[1], tx_buffer[0]);
		  }

	  } else if (xActivatedMember == usart_rx_eventHandle ) {
		  /*!
		   *  событие конца приёма в буферную переменную

		   */
		if (byte)
		{
			/*!
			 *  Если байт не нулевой, то происходит заполнение приёмного буфера
			 *  В случае переполнения иницируется добавление буфера в очередь
			 */
			rx_buffer[++buffer_itt]= byte;
			if(BUNDLE_SIZE < buffer_itt)
			{
				rx_buffer[0]=buffer_itt - 1;
				transmission = true;
			}
		}
		else
		{
			/*!
			 *  Прихода конца строки ,
			 *  запись в буфер и инициация добавления в очередь
			 *
			*/
			rx_buffer[++buffer_itt] = byte;
			rx_buffer[0]=buffer_itt - 1;
			transmission = true;
		}

		if (transmission)
		{
			if (osOK != osMessagePut(usart_to_spiHandle, *(uint32_t*)rx_buffer, 100 / portTICK_PERIOD_MS))
			{
				puts("Error  usart to spi  QUEUE inserting");
			}
			/*!
			 *  аллоцируем новый буфер.
			*/
			rx_buffer = malloc(BUNDLE_SIZE);
			buffer_itt = 0;
			transmission = false;
		}

		HAL_UART_Receive_IT(&huart1, &byte, 1); // взводим прерывание по приёму
	  };
  }
  /* USER CODE END usart_handle_function */
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
