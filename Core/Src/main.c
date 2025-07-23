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
//#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "string.h"
#include "stdio.h"
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);


/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/***QUEUEHandler */
xQueueHandle St_Queue_Handler;

//Task Handler
xTaskHandle Sender1_Task_Handler;
xTaskHandle Sender2_Task_Handler;
xTaskHandle Receiver_Task_Handler;


//Task functions
void Sender1_Task(void *argument);
void Sender2_Task(void *argument);
void Receiver_Task(void *argument);
//Structure Definition
typedef struct{
	char *str;
	int counter;
	uint16_t large_value;
} my_struct;

int indx1=0;
int indx2=0;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  //se crea la Queue
  St_Queue_Handler = xQueueCreate(2,sizeof(my_struct)); /*puede guardar dos elementos del tamaño
  de la estructura */
  char *prueba= "Hola mundo\n\n";
  	  HAL_UART_Transmit(&huart1,(uint8_t *)prueba,strlen(prueba),HAL_MAX_DELAY);

  if(St_Queue_Handler==0){
	  char *str= "Unable to create Structure Queue\n\n";
	  HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
  }else{
	  char *str= "Structure created\n\n";
	  HAL_UART_Transmit(&huart1,(uint8_t *)str,strlen(str),HAL_MAX_DELAY);
  }


  /**Se crea la tarea***/
  //función de la tarea, nombre de la tarea,stack depth, parámetro, prioridad y handler
  xTaskCreate(Sender1_Task,"Sender1",128,NULL,2,&Sender1_Task_Handler);
  xTaskCreate(Sender2_Task,"Sender2",128,NULL,2,&Sender2_Task_Handler);
  xTaskCreate(Receiver_Task,"Receiver",128,NULL,1,&Receiver_Task_Handler);

  //scheduler
  vTaskStartScheduler();

  /* USER CODE END 2 */

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
  huart1.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Sender1_Task(void *argument){
	my_struct *ptrtostruct;
	uint32_t TickDelay=pdMS_TO_TICKS(2000);
	while(1){
		char *str= "Entered SENDER1_Task\n";
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);

		//Allocate memory to the pointer
		ptrtostruct = pvPortMalloc(sizeof(my_struct));
//
		//cargar los datos a la estructura
		ptrtostruct->counter=1+indx1;
		ptrtostruct->large_value=1000+indx1*100;
		ptrtostruct->str="Hello from sender1";

		//enviar a la queue
		if(xQueueSend(St_Queue_Handler,&ptrtostruct,portMAX_DELAY)==pdPASS){
			char *str2= "Enviado a la queue con éxito, terminando Sender1_Task";
			HAL_UART_Transmit(&huart1,(uint8_t*)str2,strlen(str2),HAL_MAX_DELAY);
		}
		indx1=indx1+1;
		vTaskDelay(TickDelay);
	}
}



void Sender2_Task(void *argument){
	my_struct *ptrtostruct;
	uint32_t TickDelay=pdMS_TO_TICKS(2000);
	while(1){
		char *str= "Entered SENDER2_Task\n";
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);

		//Allocate memory to the pointer
		ptrtostruct = pvPortMalloc(sizeof(my_struct));

		//cargar los datos a la estructura
		ptrtostruct->counter=1+indx2;
		ptrtostruct->large_value=2000+indx2*200;
		ptrtostruct->str="Hello from sender2";

		//enviar a la queue
		if(xQueueSend(St_Queue_Handler,&ptrtostruct,portMAX_DELAY)==pdPASS){
			char *str2= "Enviado a la queue con éxito, terminando Sender2_Task";
			HAL_UART_Transmit(&huart1,(uint8_t*)str2,strlen(str2),HAL_MAX_DELAY);
		}
		indx2=indx2+1;
		vTaskDelay(TickDelay);
	}
}
void Receiver_Task(void *argument){
	my_struct *Rptrtostruct;
	uint32_t TickDelay=pdMS_TO_TICKS(3000);
	char *ptr;

	while(1){
		char *str= "Entered Receiver_Task\n";
		HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);

		//recibir datos de la queue
		if(xQueueReceive(St_Queue_Handler,&Rptrtostruct,portMAX_DELAY)==pdPASS){
			ptr=pvPortMalloc(100*sizeof(char));
			sprintf(ptr,"Received from QUEUE: \n Counter %d\n Large value= %u\n String %s \n\n\n",Rptrtostruct->counter,Rptrtostruct->large_value,Rptrtostruct->str);
			HAL_UART_Transmit(&huart1,(uint8_t*)ptr,strlen(ptr),HAL_MAX_DELAY);
			vPortFree(ptr); //se libera la memoria del pointer
		}
		//se debe liberar la memoria de los senders
		vPortFree(Rptrtostruct);
		vTaskDelay(TickDelay);
	}
}



/* USER CODE END 4 */


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
