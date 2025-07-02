
#include "BSP.h"
#include "Core.h"
#include "StartTask.h"
#include "usartio.h"
//#include "usart_new.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
void SystemClock_Config(void);
void BSP_Init(void);
void User_Init();

int main(void)
{
	BSP_Init();
	User_Init();
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		//matc();
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}
void User_Init()
{
	//打印用串口使能
	DEBUGC_UartInit();
	TIM5_IT_Init();

	//遥控器串口使能
	REMOTEC_Init();

	can.All_Init();

	//IMU用
	delay_init();
	DWT_Init(168);

	usart_printf("Program start!\r\n");
	osKernelStart();
}

void BSP_Init(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART6_UART_Init();
	MX_USART3_UART_Init();
	MX_CAN1_Init();
	MX_CAN2_Init();
	MX_USART1_UART_Init();

	MX_I2C3_Init();
	MX_SPI1_Init();

	MX_TIM10_Init();
	MX_TIM5_Init();

	MX_IWDG_Init();
	MX_USB_DEVICE_Init();

	FreeRTOS_Task_Init();
}

/**
  * @brief  The application entry point.
  * @retval int
  */

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
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 6;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}