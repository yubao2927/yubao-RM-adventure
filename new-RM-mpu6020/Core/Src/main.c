/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "soft_i2c.h"
#include "mpu6050.h"
#include "mahony.h"
#include "slave_comm.h"
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

/* USER CODE BEGIN PV */
MPU6050_Data_t mpu_data;  // ����MPU6050���ݽṹ����������жϺ���ѭ��ʹ��
uint8_t rx_byte;          // ���ڽ��յ����ֽ�
uint8_t rx_cmd_buf[5];    // ���ڴ���������ش����󣨹̶�5�ֽڣ�
uint8_t rx_cmd_idx = 0;   // ��������
volatile uint8_t mpu_update_flag = 0;  // ��ʱ���ж�ֻ������λ�����־
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  I2C_Init();
  if (MPU6050_Init() != 0) {
      // ��ʱע�͵���ѭ������������������
      // while (1) {
      //     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      //     HAL_Delay(200);
      // }
  } else {
      MPU6050_Calibrate(200); // ֻ�г�ʼ���ɹ���У׼
  }
  Mahony_Init(2.0f, 0.005f); 
  
  HAL_TIM_Base_Start_IT(&htim2); 
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1); 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (mpu_update_flag == 1) 
    {
        mpu_update_flag = 0; // �����־λ��׼����һ�δ���
        // 1. ����ѭ�������I2C��ȡ�����º�ʱ��
        MPU6050_ReadData(&mpu_data); 
        
        // 2. Mahony��̬���㣬dt=0.001s
        Mahony_Update(mpu_data.gx_f, mpu_data.gy_f, mpu_data.gz_f,
                      mpu_data.ax_f, mpu_data.ay_f, mpu_data.az_f, 0.001f);
                      
        // 3. ת��Ϊŷ����
        Mahony_GetEulerAngle();
        // 4. ͨ��USART1��������������֡��ȷ���õ����������ͣ���Ҫ��DMA��
        Slave_SendFrame();
    }
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
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        mpu_update_flag = 1;  // ֻ����һ���£������˳��жϣ�
    }
}
// �ӻ����ڽ�����ɻص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // ȷ������������������
    {
        rx_cmd_buf[rx_cmd_idx++] = rx_byte; // ���յ����ֽڴ��뻺����

        // �����������ش�����̶�Ϊ5���ֽڣ�����5���ʹ���
        if (rx_cmd_idx >= 5) 
        {
            // �˶�֡ͷ��֡���ͺ�֡β
            if (rx_cmd_buf[0] == 0xAA && 
                rx_cmd_buf[1] == 0x55 && 
                rx_cmd_buf[2] == 0x02 && 
                rx_cmd_buf[4] == 0x0D) 
            {
                // ȷ�����ش����������ط���ǰ֡������
                Slave_SendFrame(); 
            }
            // �����Ƿ��ش�����Ҫ����������׼����һ�ν���
            rx_cmd_idx = 0; 
        }

        // ?? ������Ҫ�����¿������ڽ����жϣ�����ֻ���յ�һ�Σ�
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1); 
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
      // ����ʲô����д�����ֿ���ѭ��
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
