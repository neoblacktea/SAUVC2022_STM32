/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "rosserial.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Propulsion_Sys/propulsion_sys.h"
#include "Datatype/dynamics.h"
#include "robot_arm.h"
#include "Sensor/mpu9250.h"
#include <stdio.h>
#include "dvl_reader.h"
#include "read_data.h"
#include "controller.h"
#include "Sensor/bar02.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Uart Communication Class
//Read_data R;
// Dvl_reader D;

//data receive from Rpi
//uint8_t zhc = 0;
//uint8_t arr_test[29];
//float desired_depth = 0.5;  //desired depth
//float yaw_sonar = 0;  //yaw angle get from sonar
//float val2 = 0;
//geometry::Vector ex = {0, 2, 0};
//geometry::Vector ev = {0};


// rosserial_parameters
float desired_depth = 0;  //desired depth
float yaw_sonar = 0;  //yaw angle get from sonar
extern geometry::Vector ex = {0, 0, 0}; // position error
extern geometry::Vector ev = {0, 0, 0};       // velocity error
double depth = 0;

Dynamics state;
// robot arm
//int arm_angle[3] = {0, 0, 0};  //-90~90

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  //debug
  //char uart_buf[100];
  //int uart_buf_len;
  
  //sensor
  Mpu9250 imu;
  Bar02 depth_sensor;

  //Dynamics state = {0};
  Kinematics control_input = {0};  //force: x, y, z; moment: x, y, z
  // Kinematics control_input = {{0, 1, 1}, {0, 0, 0}};

  Controller controller({0.2, 0.2, 0.2}, {0.2, 0.2, 0.2}, {0.2, 0.2, 0.2}, {0.2, 0.2, 0}, 0); 
  Propulsion_Sys propulsion_sys;

  //Robot Arm
  //Robot_Arm arm;

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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  //Uart Interrupt
  //HAL_UART_Receive_IT(&huart5, &zhc, 1);
  // HAL_UART_Receive_IT(&huart5, arr_test, 28);
  //HAL_UART_Receive_IT(&huart4, &D.receieve_char, 1);
  //R.receieve();

  // rosserial communication
  rosserial_init(&ex, &state, &yaw_sonar);
  rosserial_subscribe();
  

  //Sensor
  imu.set(&hspi2, GPIOB, GPIO_PIN_12);
  if (!depth_sensor.set(&hi2c1))
    return -1;
  
  //Controller
  imu.update(state);
  controller.set(state.orientation);
  //Output
  propulsion_sys.set_timer(&htim2, &htim8);

  //arm.set(&htim4, arm_angle);
  


  //Wait for motor to setup
  HAL_Delay(3000);

  /*
  // check motor direction
  propulsion_sys.motor[0].output(0.4);
  propulsion_sys.motor[1].output(-0.4);
  propulsion_sys.motor[2].output(-0.4);
  propulsion_sys.motor[3].output(0.4);
  propulsion_sys.motor[4].output(0.4);
  propulsion_sys.motor[5].output(-0.4);
  propulsion_sys.motor[6].output(-0.4);
  propulsion_sys.motor[7].output(0.4);
  HAL_Delay(15000);
  
  propulsion_sys.motor[0].output(0);
  propulsion_sys.motor[1].output(0);
  propulsion_sys.motor[2].output(0);
  propulsion_sys.motor[3].output(0);
  propulsion_sys.motor[4].output(0);
  propulsion_sys.motor[5].output(0);
  propulsion_sys.motor[6].output(0);
  propulsion_sys.motor[7].output(0);
  */

  //debug
  // uart_buf_len = sprintf(uart_buf, "ready\r\n");
  // HAL_UART_Transmit(&huart5, (uint8_t*) uart_buf, uart_buf_len, 1000);
  // while(zhc!='\n');

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /**/
    
    //IMU
    //imu.update(state);
    //Depth Sensor
    depth_sensor.read_value();
    depth = depth_sensor.depth();
    //ex.z = desired_depth - depth_sensor.depth();
    ex.z = 0;
    //Controller
    controller.update(state, ex, ev, yaw_sonar, control_input);

    //Allocate and Output
    propulsion_sys.allocate(control_input);  //T200 Motor Output
    
    //Motor take turns test*-------------------------------------------
    /*
    propulsion_sys.motor[0].output(-0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[0].output(0);
    propulsion_sys.motor[1].output(0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[1].output(0);
    propulsion_sys.motor[2].output(0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[2].output(0);
    propulsion_sys.motor[3].output(-0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[3].output(0);
    propulsion_sys.motor[4].output(-0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[4].output(0);
    propulsion_sys.motor[5].output(0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[5].output(0);
    propulsion_sys.motor[6].output(0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[6].output(0);
    propulsion_sys.motor[7].output(-0.2);
    HAL_Delay(2000);
    propulsion_sys.motor[7].output(0);
    */
    //-----------------------------------------------------------------


    //Robot arm
    // arm.move(arm_angle);  //Robot Arm Output
    // HAL_Delay(1500);
    // arm_angle[2] = 10;
    // arm.move(arm_angle);
    // HAL_Delay(1500);
    // arm_angle[2] = -10;
    // arm.move(arm_angle);
    // HAL_Delay(1500);

    rosserial_publish(state.orientation.w, state.orientation.x, state.orientation.y, state.orientation.z);
    
    /*
    // receieve data from rpi
    // 45 is total_byte and 44 is size of data
    if( HAL_UART_Receive(&huart5, arr_test, 45,1000) == HAL_OK)
    {
      uint8_t i = 0;
      if(arr_test[0] != 'n')
      {
        while((arr_test[i] != '\n'))
        {
          R.receieved_data[44-i] = arr_test[i];
          i++;
          
          if(arr_test[i] == '\n')
          {
            for(uint8_t j = 0; j<(45-i); j++)
            {
              R.receieved_data[j] = arr_test[i+j];
            }
            break;
          }
        }
      }
      else
        for(i=0;i<45;i++)
          R.receieved_data[i] = arr_test[i];

      R.assign_num();
      yaw_sonar = R.get_yaw();
      ex = R.get_geometry_vector();
      ev.x = R.get_vel0();
      ev.y = R.get_vel1();
      ev.z = R.get_vel2();
      arm_angle[0] = R.get_joint0();
      arm_angle[1] = R.get_joint1();
      arm_angle[2] = R.get_joint2();
      desired_depth = R.get_depth();
      
    }
    */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   //if(huart->Instance == UART5)
//   R.receieve();
//   if(R.access_ok() == true)
//   {
//     yaw_sonar = R.get_yaw();
//     ex = R.get_geometry_vector();
//     ev.x = R.get_vel0();
//     ev.y = R.get_vel1();
//     ev.z = R.get_vel2();
//     // arm_angle[0] = R.get_joint0();
//     // arm_angle[1] = R.get_joint1();
//     // arm_angle[2] = R.get_joint2();
//     // desired_depth = R.get_depth();
//     R.access_init();
//   }
//   // else if(huart->Instance == UART4)
//   // {
//   //   D.filling();
//   //   HAL_UART_Receive_IT(&huart4, &D.receieve_char, 1);
//   // }
// }
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
  // //if(huart->Instance == UART5)
  // R.receieve();
  // if(R.access_ok() == true)
  // {
  //   yaw_sonar = R.get_yaw();
  //   ex = R.get_geometry_vector();
  //   ev.x = R.get_vel0();
  //   ev.y = R.get_vel1();
  //   ev.z = R.get_vel2();
  //   R.access_init();
  // }
  //HAL_UART_Receive_IT(&huart5, arr_test, 28);
//}
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
