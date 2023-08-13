
#include "main.h"
#include "ssd1306.h"
#include "SSD1306I2C.h"
#include <string>
#include "usart.h"
#include "i2c.h"
#include "gpio.h"
int desiredEncoderValue = 200; // Change this to your desired encoder value
bool rotate = true;
#define I2C_ENCODER_ADDRESS 0x36 // Replace with your encoder's I2C address
char message[] = "Hello from STM32!\r\n";
UART_HandleTypeDef huart2; // Change this to your UART handle
UART_HandleTypeDef huart1;
int encoderValue = 100;
int lastEncoderValue = 10;
int realEncoderValue = 0;
uint8_t data;
HAL_StatusTypeDef status;
uint8_t low_byte, high_byte;
uint16_t raw_angle;
float deg_angle, start_angle = 0.0, corrected_angle = 0.0, previous_corrected_angle = 0.0;
int quadrant_number = 0, previous_quadrant_number = 0, number_of_turns = 0;
char uartBuffer[32]; // Buffer to store UART data

void HAL_UART_MspInit(UART_HandleTypeDef *huart);

int x;
int stepCount = 0; // Initialize step count to zero
/* USER CODE END PM */
// PID gains
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.01;

// Other global variables
float integral = 0.0;
float prev_error = 0.0;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define DIR_PIN GPIO_PIN_1
#define DIR_PORT GPIOA
#define STEP_PIN GPIO_PIN_4
#define STEP_PORT GPIOA

void microDelay(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay)
    ;
}

/* USER CODE END 0 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
   */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin | STEP_Pin | LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin LD2_Pin */
  GPIO_InitStruct.Pin = DIR_Pin | LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP_Pin */
  GPIO_InitStruct.Pin = STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEP_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

int encoder()
{
  // Read low byte of raw angle data from AS5600 sensor
  HAL_StatusTypeDef statuss = HAL_I2C_Mem_Read(&hi2c2, I2C_ENCODER_ADDRESS << 1, 0x0D, 1, &low_byte, 1, HAL_MAX_DELAY);

  // Read high byte of raw angle data from AS5600 sensor
  HAL_StatusTypeDef statuss = HAL_I2C_Mem_Read(&hi2c2, I2C_ENCODER_ADDRESS << 1, 0x0C, 1, &high_byte, 1, HAL_MAX_DELAY);

  if (statuss == HAL_OK)
  {
    raw_angle = (high_byte << 8) | low_byte;

    // Process raw angle data as needed
    // For example, you can print the raw_angle value
    char uartBuffer[32];
    snprintf(uartBuffer, sizeof(uartBuffer), "Raw Angle: %d\r\n", raw_angle);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    // Convert raw angle to degrees
    deg_angle = raw_angle * 0.087890625;

    // Correct the angle
    if (start_angle == 0.0)
    {
      start_angle = deg_angle;
    }
    corrected_angle = deg_angle - start_angle;
    if (corrected_angle < 0)
    {
      corrected_angle += 360.0;
    }

    // Determine the quadrant
    if (0 <= corrected_angle && corrected_angle <= 90)
    {
      quadrant_number = 1;
    }
    else if (90 < corrected_angle && corrected_angle <= 180)
    {
      quadrant_number = 2;
    }
    else if (180 < corrected_angle && corrected_angle <= 270)
    {
      quadrant_number = 3;
    }
    else
    {
      quadrant_number = 4;
    }

    // Check for quadrant transition
    if (quadrant_number != previous_quadrant_number)
    {
      if (quadrant_number == 1 && previous_quadrant_number == 4)
      {
        number_of_turns += 1;
      }
      else if (quadrant_number == 4 && previous_quadrant_number == 1)
      {
        number_of_turns -= 1;
      }
      previous_quadrant_number = quadrant_number;
    }

    // Calculate the total angle
    float total_angle = (number_of_turns * 360) + corrected_angle;
    encoderValue = total_angle;
    SSD1306 DISPLAY;
    HAL_TIM_Base_Start(&htim2);
    DISPLAY.SSD1306_Init();
    // Process data received from the encoder
    // Example: If the encoder sends position data, you can use it here.
    // For example, assuming the encoder data represents an integer position:
    // Convert data to actual encoder value
    // snprintf(uartBuffer, sizeof(uartBuffer), "Encoder Value: %d\r\n", encoderValue);
    // HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);

    // Store the corrected angle for the next iteration
    previous_corrected_angle = corrected_angle;
  }
  else
  {
    // Handle I2C communication error
    Error_Handler();
  }
  return encoderValue;
}
void step(int steps, uint8_t direction, uint16_t delay)
{
  desiredEncoderValue = steps;
  SSD1306 DISPLAY;
  HAL_TIM_Base_Start(&htim2);
  DISPLAY.SSD1306_Init();
  realEncoderValue = 0;
  display(realEncoderValue, desiredEncoderValue);
  if (direction == 0)
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
  while (realEncoderValue < desiredEncoderValue)
  {
    // DISPLAY.SSD1306_Clear();
    float encoder_value = encoder();
    if (encoder_value != lastEncoderValue)
    {
      realEncoderValue++;
      lastEncoderValue = encoder_value;
    }

    display(realEncoderValue, desiredEncoderValue);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
    microDelay(delay);
    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
    microDelay(delay);
    x++;
    snprintf(uartBuffer, sizeof(uartBuffer), "Encoder Value: %d\r\n\n", encoder_value);
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    // snprintf(uartBuffer, sizeof(uartBuffer), "     step value: %d\r\n", desiredEncoderValue);
    // HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
  }
  realEncoderValue = 0;
}

float CalculateError(float encoder_value, int maintain)
{
  // Calculate and return error between desired position and encoder value
  maintain - encoder_value; // 200 steps as a sample
}
float CalculatePIDControlSignal(float error)
{
  // Calculate and return PID control signal
  integral += error;
  float derivative = error - prev_error;
  prev_error = error;
  return (Kp * error + Ki * integral + Kd * derivative) / 1000000; // angle to steps
}
void ControlStepperMotor(float control_signal)
{
  if (rotate) // rotate if there is  a difference in encoder value so it mainains 200
  {
    // Apply control signal to stepper motor (e.g., adjust PWM duty cycle)
    if (control_signal > desiredEncoderValue)
    {
      step(control_signal - desiredEncoderValue, 1, 5000); // forward
    }
    else if (control_signal < desiredEncoderValue)
    {
      step(desiredEncoderValue - control_signal, 0, 5000); // reverse
    }
    else
    {
      // do nothing
    }
    rotate = false;
  }
}
void UpdateIntegralAndDerivative(float error)
{
  // Update integral and derivative terms for the next iteration
  integral += error;
}
void display(int enc, int ste)
{
  std::string displayStr = "steps: " + std::to_string(ste);
  DISPLAY.SSD1306_GotoXY(0, 0);
  // DISPLAY.SSD1306_UpdateScreen();
  DISPLAY.SSD1306_Puts(const_cast<char *>(displayStr.c_str()), &Font_11x18, 0x01);
  // DISPLAY.SSD1306_UpdateScreen();
  displayStr = "enc: " + std::to_string(enc);
  DISPLAY.SSD1306_GotoXY(0, 30);
  // DISPLAY.SSD1306_UpdateScreen();
  DISPLAY.SSD1306_Puts(const_cast<char *>(displayStr.c_str()), &Font_11x18, 0x01);
  DISPLAY.SSD1306_UpdateScreen();
}
void stepScreen(int enc)
{
  std::string displayStr = std::to_string(enc);
  DISPLAY.SSD1306_GotoXY(6, 30);
  // DISPLAY.SSD1306_UpdateScreen();
  DISPLAY.SSD1306_Puts(const_cast<char *>(displayStr.c_str()), &Font_11x18, 0x01);
  DISPLAY.SSD1306_UpdateScreen();
}
int main(void)
{

  HAL_Init();

  /* USER CODE BEGIN Init */
  // SSD1306_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init(); // Initialize your UART peripheral
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306 DISPLAY;
  HAL_TIM_Base_Start(&htim2);
  DISPLAY.SSD1306_Init();
  step(200, 1, 5000); // move to the set 200th step at boot
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /*
    int y;

    for (y = 0; y < 8; y = y + 1) // 8 times
    {
      DISPLAY.SSD1306_Clear();
      // Update the step count
      step(25, 0, 800); // 25 steps (45 degrees) CC
      HAL_Delay(100);
    }
   // step(800, 1, 5000); // 800 steps (4 revolutions ) CV
    encoder();
    // Read encoder value
    DISPLAY.SSD1306_Clear();
    display(realEncoderValue, 200);*/
    float encoder_value = encoder();
    if (encoder_value != lastEncoderValue)
    {
      realEncoderValue++;        // encoder step counter
      if (realEncoderValue > 10) // debounce
      {
        realEncoderValue = realEncoderValue - lastEncoderValue; // changes in value is what will be rotated
        lastEncoderValue = encoder_value;
        rotate = true;
      }
    }
    DISPLAY.SSD1306_Clear();
    display(realEncoderValue, 200);
    // Calculate error// the error is the actual distance the stepper is going to move
    float error = CalculateError(realEncoderValue, 200); // the 200 is what  i assumed will be the target step
    snprintf(uartBuffer, sizeof(uartBuffer), "Error: %d\r\n\n", realEncoderValue);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    // Calculate PID control signal according to the error above
    float control_signal = realEncoderValue; // CalculatePIDControlSignal(error); 400 is for testung deviation correction

    snprintf(uartBuffer, sizeof(uartBuffer), "control signal pid: %d\r\n\n", control_signal);
    HAL_UART_Transmit(&huart2, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    // Apply control signal to stepper motor
    ControlStepperMotor(control_signal);

    // Update integral and derivative terms
    UpdateIntegralAndDerivative(error);

    HAL_Delay(5000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
