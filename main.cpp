#include "main.h"
#include "i2c.h"
#include "gpio.h"

#define I2C_ENCODER_ADDRESS 0x30  // Replace with your encoder's I2C address

UART_HandleTypeDef huart2; // Change this to your UART handle

void HAL_UART_MspInit(UART_HandleTypeDef* huart);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init(); // Initialize your UART here

  uint8_t data;
  HAL_StatusTypeDef status;
  
  char uartBuffer[32]; // Buffer to store UART data

  while (1)
  {
    status = HAL_I2C_Master_Receive(&hi2c1, I2C_ENCODER_ADDRESS << 1, &data, 1, HAL_MAX_DELAY);

    if (status == HAL_OK)
    {
      // Process data received from the encoder
      // Example: If the encoder sends position data, you can use it here.
      // For example, assuming the encoder data represents an integer position:
      int encoderValue = data; // Convert data to actual encoder value
      snprintf(uartBuffer, sizeof(uartBuffer), "Encoder Value: %d\r\n", encoderValue);
      HAL_UART_Transmit(&huart2, (uint8_t*)uartBuffer, strlen(uartBuffer), HAL_MAX_DELAY);
    }
    else
    {
      // Handle I2C communication error
    }

    HAL_Delay(10); // Delay for debouncing or as needed
  }
}
