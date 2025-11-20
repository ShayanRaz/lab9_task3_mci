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
#include <stdio.h>
#include <math.h>
#include <usb_device.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

volatile float shared_angle = 0;
volatile uint8_t display_flag = 0;
volatile float shared_accAngleY = 0;
volatile float shared_gyroY = 0;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

#define LSM303AGR_ACC_ADDR (0x19 << 1) 
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23
#define OUT_X_L_A 0x28

#define WHO_AM_I_G 0x0F
#define CTRL_REG1_G 0x20
#define CTRL_REG4_G 0x23
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_G 0x2B

// ===== SPI CS Control =====
#define CS_LOW() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)

typedef struct
{
    int16_t raw_ax, raw_ay, raw_az;
    float ax, ay, az;
    float ax_offset, ay_offset, az_offset;
    int16_t raw_gx, raw_gy, raw_gz;
    float gx, gy, gz;
    float gy_offset;
} SensorData;

SensorData c;

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void Init_LSM(void);
void Init_Gyro(void);
void Read_Accel(SensorData *s, float apply_offset);
void Read_Gyro(SensorData *s);
void Calibrate_Gyro(SensorData *s);
void Offset_LSM(SensorData *data);
/* USER CODE END 0 */
float accAngleY = 0, gyroAngleX = 0, angleX = 0;
float dt = 0.1f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// volatile uint8_t display_flag = 0;   
volatile int ax_int, ay_int, az_int;

/* USER CODE END 0 */

extern float asinf(float);
extern double fabs(double);

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  
  /* USER CODE BEGIN 2 */
  Init_LSM();
  Init_Gyro();
  Offset_LSM(&c);            
  Calibrate_Gyro(&c); 
  /* USER CODE END 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(display_flag)
    {
        display_flag = 0;

        printf("%.2f\t %.2f\t %.2f\r\n",
               shared_accAngleY,
               shared_gyroY,
               shared_angle);
    }
}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        static float tilt_angle = 0;
        static int counter = 0;
        Read_Accel(&c, 1);
        Read_Gyro(&c);
        float y_norm = c.ay / 9.81f;
        if (y_norm > 1.0f)  y_norm = 1.0f;
        if (y_norm < -1.0f) y_norm = -1.0f;
        float acc_y = asinf(y_norm) * (180.0f / M_PI);
        tilt_angle = 0.98f * (tilt_angle + c.gy * dt) + 0.02f * acc_y;
        shared_angle = tilt_angle;
        shared_accAngleY = acc_y;
        shared_gyroY     = c.gy;
        counter++;
        if(counter >= 9)
        {
            display_flag = 1;
            counter = 0;
        }
    }
}

void Init_LSM(void)
{
  uint8_t data[2];

  // Enable all axes, 100 Hz, normal mode
  data[0] = CTRL_REG1_A;
  data[1] = 0x67;
  HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x20,I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // ±2g, continuous update
  data[0] = CTRL_REG4_A;
  data[1] = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, 0x32, 0x23,I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  HAL_Delay(50);

  // Verify configuration
  uint8_t whoami = 0, ctrl1 = 0;
  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACC_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &whoami, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, LSM303AGR_ACC_ADDR, CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, HAL_MAX_DELAY);
  printf("WHO_AM_I = 0x%02X, CTRL_REG1_A = 0x%02X\r\n", whoami, ctrl1);

  printf("LSM303AGR Initialized\r\n");
}

void spi_write(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg & 0x7F, value};  // Write: MSB = 0
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

uint8_t spi_read(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];
    tx[0] = reg | 0x80; // Read: MSB = 1
    tx[1] = 0x00;

    CS_LOW();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    CS_HIGH();

    return rx[1];
}
void spi_read_axis(uint8_t reg_low, int16_t *value)
{
    uint8_t low = spi_read(reg_low);
    uint8_t high = spi_read(reg_low + 1);
    *value = (int16_t)(high << 8 | low);
}

void Init_Gyro(void)
{
    HAL_Delay(100);
    
    uint8_t whoami = spi_read(WHO_AM_I_G);

    if (whoami != 0xD3) // Expected value for I3G4250D
    {
        printf("Gyro WHO_AM_I error: 0x%02X (expected 0xD3)\r\n", whoami);
    }
    else
    {
        printf("Gyro WHO_AM_I: 0x%02X - OK\r\n", whoami);
    }

    // CTRL_REG1: Power on, enable X/Y/Z, ODR = 100Hz, BW = 25Hz
    // 0x0F = 0000 1111 (DR=00, BW=00, PD=1, Zen=1, Yen=1, Xen=1)
    spi_write(CTRL_REG1_G, 0x0F);

    // CTRL_REG4: ±245 dps full-scale, continuous update
    // 0x00 = 0000 0000 (BDU=0, BLE=0, FS=00, -)
    spi_write(CTRL_REG4_G, 0x00);

    HAL_Delay(50);
    printf("I3G4250D Gyroscope Initialized\r\n");
}

void Read_Gyro(SensorData *s)
{
    int16_t gy_raw;
    uint8_t yl = spi_read(OUT_Y_L_G);
    uint8_t yh = spi_read(OUT_Y_H_G);
    
    gy_raw = (int16_t)(yh << 8 | yl);
    s->raw_gy = gy_raw;
    s->gy = (gy_raw * 8.75f / 1000.0f) - s->gy_offset;

    // Dead-zone: ignore very small gyro values
    if (fabs(s->gy) < 0.1f)  // 0.1 deg/s threshold
        s->gy = 0.0f;
}

void Calibrate_Gyro(SensorData *s)
{
    float sum_gy = 0;
    uint16_t samples = 500;  // Increase samples

    printf("Calibrating gyroscope in 2 seconds...\r\n");
    // HAL_Delay(2000);  // Give time to place device flat
    printf("Calibrating... Keep sensor STILL!\r\n");

    for (uint16_t i = 0; i < samples; i++)
    {
        int16_t gy_raw;
        uint8_t yl = spi_read(OUT_Y_L_G);
        uint8_t yh = spi_read(OUT_Y_H_G);
        
        gy_raw = (int16_t)(yh << 8 | yl);
        sum_gy += gy_raw * 8.75f / 1000.0f;

        // HAL_Delay(10);
    }

    s->gy_offset = sum_gy / samples;

    printf("Gyro Y Offset: %.3f deg/s\r\n", s->gy_offset);
}

void Read_Accel(SensorData *data, float apply_offset)
{
  uint8_t raw[6];
  HAL_I2C_Mem_Read(&hi2c1, 0x32, OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, raw, 6, HAL_MAX_DELAY);


  data->raw_ay = (int16_t)((raw[3] << 8) | raw[2]);

  data->raw_ay = (data->raw_ay) >> 6 ;


  float ay_scaled = data->raw_ay * 0.004f * 9.81f;


  if (apply_offset)
  {
    data->ay = ay_scaled - data->ay_offset;
  }
  else
  {
    data->ay = ay_scaled;
  }
}


void Offset_LSM(SensorData *data)
{
  float sum_y = 0;
  uint8_t i;

  printf("Calibrating offsets... Keep sensor still.\r\n");

  for (i = 0; i < 20; i++)
  {
    Read_Accel(data, 0);
    sum_y += data->ay;
    // HAL_Delay(100);
  }

  data->ay_offset = sum_y / 20.0f;

  printf("Offsets -> X: %.2f, Y: %.2f, Z: %.2f\r\n", data->ax_offset, data->ay_offset, data->az_offset);
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 5999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 79;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif