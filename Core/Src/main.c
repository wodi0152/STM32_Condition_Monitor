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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static int16_t mon_shunt_raw = 0;
volatile uint8_t oled_address = 0;
volatile uint8_t ina219_found = 0;
volatile uint8_t mpu6050_found = 0;

uint32_t adc_value = 0;
uint32_t pot_percent = 0;
uint32_t pwm_percent = 0;

char oled_text[32];

/* Sensor readings */
static int32_t mon_ax = 0, mon_ay = 0, mon_az = 0;
static uint32_t mon_bus_mv = 0;
static int32_t mon_current_ma = 0;
static int32_t mon_temp10 = 0;

static uint8_t mon_mpu_ok = 0;
static uint8_t mon_ina_ok = 0;
static uint8_t mon_temp_ok = 0;
static uint8_t mon_ds_busy = 0;
static uint32_t mon_ds_stamp = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Shunt resistor: 100 mOhm if marked R100.
   Check actual resistor marking before trusting mA. */
#define INA_SHUNT_MOHM 100

/* Microsecond delay using Cortex-M4 DWT */
static void MON_DelayUs(uint32_t us)
{
    uint32_t cycles = (SystemCoreClock / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles)
    {
    }
}

/* DS18B20 1-Wire, A8, open-drain */
static void OW_Low(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

static void OW_Release(void)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

static uint8_t OW_Reset(void)
{
    OW_Low();
    MON_DelayUs(480);
    OW_Release();
    MON_DelayUs(70);

    uint8_t present =
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET;

    MON_DelayUs(410);
    return present;
}

static void OW_WriteBit(uint8_t bit)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();

    OW_Low();
    if (bit)
    {
        MON_DelayUs(6);
        OW_Release();
        MON_DelayUs(64);
    }
    else
    {
        MON_DelayUs(60);
        OW_Release();
        MON_DelayUs(10);
    }

    if (mask == 0U) __enable_irq();
}

static uint8_t OW_ReadBit(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();

    OW_Low();
    MON_DelayUs(3);
    OW_Release();
    MON_DelayUs(12);

    uint8_t bit =
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET;

    MON_DelayUs(55);

    if (mask == 0U) __enable_irq();
    return bit;
}

static void OW_WriteByte(uint8_t value)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        OW_WriteBit(value & 1U);
        value >>= 1;
    }
}

static uint8_t OW_ReadByte(void)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; i++)
        value |= OW_ReadBit() << i;

    return value;
}

static uint8_t OW_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t value = data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            uint8_t mix = (crc ^ value) & 1U;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            value >>= 1;
        }
    }
    return crc;
}

/* Non-blocking DS18B20 temperature measurement.
   Assumes one externally powered sensor on A8. */
static void MON_DS18B20_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (!mon_ds_busy)
    {
        if ((uint32_t)(now - mon_ds_stamp) >= 1000U)
        {
            mon_ds_stamp = now;

            if (OW_Reset())
            {
                OW_WriteByte(0xCC); // Skip ROM
                OW_WriteByte(0x44); // Convert T
                mon_ds_busy = 1;
            }
            else
            {
                mon_temp_ok = 0;
            }
        }
    }
    else if ((uint32_t)(now - mon_ds_stamp) >= 800U)
    {
        mon_ds_busy = 0;
        mon_temp_ok = 0;

        if (!OW_Reset()) return;

        OW_WriteByte(0xCC);
        OW_WriteByte(0xBE); // Read scratchpad

        uint8_t data[9];

        for (uint8_t i = 0; i < 9; i++)
            data[i] = OW_ReadByte();

        if (OW_CRC8(data, 8) != data[8]) return;

        int16_t raw =
            (int16_t)(((uint16_t)data[1] << 8) | data[0]);

        mon_temp10 = ((int32_t)raw * 10) / 16;
        mon_temp_ok = 1;
    }
}

/* Read MPU6050 acceleration, units: mg */
static void MON_ReadMPU(void)
{
    uint8_t data[6];

    mon_mpu_ok = (HAL_I2C_Mem_Read(
        &hi2c1, 0x68 << 1, 0x3B,
        I2C_MEMADD_SIZE_8BIT, data, 6, 100) == HAL_OK);

    if (!mon_mpu_ok) return;

    int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    int16_t z = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

    mon_ax = ((int32_t)x * 1000) / 16384;
    mon_ay = ((int32_t)y * 1000) / 16384;
    mon_az = ((int32_t)z * 1000) / 16384;
}

/* Read a 16-bit INA219 register */
static uint8_t MON_INAReg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (HAL_I2C_Mem_Read(
        &hi2c1, 0x40 << 1, reg,
        I2C_MEMADD_SIZE_8BIT, data, 2, 100) != HAL_OK)
        return 0;

    *value = ((uint16_t)data[0] << 8) | data[1];
    return 1;
}


static void MON_ReadINA(void)
{
    uint16_t bus_raw = 0;
    uint16_t shunt_raw = 0;
    uint16_t current_raw = 0;

    uint8_t ok_bus = MON_INAReg(0x02, &bus_raw);
    uint8_t ok_shunt = MON_INAReg(0x01, &shunt_raw);
    uint8_t ok_current = MON_INAReg(0x04, &current_raw);

    mon_ina_ok = ok_bus && ok_shunt && ok_current;

    if (!mon_ina_ok)
    {
        mon_bus_mv = 0;
        mon_current_ma = 0;
        mon_shunt_raw = 0;
        return;
    }

    /* Bus voltage in millivolts */
    mon_bus_mv = (bus_raw >> 3) * 4U;

    /* Raw shunt voltage: 10 uV per bit */
    mon_shunt_raw = (int16_t)shunt_raw;

    /* Current register: 0.1 mA per bit */
    mon_current_ma =
        ((int32_t)(int16_t)current_raw) / 10;
}


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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ===== MOTOR INITIALIZATION ===== */

  /* Motor OFF at startup */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /* Start PWM at 0% */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
      Error_Handler();
  }

  /* ===== OLED INITIALIZATION ===== */

  ssd1306_Init();

  /* ===== CHECK I2C SENSORS ===== */

  ina219_found =
      (HAL_I2C_IsDeviceReady(
          &hi2c1, 0x40 << 1, 3, 100) == HAL_OK);

  mpu6050_found =
      (HAL_I2C_IsDeviceReady(
          &hi2c1, 0x68 << 1, 3, 100) == HAL_OK);

  /* ===== MPU6050 INITIALIZATION ===== */

  /* Wake up MPU6050 */
  uint8_t mpu_cfg = 0x00;

  if (mpu6050_found)
  {
      HAL_I2C_Mem_Write(
          &hi2c1,
          0x68 << 1,
          0x6B,
          I2C_MEMADD_SIZE_8BIT,
          &mpu_cfg,
          1,
          100
      );

      HAL_Delay(100);

      /* Accelerometer range +/-2g */
      HAL_I2C_Mem_Write(
          &hi2c1,
          0x68 << 1,
          0x1C,
          I2C_MEMADD_SIZE_8BIT,
          &mpu_cfg,
          1,
          100
      );
  }

  /* ===== INA219 INITIALIZATION ===== */


  /* ===== INA219 INITIALIZATION ===== */

  /* 32V range, 320mV shunt, continuous mode */
  uint8_t ina_cfg[2] = {0x39, 0x9F};

  /* For R100 shunt (0.1 Ohm):
     Current LSB = 0.1 mA
     Calibration = 4096 */
  uint8_t ina_cal[2] = {0x10, 0x00};

  if (ina219_found)
  {
      HAL_I2C_Mem_Write(
          &hi2c1, 0x40 << 1, 0x00,
          I2C_MEMADD_SIZE_8BIT,
          ina_cfg, 2, 100
      );

      HAL_I2C_Mem_Write(
          &hi2c1, 0x40 << 1, 0x05,
          I2C_MEMADD_SIZE_8BIT,
          ina_cal, 2, 100
      );
  }

  /* ===== DS18B20 INITIALIZATION ===== */

  /* Enable DWT microsecond timer */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Release 1-Wire data line A8 */
  HAL_GPIO_WritePin(
      GPIOA,
      GPIO_PIN_8,
      GPIO_PIN_SET
  );

  /* Schedule first temperature measurement */
  mon_ds_stamp = HAL_GetTick() - 1000U;

  /* ===== INITIAL DISPLAY ===== */

  ssd1306_Fill(Black);

  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString(
      "STM32 MONITOR",
      Font_7x10,
      White
  );

  ssd1306_SetCursor(0, 20);
  ssd1306_WriteString(
      "INITIALIZING...",
      Font_6x8,
      White
  );

  ssd1306_UpdateScreen();

  HAL_Delay(1000);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  /* ===== READ POTENTIOMETER ===== */

	  uint32_t target_pwm = 0;

	  if (HAL_ADC_Start(&hadc1) == HAL_OK)
	  {
	      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	      {
	          adc_value = HAL_ADC_GetValue(&hadc1);

	          pot_percent = (adc_value * 100U) / 4095U;

	          target_pwm = pot_percent;
	      }

	      HAL_ADC_Stop(&hadc1);
	  }

	  /* Stop command at low potentiometer position */
	  if (target_pwm <= 5U)
	  {
	      target_pwm = 0;
	  }

	  /* ===== SMOOTH MOTOR CONTROL ===== */

	  if (pwm_percent < target_pwm)
	  {
	      uint32_t step = target_pwm - pwm_percent;

	      pwm_percent += (step > 5U) ? 5U : step;
	  }
	  else if (pwm_percent > target_pwm)
	  {
	      uint32_t step = pwm_percent - target_pwm;

	      pwm_percent -= (step > 5U) ? 5U : step;
	  }

	  /* ===== MOTOR PWM ===== */

	  if (pwm_percent == 0U)
	  {
	      /* Motor STOP */
	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	  }
	  else
	  {
	      /* Motor forward */
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	      uint32_t period =
	          __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;

	      uint32_t compare =
	          (pwm_percent * period) / 100U;

	      __HAL_TIM_SET_COMPARE(
	          &htim3, TIM_CHANNEL_1, compare
	      );
	  }

	  /* ===== READ SENSORS ===== */

	  /* DS18B20 temperature */
	  MON_DS18B20_Task();

	  /* MPU6050 acceleration */
	  MON_ReadMPU();

	  /* INA219 voltage and current */
	  MON_ReadINA();

	  /* ===== OLED DISPLAY ===== */

	  char line[32];

	  ssd1306_Fill(Black);

	  /* PWM */
	  snprintf(line, sizeof(line), "PWM:%lu%%",
	           (unsigned long)pwm_percent);

	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString(line, Font_6x8, White);

	  /* Temperature */
	  if (mon_temp_ok)
	  {
	      int32_t abs_temp =
	          (mon_temp10 < 0) ? -mon_temp10 : mon_temp10;

	      snprintf(line, sizeof(line), "TEMP:%s%ld.%ld C",
	               (mon_temp10 < 0) ? "-" : "",
	               (long)(abs_temp / 10),
	               (long)(abs_temp % 10));
	  }
	  else
	  {
	      snprintf(line, sizeof(line), "TEMP: WAIT/ERR");
	  }

	  ssd1306_SetCursor(0, 9);
	  ssd1306_WriteString(line, Font_6x8, White);

	  /* INA219 bus voltage */
	  if (mon_ina_ok)
	  {
	      snprintf(line, sizeof(line), "BUS:%lumV",
	               (unsigned long)mon_bus_mv);
	  }
	  else
	  {
	      snprintf(line, sizeof(line), "BUS: ERROR");
	  }

	  ssd1306_SetCursor(0, 18);
	  ssd1306_WriteString(line, Font_6x8, White);


	  /* ===== INA219 CURRENT DIAGNOSTIC ===== */

	  uint16_t shunt_raw = 0;

	  /* Read actual shunt voltage register */
	  if (MON_INAReg(0x01, &shunt_raw))
	  {
	      int16_t shunt_signed = (int16_t)shunt_raw;

	      /* R100 shunt = 0.1 Ohm
	         1 bit = 10 uV = 0.1 mA */
	      int32_t current_ma = shunt_signed / 10;

	      snprintf(
	          line,
	          sizeof(line),
	          "I:%ldmA S:%d",
	          (long)current_ma,
	          (int)shunt_signed
	      );
	  }
	  else
	  {
	      snprintf(
	          line,
	          sizeof(line),
	          "INA219 ERROR"
	      );
	  }

	  ssd1306_SetCursor(0, 27);
	  ssd1306_WriteString(line, Font_6x8, White);

	  /* MPU6050 X */
	  if (mon_mpu_ok)
	  {
	      snprintf(line, sizeof(line), "X:%ldmg",
	               (long)mon_ax);
	  }
	  else
	  {
	      snprintf(line, sizeof(line), "MPU: ERROR");
	  }

	  ssd1306_SetCursor(0, 36);
	  ssd1306_WriteString(line, Font_6x8, White);

	  /* MPU6050 Y */
	  if (mon_mpu_ok)
	  {
	      snprintf(line, sizeof(line), "Y:%ldmg",
	               (long)mon_ay);
	  }
	  else
	  {
	      snprintf(line, sizeof(line), "Y:---");
	  }

	  ssd1306_SetCursor(0, 45);
	  ssd1306_WriteString(line, Font_6x8, White);

	  /* MPU6050 Z */
	  if (mon_mpu_ok)
	  {
	      snprintf(line, sizeof(line), "Z:%ldmg",
	               (long)mon_az);
	  }
	  else
	  {
	      snprintf(line, sizeof(line), "Z:---");
	  }

	  ssd1306_SetCursor(0, 54);
	  ssd1306_WriteString(line, Font_6x8, White);

	  ssd1306_UpdateScreen();

	  /* ===== LOOP DELAY ===== */

	  HAL_Delay(100);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS485_EN_Pin|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEMP_DQ_GPIO_Port, TEMP_DQ_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_EN_Pin PB5 PB9 */
  GPIO_InitStruct.Pin = RS485_EN_Pin|GPIO_PIN_5|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TEMP_DQ_Pin */
  GPIO_InitStruct.Pin = TEMP_DQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEMP_DQ_GPIO_Port, &GPIO_InitStruct);

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
