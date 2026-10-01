/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author         : Luca Obwegs
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 Luca Obwegs.
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
#include "led.h"
#include "battery.h"
#include "button.h"
#include "buzzer.h"
#include "vario.h"
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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

/* USER CODE BEGIN PV */
volatile SystemError_t g_system_error = ERR_UNKNOWN;
static uint8_t initial_turn_on_released = 0;
SystemState_t current_state = STATE_OFF;
static uint32_t vbat = 0;
volatile uint8_t battery_soc_pct = 0;
static uint8_t stall_count = 0;  /* consecutive IMU stall events; reboot at threshold */
static uint32_t last_batt_check_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
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
  // Keep SWD clocks running during Standby and Stop modes for debugging ---> REMOVE IN PRODUCTION CODE
  HAL_DBGMCU_EnableDBGStandbyMode();
  HAL_DBGMCU_EnableDBGStopMode();
  /* IWDG runs in standby; if it fired while sleeping re-enter standby silently (no LED animation) */
  if (RCC->CSR & RCC_CSR_IWDGRSTF) {
      RCC->CSR |= RCC_CSR_RMVF;                     /* clear all reset flags */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
      HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3);
      IWDG->KR = 0xAAAA;                             /* pet before re-entering standby */
      HAL_PWR_EnterSTANDBYMode();
      while (1);                                     /* unreachable; satisfies compiler */
  }
  /* IWDG: ~26 s hardware watchdog — automatic reboot if firmware freezes for any reason.
     LSI ~37 kHz / prescaler-256 / reload-4095 ≈ 28 s (worst-case 47 kHz ≈ 22 s). */
  IWDG->KR = 0xCCCC;                  /* start IWDG */
  IWDG->KR = 0x5555;                  /* unlock PR/RLR registers */
  IWDG->PR = 6; IWDG->RLR = 0x0FFF;  /* /256 prescaler, max reload 4095 */
  while (IWDG->SR & 0x3);             /* wait for registers to update */
  IWDG->KR = 0xAAAA;                  /* reload — countdown starts */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  Battery_Load_State_From_EEPROM();
  /* USER CODE BEGIN 2 */
  /* 1. Check whether the button was held for the full power-on gesture (2 s) */
  if (Is_Button_Held_2s()) {
    current_state = STATE_ACTIVE;
    initial_turn_on_released = 0; // lock out shutdown until the finger is lifted once

    /* --- Turn-on animation: LEDs light up one after another --- */
    HAL_GPIO_WritePin(GPIOA, BATled1_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOA, BATled2_Pin, GPIO_PIN_SET);
    HAL_Delay(250);
    HAL_GPIO_WritePin(GPIOA, BATled3_Pin, GPIO_PIN_SET);
    HAL_Delay(400); // brief pause so the user registers that all 3 are lit

    /* --- Battery level readout, shown for ~3 s --- */
    vbat = Read_Battery_MV();
    battery_soc_pct = Battery_Get_Fused_SOC(vbat, 25);

    if (vbat <= BAT_SHUTDOWN_MV) {
      /* Battery is already critical at power-on: skip the normal level display and
         don't start the sensors, buzzer, or IMU interrupt at all. Flash the warning
         LED and fall through to a clean shutdown; sensors_initialized stays 0, so
         STATE_OFF below skips sensor standby. */
      HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);
      Handle_Critical_Battery();
    } else {
      /* Only turn off whichever LEDs don't belong at this SOC level - the ones that do
         belong just stay lit from the turn-on animation, so there's no off/on flicker. */
      Display_Battery_Level_Static(battery_soc_pct);
      HAL_Delay(3000);
      HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);

      /* 2. Initialize sensors */
      if (SPL06_Init(&hi2c1) != HAL_OK) {
        HALT_WITH_ERROR(ERR_BARO_SPL06);
      }
      dbg_spl06_active = 1;

      if (LSM6DS3_Init(&hi2c1) != HAL_OK) {
        HALT_WITH_ERROR(ERR_IMU_LSM6DS3);
      }
      dbg_imu_active = 1;

      /* 3. Clear latched IMU interrupts */
      uint8_t dummy_buf[12];
      for (uint8_t i = 0; i < 3; i++) {
        LSM6DS3_ReadRaw_Poll(&hi2c1, dummy_buf);
        HAL_Delay(10);
      }

      /* 4. Barometer stabilization */
      HAL_Delay(100);
      SPL06_GetTemperature(&hi2c1); // populate calib.last_t_raw_sc first
      reference_pressure_pa = SPL06_GetPressure(&hi2c1);

      /* 5. Start PWM for the buzzer */
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      g_volume = Load_Volume();
      Set_Volume(g_volume);

      /* 5b. Start TIM21 as a free-running timestamp counter for IMU dt measurement */
      HAL_TIM_Base_Start(&htim21);

      /* 6. Arm the EXTI interrupt */
      HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

      /* Seed the stall watchdog so it doesn't fire immediately on entry */
      last_imu_sample_tick = HAL_GetTick();
    }
  } else if (HAL_GPIO_ReadPin(USB_PORT, USB_PIN) == GPIO_PIN_SET) {
    current_state = STATE_CHARGING; // USB plugged in at boot, button not held
  } else {
    current_state = STATE_OFF;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    switch (current_state) {
      case STATE_OFF:
        if (HAL_GPIO_ReadPin(USB_PORT, USB_PIN) == GPIO_PIN_SET) {
          current_state = STATE_CHARGING;
          break; // skip the standby-entry code below this cycle
        }

        /* 1. Shut down audio, interrupts, and LEDs immediately at t = 2.0 s */
        HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel2_3_IRQn);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        Set_Volume(0); // shut down PAM8904

        if (sensors_initialized) {
          if (!Sensors_Safe_Shutdown()) {
            /* Couldn't confirm both sensors are powered down even after retries and a
               bus reset - make this visible instead of silently going to Standby with a
               sensor possibly still drawing current. Distinct pattern from the
               low-battery warning so it's identifiable on the bench. */
            for (uint8_t i = 0; i < 6; i++) {
              HAL_GPIO_TogglePin(GPIOA, BATled1_Pin | BATled3_Pin);
              HAL_Delay(150);
            }
            HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);
          }
          HAL_I2C_DeInit(&hi2c1); // releases I2C clock, DMA channel, and PA9/PA10 AF pins
        }

        /* --- Turn-off animation --- */
        HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_SET);
        HAL_Delay(1000);
        HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);

        /* 2. Wait silently for physical release so Standby hardware doesn't instant-reboot */
        while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
          HAL_Delay(10);
        }

        /* 3. Debounce: wait 100 ms for mechanical switch bouncing to fully settle */
        HAL_Delay(100);

        /* Remove the pull-up on INT1 - ~82 uA flows through it in Standby when the pin is driven low */
        {
          GPIO_InitTypeDef g = {0};
          g.Pin  = acc_INT_Pin;
          g.Mode = GPIO_MODE_INPUT;
          g.Pull = GPIO_NOPULL;
          HAL_GPIO_Init(acc_INT_GPIO_Port, &g);
        }

        /* USB hot-plugged during the shutdown animation - skip Standby */
        if (HAL_GPIO_ReadPin(USB_PORT, USB_PIN) == GPIO_PIN_SET) {
          current_state = STATE_CHARGING;
          break;
        }

        /* 4. Clear the wake-up flag and enter Standby mode */
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
        HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
        HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3);

        IWDG->KR = 0xAAAA; /* pet before standby - full ~26 s before next reset */
        dbg_mcu_state = DBG_MCU_STANDBY;
        HAL_PWR_EnterSTANDBYMode();
        /* If we reach this line, standby entry failed (pending wake-up flag or mis-configuration) */
        dbg_standby_fail++;
        dbg_mcu_state = DBG_MCU_ACTIVE;
        break;

      case STATE_ACTIVE:
        /* --- Shutdown timing (bypass STOP mode while the button is held) --- */
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
          if (initial_turn_on_released) {
            /* Silence the buzzer immediately so it doesn't hang mid-tone for the whole
               2 s hold - stop as soon as the press starts. */
            Set_Buzzer_Frequency(0);
            is_beeping = 0;

            uint32_t press_start = HAL_GetTick();

            /* Trap execution here so the MCU stays awake to count ticks normally */
            uint8_t consec_low = 0;
            while (1) {
              HAL_Delay(10);
              if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
                consec_low = 0;
                if ((HAL_GetTick() - press_start) >= 2000) {
                  current_state = STATE_OFF;
                  break;
                }
              } else {
                /* Require 50 ms of sustained release to guard against buzzer-induced bounce */
                if (++consec_low >= 5) break;
              }
            }
            if (current_state != STATE_OFF) {
              /* Short press: cycle volume 1 -> 2 -> 3 -> 1 */
              g_volume = (g_volume % 3) + 1;
              Set_Volume(g_volume);
              Save_Volume(g_volume);
              Display_Volume_LED(g_volume);
              last_imu_sample_tick = HAL_GetTick(); // prevent stall watchdog after the 1 s block
            }
          }
        } else {
          /* Button was completely released, arm the system for a future power-off hold */
          initial_turn_on_released = 1;
        }

        /* If the inner loop triggered a state change to OFF, jump out of this switch case instantly */
        if (current_state == STATE_OFF) {
          break;
        }

        /* --- Battery monitoring (every ~2 s - no need to sample at the 26 Hz IMU rate) ---
           Sample only when the system is quiescent enough that the regulator and sensor
           load are not biasing the battery divider measurement. */
        if ((HAL_GetTick() - last_batt_check_tick) >= BATT_CHECK_INTERVAL_MS) {
          last_batt_check_tick = HAL_GetTick();
          vbat = Battery_Read_MV_Quiescent();
          Battery_Update_Estimate(vbat);
          int32_t batt_temp_c = 25;
          if (sensors_initialized) {
            batt_temp_c = SPL06_GetTemperature(&hi2c1) / 1000;
          }
          battery_soc_pct = Battery_Get_Fused_SOC(vbat, batt_temp_c);

          if (vbat <= BAT_SHUTDOWN_MV) {
            Handle_Critical_Battery();
            break; // let STATE_OFF take over on the next loop iteration
          }
        }

        /* --- Main vario logic --- */
        if (imu_data_ready) {
          imu_data_ready = 0;
          last_imu_sample_tick = HAL_GetTick();
          stall_count = 0;
          IWDG->KR = 0xAAAA; /* pet the watchdog on every healthy IMU sample */
          Process_Vario_Math();
        } else if ((HAL_GetTick() - last_imu_sample_tick) > IMU_STALL_TIMEOUT_MS) {
          /* No IMU sample for far longer than the ~19 ms expected at 26 Hz - the I2C/DMA
             pipeline (or the sensor's INT1 line) is wedged. Reset the bus and sensor
             configuration instead of sitting silently until the user power-cycles. */
          if (++stall_count >= 5) NVIC_SystemReset(); // 5 consecutive stalls = ~2.5 s without data
          Recover_IMU_Bus();
        }

        /* Low-power sleep between samples */
        dbg_mcu_state = DBG_MCU_SLEEP;
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        dbg_mcu_state = DBG_MCU_ACTIVE;
        break;

      case STATE_CHARGING:
        /* --- Charging mode (stay awake) --- */
        IWDG->KR = 0xAAAA; /* pet the watchdog */

        vbat = Battery_Read_MV_Quiescent();
        Battery_Update_Estimate(vbat);
        int32_t batt_temp_c = 25;
        if (sensors_initialized) {
          batt_temp_c = SPL06_GetTemperature(&hi2c1) / 1000;
        }
        battery_soc_pct = Battery_Get_Fused_SOC(vbat, batt_temp_c);
        Display_Charging_Status(battery_soc_pct); // keep LEDs flashing/solid based on SOC

        /* Exit if USB is unplugged */
        if (HAL_GPIO_ReadPin(USB_PORT, USB_PIN) == GPIO_PIN_RESET) {
          current_state = STATE_OFF;
        }

        /* If the user holds the button for 2 s while charging, switch to vario mode */
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
          uint32_t press_start = HAL_GetTick();
          while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == GPIO_PIN_SET) {
            if ((HAL_GetTick() - press_start) > 2000) {
              current_state = STATE_ACTIVE;
              break;
            }
            HAL_Delay(10);
          }
        }
        HAL_Delay(50); // fast response for the button check
        break;
    }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = ENABLE;
  hadc.Init.Oversample.Ratio = ADC_OVERSAMPLING_RATIO_16;
  hadc.Init.Oversample.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc.Init.Oversample.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the default battery sense channel. VREFINT is measured separately
      by clearing the channel-selection register before each conversion so the
      internal reference path is not left selected alongside the battery pin. */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00000608;
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 4-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BATled1_Pin|BATled2_Pin|BATled3_Pin|BUZ_EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZ_EN1_GPIO_Port, BUZ_EN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CHstat_Pin */
  GPIO_InitStruct.Pin = CHstat_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHstat_GPIO_Port, &GPIO_InitStruct);

  /* Battery sense pin must be analog input for ADC1_IN1 */
  GPIO_InitStruct.Pin = BATvoltage_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BATvoltage_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BATled1_Pin BATled2_Pin BATled3_Pin BUZ_EN2_Pin */
  GPIO_InitStruct.Pin = BATled1_Pin|BATled2_Pin|BATled3_Pin|BUZ_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : acc_INT_Pin */
  GPIO_InitStruct.Pin = acc_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(acc_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZ_EN1_Pin */
  GPIO_InitStruct.Pin = BUZ_EN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZ_EN1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* ButtonIn (PA0) and USBsens (PA2) are wakeup pins — CubeMX skips their GPIO init */
  GPIO_InitStruct.Pin = ButtonIn_Pin | USBsens_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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
  __disable_irq();
  /* Blink LED1 for ~5 s then reboot — prevents permanent freeze on peripheral or sensor fault */
  for (uint32_t b = 0; b < 26; b++) {
    HAL_GPIO_TogglePin(GPIOA, BATled1_Pin);
    for (volatile uint32_t i = 0; i < 100000; i++);
  }
  NVIC_SystemReset();
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
