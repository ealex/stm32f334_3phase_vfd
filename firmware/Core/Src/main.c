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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// these defines are for the sine table and 3 phase output
#define SINE_SAMPLES		128           // samples per sine wave
#define SINE_SAMPLE_MIN_VAR	((uint16_t)(DT_RISING+0))
#define SINE_SAMPLE_MAX_VAR	((uint16_t)(HRTIM_PERIOD-(DT_FALLING+0)))
#define OUTPUT_FREQ_MIN		1				// minimum output frequency
#define OUTPUT_FREQ_MAX		400				// maximum output frequency
#define OUTPUT_FREQ_DEF		10				// default output frequency
#define OUTPUT_SCALING_DEF	((float)1.0)	// default output scaling

// these defines handle the input voltage to frequency conversion
#define SPEED_BUTTON_ALPHA		((float)0.1)
#define SPEED_BUTTON_BETA		((float)10)
#define SPEED_BUTTON_MIN		((float)0.1)
#define SPEED_BUTTON_MAX		((float)3.2)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Single sine lookup table (0-100% as 16-bit values scaled to HRTIM_PERIOD)
static volatile uint16_t sine_table[SINE_SAMPLES];

// this controls the phasing
static volatile uint8_t sample_index_a = 0;   // Phase A: 0°
static volatile uint8_t sample_index_b = 43;  // Phase B: 120° (64/3 = 21.33 ≈ 21)
static volatile uint8_t sample_index_c = 85;  // Phase C: 240° (64*2/3 = 42.67 ≈ 43)
static volatile uint32_t sine_frequency;			// requested output sine frequency
static volatile float outputScalingFactor = 0.1;	// requested output sine amplitude

// ADC results buffer
static volatile uint16_t adc_results[3]; // Phase A, B, C current readings

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void generateSineTables(void);
void updatePWMValues(void);
void setOutputScaling(float amplitude_proportional);
void setSineFrequency(uint32_t freq_hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/**
 * @brief Initialize single sine lookup table
 * checked
 */
void generateSineTables(void) {
	float step = (2.0 * M_PI) / SINE_SAMPLES;
    for (int i = 0; i < SINE_SAMPLES; i++) {
    	float x = i * step;
    	float sine_val = sinf(x);
    	float range = outputScalingFactor*((float)SINE_SAMPLE_MAX_VAR - (float)SINE_SAMPLE_MIN_VAR);
    	sine_table[i] = (uint16_t)(((sine_val + (float)1.0) / (float)2.0) * range) + SINE_SAMPLE_MIN_VAR;
    }
}


/**
 * @brief Set sine wave output frequency (10-100 Hz)
 */
void setSineFrequency(uint32_t freq_hz) {
    if (freq_hz < OUTPUT_FREQ_MIN) freq_hz = OUTPUT_FREQ_MIN;
    if (freq_hz > OUTPUT_FREQ_MAX) freq_hz = OUTPUT_FREQ_MAX;

    sine_frequency = freq_hz;

    /* Update TIM6 period for new frequency */
    uint32_t new_period = (1000000UL / (sine_frequency * SINE_SAMPLES)) - 1;
    LL_TIM_SetAutoReload(TIM6, new_period);
}

void setOutputScaling(float amplitude_proportional) {
	// set the new value
	// regenerate the table, to keep processing time small in the IRQ
	outputScalingFactor = amplitude_proportional;
	generateSineTables();
}


/**
 * @brief Update PWM compare values from single sine table (synchronized)
 * checked
 */
void updatePWMValues(void) {
    /* Disable preload to prevent updates during write sequence */
	LL_HRTIM_SuspendUpdate(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_C);

    /* Update all three timers using phase-shifted indices */
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, sine_table[sample_index_a]);
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, sine_table[sample_index_b]);
    LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, sine_table[sample_index_c]);

    /* Trigger synchronized update for all timers */
    LL_HRTIM_ResumeUpdate(HRTIM1, LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_C);

    /* Advance all phase indices */
    sample_index_a++;
    sample_index_a = sample_index_a % SINE_SAMPLES;
    //if (sample_index_a >= SINE_SAMPLES) sample_index_a = 0;

    sample_index_b++;
    sample_index_b = sample_index_b % SINE_SAMPLES;
    //if (sample_index_b >= SINE_SAMPLES) sample_index_b = 0;

    sample_index_c++;
    sample_index_c = sample_index_c % SINE_SAMPLES;
    //if (sample_index_c >= SINE_SAMPLES) sample_index_c = 0;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_HRTIM1_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  // fill up the sine table and everything else we need
  generateSineTables();

  // start all HRTIM channels and enable outputs
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TA1 | LL_HRTIM_OUTPUT_TA2);
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TB1 | LL_HRTIM_OUTPUT_TB2);
  LL_HRTIM_EnableOutput(HRTIM1, LL_HRTIM_OUTPUT_TC1 | LL_HRTIM_OUTPUT_TC2);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER | LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_CounterEnable(HRTIM1, LL_HRTIM_TIMER_MASTER | LL_HRTIM_TIMER_A | LL_HRTIM_TIMER_B | LL_HRTIM_TIMER_C);

  // start TIM6
  setSineFrequency(OUTPUT_FREQ_DEF);
  setOutputScaling(OUTPUT_SCALING_DEF);

  LL_TIM_ClearFlag_UPDATE(TIM6);
  LL_TIM_EnableIT_UPDATE(TIM6);
  LL_TIM_EnableCounter(TIM6);

  // start the first ADC2 conversion
  LL_ADC_REG_StartConversion(ADC2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float adcSpeedButton = 0.00;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // start a new ADC read for the user input
	  if(!LL_ADC_REG_IsConversionOngoing(ADC2)) {
		  // get the data and add it to the new value
		  float tempValue = (float)3.3*((float)LL_ADC_REG_ReadConversionData10(ADC2)/(float)1024);
		  if(SPEED_BUTTON_MIN > tempValue) {
			  tempValue = 0;
		  }
		  if(tempValue > SPEED_BUTTON_MAX) {
			  tempValue = SPEED_BUTTON_MAX;
		  }
		  adcSpeedButton = (tempValue*(1/SPEED_BUTTON_BETA)) + adcSpeedButton*(1.0-(1.0/SPEED_BUTTON_BETA));
		  LL_ADC_REG_StartConversion(ADC2);

		  // set the new frequency
		  uint16_t freqNew =(uint16_t)((float)(OUTPUT_FREQ_MAX-OUTPUT_FREQ_MIN)*adcSpeedButton/(SPEED_BUTTON_MAX)) + OUTPUT_FREQ_MIN;
		  setSineFrequency(freqNew);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(64000000);
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetHRTIMClockSource(RCC_CFGR3_HRTIM1SW_PLL);
  LL_RCC_SetADCClockSource(LL_RCC_ADC12_CLKSRC_PLL_DIV_1);
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

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC1 GPIO Configuration
  PA0   ------> ADC1_IN1
  PA1   ------> ADC1_IN2
  PA2   ------> ADC1_IN3
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_8B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_HRTIM_TRG1;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_ENABLE_3RANKS;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_2);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_2, LL_ADC_SINGLE_ENDED);

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_3);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_3, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**ADC2 GPIO Configuration
  PA4   ------> ADC2_IN1
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_Init 1 */
  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_10B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC2, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC2, &ADC_REG_InitStruct);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC2);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_1);
  LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SAMPLINGTIME_601CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC2, LL_ADC_CHANNEL_1, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC2_Init 2 */
  LL_ADC_StartCalibration(ADC2, LL_ADC_SINGLE_ENDED);
  while(LL_ADC_IsCalibrationOnGoing(ADC2)) {
	  //TODO - add time-out
  }
  LL_ADC_Enable(ADC2);
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_HRTIM1);

  /* HRTIM1 interrupt Init */
  NVIC_SetPriority(HRTIM1_Master_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(HRTIM1_Master_IRQn);

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  LL_HRTIM_ConfigDLLCalibration(HRTIM1, LL_HRTIM_DLLCALIBRATION_MODE_CONTINUOUS, LL_HRTIM_DLLCALIBRATION_RATE_14);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_ConfigADCTrig(HRTIM1, LL_HRTIM_ADCTRIG_1, LL_HRTIM_ADCTRIG_UPDATE_MASTER, LL_HRTIM_ADCTRIG_SRC13_MCMP1);
  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_PRESCALERRATIO_MUL4);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_MASTER, HRTIM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_MASTER, 0x00);
  LL_HRTIM_TIM_EnableHalfMode(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_MASTER);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_MASTER, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_MASTER);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_PRESCALERRATIO_MUL4);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_A, HRTIM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_A, 0x00);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_A);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_A, TA_START);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_PRESCALER_DIV1);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_A, DT_RISING);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_A, DT_FALLING);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_A, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_CROSSBAR_TIMPER);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_CROSSBAR_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TA2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_PRESCALERRATIO_MUL4);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_B, HRTIM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_B, 0x00);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_B);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_B, TB_START);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_DT_PRESCALER_DIV1);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_B, DT_RISING);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_B, DT_FALLING);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_B, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_CROSSBAR_TIMPER);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_CROSSBAR_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TB1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TB2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);

  /* Poll for DLL end of calibration */
#if (USE_TIMEOUT == 1)
  uint32_t Timeout = 10; /* Timeout Initialization */
#endif  /*USE_TIMEOUT*/

  while(LL_HRTIM_IsActiveFlag_DLLRDY(HRTIM1) == RESET){
#if (USE_TIMEOUT == 1)
    if (LL_SYSTICK_IsActiveCounterFlag())  /* Check Systick counter flag to decrement the time-out value */
    {
        if(Timeout-- == 0)
        {
          Error_Handler();  /* error management */
        }
    }
#endif  /* USE_TIMEOUT */
  }

  LL_HRTIM_TIM_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_PRESCALERRATIO_MUL4);
  LL_HRTIM_TIM_SetCounterMode(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_MODE_CONTINUOUS);
  LL_HRTIM_TIM_SetPeriod(HRTIM1, LL_HRTIM_TIMER_C, HRTIM_PERIOD);
  LL_HRTIM_TIM_SetRepetition(HRTIM1, LL_HRTIM_TIMER_C, 0x00);
  LL_HRTIM_TIM_SetDACTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DACTRIG_NONE);
  LL_HRTIM_TIM_DisableHalfMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableStartOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_DisableResetOnSync(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnablePreload(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetUpdateGating(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATEGATING_INDEPENDENT);
  LL_HRTIM_TIM_SetUpdateTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_UPDATETRIG_NONE|LL_HRTIM_UPDATETRIG_REPETITION);
  LL_HRTIM_TIM_SetResetTrig(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_RESETTRIG_NONE);
  LL_HRTIM_TIM_DisablePushPullMode(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_EnableDeadTime(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetBurstModeOption(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_BURSTMODE_MAINTAINCLOCK);
  LL_HRTIM_ForceUpdate(HRTIM1, LL_HRTIM_TIMER_C);
  LL_HRTIM_TIM_SetCompare1(HRTIM1, LL_HRTIM_TIMER_C, TC_START);
  LL_HRTIM_DT_SetPrescaler(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_PRESCALER_DIV1);
  LL_HRTIM_DT_SetRisingValue(HRTIM1, LL_HRTIM_TIMER_C, DT_RISING);
  LL_HRTIM_DT_SetRisingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_RISING_POSITIVE);
  LL_HRTIM_DT_SetFallingValue(HRTIM1, LL_HRTIM_TIMER_C, DT_FALLING);
  LL_HRTIM_DT_SetFallingSign(HRTIM1, LL_HRTIM_TIMER_C, LL_HRTIM_DT_FALLING_POSITIVE);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_CROSSBAR_TIMPER);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_CROSSBAR_TIMCMP1);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC1, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  LL_HRTIM_OUT_SetPolarity(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_POSITIVE_POLARITY);
  LL_HRTIM_OUT_SetOutputSetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetOutputResetSrc(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_CROSSBAR_NONE);
  LL_HRTIM_OUT_SetIdleMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_NO_IDLE);
  LL_HRTIM_OUT_SetIdleLevel(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_IDLELEVEL_INACTIVE);
  LL_HRTIM_OUT_SetFaultState(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_FAULTSTATE_NO_ACTION);
  LL_HRTIM_OUT_SetChopperMode(HRTIM1, LL_HRTIM_OUTPUT_TC2, LL_HRTIM_OUT_CHOPPERMODE_DISABLED);
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**HRTIM1 GPIO Configuration
  PB12   ------> HRTIM1_CHC1
  PB13   ------> HRTIM1_CHC2
  PA8   ------> HRTIM1_CHA1
  PA9   ------> HRTIM1_CHA2
  PA10   ------> HRTIM1_CHB1
  PA11   ------> HRTIM1_CHB2
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_8|LL_GPIO_PIN_9|LL_GPIO_PIN_10|LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_13;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM6);

  /* TIM6 interrupt Init */
  NVIC_SetPriority(TIM6_DAC1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM6_DAC1_IRQn);

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  TIM_InitStruct.Prescaler = 64;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  LL_TIM_Init(TIM6, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM6);
  LL_TIM_SetTriggerOutput(TIM6, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM6);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

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
