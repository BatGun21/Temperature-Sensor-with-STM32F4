/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx.h"
#include <string.h>
#include "stdio.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TARGET_TEMPERATURE 20.0// Target temperature
#define KP 27.0 // Proportional gain
#define KI 1.0// Integral gain
#define KD 0.0 // Derivative gain
#define PID_MAX_VALUE 100 // Maximum PWM value for the relay control
#define PID_MIN_VALUE 0 // Minimum PWM value for the relay control
#define DUTYCYCLE_MAX_VALUE 75 // per cent
#define DUTYCYCLE_MIN_VALUE 25 // per cent
#define Clock_Frequency 16000 // in KHz
#define GPIO_PORT_LEDS GPIOD
#define GPIO_PORT_ADC GPIOB
#define ADC_CHANNEL 9
#define ON 1
#define OFF 0
#define GPIO_PIN_Compressor_Relay 0x00000002 //PC1
#define GPIO_PIN_SWITCH GPIO_IDR_ID0 //PA0
#define GPIO_PORT_RELAY GPIOC
#define PWM_PERIOD 60000 // 60 seconds
#define MIN_DUTY_CYCLE 25 // per cent
#define MOVING_AVERAGE_SIZE 10
#define NOISE_THRESHOLD 3.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile float currentTemperature = 0.0; // Current temperature
volatile float error = 0.0; // Error term
volatile float integral = 0.0; // Integral term
volatile float derivative = 0.0; // Derivative term
volatile float pidOutput = 0.0; // PID output value for relay control
int counter = 0;
char msg2 [50];
char msg [20];
char msg3 [50];
char msg4 [50];
float temperatureBuffer[MOVING_AVERAGE_SIZE]; // Buffer for temperature readings
int bufferIndex = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
float Average(float array[], int size);
void PIDControlLoop(void);
uint16_t read_adc(uint8_t channel);
void ControlRelay(float dutyCycle);
void LED_init(void);
void DelayMSW(unsigned int time);
int time_expired (int delayTime, int currentTime);
void adc_init(void);
void SysTick_Init(uint32_t ticks);
void DelayMSW(unsigned int time);
void Voltage_Print(void);
void printTimestamp(void);
float adcValtoVolts (uint16_t adcVal);
void print_adcval(void);
float readTemperature(void);
void TemperaturePrint (void);
void ConfigureVoltageSourcePin(void);
void Config_RelayCTRL_Pin(void);
void Timer3_Init(void);
float Percentage(float currentValue, float maxValue);
void Timer2_Init(void);
void SetWaitPidLoopUpdate(void);
void SetWaitOneSec(void);
void print_pidOutpuVal(void);
float MovingAverage(float newValue);
float RemoveOutliers(float newValue, float movingAverage);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct Wait {
	int currentTime;
	int delayTime ;
	int activeFlag;
};

struct Wait pidLoopUpdateWait = {0, 15000, 0};
struct Wait OneSec = {0, 1000, 0};

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
  SysTick_Init(Clock_Frequency);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_init();
  adc_init();
  ConfigureVoltageSourcePin();
  Timer3_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  SetWaitPidLoopUpdate();

	  if (time_expired(pidLoopUpdateWait.delayTime, pidLoopUpdateWait.currentTime)){
		  PIDControlLoop();
		  pidLoopUpdateWait.activeFlag = 0;
	  }

	  SetWaitOneSec();

	  if (time_expired(OneSec.delayTime, OneSec.currentTime)){\
		  print_pidOutpuVal();
	  	  TemperaturePrint();
//	  	  Voltage_Print();
		  OneSec.activeFlag = 0;
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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void print_adcval(void){ // Debug
	uint16_t adcVal = read_adc(ADC_CHANNEL);
	sprintf(msg2, " ADC Value = %d ", adcVal);
	printTimestamp();
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg2), strlen(msg2), 200);
}

void print_pidOutpuVal(void){ // Debug
	sprintf(msg4, "%.3f,", pidOutput);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg4), strlen(msg4), 200);
}

void SetWaitPidLoopUpdate(void){
	  if (!pidLoopUpdateWait.activeFlag){
		  pidLoopUpdateWait.currentTime = counter;
		  pidLoopUpdateWait.activeFlag = 1;
	  }
}

void SetWaitOneSec(void){
	  if (!OneSec.activeFlag){
		  OneSec.currentTime = counter;
		  OneSec.activeFlag = 1;
	  }
}

void ControlRelay(float dutycycle) {

    uint32_t dutyValue = (uint32_t)((100 - dutycycle) * (PWM_PERIOD/ 100)); // NPN Transistor to relay

    // Set the duty cycle
    TIM3->CCR1 = dutyValue;
}

void PIDControlLoop(void) {
    currentTemperature = readTemperature();

    error = (currentTemperature - TARGET_TEMPERATURE);

    integral += error;

    derivative = error - derivative;

    pidOutput = KP * error + KI * integral + KD * derivative;

    // Limit the PWM output to the maximum value
    if (pidOutput > DUTYCYCLE_MAX_VALUE) {
        pidOutput = PID_MAX_VALUE;
    }
    // If too low value set to 0
    if (pidOutput < DUTYCYCLE_MIN_VALUE) {
        pidOutput = PID_MIN_VALUE;
    }
    pidOutput = Percentage(pidOutput, PID_MAX_VALUE);

    // Control the relay using PWM
    ControlRelay(pidOutput);
}

void Timer2_Init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Configure PA15 as alternate function (TIM2 CH1)
    GPIOA->MODER |= GPIO_MODER_MODER15_1;  // Alternate function mode
    GPIOA->AFR[1] |= 0x00002000;  // AF01 for PA15 (TIM2 CH1)
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;  // High speed
    GPIOA->OTYPER |= GPIO_OTYPER_OT_15;  // Open-Drain

    // TIM2
    TIM2->PSC = (Clock_Frequency / 1000) - 1;  // Prescaler for 1ms tick
    TIM2->ARR = PWM_PERIOD - 1;
    TIM2->CCR1 = (uint32_t) (MIN_DUTY_CYCLE * PWM_PERIOD / 100);  // Initial duty cycle

    // Configure TIM2 CH1 for PWM mode 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM2->CCER |= TIM_CCER_CC1E;  // Enable capture/compare channel 1

    // Trigger an update event to load new values
    TIM2->EGR |= TIM_EGR_UG;

    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
}

void Timer3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // PC6 as alternate function (TIM3 CH1)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIO_PORT_RELAY->MODER |= GPIO_MODER_MODER6_1;  // Alternate function mode
    GPIO_PORT_RELAY->AFR[0] |= 0x02000000;  // AF02 for PC6 (TIM3 CH1)
    GPIO_PORT_RELAY->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6;  // High speed
    GPIO_PORT_RELAY->OTYPER |= 0x0040; // Open Drain

    // TIM3
    TIM3->PSC = Clock_Frequency - 1;  // Prescaler to achieve 1ms tick with 16MHz clock
    TIM3->ARR = PWM_PERIOD - 1;  // Auto-reload value
    TIM3->CCR1 = (uint16_t) (MIN_DUTY_CYCLE * PWM_PERIOD / 100);  // Initial duty cycle

    // Configure TIM3 CH1 for PWM mode 1
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
    TIM3->CCER |= TIM_CCER_CC1E;  // Enable capture/compare channel 1

    // Trigger an update event to load new values
    TIM3->EGR |= TIM_EGR_UG;

    TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3
}

float Average(float array[], int size) {
    float sum = 0.0;
    float count = 0;
    for (int i = 0; i < size; i++) {
        if (array[i] != 0) {
            sum += array[i];
            count++;
        }
    }
    // Avoid division by zero
    if (count == 0) {
        return 0.0;
    }
    return sum / count;
}

void ConfigureVoltageSourcePin(void) {
    // Enable the GPIO port clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure PC1 as general purpose output
    GPIO_PORT_RELAY->MODER |= GPIO_MODER_MODER1_0;

    // Configure PC1 as open drain
    GPIO_PORT_RELAY->OTYPER |= GPIO_OTYPER_OT_1;

    // Configure PC1 to high speed
    GPIO_PORT_RELAY->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
}

void LED_init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; // Enabling Clock for GPIOD
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER14_0; //Set bit 0 to 1 Red
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER15_0; //Set bit 0 to 1 Blue
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER13_0; //Set bit 0 to 1 Orange
	GPIO_PORT_LEDS->MODER |= GPIO_MODER_MODER12_0; //Set bit 0 to 1 Green
}

int time_expired (int delayTime, int currentTime){
	int timeExpiredFlag = 0;
	if (counter> currentTime+delayTime){
		timeExpiredFlag = 1;
	}else{
		timeExpiredFlag = 0;
	}
	return timeExpiredFlag;
}

void adc_init(void) {
    // Enable the ADC1 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Enable the GPIOB clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Configure PB1 as analog input
    GPIO_PORT_ADC->MODER |= GPIO_MODER_MODER1; // Analog mode
    GPIO_PORT_ADC->OTYPER |= GPIO_OTYPER_OT1; // Open Drain PB1

    // Configure ADC settings
    ADC1->CR1 &= ~ADC_CR1_RES; // Clear the RES bits for 12-bit resolution
    ADC1->CR2 &= ~ADC_CR2_ALIGN; // Data right-aligned
    ADC1->CR2 |= ADC_CR2_CONT; // Continuous conversion mode
    ADC1->SQR3 &= ~ADC_SQR3_SQ1; // Clear the SQ1 bits
    ADC1->SQR3 |= 9 << ADC_SQR3_SQ1_Pos; // Set the channel number in SQ1 bits (Channel 9 for PB1)
    ADC1->SQR3 |= 8 << ADC_SQR3_SQ1_Pos; // Set the channel number in SQ1 bits (Channel 8 for PB0)
    // Enable the ADC
    ADC1->CR2 |= ADC_CR2_ADON;

    // Wait for ADC to be ready
    DelayMSW(100);

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

uint16_t read_adc(uint8_t channel) {
    ADC1->SQR3 = (channel & 0x1F);  // Assuming channel is less than 16

    // Start the conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for the end of conversion
    while (!((ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC)) {}

    // Read the converted value
    uint16_t result = ADC1->DR;

    return result;
}

void SysTick_Init(uint32_t ticks){

	SysTick->CTRL = 0; // Disable SysTick

	SysTick->LOAD = ticks-1; // Set Reload Register

	// Setting Interrupt Priority to the highest
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS)-1);

	SysTick->VAL = 0; // Reset the SysTick counter value

	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk; // Selecting internal clock source
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enabling SysTick exception Request when 0


	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // Enable SysTick
}

void DelayMSW(unsigned int time){
	for(int i=0; i<=time; i++){
		while ((SysTick->CTRL & 0x00010000) == 0){
				//Wait for 1 millisec.
		}
	}
}

void SysTick_Handler(void) {

	if (counter == 0xffffffff) {
        counter = 0; // Reset the counter if the maximum value is reached
    } else {
        counter++; // Increment the counter
    }
}

void Voltage_Print(void){ // Debug function
	uint16_t adcVal = read_adc(ADC_CHANNEL);
	float Vin = adcValtoVolts(adcVal);
	sprintf(msg2, "Vol = %.3f/n ", Vin);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg2), strlen(msg2), 200);
}

void printTimestamp(void) { // Debug
	sprintf(msg, "%d,", counter);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg), strlen(msg), 200);
}

float adcValtoVolts (uint16_t adcVal){
	float Vin = (adcVal/4096.0)*2.9;
//	Vin = Vin*(48.0/2.70); //Correction for Voltage divider for 48V
//	Vin += (0.6/30.0)*Vin; //Correction using observation
	return Vin;
}

void TemperaturePrint (void){ //Debug
	float temp_in = readTemperature();
	sprintf(msg3, "%.1f,", temp_in);
	HAL_UART_Transmit(&huart2, (uint8_t*)(msg3), strlen(msg3), 200);
}

float readTemperature(void) {

	uint16_t adcVal = read_adc(ADC_CHANNEL);
	float voltage = adcValtoVolts(adcVal);

    // Coefficients of the polynomial equation
    const float coefficients[] = { 13000.8914445 ,  -99474.67447538,  285225.49823225, -363335.0395916 ,  173570.90571022}; // Range is 25 deg C to 60 deg C

    float result = 0.0;
    for (int i = 0; i <= 4; ++i) {
        result += coefficients[i] * powf(voltage, 4 - i);
    }

    // Apply the moving average filter
    float movingAverage = MovingAverage(result);

    // Remove outliers
    float filteredTemperature = RemoveOutliers(result, movingAverage);

    return filteredTemperature;
}

float RemoveOutliers(float newValue, float movingAverage) {

    if (fabs(newValue - movingAverage) > NOISE_THRESHOLD) {
        return movingAverage;
    } else {
        return newValue;
    }
}

float MovingAverage(float newValue) {

    temperatureBuffer[bufferIndex] = newValue;

    bufferIndex = (bufferIndex + 1) % MOVING_AVERAGE_SIZE;

    float sum = 0.0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        sum += temperatureBuffer[i];
    }
    float movingAverage = sum / MOVING_AVERAGE_SIZE;

    return movingAverage;
}

float Percentage(float currentValue, float maxValue) {
    if (maxValue == 0.0) {
        // Avoid division by zero
        return 0.0;
    }
    float percentage = (currentValue / maxValue) * 100.0;

    return percentage;
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
