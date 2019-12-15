/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t buffer1[8];
uint8_t buffer2[8];
char bt_char[10];
char jet_char[15];
unsigned char rxData[1];
unsigned char rxData_j[1];
uint8_t rx_index=0, rx_index_j=0;
uint8_t hall_counter=0;
bool rwheel_rising = false;
bool lwheel_rising = false;

// DiffDrive Variables
#define minJoystick -50
#define maxJoystick 50
#define minSpeed 45
#define maxSpeed 127
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SR1 70
#define SL1 70

int nMotLeft, nMotRight;
bool toggleDirection = false;
bool brake = false;
char state = 'F';
int yEnd, lEnd;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t c1, c2;
char state;
uint8_t recieveFlag = 0, recieveFlag_jetson=0;
uint8_t doneFlag = 0;
float r_speed, l_speed;
unsigned int rwheel_counter = 0;
unsigned int lwheel_counter = 0;
unsigned char hall_counter_buffer [16];
unsigned int diff;
unsigned int start_of_ride;
TIM_HandleTypeDef* timer_hal = &htim4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);  
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

unsigned char *dec(unsigned x,  unsigned char *s)
{
    *--s = 0;
    if (!x) *--s = '0';
    for (; x; x/=10) *--s = '0'+x%10;
    return s;
}


void mapRange(int x, int y){

if(x==0&&y==0)
{
nMotRight = 0;
nMotLeft = 0;
return;
}

if (y >= 0 && x >= 0){

if(y >= x) {
toggleDirection = (state == 'B')? true: false;
state = 'F';
nMotLeft = SL1;
nMotRight = SR1;
}
else {
nMotRight = SR1;
nMotLeft = 0;
}
return;
}
if (y <= 0 && x <= 0){

if(x >= y) {
toggleDirection = (state == 'F')? true: false;
state = 'B';
nMotLeft = SL1;
nMotRight = SR1;
}
else {
nMotRight = 0;
nMotLeft = SL1;
}
return;
}

if (y >= 0 && x <= 0){

if(x*-1 > y) {
nMotLeft = SL1;
nMotRight = 0;
}
else {
toggleDirection = (state == 'B')? true: false;
state = 'F';
nMotRight = SR1;
nMotLeft = SL1;
}
return;
}

if (y <= 0 && x >= 0){
//toggleDirection = (state == 'F')? true: false;
//state = 'B';
if(x > y*-1) {
nMotLeft = 0;
nMotRight = SR1;
}
else {
toggleDirection = (state == 'F')? true: false;
 state = 'B';
nMotRight = SR1;
nMotLeft = SL1;
}
return;
}
}
int map(int v)
{
    int joystickRange = (maxJoystick - minJoystick);  
    int speedRange = (maxSpeed - minSpeed);

    return  (((v - minJoystick) * speedRange) / joystickRange) + minSpeed;
}


void joystickToDiff(int x, int y){
float rawLeft, rawRight;

if (x == 0 && y == 0){
nMotRight = 0;
nMotLeft = 0;
return;
}
float z = sqrt( x* x + y * y);

float rad = acos(abs(x) / z);

float angle = rad * 180 / 3.14;
float tcoeff = -1 + (angle / 90) * 2;
float turn = tcoeff * abs(abs(y) - abs(x));
turn = round(turn * 100) / 100;
float mov = MAX(abs(y), abs(x));

// First and third quadrant
if ((x >= 0 && y >= 0) || (x < 0 && y < 0)){
rawLeft = mov;
rawRight = turn;
}
else {
rawRight = mov;
rawLeft = turn;
}

// Reverse polarity
if (y < 0){
rawLeft = 0 - rawLeft;
rawRight = 0 - rawRight;
toggleDirection = (state == 'F') ? true: false;
state = 'B';
}
else{
toggleDirection = (state == 'B') ? true: false;
state = 'F';
}
// minJoystick, maxJoystick, minSpeed, maxSpeed
// Map the values onto the defined range
nMotRight = map(rawRight);
nMotLeft = map(rawLeft);
}


void MoveBase(){

HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
if(toggleDirection){ // Toggle Direction Right: PA1, Left: PA2
// buffer1[1] = 70;
// HAL_I2C_Master_Transmit(&hi2c1,0x90,buffer1,2,100);  
//
// buffer2[1] = 70;
// HAL_I2C_Master_Transmit(&hi2c2,0x90,buffer2,2,100);
// HAL_Delay(5000);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 1);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
HAL_Delay(500);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
toggleDirection = false;
}
if (brake) { // Brake Right : PA3, Left: PA4
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
brake = false;
}

buffer2[1] = nMotRight;
HAL_I2C_Master_Transmit(&hi2c2,0x90,buffer2,2,100);

//HAL_Delay(80);

buffer1[1] = nMotLeft;
HAL_I2C_Master_Transmit(&hi2c1,0x90,buffer1,2,100);  

// HAL_Delay(80);
// buffer2[1] = nMotRight;
// HAL_I2C_Master_Transmit(&hi2c2,0x90,buffer2,2,100);
//
}

int extractNumSigned(char bt_char[], int start, int end){
	int num = 0;
	int radixMultiple[] = {1, 10, 100};
	bool isSigned = false;

	if (bt_char[start] == '-'){
			start = start + 1;
			isSigned = true;
	}
	int numDigits = end - start;
	int indx = numDigits - 1;
	for (int i=start; i<end; i++){
		num += (bt_char[i] - '0') * radixMultiple[indx--];
	}
	if(isSigned){
		num*=-1;
	}
	return num;
}

int extractSpeedSigned(char jet_char[], int start, int end){

	bool isSigned = false;
	char sub[5];
	int j=0;
	
	if (jet_char[start] == '-'){
			start = start + 1;
			isSigned = true;
	}

	for (int i=start; i<end; i++){
		sub[j] = jet_char[i];
	}
	
	float f2 = strtof(sub, NULL);
	if(isSigned){
		f2*=-1;
	}
	return f2;
}
void readJoyStick(char bt_char[], int * x, int * y){
	
	int xEnd, xStart = 2;
	int yStart;

	for (int i= xStart; i<11; i++){
		if(bt_char[i] == 'Y')
			xEnd = i;
	}
	yStart = xEnd + 2;

	*x = extractNumSigned(bt_char, xStart, xEnd);
	*y = extractNumSigned(bt_char, yStart, yEnd);
	
	if(*x== 100 && *y == 100)
		brake = true;
}

void readSpeed(char jet_char[], float * r, float * l){
	
	int rEnd, rStart = 2;
	int lStart;

	for (int i= rStart; i<15; i++){
		if(jet_char[i] == 'l')
			rEnd = i;
	}
	lStart = rEnd + 2;

	*r = extractSpeedSigned(jet_char, rStart, rEnd);
	*l = extractSpeedSigned(jet_char, lStart, lEnd);

}

bool checkData(char bt_char[]){
bool foundX = (bt_char[0] == 'X');
bool foundEnd = false;
for (int i =0; i<11; i++ ){
if(bt_char[i] == 'Y')
foundEnd = true;
}
return (foundX && foundEnd);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart ->Instance == USART2)
	{
		if (rxData[0]==';')
		{
			yEnd = rx_index;
			rx_index=0;
			recieveFlag=1;
		}
		else if (rxData[0] == '\n' || rxData[0] == '\r'){}
		else
		{
			bt_char[rx_index++]=rxData[0];
		}
		HAL_UART_Receive_IT(&huart2,rxData,1);
	}
	
	if(huart ->Instance == USART1)
	{
		if (rxData_j[0]==';')
		{
			lEnd = rx_index_j;
			rx_index_j=0;
			recieveFlag_jetson=1;
		}
		else if (rxData_j[0] == '\n' || rxData_j[0] == '\r'){}
		else
		{
			jet_char[rx_index_j++]=rxData_j[0];
		}
		HAL_UART_Receive_IT(&huart1,rxData_j,1);
	}
}  

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)  
{
   if (GPIO_Pin==GPIO_PIN_1) //x:0-15
   {
			if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 1)
			{
				diff = __HAL_TIM_GET_COUNTER(timer_hal);

				if(diff > 100)  // else, just discard it
				{
					rwheel_counter +=1;
					__HAL_TIM_SET_COUNTER(&htim4, 0);
					sprintf(hall_counter_buffer,"r%dl%d;",rwheel_counter, lwheel_counter);
					//HAL_UART_Transmit_IT(&huart1, hall_counter_buffer,  16);
					//last_reading_time = __HAL_TIM_GET_COUNTER(&htim4);
				}
			}
		}

		if (GPIO_Pin==GPIO_PIN_0) //x:0-15
		{
				if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == 1)
				{
					diff = __HAL_TIM_GET_COUNTER(timer_hal);

					if(diff > 100)  // else, just discard it
					{
						lwheel_counter +=1;
						__HAL_TIM_SET_COUNTER(&htim4, 0);
						sprintf(hall_counter_buffer,"r%dl%d;",rwheel_counter, lwheel_counter);
						//HAL_UART_Transmit_IT(&huart1, hall_counter_buffer,  16);
						//last_reading_time = __HAL_TIM_GET_COUNTER(&htim4);
					}
				}
		 }

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

	
 
  /* USER CODE BEGIN Callback 1 */

  if (htim->Instance == timer_hal->Instance) {
		start_of_ride = 1;
		HAL_UART_Transmit_IT(&huart1, hall_counter_buffer,  16);
  }

  /* USER CODE END Callback 1 */
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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	// HAL_Delay(15);  
	// buffer[0]=0xFF; //Pointer buffer  
	// buffer[1]=100;
	// HAL_I2C_Master_Transmit(&hi2c1,0x90,buffer,2,100);
		/* USER CODE END 2 */

		/* Infinite loop */
		/* USER CODE BEGIN WHILE */
	state = 'F';
	buffer1[0]=0xFF;    
	buffer1[1]=80;
	buffer2[0]=0xFF;    
	buffer2[1]=80;
	//HAL_I2C_Master_Transmit(&hi2c1,0x90,buffer1,2,100);  
	//HAL_I2C_Master_Transmit(&hi2c2,0x90,buffer2,2,100);
	int x,y;
	HAL_TIM_Base_Start_IT(timer_hal);
	HAL_UART_Receive_IT(&huart1,rxData_j,1);
	HAL_UART_Receive_IT(&huart2,rxData,1);

  while (1)
  {
		if (recieveFlag)
		{
			//HAL_UART_Receive(&huart2, (uint8_t *)&bt_char,11, 100);  
			if(checkData(bt_char)){
				// HAL_UART_Transmit(&huart1, (uint8_t *)&bt_char,11,1);
				 HAL_Delay(100);
				readJoyStick(bt_char,&x, &y);
				mapRange(x,y);
				// joystickToDiff(x, y);
				MoveBase();
				// HAL_Delay(400);
			}
			recieveFlag=0;
		}
		
		if (recieveFlag_jetson)
		{
		
				HAL_Delay(100);
				readSpeed(jet_char,&r_speed, &l_speed);
				mapRange(r_speed,l_speed);
				MoveBase();
			
			recieveFlag_jetson=0;
		}
		// bt_char[0]= '\0';
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

  /** Initializes the CPU, AHB and APB busses clocks
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
  /** Initializes the CPU, AHB and APB busses clocks
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
  hi2c2.Init.ClockSpeed = 100000;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 35999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/