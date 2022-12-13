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
#include "stm32f3xx.h"
#include <button.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  // Read the current state of the limit switches and start button
	    limitXpState = digitalRead(limitXp);
	    limitYpState = digitalRead(limitYp);
	    limitXnState = digitalRead(limitXn);
	    limitYnState = digitalRead(limitYn);
	    bStartState = bStart.pressed();

	    // Mode Switch Logic and Mode Logic that has to happen once
	    switch (mode) {
	      case 0:
	        // Mode 1: Check if the system is safe to start
	        // Requirement: Button Start must be pressed
	        if (bStartState && !paused) {
	          Serial.println("Is System Safe to Start?");
	          lcd.clear();
	          lcd.setCursor(0,0);
	          lcd.print("Is System safe?");
	          lcd.setCursor(0,1);
	          lcd.print(mode, 1);
	          calibrated = false;
	          delay(200);

	          statusLed(2);
	          mode = mode + 1;
	        }
	        break;

	      case 1:
	      case 10:
	        // Mode 2: Calibrate the light barrier
	        // Requirement: Button Start must be pressed and the limit switches must not be pressed
	        if (((!limitXpState && !limitYpState && !limitXnState && !limitYnState && bStartState) || mode == 10) && !paused) {
	          Serial.println("Begin Light Barrier Calibration:");
	          lcd.clear();
	          lcd.setCursor(0,0);
	          lcd.print("Begin");
	          lcd.setCursor(0,1);
	          lcd.print("Calibration");

	          //attachInterrupt(digitalPinToInterrupt(limitXp), stopSystem, RISING);
	          //attachInterrupt(digitalPinToInterrupt(limitYp), stopSystem, RISING);
	          //attachInterrupt(digitalPinToInterrupt(limitXn), stopSystem, RISING);
	          //attachInterrupt(digitalPinToInterrupt(limitYn), stopSystem, RISING);

	          mode = 2;
	        }
	        break;

	      case 2:
	        // Mode 3: Divide the goods into boxes
	        // Requirement: The light barrier must be calibrated
	        if (calibrated && !paused) {
	          stepperX.disableOutputs();                  // Enables the Stepper Motors
	          Serial.println("Start of Box filling");

	          lcd.clear();
	          lcd.setCursor(0,0);
	          lcd.print("Start");
	          lcd.setCursor(0,1);
	          lcd.print("Box Filling");

	          statusLed(3);
	          stepperZ.setSpeed(moZDirection*moZSpeed);
	          stepperA.setSpeed(moADirection*moASpeed);
	          mode = mode + 1;
	        }
	        break;

	      case 3:
	        // Mode 4: Reference the rotation axis
	        // Requirement: The number of counted items must be greater than or equal to the maximum number of items per box
	        if (countVar >= countMax && !paused) {
	          stepperZ.setSpeed(0);
	          stepperA.setSpeed(0);

	          stepperY.setSpeed(-moYDirection*moYSpeed);

	          Serial.println("Box filled ... delivering...");

	          lcd.clear();
	          lcd.setCursor(0,0);
	          lcd.print("Box filled");
	          lcd.setCursor(0,1);
	          lcd.print("Start delivering");

	          //detachInterrupt(digitalPinToInterrupt(limitYn));

	          countVar = 0;
	          mode = mode + 1;
	        }
	        break;

	      case 4:
	        // Mode 5: Reference the lift axis
	        // Requirement: Limit Switch Rot reached
	        if (limitYnState && !paused) {
	          stepperY.setSpeed(0);
	          stepperY.setCurrentPosition(0);

	          stepperX.setSpeed(-moXDirection*moXSpeed);

	          //detachInterrupt(digitalPinToInterrupt(limitXn));

	          mode = mode + 1;
	        }
	        break;

	      case 5:
	        // Mode 6: Move to phiPos1
	        // Requirement: Limit Switch Lift reached
	        if (limitXnState && !paused) {
	          stepperX.setSpeed(0);
	          stepperX.setCurrentPosition(0);

	          stepperY.setSpeed(moYDirection*moYSpeed);

	          mode = mode + 1;
	        }
	        break;

	      case 6:
	        // Mode 7: Move to xPos1
	        // Requirement: Rotation Position 1 reached
	        if (abs(stepperY.currentPosition()) >= phiPos1 && !paused) {
	          stepperY.setSpeed(0);

	          stepperX.setSpeed(moXDirection*moXSpeed);

	          attachInterrupt(digitalPinToInterrupt(limitYn), stopSystem, RISING);

	          mode = mode + 1;
	        }
	        break;

	      case 7:
	        // Mode 8: Move to phiPo2
	        // Requirement: Lift Position 1 reached
	        if (abs(stepperX.currentPosition()) >= xPos1 && !paused) {
	          stepperX.setSpeed(0);

	          stepperY.setSpeed(moYDirection*moYSpeed);

	          attachInterrupt(digitalPinToInterrupt(limitXn), stopSystem, RISING);

	          mode = mode + 1;
	        }
	        break;

	      case 8:
	        // Mode 9: Move to xPos2
	        // Requirement: Rotation Position 2 reached
	        if (abs(stepperY.currentPosition()) >= phiPos2 && !paused) {
	          stepperY.setSpeed(0);

	          stepperX.setSpeed(-moXDirection*moXSpeed);

	          mode = mode + 1;
	        }
	        break;

	      case 9:
	        // Mode 10: Check Box count
	        // Requirement: Lift Position 2 reached
	        if (abs(stepperX.currentPosition()) <= xPos2 && !paused) {
	          stepperX.setSpeed(0);
	          countBox = countBox + 1;
	          calibrated = false;
	          idx = 0;
	          threshold = 0;
	          stepperX.enableOutputs();    // Disables All Steppers

	          if (countBox < countBoxMax){
	            Serial.print("Box delivered: ");
	            Serial.println(countBox);

	            lcd.clear();
	            lcd.setCursor(0,0);
	            lcd.print("Box delivered");
	            lcd.setCursor(0,1);
	            lcd.print("Box count: ");
	            lcd.print(countBox, 1);
	            mode = 10;
	          }
	          else{
	            mode = 0;
	            countBox = 0;
	            Serial.println("Last Box delivered");

	            lcd.clear();
	            lcd.setCursor(0,0);
	            lcd.print("All boxes");
	            lcd.setCursor(0,1);
	            lcd.print("delivered");

	            statusLed(1);
	          }
	        }
	        break;
	    }

	    if (!paused){
	      stepperX.runSpeed();
	      stepperY.runSpeed();
	      stepperZ.runSpeed();
	      stepperA.runSpeed();
	    }
	    else if (paused && bStartState){
	      paused = false;
	      if (mode > 2){
	        stepperX.disableOutputs();            // Enables the Stepper Motors
	        lcd.clear();
	      }
	    }

	    // Mode Logic that has to be run each cycle
	    if (mode == 2 && !paused){
	      barrierValue = analogRead(photoRes);
	      calibratePhotoresistor();              // call of calibration algorithm
	    }
	    else if (mode == 3 && !paused){
	      barrierValue = analogRead(photoRes);    // current light barrier sensor value
	      countGoods();                          // counter logic checks if light barrier detects goods
	    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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


// The calibratePhotoresistor() function calibrates the light barrier
// by calculating a threshold value that is used to determine when the barrier is triggered.
// The threshold is calculated as the arithmetic mean of `numCalibrate` measurements,
// with an offset defined by `thresholdOffset`.
//
// After `numCalibrate` calls to this function, the `calibrated` flag is set to `true`.
void calibratePhotoresistor()
{
  // Print the current barrier value to the serial console
  Serial.println(barrierValue);

  // Add the current barrier value to the threshold sum
  threshold = threshold + barrierValue;

  // Increment the index
  idx = idx + 1;

  // If the index has reached numCalibrate, calculate the mean threshold value
  // and set the calibrated flag to true
  if (idx >= numCalibrate){
    threshold = threshold / numCalibrate;

    Serial.println("Sensor calibrated");
    Serial.println(threshold);

    // Print a message to the LCD
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Sensor Calibrated");

    // Subtract the threshold offset
    threshold = threshold - thresholdOffset;
    calibrated = true;
  }

  // Delay for 100 milliseconds
  delay(100);
}


// The statusLed() function sets the state of the red, yellow, and green LEDs
// based on the value of the ledMode parameter.
//
// @param {number} ledMode - An integer representing the desired LED state.
//                            1 = red LED on, yellow and green LEDs off
//                            2 = yellow LED on, red and green LEDs off
//                            3 = green LED on, red and yellow LEDs off
//                            Any other value = all LEDs off
void statusLed(int ledMode)
{
  if (ledMode == 1)
  {
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
  }
  else if (ledMode == 2)
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledGreen, LOW);
  }
  else if (ledMode == 3)
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, HIGH);
  }
  else
  {
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
  }
}


// The countGoods() function handles the detection of goods using edge detection.
// If the barrier sensor value falls below the threshold, an item is counted.
// The function compares the current state of the sensor to its previous state,
// and increments the counter if the state has changed from LOW to HIGH.
//
// The current count value is printed to the serial console and LCD display.
void countGoods()
{
  // If the barrier sensor value is below the threshold, set the count state to HIGH
  if (barrierValue < threshold){
    countState = HIGH;
  }
  // Otherwise, set the count state to LOW
  else{
    countState = LOW;
  }

  // compare the countState to its previous state
  if (countState != countLastState) {
    // If the state has changed, increment the counter
    if (countState == HIGH)
    {
      // If the current state is HIGH, then the sensor went from off to on
      countVar++;

      // Print the current count value to the serial console
      Serial.print("Boxlevel: ");
      Serial.println(countVar);

      // Print the current count value to the LCD display
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Counting: ");
      lcd.setCursor(0,1);
      lcd.print("actual goods: ");
      lcd.print(countVar, 1);
    }
  }
  // Save the current state as the last state, for the next iteration of the loop
  countLastState = countState;
}


// The pauseSystem() function is an interrupt handler that is called when the pause button is pressed.
// It stops all the motors and saves the current state of the system.
// The system can be resumed by pressing the start button.
// The function also prints a message to the serial console and LCD display,
// and turns on the yellow LED to indicate that the system is paused.
void pauseSystem() {
  // Stop all the motors
  stepperX.enableOutputs();     // Disables the Stepper Motors
  paused = true;

  // Print a message to the serial console
  Serial.println("Pause");

  // Print a message to the LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Paused");

  // Turn on the yellow LED to indicate that the system is paused
  statusLed(2);
}


// The stopSystem() function is an interrupt handler that is called when the stop button is pressed.
// It stops all the motors, resets all the variables, and sets the system into the initial mode.
// The function also prints a message to the serial console and LCD display,
// and turns on the red LED to indicate that the system is stopped.
void stopSystem() {
  // Stop all the motors
  stepperX.setSpeed(0);
  stepperX.runSpeed();
  stepperY.setSpeed(0);
  stepperY.runSpeed();
  stepperZ.setSpeed(0);
  stepperZ.runSpeed();
  stepperA.setSpeed(0);
  stepperA.runSpeed();
  stepperX.enableOutputs();

  // Reset all the variables
  calibrated = false;
  idx = 0;
  threshold = 0;
  mode = 0;

  // Print a message to the serial console
  Serial.println("Stopp");

  // Print a message to the LCD display
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Stopped");

  // Turn on the red LED to indicate that the system is stopped
  statusLed(1);
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
