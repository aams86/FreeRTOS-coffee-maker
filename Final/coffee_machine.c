/*
 * coffee_machine.c
 *
 *  Created on: May 28, 2022
 *      Author: aaronstafford
 */
#include "main.h"
#include "coffee_machine.h"
#include "cmsis_os.h"
#include "sounds.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

machineState machineStatus = OFF;

size mode = SMALL;
char sizes[][8] = {"Small", "Medium", "Large", "XL"};
char machineStates[][16] = {"Brewing", "Keeping Warm"};

uint32_t brewStart;
uint32_t warmStart;
uint32_t onStart;
uint16_t waterLevel;

ADC_HandleTypeDef *hadc;

TaskHandle_t xLEDUpdateTaskHandle = NULL;
TaskHandle_t xOLEDTaskHandle = NULL;
TaskHandle_t xShutdownTaskHandle = NULL;

void vOLEDManagerTask(void const * argument);
void vCoffeeMakerShutdownTask(void const * argument);
void vControlCoffeeMakerTask(void const * argument);
void vUpdateLEDs(void const * argument);
void displayTime();
void displayMachineState();
void displayMode();
void displayWaterLevel();
void SSD1306_UpdateScreen();




void prvTimeoutTimer(xTimerHandle xTimer) {
	switch(machineStatus) {
	case ON:
		xSemaphoreGive(xOffSemaphore);
		break;
	case BREWING:
		xSemaphoreGive(xWarmingSemaphore);
		break;
	case WARMING:
		xSemaphoreGive(xOnSemaphore);
		break;
	case OFF:
		break;
	}
}

void setCoffeeMakerMode(size size_setting) {
	mode = size_setting;
}


void prvReadTempSensorTimer(xTimerHandle xTimer) {
	xSemaphoreGive(xOLEDUpdateSemaphore);
}
void prvReadWaterLevelSensorTimer(xTimerHandle xTimer) {
	  HAL_ADC_Start(hadc);
	  HAL_ADC_PollForConversion(hadc, 100);
	  uint32_t adcResult = HAL_ADC_GetValue(hadc);
	  uint16_t level = 100 * (adcResult - SENSOR_MIN) / (SENSOR_MAX - SENSOR_MIN);
	  if (level > 100)
		  level = 100;
	  if (level < 0)
		  level = 0;
	  waterLevel = level;
	  xSemaphoreGive(xOLEDUpdateSemaphore);
}

void CoffeeMaker_init(ADC_HandleTypeDef *_hadc) {
	hadc = _hadc;
}

void createCoffeeMakerTasks() {
	xTaskCreate((TaskFunction_t)vCoffeeMakerResetTask, "CoffeeMakerResetTask", 128, NULL, 2, NULL);
	xTaskCreate((TaskFunction_t)vControlCoffeeMakerTask, "CoffeeMakerControlTask", 128, NULL, 1, NULL);
	xTaskCreate((TaskFunction_t)vOLEDManagerTask, "OLEDManagerTask", 128, NULL, 1, &xOLEDTaskHandle);
	xTaskCreate((TaskFunction_t)vUpdateLEDs, "UpdateIndicatorLEDs", 128, NULL, 1, &xLEDUpdateTaskHandle);
	xTaskCreate((TaskFunction_t)vCoffeeMakerShutdownTask, "CoffeeMakerShutdownTask", 128, NULL, 1, &xShutdownTaskHandle);
	vTaskSuspend(xOLEDTaskHandle);
	vTaskSuspend( xShutdownTaskHandle );
}

machineState getMachineState() {
	return machineStatus;
}

void vControlCoffeeMakerTask(void const * argument) {
	BaseType_t xSemaphoreReceived;
	for(;;) {
		switch(machineStatus) {
		case ON:
			//turn off relay
			HAL_GPIO_WritePin(ARD_A0_GPIO_Port, ARD_A0_Pin, RESET);
			if(xSemaphoreTake(xBrewingSemaphore, portMAX_DELAY)==pdTRUE) {
				machineStatus = BREWING;
				HAL_GPIO_WritePin(ARD_A0_GPIO_Port, ARD_A0_Pin, SET);
			}
			break;
		case BREWING:
			//config brew settings
			xTimerChangePeriod( xTimeoutTimer, pdMS_TO_TICKS(BREW_DELAY), 50 );
			xSemaphoreReceived = xSemaphoreTake(xWarmingSemaphore, portMAX_DELAY);
			xTimerChangePeriod( xTimeoutTimer, pdMS_TO_TICKS(WARM_DELAY), 50 );
			machineStatus = WARMING;
			break;
		case WARMING:

			break;
		default:
			break;
		}
	}
}

void vCoffeeMakerResetTask(void const * argument) {
	for(;;) {
		BaseType_t xSemaphoreReceived = xSemaphoreTake(xOnSemaphore, portMAX_DELAY);
		//start timeout timer
		if(machineStatus == OFF) {
			xTimerStart(xBeatTimer,0);
			xTimerStart(xReadWaterLevelSensorTimer,0);
			xTimerStart(xPollGPIOTimer,0);
			vTaskResume(xOLEDTaskHandle);
			vTaskResume(xLEDUpdateTaskHandle);
			vTaskResume(xShutdownTaskHandle);
		}
		machineStatus = ON;
		xTimerChangePeriod( xTimeoutTimer, pdMS_TO_TICKS(ON_DELAY), 50 );
		xSemaphoreGive(xChangeSizeSemaphore);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);


	}
}

void vCoffeeMakerShutdownTask(void const * argument) {
	for(;;) {
		BaseType_t xSemaphoreReceived = xSemaphoreTake(xOffSemaphore, portMAX_DELAY);
		machineStatus = OFF;
		xTimerStop(xReadWaterLevelSensorTimer, 100);
		xTimerStop(xPollGPIOTimer,0);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);


		HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ARD_D11_GPIO_Port, ARD_D11_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);
		vTaskSuspend(xLEDUpdateTaskHandle);


		xSemaphoreTake(xOLEDMutex,portMAX_DELAY);
		vTaskSuspend(xOLEDTaskHandle);
		ssd1306_Fill(Black);
		ssd1306_UpdateScreen();
		xSemaphoreGive(xOLEDMutex);
		vTaskSuspend( NULL );

	}
}


void vOLEDManagerTask(void const * argument) {
	for(;;) {
		if(xSemaphoreTake(xOLEDUpdateSemaphore, portMAX_DELAY) == pdTRUE) {
			if(xSemaphoreTake(xOLEDMutex,0) == pdTRUE) {
			  //clear LCD
			  ssd1306_Fill(Black);

			  //display the time on the LCD
			  displayTime();
			  //display the machine State on the LCD
			  displayMachineState();
			  //display the size selection on the LCD
			  displayMode();
			  //display the water level on the LCD
			  displayWaterLevel();
			  //updates the LCD with the new information
			  ssd1306_UpdateScreen();
			  HAL_Delay(50);
			  xSemaphoreGive(xOLEDMutex);
			}
		}
	}
}

void displayMode(void) {
		ssd1306_SetCursor(5, 50);
		ssd1306_WriteString(sizes[mode], Font_7x10, White);
}



//Function displays the time on the LCD (note that the value for time is static currently)
void displayTime(void) {
	  ssd1306_SetCursor(2, 17);
	  ssd1306_WriteString("07:35", Font_16x26, White);
	  ssd1306_SetCursor(70, 2);
	  ssd1306_WriteString("AM", Font_7x10, White);
}

//when the machine is in brewing or warming mode, it displays the status on the LCD
void displayMachineState(void) {
	if(machineStatus == BREWING) {
		ssd1306_SetCursor(58, 50);
		ssd1306_WriteString("Brewing", Font_7x10, White);
	} else 	if(machineStatus == WARMING) {
		ssd1306_SetCursor(58, 50);
		ssd1306_WriteString("Warming", Font_7x10, White);
	}
}

void displayWaterLevel(void) {
	uint16_t xCorner = 112;
	uint16_t yCorner = 20;
	static uint16_t filledHeight = 0;
	uint16_t rectHeight = 35;
	uint16_t rectWidth = 8;
	//set height of bar indicating water level
	filledHeight = waterLevel * rectHeight/100;

	ssd1306_SetCursor(106, 5);
	ssd1306_WriteString("H2O", Font_7x10, White);
	//draw rectangle outline (add some padding so there is gap between outline and fill
	ssd1306_DrawRectangle(xCorner-2, yCorner-2, xCorner+rectWidth+4, yCorner+rectHeight+4, White);
	//draw filled bar
	ssd1306_DrawRectangle(xCorner, yCorner + rectHeight - filledHeight, xCorner+rectWidth, yCorner + rectHeight, White);
}


void prvPollGPIOTimer(xTimerHandle xTimer) {

	if(HAL_GPIO_ReadPin(ARD_D3_GPIO_Port, ARD_D3_Pin) == 0) {
		xSemaphoreGive(xBrewingSemaphore);
	}
	if(HAL_GPIO_ReadPin(ARD_D6_GPIO_Port, ARD_D6_Pin) == 0) {
		setCoffeeMakerMode(XLARGE);
		xSemaphoreGive(xChangeSizeSemaphore);
	}
	if(HAL_GPIO_ReadPin(ARD_D5_GPIO_Port, ARD_D5_Pin) == 0) {
		setCoffeeMakerMode(SMALL);
		xSemaphoreGive(xChangeSizeSemaphore);
	}
	if(HAL_GPIO_ReadPin(ARD_A1_GPIO_Port, ARD_A1_Pin) == 0) {
		setCoffeeMakerMode(LARGE);
		xSemaphoreGive(xChangeSizeSemaphore);
	}
	if(HAL_GPIO_ReadPin(ARD_D8_GPIO_Port, ARD_D8_Pin) == 0) {
		setCoffeeMakerMode(MEDIUM);
		xSemaphoreGive(xChangeSizeSemaphore);
	}
}

void vUpdateLEDs(void const * argument)
{
	for(;;) {
		if(xSemaphoreTake(xChangeSizeSemaphore, portMAX_DELAY)==pdTRUE) {
		  if (mode == MEDIUM) {
			  HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(ARD_D11_GPIO_Port, ARD_D11_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);

		  } else if(mode == SMALL) {
			  HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D11_GPIO_Port, ARD_D11_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);

		  } else if(mode == LARGE) {
			  HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D11_GPIO_Port, ARD_D11_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_RESET);

		  } else {
			  HAL_GPIO_WritePin(ARD_D13_GPIO_Port, ARD_D13_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D11_GPIO_Port, ARD_D11_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D10_GPIO_Port, ARD_D10_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(ARD_D9_GPIO_Port, ARD_D9_Pin, GPIO_PIN_SET);
		  }
		  xSemaphoreGive(xOLEDUpdateSemaphore);
		}
	}
}

/*
void controlCoffeeMaker() {
	  //machine status state machine
	  switch(machineStatus)
	  {
	  //in on case, turn LED on, turn relay off, set to off after ON_DELAY milliseconds
	  	case ON:
	  		HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
	  		HAL_GPIO_WritePin(Relay_D8_GPIO_Port, Relay_D8_Pin, GPIO_PIN_RESET);
	        if (HAL_GetTick() - onStart > ON_DELAY) {
	        	machineStatus = OFF;
	        }
	  		break;
	  	//in brewing case, turn relay on, switch to warming after brew_delay or when water is empty
	  	case BREWING:
	  		HAL_GPIO_WritePin(Relay_D8_GPIO_Port, Relay_D8_Pin, GPIO_PIN_SET);
	  		if (HAL_GetTick() - brewStart > BREW_DELAY || getWaterLevel() == 0) {
	  			machineStatus = WARMING;
	  			warmStart = HAL_GetTick();

	  		}
	  		break;
	  	//keep machine warm until warm delay, then switch to on state
	  	case WARMING:
	        if (HAL_GetTick() - warmStart > WARM_DELAY) {
	        	machineStatus = ON;
	        	onStart = HAL_GetTick();
	        }
	  		break;
	  	default:
	  		//off state, only time is displayed, relay is shut off
	  		HAL_GPIO_WritePin(Relay_D8_GPIO_Port, Relay_D8_Pin, GPIO_PIN_RESET);
	  		HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
	  		break;
	  }


	  //display temperature and water level through UART
	  displayUart(1000);

}
//Function draws a bar on LCD and fills it to the percentage that corresponds to the current water level, only updates the water level every
//500 ms, and if the machine is not in the OFF state



//when the machine is not in the off state, displays the selected cup size on LCD
static void displayMode(void) {
	if(machineStatus != OFF) {
		SSD1306_GotoXY(5, 50);
		SSD1306_Puts(sizes[mode], &Font_7x10, SSD1306_COLOR_WHITE);
	}
}

//function reads the adc to get the water level, and converts it to a percentage, returning the percentage
static uint16_t getWaterLevel(void) {
	  //get water level
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  uint32_t adcResult = HAL_ADC_GetValue(&hadc1);
	  uint16_t level = 100 * (adcResult - SENSOR_MIN) / (SENSOR_MAX - SENSOR_MIN);
	  if (level > 100)
		  level = 100;
	  if (level < 0)
		  level = 0;
	  return level;
}




//function to display water level and temperature on the UART
static void displayUart(uint16_t uartMessageDelay) {
	static uint32_t lastMessageTime = 0;

	if(HAL_GetTick() - lastMessageTime > uartMessageDelay) {
		lastMessageTime = HAL_GetTick();
		uint16_t waterLevel = getWaterLevel();
		float temp = BSP_TSENSOR_ReadTemp();
		char message[100];
		snprintf(message, sizeof(message), "Water level reading %%: %d, Water Temperature: %f\r\n", waterLevel, temp);
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 200);
	}
}
//this function polls all the size setting buttons and sets appropriate state while turning on the corresponding LED
static void checkMode(void)
{
	//read status of GPIOs
	  GPIO_PinState button1State = !HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
	  GPIO_PinState button2State = !HAL_GPIO_ReadPin(Button2_GPIO_Port, Button2_Pin);
	  GPIO_PinState button3State = !HAL_GPIO_ReadPin(Button3_GPIO_Port, Button3_Pin);
	  GPIO_PinState button4State = !HAL_GPIO_ReadPin(Button4_GPIO_Port, Button4_Pin);

	  //check to see if a button is pressed
	  if(button1State)
		  mode = SMALL;
	  if(button2State)
		  mode = MEDIUM;
	  if(button3State)
		  mode = LARGE;
	  if(button4State)
		  mode = XLARGE;

	  //set the corresponding LEDs
	  if (mode == SMALL) {
		  HAL_GPIO_WritePin(LED1_D4_GPIO_Port, LED1_D4_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED2_D5_GPIO_Port, LED2_D5_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED3_D6_GPIO_Port, LED3_D6_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED4_D7_GPIO_Port, LED4_D7_Pin, GPIO_PIN_RESET);

	  } else if(mode == MEDIUM) {
		  HAL_GPIO_WritePin(LED1_D4_GPIO_Port, LED1_D4_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_D5_GPIO_Port, LED2_D5_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED3_D6_GPIO_Port, LED3_D6_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED4_D7_GPIO_Port, LED4_D7_Pin, GPIO_PIN_RESET);

	  } else if(mode == LARGE) {
		  HAL_GPIO_WritePin(LED1_D4_GPIO_Port, LED1_D4_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_D5_GPIO_Port, LED2_D5_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED3_D6_GPIO_Port, LED3_D6_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LED4_D7_GPIO_Port, LED4_D7_Pin, GPIO_PIN_RESET);

	  } else {
		  HAL_GPIO_WritePin(LED1_D4_GPIO_Port, LED1_D4_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED2_D5_GPIO_Port, LED2_D5_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED3_D6_GPIO_Port, LED3_D6_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LED4_D7_GPIO_Port, LED4_D7_Pin, GPIO_PIN_SET);
	  }
}
*/
