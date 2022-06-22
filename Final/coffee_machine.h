/*
 * coffee_machine.h
 *
 *  Created on: May 28, 2022
 *      Author: aaronstafford
 */

#ifndef INC_COFFEE_MACHINE_H_
#define INC_COFFEE_MACHINE_H_


#include "cmsis_os.h"
#include "timers.h"
#include "event_groups.h"
#include "stm32l4xx_hal.h"

#define MACHINE_ON_BIT (1UL << 0UL)
#define MACHINE_SHUTDOWN_BIT (1UL << 1UL)
#define MACHINE_BREW_BIT (1UL << 2UL)
#define MACHINE_WARMING_BIT (1UL << 3UL)

EventGroupHandle_t xcoffeeMachineEventGroup;
SemaphoreHandle_t xOnSemaphore;
SemaphoreHandle_t xOffSemaphore;
SemaphoreHandle_t xWarmingSemaphore;
SemaphoreHandle_t xBrewingSemaphore;
SemaphoreHandle_t xChangeSizeSemaphore;
SemaphoreHandle_t xOLEDUpdateSemaphore;
SemaphoreHandle_t xOLEDMutex;
typedef enum {
    SMALL = 0,
    MEDIUM = 1,
    LARGE = 2,
    XLARGE = 3
} size;

typedef enum {
    OFF,
    ON,
    BREWING,
    WARMING
} machineState;

TimerHandle_t xTimeoutTimer;
TimerHandle_t xPollGPIOTimer;
TimerHandle_t xReadWaterLevelSensorTimer;
TimerHandle_t xReadTempSensorTimer;

#define SENSOR_MIN 400
#define SENSOR_MAX 1400
#define BREW_DELAY 6000
#define WARM_DELAY 8000
#define ON_DELAY 30000
#define DEBOUNCE_TIME 200

void prvTimeoutTimer(xTimerHandle xTimer);
void prvReadTempSensorTimer(xTimerHandle xTimer);
void prvReadWaterLevelSensorTimer(xTimerHandle xTimer);
void prvPollGPIOTimer(xTimerHandle xTimer);
void vCoffeeMakerResetTask(void const * argument);
void createCoffeeMakerTasks();
void CoffeeMaker_init(ADC_HandleTypeDef *_hadc);
void setCoffeeMakerMode(size size_setting);

machineState getMachineState();

#endif /* INC_COFFEE_MACHINE_H_ */
