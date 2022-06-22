/*
 * sounds.h
 *
 *  Created on: May 28, 2022
 *      Author: aaronstafford
 */

#ifndef INC_SOUNDS_H_
#define INC_SOUNDS_H_

#include "cmsis_os.h"
#include "timers.h"
#include "event_groups.h"
#include "stm32l4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *timerHandle;
	uint32_t PWMChannel;
	uint8_t buzzerID;
	uint16_t *buzzerNotes;
	uint16_t *buzzerLengths;
	uint16_t numNotes;
} buzzerDataType;

typedef enum  {
	SOUND_ON,
	PAUSE
} note_status_t;

#define SIXTEENTH_NOTE_BIT (1UL << 0UL)
#define NOTE_GAP_BIT (1UL << 1UL)

#define BPM 75
#define NOTE_GAP_MS 10
#define PERIOD_SIXTEENTH_MS (BPM*1000/60/16)
#define LOW_PRIORITY_BUZZER_ID 1
#define TIMER_FREQUENCY 80000000

TimerHandle_t xBeatTimer;
uint8_t noteCount;
EventGroupHandle_t xNoteEventGroup;


void vBuzzerTask(void const * argument);

#endif /* INC_SOUNDS_H_ */
