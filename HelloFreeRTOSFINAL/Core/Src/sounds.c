/*
 * sounds.c
 *
 *  Created on: May 28, 2022
 *      Author: aaronstafford
 */
#include "sounds.h"

#include "timers.h"
#include "event_groups.h"
#include "cmsis_os.h"

note_status_t xNoteStatus = PAUSE;


int calcARR(TIM_HandleTypeDef *htim, int frequency) {
	return TIMER_FREQUENCY/(htim->Instance->PSC + 1)/frequency - 1;
}



void vBuzzerTask(void const * argument) {
	TIM_HandleTypeDef *ptimerHandle = ((buzzerDataType *)argument)->timerHandle;
	uint32_t PWMChannel = ((buzzerDataType *)argument)->PWMChannel;
	uint8_t buzzerNumber = ((buzzerDataType *)argument)->buzzerID;
	uint16_t *notes = ((buzzerDataType *)argument)->buzzerNotes;
	uint16_t *delays = ((buzzerDataType *)argument)->buzzerLengths;
	uint16_t numNotes = ((buzzerDataType *)argument)->numNotes;
	note_status_t buzzer_status = PAUSE;
	EventBits_t xEventGroupValue;
	int currentNote = 0;
	int noteLength = delays[currentNote];
	for(;;) {
		uint8_t clearBits = (buzzerNumber == LOW_PRIORITY_BUZZER_ID);
		xEventGroupValue = xEventGroupWaitBits(xNoteEventGroup,
											   SIXTEENTH_NOTE_BIT,
											   clearBits, //do not clear bits
											   pdTRUE, // Wait for all bits
											   portMAX_DELAY);
		if(buzzer_status == PAUSE) {
			buzzer_status = SOUND_ON;
			int current_arr = calcARR(ptimerHandle, notes[currentNote]);
			ptimerHandle->Instance->ARR = current_arr;
			__HAL_TIM_SET_COMPARE(ptimerHandle, PWMChannel, ((current_arr + 1)/2)-1);
			//ptimerHandle->Instance->CCR2 = ((current_arr + 1)/2)-1;
			HAL_TIM_PWM_Start(ptimerHandle, PWMChannel);
		}
		noteLength--;
		if(noteLength <= 0) {
			currentNote++;
			buzzer_status = PAUSE;
			HAL_TIM_PWM_Stop(ptimerHandle, PWMChannel);
			if(currentNote >= numNotes) {
				currentNote = 0;
				noteLength = delays[currentNote];
				if(buzzerNumber == LOW_PRIORITY_BUZZER_ID) {
					xTimerStop(xBeatTimer,portMAX_DELAY);
				}
			} else {
				noteLength = delays[currentNote];
			}
		}
	}
}
