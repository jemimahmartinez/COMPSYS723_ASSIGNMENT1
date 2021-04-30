// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h> //for usleep
#include "system.h"
#include "sys/alt_irq.h"

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include <altera_avalon_pio_regs.h>

// IO includes
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"

// Definition of Task Stacks
#define TASK_STACKSIZE 2048

// Definition of Task Priorities
#define LOAD_CTRL_TASK_PRIORITY 1
#define STABILITY_MONITOR_TASK_PRIORITY 1
#define SWITCH_POLLING_TASK_PRIORITY 2
#define LED_HANDLER_TASK_PRIORITY 3
#define VGA_DISPLAY_TASK_PRIORITY 1

// Definition of queues
#define MSG_QUEUE_SIZE 30
QueueHandle_t msgqueue;
#define LOAD_CTRL_QUEUE_SIZE 100
QueueHandle_t loadCtrlQ;
#define NEW_FREQ_QUEUE_SIZE 100
QueueHandle_t signalFreqQ;

// Definition of Semaphores
xSemaphoreHandle stabilitySemaphore;
xSemaphoreHandle loadSemaphore;
xSemaphoreHandle shedSemaphore;
xSemaphoreHandle ledStatusSemaphore;
xSemaphoreHandle thresholdFreqSemaphore;
xSemaphoreHandle thresholdROCSemaphore;

// used to delete a task
TaskHandle_t xHandle;

// Timer handle
 TimerHandle_t timer_500;

// Global variables
bool stabilityFlag = true;
bool prevStabilityFlag = true;
bool timerHasFinished = false;
bool timer200HasFinished = false;
bool buttonStateFlag = false;
bool configureThresholdFlag = false;
int switchArray[5];
int loadArray[5];
int	tempLoadArray[5];
int shedArray[5];
double freqThre[1000];
double freqROC[1000];
int n = 99; // Frequency array count
double thresholdFreq = 49; // 50;
double thresholdROC = 60; // 50;
int ledOnVals[5] = {0x01, 0x02, 0x04, 0x08, 0x10};
int ledOffVals[5] = {0x1E, 0x1D, 0x1B, 0x17, 0x0F};
int loadIndex = 0;
int loadPriorities[5];

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
} Line;

// Operation State enum declaration
/*********** CHANGE NAMES **********/
typedef enum
{
	IDLE,
	SHEDDING,
	MONITORING,
	LOADING,
	MANUAL,
	AUTO
} state;

state operationState = IDLE;
state buttonState = MANUAL;

#define CLEAR_LCD_STRING "[2J"
#define ESC 27
int buttonValue = 0;
#define SAMPLING_FREQ 16000.0 // 16kHz

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

// VGA Display Task
void VGADisplayTask(void *pvParameters) {
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);


//	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;

	while(1){

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freqThre[(i+j)%100]) > MIN_FREQ) && ((int)(freqThre[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freqThre[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freqThre[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * freqROC[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * freqROC[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);
			}
		}
		vTaskDelay(10);

	}
}

// Switch Polling Task
void SwitchPollingTask(void *pvParameters)
{
	while (1)
	{
		int i;
		int switchState = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		xSemaphoreTake(ledStatusSemaphore, portMAX_DELAY);
		// Loads can be turned on and off using the switches when the system is STABLE or in MANUAL mode.
		if (buttonState == MANUAL) //|| stabilityFlag == true
		{
			for (i = 0; i < 5; i++)
			{
				if (!(switchState & (1 << i)))
				{
					switchArray[i] = 0;
					loadArray[i] = 0;
					tempLoadArray[i] = 0;
				}
				else
				{
					switchArray[i] = 1;
					loadArray[i] = 1;
					tempLoadArray[i] = 1;
				}
			}
		} // When the frequency relay is managing loads (AUTO), only loads that are currently on can be turned off. No new loads can be turned on.
		else if (buttonState == AUTO)
		{
			for (i = 0; i < 5; i++)
			{
				if (!(switchState & (1 << i)))
				{
					switchArray[i] = 0;
					loadArray[i] = 0;
					tempLoadArray[i] = 0;
				}
			}
		}
		xSemaphoreGive(ledStatusSemaphore);

		if (switchState & (1 << 17))
		{
			// We want to configure the thresholdFreq
			configureThresholdFlag = true;
		}
		else
		{
			// We want to configure the thresholdROC
			configureThresholdFlag = false;
		}

//		xSemaphoreTake(stabilitySemaphore, portMAX_DELAY);
//		prevStabilityFlag = stabilityFlag;
//		if (switchState & (1 << 16))
//		{
//			stabilityFlag = true;
//			printf("stable\n");
//
//		}
//		else
//		{
//			stabilityFlag = false;
//			printf("unstable\n");
//
//		}
//		xSemaphoreGive(stabilitySemaphore);

		vTaskDelay(50);
	}
}

// ISRs

// Handles button input on interrupt to determine whether or not the system is in the MANUAL state
void button_isr(void *context, alt_u32 id)
{
	// need to cast the context first before using it
	int *temp = (int *)context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	// Code
	switch (buttonValue)
	{
	case 1:
		buttonStateFlag = !buttonStateFlag;
		if (buttonStateFlag == false)
		{
			buttonState = MANUAL;
			printf("MANUAL state\n");
		}
		else
		{
			buttonState = AUTO;
			printf("AUTO state\n");
		}
		break;
		// Configuring the threshold frequency and threshold ROC
	case 2:
		if (configureThresholdFlag)
		{
			printf("Increment threshold frequency\n");
			thresholdFreq += 0.05;
			printf("threshold freq: %f\n", thresholdFreq);
		}
		else
		{
			printf("Increment ROC frequency\n");
			thresholdROC += 0.05;
			printf("threshold ROC: %f\n", thresholdROC);
		}
		printf("freqROC: %f\n", freqROC[n]);
		printf("freqThre: %f\n", freqThre[n]);
		break;
	default:
		if (configureThresholdFlag)
		{
			printf("Decrement threshold frequency\n");
			thresholdFreq -= 0.05;
			printf("threshold freq: %f\n", thresholdFreq);
		}
		else
		{
			printf("Decrement ROC frequency\n");
			thresholdROC -= 0.05;
			printf("threshold ROC: %f\n", thresholdROC);
		}
		printf("freqROC: %f\n", freqROC[n]);
		printf("freqThre: %f\n", freqThre[n]);
		break;
	};
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

void LEDHandlerTask(void *pvParameters)
{
	int redLEDs = 0x00;
	int greenLEDs = 0x00;
	int tempArray[5];
	while (1)
	{
		xSemaphoreTake(ledStatusSemaphore, portMAX_DELAY);
		// When in MANUAL mode, loads should be able to be turned on and off using the switches. Red LEDs should be turned on and off accordingly.
		// Red LEDs should remain on when transitioning to AUTO mode until those loads are shed (green LED turns on) or until manually switched off.
		int i;
		if (buttonState == AUTO)
		{
			for (i = 0; i < 5; i++)
			{
				tempArray[i] = loadArray[i];
			}
		} else {
			for (i = 0; i < 5; i++)
			{
				tempArray[i] = switchArray[i];
			}
		}
		// Prepapre mask for red LEDs
		for (i = 0; i < 5; i++)
		{
			if (tempArray[i] == 1)
			{
				redLEDs = (redLEDs | ledOnVals[i]);
			}
			else
			{
				redLEDs = (redLEDs & ledOffVals[i]);
			}
		}
		// Prepare mask for green LEDs
		for (i = 0; i < 5; i++)
		{
			if (shedArray[i] == 1)
			{
				greenLEDs = (greenLEDs | ledOnVals[i]);
			}
			else
			{
				greenLEDs = (greenLEDs & ledOffVals[i]);
			}
		}
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, redLEDs);
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLEDs);
		xSemaphoreGive(ledStatusSemaphore);
		vTaskDelay(50);
	}
}

void freq_analyser_isr(void *context, alt_u32 id)
{
	double signalFreq = SAMPLING_FREQ / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR(signalFreqQ, &signalFreq, pdFALSE);
}

void StabilityMonitorTask(void *pvParameters)
{
//	double currentFreq;
	while (1)
	{
		while (uxQueueMessagesWaiting(signalFreqQ) != 0)
		{
			xQueueReceive(signalFreqQ, freqThre+n, 0); // portMAX_DELAY
			// ROC calculation
//			freqThre[n] = currentFreq;
			if (n == 0)
			{
//				freqROC[0] = ((freqThre[0] - freqThre[999]) * SAMPLING_FREQ) / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);
				freqROC[0] = (freqThre[0] - freqThre[99]) * 2.0 * freqThre[0] * freqThre[99] / (freqThre[0] + freqThre[99]);
			}
			else
			{
//				freqROC[n] = ((freqThre[n] - freqThre[n - 1]) * SAMPLING_FREQ) / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);
				freqROC[n] = (freqThre[n] - freqThre[n - 1]) * 2.0 * freqThre[n] * freqThre[n-1] / (freqThre[n] + freqThre[n-1]);
			}
			xSemaphoreTake(stabilitySemaphore, portMAX_DELAY);
			prevStabilityFlag = stabilityFlag;
			// (/* instantaneous frequency */ < thresholdFreq) || (/* too high abs(ROC of frequency) */ > thresholdROC)
			if ((freqThre[n] < thresholdFreq) || (abs(freqROC[n]) > thresholdROC)) { //&& (buttonState == AUTO)
				// system is unstable, operationState = SHEDDING
				stabilityFlag = false;

			} else { // freqThre[n] > thresholdFreq || abs(freqROC[n] < thresholdROC
				// system is stable
				stabilityFlag = true;
			}
//			printf("n: %d\n", n);
//			printf("freqROC: %f\n", freqROC[n]);
//			printf("freqThre: %f\n", freqThre[n]);
			xSemaphoreGive(stabilitySemaphore);
			if (freqROC[n] > 100.0){
				freqROC[n] = 100.0;
			}
			n =	++n%100; //point to the next data (oldest) to be overwritten
			vTaskDelay(50);
		}
	}
}

void stabilityTimer(xTimerHandle t_timer500)
{
	timerHasFinished = true;
}

// For testing
bool printState = false;
bool printState1 = false;

void LoadCtrlTask(void *pvParameters)
{
	// switches cannot turn on new loads but can turn off loads that are currently on
	while(1) {
		switch (buttonState)
		{
		case MANUAL:
			printf("MANUAL state \n");
			break;

		// AUTO is used by buttonState to represent whether the switches or the frequency relay are managing loads
		// When switching to AUTO, operationState defaults to IDLE
		case AUTO:
			if (printState == false) {
				printf("AUTO state \n");
				printState = true;
			}
			int i;
			switch (operationState)
			{
			case IDLE:
				printf("IDLE state \n");
				if (stabilityFlag) {
					break;
				} else {
					operationState = SHEDDING;
				}

				break;

			case SHEDDING:
				printf("SHEDDING state \n");
				xSemaphoreTake(shedSemaphore, portMAX_DELAY);
				// Shedding loads that are on from lowest to highest priority
				for (i = 0; i < 5; i++) {
					if (loadArray[i] == 1) {
						loadArray[i] = 0;
						shedArray[i] = 1;
						break;
					}
				}
				// Start 500 ms stability timer for MONITORING state
				xTimerReset(timer_500, 0);
				timerHasFinished = false;
				printf("timer started\n");
//				prevStabilityFlag = stabilityFlag;
				operationState = MONITORING;
				xSemaphoreGive(shedSemaphore);
				// Shedding loads that are on from lowest priority to highest

				break;

			case MONITORING:
				if (printState1 == false) {
					printf("MONITORING state \n");
					printState = true;
				}
				xSemaphoreTake(stabilitySemaphore, portMAX_DELAY);
				// 500 ms timer should be reset if the network status changes from stable to unstable or vice versa
				// before the 500 ms period ends.
				if (stabilityFlag != prevStabilityFlag && timerHasFinished == false)
				{
					xTimerReset(timer_500, 0);
				}
				if (timerHasFinished == true)
				{
					if (stabilityFlag == true)
					{
						// if network is stable for 500ms, highest priority load that has been shed should be reconnected
						// switch state to loading
						operationState = LOADING;
						xTimerReset(timer_500, 0);
						printf("reset timer_t\n");
						timerHasFinished = false;
						// process can repeat until all loads are off
					}
					else
					{
						// if network is unstable for 500ms, the next lowest priority load should be shed
						// switch state to shed
						operationState = SHEDDING;
						xTimerReset(timer_500, 0);
						printf("reset timer_e\n");
						timerHasFinished = false;
					}

				}
				// if network switches from stable <-> unstable, reset 500ms at time of change
				xSemaphoreGive(stabilitySemaphore);

				break;

			case LOADING:
				printf("LOADING state \n");
				xSemaphoreTake(loadSemaphore, portMAX_DELAY);
				// Load from highest priority to lowest priority that have been shed
				for (i = 4; i >= 0; i--) {
					if (loadArray[i] != tempLoadArray[i]) {
						loadArray[i] = 1;
						shedArray[i] = 0;
						break;
					}
				}
				bool reconnected = true;
				for (i = 0; i < 5; i++)
				{
					if (shedArray[i] == 1)
					{
						reconnected = false;
					}
				}
				if (reconnected == false)
				{
					operationState = MONITORING;
				}
				else
				{
					operationState = IDLE;
				}
				// Switch to AUTO state once all loads have been reconnected
				xSemaphoreGive(loadSemaphore);

				break;
			default:
				break;
			}
			break;
		default:
			break;
		}
		vTaskDelay(50);
	}
}

int main(int argc, char *argv[], char *envp[])
{
	initOSDataStructs();
	initCreateTasks();
	initISRs();
	vTaskStartScheduler();
	for (;;)
		;
	return 0;
}

// This function simply creates the ISRs
int initISRs(void)
{
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	alt_irq_register(PUSH_BUTTON_IRQ, (void *)&buttonValue, button_isr);

	// enable interrupt for frequency analyser isr
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_analyser_isr);
	return 0;
}

// This function simply creates message queues and semaphores
int initOSDataStructs(void)
{
	signalFreqQ = xQueueCreate(NEW_FREQ_QUEUE_SIZE, sizeof(double));
	loadCtrlQ = xQueueCreate(LOAD_CTRL_QUEUE_SIZE, sizeof(void *));

	stabilitySemaphore = xSemaphoreCreateMutex();
	loadSemaphore = xSemaphoreCreateMutex();
	shedSemaphore = xSemaphoreCreateMutex();
	ledStatusSemaphore = xSemaphoreCreateMutex();
	thresholdFreqSemaphore = xSemaphoreCreateMutex();
	thresholdROCSemaphore = xSemaphoreCreateMutex();

	// timers
	timer_500 = xTimerCreate("500ms timer", 500, pdFALSE, NULL, stabilityTimer);
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0);
	xTaskCreate(SwitchPollingTask, "SwitchPollingTask", TASK_STACKSIZE, NULL, SWITCH_POLLING_TASK_PRIORITY, NULL);
	xTaskCreate(LEDHandlerTask, "LEDHandlerTask", TASK_STACKSIZE, NULL, LED_HANDLER_TASK_PRIORITY, NULL);
//	xTaskCreate(VGADisplayTask, "VGADisplayTask", TASK_STACKSIZE, NULL, VGA_DISPLAY_TASK_PRIORITY, NULL);
	xTaskCreate(LoadCtrlTask, "LoadCtrlTask", TASK_STACKSIZE, NULL, LOAD_CTRL_TASK_PRIORITY, NULL);
	xTaskCreate(StabilityMonitorTask, "StabilityMonitorTask", TASK_STACKSIZE, NULL, STABILITY_MONITOR_TASK_PRIORITY, NULL);
	return 0;
}
