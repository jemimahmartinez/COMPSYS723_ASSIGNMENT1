// Standard includes
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h> //for usleep

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <altera_avalon_pio_regs.h>

// IO includes
#include "io.h"
#include "altera_up_avalon_ps2.h"
//#include "avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
//#include "ps2_keyboard.h"

// Definition of Task Stacks
#define TASK_STACKSIZE 2048

// Definition of Task Priorities
#define LOAD_CNTRL_TASK_PRIORITY 4		  // 1
#define STABILITY_MONITOR_TASK_PRIORITY 4 // 1
#define SWITCH_POLLING_TASK_PRIORITY 3	  // 2
#define KEYBOARD_TASK_PRIORITY 3		  // 2
#define LED_HANDLER_TASK_PRIORITY 1		  //3
#define VGA_DISPLAY_TASK_PRIORITY 1		  // 4

// Definition of queues
#define MSG_QUEUE_SIZE 30
QueueHandle_t msgqueue;
#define LOAD_CTRL_QUEUE_SIZE 100
QueueHandle_t loadCtrlQ;
#define KEYBOARD_DATA_QUEUE_SIZE 100
QueueHandle_t keyboardDataQ;
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
// TimerHandle_t timer_500;

// Global variables
bool stabilityFlag = true;
bool prevStabilityFlag = stabilityFlag;
bool timer500HasFinished = false;
bool buttonStateFlag = false;
bool configureThresholdFlag = false;
int switchArray[5];
int loadArray[5];
int shedArray[5];
double freqThre[1000];
double freqROC[1000];
int n = 0; // Frequency array count
unsigned int thresholdFreq = 0;
unsigned int thresholdROC = 0;
int ledOnVals[5] = {0x01, 0x02, 0x04, 0x08, 0x10};
int ledOffVals[5] = {0x1E, 0x1D, 0x1B, 0x17, 0x0F};
int lowestPriorityLoadOn = 0;
int loadPriorities[5];

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
state buttonState = AUTO;

#define CLEAR_LCD_STRING "[2J"
#define ESC 27
int buttonValue = 0;
#define SAMPLING_FREQ 16000.0 // 16kHz

KB_CODE_TYPE decode_mode;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

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
				}
				else
				{
					switchArray[i] = 1;
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
				}
			}
		}
		// Populate loadPriorities with the priorities of the loads that are currently on
		int j;
		int k = 0;
		for (j = 0; j < 5; j++)
		{
			loadPriorities[j] = 9;
		}

		for (j = 0; j < 5; j++)
		{
			if (switchArray[j] == 1)
			{
				loadPriorities[k] = j;
				k++;
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

		vTaskDelay(5);
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
			thresholdFreq++;
			printf("threshold freq: %d\n", thresholdFreq);
		}
		else
		{
			printf("Increment ROC frequency\n");
			thresholdROC++;
			printf("threshold ROC: %d\n", thresholdROC);
		}
		break;
	default:
		if (configureThresholdFlag)
		{
			printf("Decrement threshold frequency\n");
			thresholdFreq--;
			printf("threshold freq: %d\n", thresholdFreq);
		}
		else
		{
			printf("Decrement ROC frequency\n");
			thresholdROC--;
			printf("threshold ROC: %d\n", thresholdROC);
		}
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
			int i;
			for (i = 0; i < 5; i++)
			{
				tempArray[i] = loadArray[i];
			}
		}
		// Prepapre mask for red LEDs
		for (i = 0; i < 5; i++)
		{
			if (switchArray[i] == 1)
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
		vTaskDelay(5);
	}
}

void freq_analyser_isr(void *context, alt_u32 id)
{
	double signalFreq = SAMPLING_FREQ / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR(signalFreqQ, &signalFreq, pdFALSE);
}

//void StabilityMonitorTask(void *pvParameters)
//{
//	double currentFreq;
//	while (1)
//	{
//		while (uxQueueMessagesWaiting(signalFreqQ) != 0)
//		{
//			xQueueReceive(signalFreqQ, &currentFreq, portMAX_DELAY);
//			// ROC calculation
//			freqThre[n] = currentFreq;
//			if (n == 0)
//			{
//				freqROC[0] = ((freqThre[0] - freqThre[999]) * SAMPLING_FREQ) / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);
//			}
//			else
//			{
//				freqROC[n] = ((freqThre[n] - freqThre[n - 1]) * SAMPLING_FREQ) / (double)IORD(FREQUENCY_ANALYSER_BASE, 0);
//			}
//			xSemaphoreTake(stabilitySemaphore, portMAX_DELAY);
//			// (/* instantaneous frequency */ < thresholdFreq) || (/* too high abs(ROC of frequency) */ > thresholdROC)
//			if (((freqThre[n] < thresholdFreq) || (abs(freqROC) > thresholdROC)) && (buttonState == AUTO)) {
//				// system is unstable, operationState = SHEDDING
//				stabilityFlag = false;
//				operationState = SHEDDING;
//			} else {
//				// system is stable
//				stabilityFlag = true;
//			}
//			printf("n: %d\n", n);
//			printf("freqROC: %f\n", freqROC[n]);
//			printf("freqThre: %f\n", freqThre[n]);
//			xSemaphoreGive(stabilitySemaphore);
//			if (n == 999)
//			{
//				n = 0;
//			}
//			else
//			{
//				n++;
//			}
//		}
//	}
//}

void stabilityTimer(xTimerHandle stabilityTimer500)
{
	timer500HasFinished = true;
}

void loadCtrlTask(void *pvParameters)
{
	// switches cannot turn on new loads but can turn off loads that are currently on

	switch (operationState)
	{
	case IDLE:
		printf("IDLE state \n");
		break;

	case SHEDDING:
		printf("SHEDDING state \n");
		xSemaphoreTake(shedSemaphore, portMAX_DELAY);
		// Shedding loads that are on from lowest to highest priority
		loadArray[loadPriorities[lowestPriorityLoadOn]] = 0;
		if (loadPriorities[lowestPriorityLoadOn++] != 9)
		{
			lowestPriorityLoadOn++;
		}
		operationState = MONITORING;
		prevStabilityFlag = stabilityFlag;
		// Start 500 ms stability timer for MONITORING state
		xTimerStart(timer_500, 0);
		xSemaphoreGive(shedSemaphore);
		// Shedding loads that are on from lowest priority to highest

		break;
	case MONITORING:
		printf("MONITORING state \n");
		xSemaphoreTake(stabilitySemaphore, portMAX_DELAY);
		// 500 ms timer should be reset if the network status changes from stable to unstable or vice versa
		// before the 500 ms period ends.
		if (stabilityFlag != prevStabilityFlag && timer500HasFinished == false)
		{
			xTimerReset(timer_500, 0);
		}
		if (timer500HasFinished == true)
		{
			if (stabilityFlag == true)
			{
				// if network is stable for 500ms, highest priority load that has been shed should be reconnected
				// switch state to loading
				operationState = LOADING;
				// process can repeat until all loads are off
			}
			else
			{
				// if network is unstable for 500ms, the next lowest priority load should be shed
				// switch state to shed
				operationState = SHEDDING;
				// process can repeat until all loads are reconnected
			}
		}
		// if network switches from stable <-> unstable, reset 500ms at time of change
		xSemaphoreGive(stabilitySemaphore);

		break;
	case LOADING:
		printf("LOADING state \n");
		xSemaphoreTake(loadSemaphore, portMAX_DELAY);
		// Load from highest priority to lowest priority that have been shed
		loadArray[loadPriorities[lowestPriorityLoadOn--]] = 1;
		lowestPriorityLoadOn--;
		if (lowestPriorityLoadOn >= 0)
		{
			lowestPriorityLoadOn--;
		}
		bool noShedding = false;
		int i;
		for (i = 0; i < 5; i++)
		{
			if (shedArray[i] != 0)
			{
				noShedding = false;
			}
			else
			{
				noShedding = true;
			}
		}
		if (noShedding == true)
		{
			operationState = IDLE;
		}
		else
		{
			operationState = MONITORING;
		}
		// Switch to AUTO state once all loads have been reconnected
		xSemaphoreGive(loadSemaphore);

		break;

	case MANUAL:
		printf("MANUAL state \n");

		break;
	// AUTO is used by buttonState to represent whether the switches or the frequency relay are managing loads
	// When switching to AUTO, operationState defaults to IDLE
	case AUTO:
		printf("AUTO state \n");

		break;
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
	//
	//	// enable interrupt for keyboard
	//	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	//
	//	if(ps2_device == NULL){
	//		printf("can't find PS/2 device\n");
	//		return 1;
	//	}
	//
	//	alt_up_ps2_clear_fifo (ps2_device) ;
	//	alt_irq_register(PS2_IRQ, ps2_device, keyboard_isr);
	//	// register the PS/2 interrupt
	//	IOWR_8DIRECT(PS2_BASE,4,1);

	// enable interrupt for frequency analyser isr
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_analyser_isr);
	return 0;
}

// This function simply creates message queues and semaphores
int initOSDataStructs(void)
{
	signalFreqQ = xQueueCreate(NEW_FREQ_QUEUE_SIZE, sizeof(double));
	loadCtrlQ = xQueueCreate(LOAD_CTRL_QUEUE_SIZE, sizeof(void *));
	keyboardDataQ = xQueueCreate(KEYBOARD_DATA_QUEUE_SIZE, sizeof(unsigned char));

	stabilitySemaphore = xSemaphoreCreateMutex();
	loadSemaphore = xSemaphoreCreateMutex();
	shedSemaphore = xSemaphoreCreateMutex();
	ledStatusSemaphore = xSemaphoreCreateMutex();
	thresholdFreqSemaphore = xSemaphoreCreateMutex();
	thresholdROCSemaphore = xSemaphoreCreateMutex();

	// timers
	timer_500 = xTimerCreate("500ms timer", 500, pdTRUE, NULL, stabilityTimerFinish);
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, 0);
	xTaskCreate(SwitchPollingTask, "SwitchPollingTask", TASK_STACKSIZE, NULL, SWITCH_POLLING_TASK_PRIORITY, NULL);
	//	xTaskCreate(KeyboardTask, "KeyboardTask", TASK_STACKSIZE, NULL, KEYBOARD_TASK_PRIORITY, NULL);
	xTaskCreate(LEDHandlerTask, "LEDHandlerTask", TASK_STACKSIZE, NULL, LED_HANDLER_TASK_PRIORITY, NULL);
	//	xTaskCreate(VGADisplayTask, "VGADisplayTask", TASK_STACKSIZE, NULL, VGA_DISPLAY_TASK_PRIORITY, NULL);
	//	xTaskCreate(LoadCtrlTask, "LoadCntrlTask", TASK_STACKSIZE, NULL, LOAD_CNTRL_TASK_PRIORITY, NULL);
	//	xTaskCreate(StabilityMonitorTask, "StabilityMonitorTask", TASK_STACKSIZE, NULL, STABILITY_MONITOR_TASK_PRIORITY, NULL);
	return 0;
}
