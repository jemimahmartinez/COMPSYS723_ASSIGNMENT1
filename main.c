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
//#include "altera_up_avalon_ps2.h"
#include "avalon_ps2.h"
// #include "altera_up_ps2_keyboard.h"
#include "ps2_keyboard.h"

// Definition of Task Stacks
#define TASK_STACKSIZE 2048

// Definition of Task Priorities
#define LOAD_CNTRL_TASK_PRIORITY 1
#define STABILITY_MONITOR_TASK_PRIORITY 1
#define SWITCH_POLLING_TASK_PRIORITY 2
#define KEYBOARD_TASK_PRIORITY 2
#define LED_HANDLER_TASK_PRIORITY 3
#define VGA_DISPLAY_TASK_PRIORITY 4

// Definition of queues
#define MSG_QUEUE_SIZE 30
QueueHandle_t msgqueue;
#define LOAD_CTRL_QUEUE_SIZE 100
QueueHandle_t loadCtrlQ;
#define KEYBOARD_DATA_QUEUE_SIZE 100
QueueHandle_t keyboardDataQ;
#define NEW_FREQ_QUEUE_SIZE 100
QueueHandle_t newFreqQ;

// used to delete a task
TaskHandle_t xHandle;

// Global variables
bool systemStatusFlag = false;
unsigned int led0StatusFlag = 0;
unsigned int led1StatusFlag = 0;
unsigned int led2StatusFlag = 0;
unsigned int led3StatusFlag = 0;
unsigned int led4StatusFlag = 0;
unsigned int thresholdFreq = 0;
unsigned int thresholdROC = 0;

// Operation State enum declaration
/*********** CHANGE NAMES **********/
typedef enum
{
	DEFAULT,
	SHEDDING,
	MONITORING,
	LOADING,
	MAINTENANCE,
	NORMAL
} state;

state operationState = NORMAL;
state currentState = NORMAL;

#define CLEAR_LCD_STRING "[2J"
#define ESC 27
int buttonValue = 0;

// Local Function Prototypes
int initOSDataStructs(void);
int initCreateTasks(void);

// Switch Polling Task
//void SwitchPollingTask(void *pvParameters)
//{
//	while (1)
//	{
//		int mode = IORD_ALTERA_AVALON_PIO_DATA(SWITCHES_BASE);
//		// Carry out switch logic
//	}
//}

// ISRs

// Handles button input on interrupt to determine whether or not the system is in the maintenance state
void button_isr(void *context, alt_u32 id)
{
	// need to cast the context first before using it
	int *temp = (int *)context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	// Code
	switch (buttonValue)
	{
	case 1:
		buttonValue = 0;
		operationState = MAINTENANCE;
		break;
	default:
		// if buttonValue === 0
		buttonValue = 1;
		operationState = NORMAL;
		break;
	};
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
}

// void keyboard_isr(void *context, alt_u32 id)
// {
// 	char ascii;
// 	int status = 0;
// 	unsigned char key = 0;
// 	KB_CODE_TYPE decode_mode;
// 	while (1)
// 	{
// 		// blocking function call
// 		status = decode_scancode(ps2_device, &decode_mode, &key, &ascii);
// 		if (status == 0) //success
// 		{
// 			// print out the result
// 			switch (decode_mode)
// 			{
// 			case KB_ASCII_MAKE_CODE:
// 				printf("ASCII   : %x\n", key);
// 				break;
// 			case KB_LONG_BINARY_MAKE_CODE:
// 				// do nothing
// 			case KB_BINARY_MAKE_CODE:
// 				printf("MAKE CODE : %x\n", key);
// 				break;
// 			case KB_BREAK_CODE:
// 				// do nothing
// 			default:
// 				printf("DEFAULT   : %x\n", key);
// 				break;
// 			}
// 			IOWR(SEVEN_SEG_BASE, 0, key);
// 		}
// 	}
// 	return 0;
// }

//void KeyboardTask(void *pvParameters)
//{
//	alt_up_ps2_dev *ps2_device = alt_up_ps2_open_dev(PS2_NAME);
//
//	if (ps2_device == NULL)
//	{
//		printf("can't find PS/2 device\n");
//		return;
//	}
//
//	alt_up_ps2_clear_fifo(ps2_device);
//
//	alt_irq_register(KEYBOARD_IRQ, ps2_device, keyboard_isr);
//	// register the PS/2 interrupt
//	IOWR_8DIRECT(PS2_BASE, 4, 1);
//	while (1)
//	{
//	}
//}

void LEDHandlerTask(void *pvParameters)
{
}

void freq_analyser_isr(void *context, alt_u32 id)
{
}

void loadCtrlTask(void *pvParameters) {
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
	//	alt_irq_register(KEYBOARD_IRQ, (void *)&keyboardValue, keyboard_isr);
	//	alt_irq_register(FREQ_ANALYSER_IRQ, (void *)&frequencyValue, freq_analyser_isr);
}

// This function simply creates a message queue and a semaphore
int initOSDataStructs(void)
{
	newFreqQ = xQueueCreate(NEW_FREQ_QUEUE_SIZE, sizeof(double));
	loadCtrlQ = xQueueCreate(LOAD_CTRL_QUEUE_SIZE, sizeof(void *));
	keyboardDataQ = xQueueCreate(KEYBOARD_DATA_QUEUE_SIZE, sizeof(unsigned char));
	return 0;
}

// This function creates the tasks used in this example
int initCreateTasks(void)
{
	xTaskCreate(SwitchPollingTask, "SwitchPollingTask", TASK_STACKSIZE, NULL, PRINT_STATUS_TASK_PRIORITY, NULL);
	xTaskCreate(KeyboardTask, "KeyboardTask", TASK_STACKSIZE, NULL, KEYBOARD_TASK_PRIORITY, NULL);
	xTaskCreate(LEDHandlerTask, "LEDHandlerTask", TASK_STACKSIZE, NULL, LED_HANDLER_TASK_PRIORITY, NULL);
	xTaskCreate(VGADisplayTask, "VGADisplayTask", TASK_STACKSIZE, NULL, VGA_DISPLAY_TASK_PRIORITY, NULL);
	xTaskCreate(LoadCtrlTask, "LoadCntrlTask", TASK_STACKSIZE, NULL, LOAD_CNTRL_TASK_PRIORITY, NULL);
	xTaskCreate(StabilityMonitorTask, "StabilityMonitorTask", TASK_STACKSIZE, NULL, STABILITY_MONITOR_TASK_PRIORITY, NULL);
	return 0;
}
