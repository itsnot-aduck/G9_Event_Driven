/*
 * queue.h
 *
 *  Created on: Dec 29, 2023
 *      Author: DELL
 */

#ifndef __TASK_QUEUE_H
#define __TASK_QUEUE_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

#define MAX_TASKS 100

typedef void (*pTaskFunction)();

typedef struct
{
    pTaskFunction Task[MAX_TASKS]; // Array to store task addresses
    uint8_t Front;                     // Index of the front of the queue
    uint8_t Rear;                      // Index of the rear of the queue
    uint8_t Size;                      // Current size of the queue
    uint8_t Capacity;
} TaskQueue_t;

typedef enum
{
	QUEUE_OK,
	QUEUE_ERR_MEMORY,
	QUEUE_FULL,
	QUEUE_EMPTY,
} Queue_Status;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void Queue_Init(TaskQueue_t *queue, uint8_t capacity);
Queue_Status Queue_PushRear(TaskQueue_t* queue, pTaskFunction pTask);
Queue_Status Queue_PushFront(TaskQueue_t* queue, pTaskFunction pTask);
pTaskFunction Queue_Pop(TaskQueue_t* queue);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#endif /* __TASK_QUEUE_H */
