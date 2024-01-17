/*
 * queue.c
 *
 *  Created on: Dec 29, 2023
 *      Author: DELL
 */

/* Private includes ----------------------------------------------------------*/
#include <taskQueue.h>

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/

void Queue_Init(TaskQueue_t *queue, uint8_t capacity)
{
	queue->Size = 0;
	queue->Front = capacity - 1;
	queue->Rear = capacity;
	queue->Capacity = capacity;
}

Queue_Status Queue_PushRear(TaskQueue_t* queue, pTaskFunction pTask)
{
	Queue_Status status = QUEUE_FULL;

	if (queue->Size < MAX_TASKS)
	{
		(queue->Rear)--;
		queue->Task[queue->Rear] = pTask;
		(queue->Size)++;
		status = QUEUE_OK;
	}

	return status;
}

Queue_Status Queue_PushFront(TaskQueue_t* queue, pTaskFunction pTask)
{
	Queue_Status status = QUEUE_FULL;
	uint8_t idx;

	if (queue->Size < MAX_TASKS)
	{
		for (idx = 0; idx < queue->Size; idx++)
		{
			queue->Task[queue->Rear + idx - 1] = queue->Task[queue->Rear + idx];
		}
		queue->Task[queue->Front] = pTask;
		(queue->Rear)--;
		(queue->Size)++;

		status = QUEUE_OK;
	}

	return status;
}

pTaskFunction Queue_Pop(TaskQueue_t* queue)
{
	pTaskFunction pTask = NULL;
	uint8_t idx;

	if (queue->Size >= 0)
	{
		pTask = queue->Task[queue->Front];
		(queue->Size)--;
		(queue->Rear)++;
		for (idx = 0; idx < queue->Size; idx++)
		{
			queue->Task[queue->Front - idx] = queue->Task[queue->Front - idx - 1];
		}
	}

	return pTask;
}
