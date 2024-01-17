
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DTH11_H
#define __DHT11_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef* Port;
    uint16_t Pin;
    TIM_HandleTypeDef* Timer;
    float Temp;
    float Humi;
} DHT11_Sensor;

typedef enum
{
	DHT11_OK,
    DHT11_ERR_RESPONSE,
    DHT11_ERR_CHECKSUM,
} DHT11_Status;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
DHT11_Status DHT11_GetData(DHT11_Sensor* dht);
void DHT11_Init(DHT11_Sensor* dht, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* timer);
/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#endif /* __DHT11_H */
