/* Private includes ----------------------------------------------------------*/
#include "dht11.h"
#include "delay_timer.h"

/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define INIT_TEMP_VALUE -1
#define INIT_HUMI_VALUE -1

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static void DHT11_SetPinIn(DHT11_Sensor* dht);
static void DHT11_SetPinOut(DHT11_Sensor* dht);
static void DHT11_WritePin(DHT11_Sensor* dht, uint8_t value);
static uint8_t DHT11_ReadPin(DHT11_Sensor* dht);
static DHT11_Status DHT11_Start(DHT11_Sensor* dht);
static uint8_t DHT11_ReadByte(DHT11_Sensor* dht);

/* Private user code ---------------------------------------------------------*/
static void DHT11_SetPinIn(DHT11_Sensor* dht)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = dht->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(dht->Port, &GPIO_InitStruct);
}

static void DHT11_SetPinOut(DHT11_Sensor* dht)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = dht->Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(dht->Port, &GPIO_InitStruct);
}

static void DHT11_WritePin(DHT11_Sensor* dht, uint8_t value)
{
    HAL_GPIO_WritePin(dht->Port, dht->Pin, value);
}

static uint8_t DHT11_ReadPin(DHT11_Sensor* dht)
{
    uint8_t value = HAL_GPIO_ReadPin(dht->Port, dht->Pin);
    return value;
}

static void DHT11_DelayInit(DHT11_Sensor* dht)
{
	DELAY_TIM_Init(dht->Timer);
}

static void DHT11_DelayUs(DHT11_Sensor* dht, uint16_t time)
{
	DELAY_TIM_Us(dht->Timer, time);
}

static DHT11_Status DHT11_Start(DHT11_Sensor* dht)
{
    DHT11_Status status = DHT11_ERR_RESPONSE;

    DHT11_SetPinOut(dht);

    /* MCU sends out start signal and pulls down  voltage for at least 18 ms */
    DHT11_WritePin(dht, 0);
    DHT11_DelayUs(dht, 18000);

    /* MCU waits for DHT response (20-40us) */
    DHT11_SetPinIn(dht);
    DHT11_DelayUs(dht, 50);

    /* DHT sends out response signal and keeps it for 80 us */
    if (DHT11_ReadPin(dht) == 0)
    {
        DHT11_DelayUs(dht, 80);
        if (DHT11_ReadPin(dht) == 1)
        {
            status = DHT11_OK;
            /* Wait for data transmission */
            while(DHT11_ReadPin(dht) == 1);
        }
    }

    return status;
}

static uint8_t DHT11_ReadByte(DHT11_Sensor* dht)
{
    uint8_t value = 0;
    uint8_t idx;

    DHT11_SetPinIn(dht);

    for (idx = 0; idx < 8; idx++)
    {
        /* Wait for old signal to end */
        while(DHT11_ReadPin(dht) == 0);

        DHT11_DelayUs(dht, 40);

        /* Bit 1 */
        if (DHT11_ReadPin(dht) == 1)
        {
            /* Set bit at (7-idx) position */
            value |= (0x01 << (7 - idx));
        }

        /* Bit 0 */
        else
        {
            /* Clear bit at (7-idx) position */
            value &= ~(0x01 << (7-idx));
        }

        while(DHT11_ReadPin(dht) == 1);
    }

    return value;
}

void DHT11_Init(DHT11_Sensor* dht, GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* timer)
{
    dht->Port   = port;
    dht->Pin    = pin;
    dht->Timer  = timer;
    dht->Temp   = INIT_TEMP_VALUE;
    dht->Humi   = INIT_HUMI_VALUE;

    DHT11_DelayInit(dht);
}

DHT11_Status DHT11_GetData(DHT11_Sensor* dht)
{
    DHT11_Status status = DHT11_OK;

    uint8_t rh_integral, rh_decimal, temp_integral, temp_decimal, sum;

	status = DHT11_Start(dht);

    if (status == DHT11_OK)
    {
        /* Read 5 byte from DHT */
        rh_integral     = DHT11_ReadByte(dht);
        rh_decimal      = DHT11_ReadByte(dht);
        temp_integral   = DHT11_ReadByte(dht);
        temp_decimal    = DHT11_ReadByte(dht);
        sum             = DHT11_ReadByte(dht);

        if (sum == rh_integral + rh_decimal + temp_integral + temp_decimal)
        {
            dht->Humi = rh_integral + (float)(rh_decimal/10.0);
            dht->Temp = temp_integral + (float)(temp_decimal/10.0);
        }

        else
        {
            status = DHT11_ERR_CHECKSUM;
        }
    }

    return status;
}
