/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_I2C_H
#define __LCD_I2C_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct
{
	I2C_HandleTypeDef* I2c;
	uint8_t Address;
	uint8_t Columns;
	uint8_t Rows;
	uint8_t EntryMode;
	uint8_t DisplayControl;
	uint8_t CursorShift;
	uint8_t FunctionSet;
	uint8_t BackLight;
} LCD_I2C_Name;

/* Exported constants --------------------------------------------------------*/

#define LDC_DEFAULT_ADDRESS 0x4E

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void LCD_Init(LCD_I2C_Name* lcd, I2C_HandleTypeDef* i2c, uint8_t address, uint8_t columns, uint8_t rows);
void LCD_SetCursor(LCD_I2C_Name* lcd, uint8_t x_pos, uint8_t y_pos);
void LCD_WriteString(LCD_I2C_Name* lcd, char *string);
void LCD_Clear(LCD_I2C_Name* lcd);

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */


#endif /* __LCD_I2C_H */
