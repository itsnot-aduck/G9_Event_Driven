/* Private includes ----------------------------------------------------------*/
#include "lcd_i2c.h"
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

/* List of COMMANDS */
#define LCD_CLEARDISPLAY      0x01
#define LCD_RETURNHOME        0x02
#define LCD_ENTRYMODESET      0x04
#define LCD_DISPLAYCONTROL    0x08
#define LCD_CURSORSHIFT       0x10
#define LCD_FUNCTIONSET       0x20
#define LCD_SETCGRAMADDR      0x40
#define LCD_SETDDRAMADDR      0x80

/* LCD Mode */
#define LCD_COMMAND 		0x00
#define LCD_DATA 			0x01

#define LCD_EN 0x04  // Enable bit
#define LCD_RW 0x02  // Read/Write bit
#define LCD_RS 0x01  // Register select bit

/* List of commands Bitfields */
// 1. Fags for display entry mode
#define LCD_ENTRYRIGHT 			0x00
#define LCD_ENTRYLEFT 			0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00
// 2. Flags for display on/off control
#define LCD_DISPLAYON 			0x04
#define LCD_DISPLAYOFF 			0x00
#define LCD_CURSORON 			0x02
#define LCD_CURSOROFF 			0x00
#define LCD_BLINKON 			0x01
#define LCD_BLINKOFF 			0x00
// 3. Flags for display/cursor shift
#define LCD_DISPLAYMOVE 		0x08
#define LCD_CURSORMOVE 			0x00
#define LCD_MOVERIGHT 			0x04
#define LCD_MOVELEFT 			0x00
// 4. Flags for function set
#define LCD_8BITMODE 			0x10
#define LCD_4BITMODE 			0x00
#define LCD_2LINE 				0x08
#define LCD_1LINE 				0x00
#define LCD_5x10DOTS 			0x04
#define LCD_5x8DOTS 			0x00

#define LCD_BACKLIGHT 			0x08
#define LCD_NOBACKLIGHT 		0x00

/* Private macro -------------------------------------------------------------*/

#define LCD_DelayMs(X)      HAL_Delay(X)

/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

static void LCD_Write(LCD_I2C_Name* lcd, uint8_t data, uint8_t mode);
static void LCD_WriteChar(LCD_I2C_Name* lcd, char character);

/* Private user code ---------------------------------------------------------*/

static void LCD_Write(LCD_I2C_Name* lcd, uint8_t data, uint8_t mode)
{
    uint8_t Data_H, Data_L;
	uint8_t Data_I2C[4];

	Data_H = data & 0xF0;
	Data_L = (data << 4) & 0xF0;

	if(lcd->BackLight)
	{
        /* Set bit */
		Data_H |= LCD_BACKLIGHT;
		Data_L |= LCD_BACKLIGHT;
	}

	if(mode == LCD_DATA)
	{
        /* Set bit */
		Data_H |= LCD_RS;
		Data_L |= LCD_RS;
	}
	else if(mode == LCD_COMMAND)
	{
        /* Clear bit */
		Data_H &= ~LCD_RS;
		Data_L &= ~LCD_RS;
	}

    /* Assign data to higher bits */
	Data_I2C[0] = Data_H|LCD_EN;
	LCD_DelayMs(1);
	Data_I2C[1] = Data_H;

    /* Assign data to lower bits */
	Data_I2C[2] = Data_L|LCD_EN;
	LCD_DelayMs(1);
	Data_I2C[3] = Data_L;

	HAL_I2C_Master_Transmit(lcd->I2c, lcd->Address, (uint8_t *)Data_I2C, sizeof(Data_I2C), 1000);
}

void LCD_Init(LCD_I2C_Name* lcd, I2C_HandleTypeDef* i2c, uint8_t address, uint8_t columns, uint8_t rows)
{
    lcd->I2c        = i2c;
    lcd->Address    = address;
    lcd->Columns    = columns;
    lcd->Rows       = rows;

    lcd->FunctionSet    = LCD_FUNCTIONSET|LCD_4BITMODE|LCD_2LINE|LCD_5x8DOTS;
    lcd->EntryMode      = LCD_ENTRYMODESET|LCD_ENTRYLEFT|LCD_ENTRYSHIFTDECREMENT;
    lcd->DisplayControl = LCD_DISPLAYCONTROL|LCD_DISPLAYON|LCD_CURSOROFF|LCD_BLINKOFF;
    lcd->CursorShift    = LCD_CURSORSHIFT|LCD_CURSORMOVE|LCD_MOVERIGHT;
    lcd->BackLight      = LCD_BACKLIGHT;

    LCD_DelayMs(50);
	LCD_Write(lcd, 0x33, LCD_COMMAND);

	LCD_Write(lcd, 0x33, LCD_COMMAND);
	LCD_DelayMs(5);
	LCD_Write(lcd, 0x32, LCD_COMMAND);
	LCD_DelayMs(5);
	LCD_Write(lcd, 0x20, LCD_COMMAND);
	LCD_DelayMs(5);

	LCD_Write(lcd, lcd->EntryMode, LCD_COMMAND);
	LCD_Write(lcd, lcd->DisplayControl, LCD_COMMAND);
	LCD_Write(lcd, lcd->CursorShift, LCD_COMMAND);
	LCD_Write(lcd, lcd->FunctionSet, LCD_COMMAND);

	LCD_Write(lcd, LCD_CLEARDISPLAY, LCD_COMMAND);
	LCD_Write(lcd, LCD_RETURNHOME, LCD_COMMAND);
}

static void LCD_WriteChar(LCD_I2C_Name* lcd, char character)
{
	LCD_Write(lcd, character, LCD_DATA);
}

void LCD_WriteString(LCD_I2C_Name* lcd, char *string)
{
	while(*string)
	{
		LCD_WriteChar(lcd, *string++);
	}
}

void LCD_Clear(LCD_I2C_Name* lcd)
{
	LCD_Write(lcd, LCD_CLEARDISPLAY, LCD_COMMAND);
	LCD_DelayMs(5);
}

void LCD_SetCursor(LCD_I2C_Name* lcd, uint8_t x_pos, uint8_t y_pos)
{
	uint8_t DRAM_ADDRESS = 0x00;

	if(x_pos >= lcd->Columns)
	{
		x_pos = lcd->Columns - 1;
	}

	if(y_pos >= lcd->Rows)
	{
		y_pos = lcd->Rows -1;
	}

	switch (y_pos)
	{
		case 0:
			DRAM_ADDRESS = 0x00 + x_pos;
			break;
		case 1:
			DRAM_ADDRESS = 0x40 + x_pos;
			break;
		case 2:
			DRAM_ADDRESS = 0x14 + x_pos;
			break;
		case 3:
			DRAM_ADDRESS = 0x54 + x_pos;
			break;
	}

	LCD_Write(lcd, LCD_SETDDRAMADDR|DRAM_ADDRESS, LCD_COMMAND);
}
