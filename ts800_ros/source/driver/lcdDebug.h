#include <iostream>
#include <vector>

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdarg> 
#include <string.h>

#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#define LCD_TEXT_CTR CLcdTextCtr::getInstance()

class CLcdTextCtr
{
private:
	void printLCD(int idx, const char *str);
	void insertLetter(char *lcd, int letterIndex, int startRow, int startCol);
	
public:
	CLcdTextCtr(){};
	~CLcdTextCtr(){};
	static CLcdTextCtr& getInstance();
		
	void printfLCDClear();	
	void printf(int idx, const char *ucFmt, ...);

	void debugCleanMode(int type);
	void testDispaly();
	void Debug_printf(int idx, const char *ucFmt, ...);
	void DisplayQRCodeOnLCD(const char *data, int lcd_width, int lcd_height, int qr_size, int x_pos, int y_pos, int Color, int border_size, int border_color, int margin);
};





