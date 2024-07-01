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
#include "font.h"
#include "lcdDebug.h"
#include "qrencode.h"
#include "LibRobotDisplayInterface.h"

char LCD_Buffer[DisplayWidthLength];
char lcd[DisplayWidthLength * DispalyHeightLength];
char Clearletters[ALPHABAT + NUMBER + SPECIALCHA][HIGHT][HIGHT] = {0x00,};
extern CRobotDisplayInterface RoDisplayInf;

CLcdTextCtr &CLcdTextCtr::getInstance()
{
    static CLcdTextCtr s;
    
    return s;
}

void CLcdTextCtr::insertLetter(char *lcd, int letterIndex, int startRow, int startCol)
{
	for (int i = 0; i < HIGHT; i++)
	{
		for (int j = 0; j < HIGHT; j++)
		{
			lcd[((startRow + i) * DisplayWidthLength) + startCol + j] = letters[letterIndex][i][j];
		}
	}
}

void CLcdTextCtr::printfLCDClear()
{
	for (int i = 0; i < DisplayWidthLength * DispalyHeightLength; i++)
	{
		lcd[i] = 0x00;
	}
}

void CLcdTextCtr::printLCD(int idx, const char *str) {

	int startRow = STARTROW;
	int startCol = STARTCOL;

	for (int i = 0; i < MAXLINECHAR; i++) {

		int letterIndex = 0;

		if (str[i]  >= 'A' && str[i]  <= 'Z') {
			letterIndex = str[i] - 'A';
		} 
		else if (str[i]  >= 'a' && str[i]  <= 'z') {
			letterIndex = str[i] - 'a';
		} 
		else if (str[i]  >= '0' && str[i]  <= '9') {
			letterIndex = str[i] - '0' + ALPHABAT;
		}
		else if (str[i]  == ':') {
			letterIndex = SPECIALCHA2;
		}  
		else if (str[i]  == '+') {
			letterIndex = SPECIALCHA3;
		} 
		else if (str[i]  == '-') {
			letterIndex = SPECIALCHA4;
		} 
		else if (str[i]  == '.') {
			letterIndex = SPECIALCHA5;
		} 	
		else if (str[i]  == '*') {
			letterIndex = SPECIALCHA6;
		} 				
		else
		{
			letterIndex = SPECIALCHA1;
		} 

		if (idx == 1)
		{
			insertLetter(lcd, letterIndex, startRow, startCol);
		}

		if (idx == 2)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 1], letterIndex, startRow, startCol);
		}

		if (idx == 3)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 2], letterIndex, startRow, startCol);
		}
		if (idx == 4)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 3], letterIndex, startRow, startCol);
		}
		if (idx == 5)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 4], letterIndex, startRow, startCol);
		}
		if (idx == 6)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 5], letterIndex, startRow, startCol);
		}
		if (idx == 7)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 6], letterIndex, startRow, startCol);
		}
		if (idx == 8)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 7], letterIndex, startRow, startCol);
		}
		if (idx == 9)
		{
			insertLetter(&lcd[DisplayWidthLength * (HIGHT + LINESPACING) * 8], letterIndex, startRow, startCol);
		}
		startCol += HIGHT + WORDSPACING;

	}
	
	RoDisplayInf.Inf_Play_Custom((unsigned char *)lcd);
}

void CLcdTextCtr::testDispaly()
{
	printfLCDClear();
	printf(1, "debug message");
	printf(2, "*-------------------*");
	printf(3, "wait for calculate");
	printf(4, "wait for calculate");
	printf(5, "wait for calculate");
	printf(6, "wait for calculate");
	printf(7, "wait for calculate");
	printf(8, "wait for calculate");
	printf(9, "*-------------------*");
	usleep(1000 * 200);
}

void CLcdTextCtr::debugCleanMode(int type)
{	
	printfLCDClear();
	printf(1, "debug clean mode");

	if (type == 0){
		printf(2, "*-------------------*");
		printf(3, "-- auto clean --");
		printf(4, " + wall follow on + ");
		printf(5, " + s pattern on + ");
		printf(6, "empty");
		printf(7, "empty");
		printf(8, "empty");
		printf(9, "*-------------------*");
	}

	if (type == 1){
		printf(2, "*-------------------*");
		printf(3, "-- quick clean --");
		printf(4, " + wall follow off + ");
		printf(5, " + s pattern on + ");
		printf(6, "empty");
		printf(7, "empty");
		printf(8, "empty");
		printf(9, "*-------------------*");
	}

	if (type == 2){
		printf(2, "*-------------------*");
		printf(3, "-- wall clean --");
		printf(4, " + wall follow on + ");
		printf(5, " + s pattern off + ");
		printf(6, "empty");
		printf(7, "empty");
		printf(8, "empty");
		printf(9, "*-------------------*");
	}
	
	usleep(1000 * 200);
}

void CLcdTextCtr::DisplayQRCodeOnLCD(const char *data, int lcd_width, int lcd_height, int qr_size, int x_pos, int y_pos, int Color, int border_size, int border_color, int margin)
{

    QRcode *qr = QRcode_encodeString(data, 0, QR_ECLEVEL_L, QR_MODE_8, 1);

    if (qr != NULL) {
        char *lcd_data = (char *)malloc(lcd_width * lcd_height * sizeof(char));
        if (lcd_data == NULL) {
            QRcode_free(qr);
            return;
        }
        memset(lcd_data,0x00, lcd_width * lcd_height);

        int qr_with_border = qr->width + (2 * border_size) + (2 * margin);
        int scale_x = qr_size / qr_with_border;
        int scale_y = qr_size / qr_with_border;
        for (int y = 0; y < qr_with_border; y++) {
            for (int i = 0; i < scale_y; i++) {
                for (int x = 0; x < qr_with_border; x++) {
                    for (int j = 0; j < scale_x; j++) {
                        int qr_x = x - border_size - margin;
                        int qr_y = y - border_size - margin;
                        int lcd_x = x_pos + (x * scale_x) + j;
                        int lcd_y = y_pos + (y * scale_y) + i;
                        if (lcd_x >= 0 && lcd_x < lcd_width && lcd_y >= 0 && lcd_y < lcd_height) {
                            if (qr_x < 0 || qr_x >= qr->width || qr_y < 0 || qr_y >= qr->width) {
                                lcd_data[lcd_y * lcd_width + lcd_x] = border_color;
                            } else {
                                if (qr->data[qr_y * qr->width + qr_x] & 1) {
                                    lcd_data[lcd_y * lcd_width + lcd_x] = 0x00;
                                }else
                                {
                                    lcd_data[lcd_y * lcd_width + lcd_x] = 0xFF;
                                }
                            }
                        }
                    }
                }
            }
        }

        RoDisplayInf.Inf_Play_Custom((unsigned char *)lcd_data);
        free(lcd_data);
        QRcode_free(qr);
    } else {

    }
}

void CLcdTextCtr::printf(int idx, const char *ucFmt, ...) 
{
	va_list ap;
	
	memset(LCD_Buffer,0x00,DisplayWidthLength);
	va_start(ap,ucFmt);
	vsprintf(LCD_Buffer,ucFmt,ap);
	va_end(ap);

	printLCD(idx,LCD_Buffer);
}





