/*
 * main.h
 *
 * Created: 07-06-2021 13:28:22
 *  Author: Jan
 */ 

#ifndef DEVEL
#define DEVEL 1
#endif

#define F_CPU 16000000UL
#define datasize 1007

#ifndef MAIN_H_
#define MAIN_H_

// ================================================
// Includes
// ================================================
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <string.h>
#include "USART.h"
#include "ADC.h"
#include "Timer.h"
#include <math.h>

#include "I2C.h"
#include "ssd1306.h"

// ================================================
// Defines/macros
// ================================================
#define SETBIT(ADDR, BIT)(ADDR |= (1<<BIT))
#define CLRBIT(ADDR, BIT)(ADDR &= ~(1<<BIT))
#define CHKBIT(ADDR, BIT)(ADDR & (1<<BIT))
#define TOGGLEBIT(ADDR,BIT)(ADDR ^= (1<<BIT))

#define NUM_SAMPLES 64
#define BIT_DIV 256


#define AMP_THRESHOLD 1
#define NUM_MATERIALS 4
#define NUM_MATERIAL_SAMPLES 20
#define MATERIAL_DEVIATION 2	//deviation from material phase in degrees
#define AVERAGE_NUM 10


// ================================================
// Functions
// ================================================
void setup();
void nextState(char input);
void init_trigonometry();
void computeDFT();
char detectMaterial();

int intToAscii(int number);
void debug_print_char(float input, char x, char y);









#endif /* MAIN_H_ */