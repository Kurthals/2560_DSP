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

#define DEBOUNCE 10

#define NUM_SAMPLES 64
#define BIT_DIV 256


#define AMP_THRESHOLD 2
#define PHASE_TOLERANCE 5
#define NUM_MATERIALS 5
#define NUM_MATERIAL_SAMPLES 20
#define MATERIAL_DEVIATION 2	//deviation from material phase in degrees
#define AVERAGE_NUM 10
#define NUM_PHASE_STABILITY_SAMPLES 10

#define IRON_PHASE	10
#define COPPER_PHASE -40
#define BRASS_PHASE 10
#define ALUMINUM_PHASE -61

//TODO calibration

// ================================================
// Functions
// ================================================
void setup();
void nextState(char input);
void init_trigonometry();
char computeDFT();
char detectMaterial();
void loadMaterials();
char checkPhaseStability();
void calibratePhase(char materialID);
void defaultDisplay();
void calibrateDisplay();
int intToAscii(int number);
void debug_print_float(float input, char x, char y);
void printMaterial(char materialID);
void(* resetFunc) (void);








#endif /* MAIN_H_ */