/* Digital_Design.c
 *
 * Created: 04-06-2021 13:21:47
 * Author : Jan & Lars
 */ 


#include "main.h"
unsigned char timercount = 0;
unsigned char ADC_start_flag = 0;
char OLED_buffer[20];
int buffercounter = 0;
volatile char active_write = 0;

int ReTrig[NUM_SAMPLES]={0};
int ImTrig[NUM_SAMPLES]={0};
float Re = 0;
float Im = 0;

char trig_count = 0;
volatile char DFT_ready = 0; //Måske nødvendigt med Volatile
enum tilstande {reset, run, calibrate, select, store};
char tilstand = run;

double angle = 0;
double modulus = 0;

double angleThres[10] = {0};
char angleCnt = 0;

int DFT_counter = 0;
char printbuffer[4]={0};
char DFTBuffer[2][64] = {{0},{0}}; //{0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128};
volatile char init_flag = 0;
char transmitflag = 0;
volatile char BTN5_flag, BTN4_flag, BTN3_flag;
char anglecounter = 0;
double anglebuf[AVERAGE_NUM]= {0};
char anglebufcnt = 0;


enum default_materials {iron, copper, brass, aluminum, undefined};
char materialSelctor = undefined;

//Materials buffer
//0:jern, 1:kobber, 2:messing, 3:aluminum, 4:undefined
float materials[NUM_MATERIALS][NUM_MATERIAL_SAMPLES] = {
	{0xFF},
	{0xFF}
	};



 int main(void){
	setup();
	
//Main loop	
	while(1){
		switch(tilstand){
			
			case run:
				sendStrXY("Running  ",1,7);
				
				computeDFT();
				printMaterial(detectMaterial());
				
				//Cler status LED
				CLRBIT(PORTB,5);
				
				//Calibration button
				if(BTN3_flag == 1){
					CLRBIT(EIMSK,INT3);
					_delay_ms(DEBOUNCE);
					if(!CHKBIT(PIND,3)){
						nextState(select);
					}
 					BTN3_flag = 0;
					SETBIT(EIMSK,INT3);
				}

				if(BTN5_flag == 1){
					CLRBIT(EIMSK,INT5);
					_delay_ms(DEBOUNCE);
					if(!CHKBIT(PINE,5)){
						nextState(reset);
					}
					BTN5_flag =0;
					SETBIT(EIMSK,INT5);
				}
				break;	
			
			case calibrate:
				calibratePhase(materialSelctor);
				defaultDisplay();
				nextState(run);
				break; 
			
			case select:
				calibrateDisplay();
				nextState(store);				
				break;
			
			case store:
				//Select material
				if(BTN4_flag == 1){
					CLRBIT(EIMSK,INT2);
					_delay_ms(DEBOUNCE);
					if(!CHKBIT(PIND,2)){
						materialSelctor ++;
						if(materialSelctor>=NUM_MATERIALS) materialSelctor = 0;
						printMaterial(materialSelctor);
					}
					BTN4_flag = 0;
					SETBIT(EIMSK,INT2);
				}
				
				//Perform calibration when desired material has been selected
				if(BTN3_flag == 1){
					CLRBIT(EIMSK,INT3);
					_delay_ms(DEBOUNCE);
					if(!CHKBIT(PINE,3)){
						nextState(calibrate);
					}
					BTN3_flag = 0;
					SETBIT(EIMSK,INT3);
				}
				break;
			
			case reset:
				resetFunc();
				//nextState(run);
			
				break; 
		}
	}
 }
 


// ================================================
// Initialization
// ================================================

 void setup(){
	 
	 //Setup ADC PORT
	 SETBIT(DDRB,6);
	 CLRBIT(PORTB,6);
	 
	 //Setup LED PORT
	 SETBIT(DDRB,5);
	 CLRBIT(PORTB,5);
	 
	 //Setup PINS for buttons
	 CLRBIT(DDRE,5); //PE5 int5 pin3 (RESET)
	 CLRBIT(DDRD,2); //PE4 int2 pin2 (SET)
	 CLRBIT(DDRD,3); //PD3 int3 pin18 (CALIBRATE)
	 //Internal Pull-up on inputs
	 SETBIT(PORTE,5); 
	 SETBIT(PORTD,2);
	 SETBIT(PORTD,3);
	 
	 //Configure falling edge detection on pins:
	 EICRA |= (1<<ISC31) | (1<<ISC21);
	 EICRB |= (1<<ISC51) ;	
	 
	 //Enable interrupts with EIMSK:
	 EIMSK |= (1<<INT5) | (1<<INT2) | (1<<INT3);
	 
	 //Enable global interrupt
	 sei();
	 
	 //OLED-display
	 _i2c_address = 0X78;
	 I2C_Init();
	 InitializeDisplay();
	 print_fonts();
	defaultDisplay();
	
	 //init_timer0();
	 init_timer0();
	 
	 //Fill Trigonometric array
	 init_trigonometry();
	 
	 loadMaterials();
		 
 }
 
 
  //Set next state in state machine
  void nextState(char input){
	  tilstand = input;
	  return;
  }

 

 
 
 
 // ================================================
 // DFT
 // ================================================
 
 //Pre-load Trigonometric values into buffers:
 void init_trigonometry(){
	 for(int i = 0; i<NUM_SAMPLES; i++){
		switch(trig_count){
			case 0:
			ReTrig[i]=1;
			ImTrig[i]=0;
			trig_count++;
			break;
			 		 
			case 1:
			ReTrig[i]=0;
			ImTrig[i]=1;
			trig_count++;
			break;
			 		 
			case 2:
			ReTrig[i]=-1;
			ImTrig[i]=0;
			trig_count++;
			break;
			 		 
			case 3:
			ReTrig[i]=0;
			ImTrig[i]=-1;
			trig_count = 0;
			break;
		}
	 }
 }


//Compute DFT for latest sample
//Returns true if DFT was succesfully calculated, false otherwise
 char computeDFT(){
	 if(DFT_ready == 1){
		 
		 for(int i = 0; i<NUM_SAMPLES; i++){
			 Re += ReTrig[i]*(DFTBuffer[!active_write][i]*5)/BIT_DIV;
			 Im += ImTrig[i]*(DFTBuffer[!active_write][i]*5)/BIT_DIV;
		 }
		 Im = -Im;

		 modulus =(0.6*modulus)+(0.4*sqrtf((Im*Im) + (Re*Re))/16);
		 debug_print_float(modulus,2,7);

		 modulus = (0.6*modulus)+(0.4*sqrtf((Im*Im) + (Re*Re))/16);
		 debug_print_float(modulus,2,7);

		
		 if(Im == 0 && Re == 0){
			 angle = 0;
		 }
		 else{
		     angle = (0.6*angle)+((0.4*(180/M_PI)*atan2((double)Im, (double)Re)));
		 }	
		 //Save to phase history  
		 angleThres[angleCnt] = angle;
		 angleCnt ++; 
		 if(angleCnt >= 10) angleCnt = 0; 
		 
		 debug_print_float(angle,3,7);
		 Re = 0;
		 Im = 0;
		 DFT_ready = 0;
		 return 1; 
	 }
	 return 0;
 }


// ================================================
// Main functionality
// ================================================

//Detect material from signal phase
//Returns material ID if phase is matched. 0xFF otherwise
char detectMaterial(){
	
	//Check if signal amplitude is above threshold
	if(modulus>AMP_THRESHOLD){
		//Check stability of phase
		if(checkPhaseStability()){
			
			//Enable status LED
			SETBIT(PORTB,5);
			
			//Detect material from phase
			char result[2] = {0,undefined};
			char hits = 0;
			for(int i = 0; i<NUM_MATERIALS-1; i++){
				hits = 0;
				for(int j = 0; j<NUM_MATERIAL_SAMPLES; j++){
					if(materials[i][j] < (angle+MATERIAL_DEVIATION) && materials[i][j] > (angle-MATERIAL_DEVIATION)){
						hits ++; 
					}
					if(hits>result[0]){
						 result[0]=hits;
						 result[1]=i;	//Save material with most hits
					}
				}
			}
			//Return material with most matched phase "hits"
			return result[1];
		}
	}
	return undefined;
}

//Pre-load default material phases
void loadMaterials(){
	//Iron
	for(int i = 0; i< NUM_MATERIAL_SAMPLES; i++){
		materials[iron][i] = IRON_PHASE;
	}
	//Copper
	for(int i = 0; i< NUM_MATERIAL_SAMPLES; i++){
		materials[copper][i] = COPPER_PHASE; 
	}

	//Brass
	for(int i = 0; i< NUM_MATERIAL_SAMPLES; i++){
		materials[brass][i] = BRASS_PHASE;
	}
	//Aluminum
	for(int i = 0; i< NUM_MATERIAL_SAMPLES; i++){
		materials[aluminum][i] = ALUMINUM_PHASE;
	}
}


//Check if phase from latest X samples is stable(within threshold)
//Returns TRUE if phase is stable. FALSE otherwise
char checkPhaseStability(){
	for(int i = 0; i<NUM_PHASE_STABILITY_SAMPLES; i++){
		 if(angleThres[i] > angle+PHASE_TOLERANCE || angleThres[i] < angle-PHASE_TOLERANCE) return false;
	}
	return true;
}


//Calibrates phase detection by saving samples to memory. User specifies material in menu
//INPUT: material ID to sample
void calibratePhase(char materialID){	
	char numAttempts = 0; 
	//Normal materials
	if(materialID < 4){
		for(int i = 0; i<NUM_MATERIAL_SAMPLES;){
			if(computeDFT()){	
				if(modulus>AMP_THRESHOLD){
					//Check stability of phase
					if(checkPhaseStability()){
						materials[materialID][i] = angle;
						i++;
						continue;
					}
				}
				//Error handling
				numAttempts ++;
				if(numAttempts>NUM_MATERIAL_SAMPLES) return;
			}
		}
	}
	//Background noise (should be 0)
	else{
		for(int i = 0; i<NUM_MATERIAL_SAMPLES;){
			if(computeDFT()){
				//Check stability of phase
				if(checkPhaseStability()){
					materials[undefined][i] = angle;
					i++;
					continue;
				}
				//Error handling
				numAttempts ++;
				if(numAttempts>NUM_MATERIAL_SAMPLES) return;
			}
		}
	}
}



// ================================================
// Utils
// ================================================

//Reset function for manual software reset. Address 0
 void(* resetFunc) (void) = 0;


//Print default text on display
void defaultDisplay(){
	clear_display();
	sendStrXY("Mode:",1,0);
	sendStrXY("AMP",2,0);
	sendStrXY("Angle:",3,0);
	sendStrXY("Material:",5,0);
}

void calibrateDisplay(){
	clear_display();
	sendStrXY("Select material",1,0);
	sendStrXY("Material:",5,0);
}

//Prints material to display
//INPUT: material ID
void printMaterial(char materialID){
	switch(materialID){
		case iron:
			sendStrXY("IRO",5,10);
			break;
		case copper:
			sendStrXY("COP",5,10);
			break;
		case brass:
			sendStrXY("BRA",5,10);
			break;
		case aluminum:
			sendStrXY("ALU",5,10);
			break;
		case undefined:
			sendStrXY("UND",5,10);
			break;
	}
}


void debug_print_float(float input,char x, char y){
	char temp[100] = {0};
	dtostrf(input,5,2,temp);
	//sprintf(temp,"%d",input);
	sendStrXY(temp, x,y);
}









// ================================================
// Service Routines
// ================================================

//Service routine for Timer1 Compare B
ISR(TIMER0_COMPA_vect){
//	TOGGLEBIT(PORTB,5);
 	if(timercount != 1){
 		timercount++;		
 	}
 	else{
 		timercount = 0;
 		TOGGLEBIT(PORTB,6);	
		//Start ADC sampling
		if(CHKBIT(PORTB,6) == 0 && ADC_start_flag == 0){
			ADC_start_flag = 1;
			init_adc(1);
			}
		}
}



//Service routine for ADC sample ready
ISR(ADC_vect){
	if(buffercounter < NUM_SAMPLES){
		DFTBuffer[active_write][buffercounter] = ADCH;
		buffercounter++;
	}
	else if(DFT_ready){
		buffercounter = 0;
		DFTBuffer[active_write][buffercounter] = ADCH; 
		buffercounter++;
	}
	else{
		active_write = !active_write;
		DFT_ready = 1;
		buffercounter = 0;
		DFTBuffer[active_write][buffercounter] = ADCH;	//Save remaining first sample 
		buffercounter++;  
	}
}


//Service routines for external interrupts (buttons)
ISR(INT3_vect){
	if(init_flag == 1){
		init_flag = 0;
	}
	else{
	BTN3_flag = 1;
	}
}

ISR(INT2_vect){
	if(init_flag == 1){
		init_flag = 0;
	}
	else{
		BTN4_flag = 1;
	}
}

ISR(INT5_vect){
	if(init_flag == 1){
		init_flag = 0;
	}
	else{
		BTN5_flag = 1;
	}
}