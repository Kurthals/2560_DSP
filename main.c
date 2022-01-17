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
enum tilstande {reset, run, calibrate, store};
char tilstand = run;
double angle = 0;
float modulus = 0;
int DFT_counter = 0;
char printbuffer[4]={0};
char DFTBuffer[2][64] = {{0},{0}}; //{0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128};
volatile char init_flag = 0;
char transmitflag = 0;
volatile char BTN5_flag, BTN4_flag, BTN3_flag;
char anglecounter = 0;
double anglebuf[AVERAGE_NUM]= {0};
char anglebufcnt = 0;
double anglemean = 0;


//char jern[NUM_MATERIAL_SAMPLES] = {0xFF};
//char kobber[NUM_MATERIAL_SAMPLES] = {0xFF};
//char messing[NUM_MATERIAL_SAMPLES] = {0xFF};
//char aluminium[NUM_MATERIAL_SAMPLES] = {0xFF};


//0:jern, 1:kobber, 2:messing, 3:aluminium
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
				if(detectPhase()==3){
					 sendStrXY("Alu",6,0);
				}
				
				
				if(BTN3_flag == 1){
 					nextState(calibrate);
 					BTN3_flag = 0;
				}
				if(BTN4_flag ==1){
					nextState(store);
					BTN4_flag = 0;
				}
				if(BTN5_flag == 1){
					nextState(reset);
					BTN5_flag =0;
				}
				
			break;	
			
			case calibrate:
				sendStrXY("Calibrate",1,7);
				nextState(run);
			
			break; 
			
			case store:
				sendStrXY("Store",1,7);
				nextState(run);
				
			break;
			
			case reset:
				nextState(run);
			
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
	 
	 //Setup PINS for buttons
	 CLRBIT(DDRE,5); //PE5 int5 pin3
	 CLRBIT(DDRE,4); //PE4 int4 pin2
	 CLRBIT(DDRD,3); //PD3 int3 pin18
	 //Internal Pull-up on inputs
	 SETBIT(PORTE,5); 
	 SETBIT(PORTE,4);
	 SETBIT(PORTD,3);
	 
	 //Configure falling edge detection on pins:
	 EICRA |= (1<<ISC31);
	 EICRB |= (1<<ISC41) | (1<<ISC51) ;	
	 
	 //Enable interrupts with EIMSK:
	 EIMSK |= (1<<INT5) | (1<<INT4) | (1<<INT3);
	 
	 //Enable global interrupt
	 sei();
	 
	 //OLED-display
	 _i2c_address = 0X78;
	 I2C_Init();
	 InitializeDisplay();
	 print_fonts();
	 clear_display();
	 sendStrXY("AMP",2,0);
	 sendStrXY("Angle:",3,0);
	 sendStrXY("Mode:",1,0);
	 sendStrXY("Material:",5,0);
	
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
 void computeDFT(){
	 if(DFT_ready == 1){
		 
		 for(int i = 0; i<NUM_SAMPLES; i++){
			 Re += ReTrig[i]*(DFTBuffer[!active_write][i]*5)/BIT_DIV;
			 Im += ImTrig[i]*(DFTBuffer[!active_write][i]*5)/BIT_DIV;
			 //Im = -Im;
		 }
		 Im = -Im;
		 modulus =(0.6*modulus)+(0.4*sqrtf((Im*Im) + (Re*Re))/16);
		 debug_print_float(modulus,2,7);
		
		 if(Im == 0 && Re == 0){
			 angle = 0;
		 }
		 else{
		     angle = (0.6*angle)+((0.4*(180/M_PI)*atan2((double)Im, (double)Re)));
		 }
// ================================================
// Moving average function - no longer necessary
// ================================================		 
		 // Averaging with AVERAGE_NUM-number of calculated angles.
		 
// 		 anglebuf[anglebufcnt]=angle;
// 		 anglebufcnt++;
// 		 
// 		 for(int i = 0; i<AVERAGE_NUM; i++){
// 			 anglemean += anglebuf[i];
// 			 
// 		 }
// 		 anglemean = anglemean/AVERAGE_NUM; //Eller divider med Average_num
// 		 if(anglebufcnt == AVERAGE_NUM){
// 			 anglebufcnt = 0;
// 		 }
// 		 
		 
		 debug_print_float(angle,3,7);
		 Re = 0;
		 Im = 0;
		 anglemean = 0;
		 DFT_ready = 0;
	 }
 }


// ================================================
// Main functionality
// ================================================
char detectPhase(){
	
	//Check if signal amplitude is above threshold
	if(modulus>AMP_THRESHOLD){
		//Detect material from phase
		char result[2] = {0};
		char hits = 0;
		for(int i = 0; i<NUM_MATERIALS; i++){
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
	return 0xFF;
}


void loadMaterials(){
	for(int i = 0; i< NUM_MATERIAL_SAMPLES; i++){
		materials[3][i] = 116; 
	}
}



// ================================================
// Utils
// ================================================

void debug_print_float(float input,char x, char y){
	char temp[100] = {0};
	dtostrf(input,4,2,temp);
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

ISR(INT4_vect){
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