/* Digital_Design.c
 *
 * Created: 04-06-2021 13:21:47
 * Author : Jan & Lars
 */ 


#include "main.h"
unsigned char timercount = 0;
unsigned char ADC_start_flag = 0;
char OLED_buffer[20];
int ADC_value =0;
int ADC_value_output=0;

char ADC_buffer[4]={0};
int buffercounter = 0;
char active_write = 0;
char active_read = 1;
int ReTrig[NUM_SAMPLES]={0};
int ImTrig[NUM_SAMPLES]={0};
char trig_count = 0;
char DFT_ready;
enum tilstande {reset, run, calibrate, store};
char tilstand = run;
double angle = 0;
float modulus = 0;
int DFT_counter = 0;
char printbuffer[4]={0};
char DFTBuffer[64] = {0}; //{0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128,0,128,255,128};
float Re = 0; 
float Im = 0;
char transmitflag = 0;
volatile char BTN5_flag, BTN4_flag, BTN3_flag;
char anglecounter = 0;
double anglebuf[AVERAGE_NUM]= {0};
char anglebufcnt = 0;
double anglemean = 0;

//
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
				computeDFT();
				
				debug_print_char(atan2(-23,40)*(180/M_PI),5,0);
// 				if(BTN3_flag == 1){
// 					_delay_ms(20);
// 					nextState(calibrate);
// 					BTN3_flag = 0;
// 				}
// 				if(BTN4_flag ==1){
// 					_delay_ms(20);
// 					nextState(store);
// 					BTN4_flag = 0;
// 				}
// 				if(BTN5_flag == 1){
// 					_delay_ms(20);
// 					nextState(reset);
// 					BTN5_flag =0;
// 				}
				
			break;	
			
			case calibrate:
			
			
			break; 
			
			case store:
				
			break;
			
			case reset:
			
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
	 SETBIT(PORTE,5);
	 SETBIT(PORTE,4);
	 SETBIT(PORTD,3);
	 
	 //Configure Rising edge detection on pins:
	 EICRA |= (1<<ISC31) | (1<<ISC30);
	 EICRB |= (1<<ISC41) | (1<<ISC40) | (1<<ISC51) | (1<<ISC50);	
	 
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
	 sendStrXY("AMP",1,0);
	 sendStrXY("Angle:",2,0);
	
	 //init_timer0();
	 init_timer0();
	 
	 //Fill Trigonometric array
	 init_trigonometry();
		 
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
			 Re += ReTrig[i]*(DFTBuffer[i]*5)/BIT_DIV;
			 Im += ImTrig[i]*(DFTBuffer[i]*5)/BIT_DIV;
			 //Im = -Im;
		 }
		 //Im = -Im;
		 modulus =(0.9*modulus)+(0.1*sqrtf((Im*Im) + (Re*Re))/16);
		 debug_print_char(modulus,1,7);
		
		 if(Im == 0 && Re == 0){
			 angle = 0;
		 }
		 else{
			 angle = (0.9*angle)+((0.1*(180/M_PI)*atan2((double)Im, (double)Re)));
		//	 angle = (180/M_PI)*atan2((double)Im, (double)Re);
		 }
		 
		 anglebuf[anglebufcnt]=angle;
		 anglebufcnt++;
		 
		 for(int i = 0; i<AVERAGE_NUM; i++){
			 anglemean += anglebuf[i];
			 
		 }
		 anglemean = anglemean/AVERAGE_NUM;
		 if(anglebufcnt == AVERAGE_NUM){
			 anglebufcnt = 0;
		 }
// 		 anglecounter ++;
// 		 if(anglecounter == 10){
// 		 debug_print_char(anglemean,2,7);
// 		 anglecounter = 0;
// 		 }
		 debug_print_char(anglemean,2,7);
		 Re = 0;
		 Im = 0;
		 DFT_ready = 0;
	 }
 }


// ================================================
// Main functionality
// ================================================
char detectMaterial(){
	
	//Check if signal amplitude is above threshold
	if(modulus>AMP_THRESHOLD){
		//Detect material from phase
		char result[2] = {0};
		char hits = 0;
		for(int i = 0; i<NUM_MATERIALS; i++){
			hits = 0;
			for(int j = 0; j<NUM_MATERIAL_SAMPLES; j++){
				if(materials[i][j] < (angle+MATERIAL_DIVIATION) && materials[i][j] > (angle-MATERIAL_DIVIATION)){
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






// ================================================
// Utils
// ================================================

//Int to ascii conversion
int intToAscii(int number) {
	return '0' + number;
}

void debug_print_char(float input,char x, char y){
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
	ADC_value = ADCH;
	TOGGLEBIT(PORTB,5);
	
	if(buffercounter < NUM_SAMPLES && !DFT_ready){
		DFTBuffer[buffercounter] = ADCH;
		buffercounter++;
	}else{
		DFT_ready = 1;
		buffercounter = 0;
	}
}


//Service routines for external interrupts (buttons)
ISR(INT3_vect){
	BTN3_flag = 1;
}

ISR(INT4_vect){
	BTN4_flag = 1;
}

ISR(INT5_vect){
	BTN5_flag = 1;
}
























	

/*
//Holds latest adc sample -> read on adc interrupt
volatile char adc_flag = 0; 
unsigned int bufferCounter = 0;
char sampleBuffer[2][SAMPLE_BUF] = {{0},{0}};
int adc_user = 0;
int uart_user = 1;		//TODO char??

//unsigned int sampleRateTarget = 1000;
unsigned int recordLength = 500;	
unsigned int nextRecordLenght = 0;

int main(void){ 
    
	setup();
	setSampleRate(7000);
    
    while (1){
		
	//Main tilstandsmaskine
	//Reagerer p� uart-receive-flag. Scope er begyndelsestilstanden og herfra kaldes funktionen Handle_type.
	//Dermed skiftes der tilstand baseret p� den modtagne uart-type. 
	switch(tilstand){
		
		//Grundtilstand. Tjek for uart-flag. skift tilstand baseret p� uart-type. 
		case scope:
		debug_print_int(recordLength);
 			if(adc_flag){
	 			transmitUARTPackage(&sampleBuffer[uart_user][0], SCOPE_TYPE, recordLength);
				 adc_flag = 0;
				 
 			}
			if(flag_uart_rx==1){
				flag_uart_rx=0;
				tilstand = handle_type(uart_type);
			}
			break;
		
		//"Send" er modtaget. Opdat�r S_rate og RL.
		case set_sample:
			S_Rate = ((unsigned int)data[0]<<8)|(unsigned int)data[1];
			RL = ((unsigned int)data[2]<<8)|(unsigned int)data[3];
			setSampleRate(S_Rate);
			recordLength = RL;
			tilstand = scope;		
			break;
		
		//Knaptryk fra "Generator" modtaget. Funktionen handle_generator behandler tastetryk. 
		case set_gen:
		handle_generator();
		tilstand = scope;
		break;
		
		//"Start" er modtaget. Spi-pakken-opdateres og der loopes med increments af 1Hz.
		case BodePlot:
		spi_package[0]=7;
		for(int i = 1; i<=255;i++){//adjust frequency 1 hz pr step
			spi_package[1]=i;
			//send SPI package
			//vent - record sample (delay)
			//bode_data[i-1]=ADC-sample
		}
		//Transmit UART datapackage
		tilstand = scope;
		
		
		break;
	}
		
		
		
		for(int i=0;i<10;i++){
			OLED_buffer[i]=data[i]+0x30;
		}		
		sendStrXY(OLED_buffer,4,5);
		
		(uart_type,5);
		debug_print(rec_complete,6);
		debug_print(checksum_flag,7);
		sendStrXY("Data:",4,0);
 		sendStrXY("Type:",5,0);
 		sendStrXY("Rec_comp:",6,0);
 		sendStrXY("Checksum_f:",7,0);

		
		
	
	}
}

// ================================================
// Service Routines
// ================================================

//Service routine for ADC sample ready
ISR(ADC_vect){
	sampleBuffer[adc_user][5+bufferCounter++] = ADCH;
	
	if(bufferCounter >= recordLength){
		adc_flag = 1;
		
		adc_user = !adc_user;
		uart_user = !uart_user;
			
		bufferCounter = 0;
	}
	
	//Overflow
	//if(bufferCounter[adc_user][0] > SAMPLE_BUF){
		//bufferCounter[adc_user][0] = 0;
	//}
}

//Service routine for Timer1 Compare B
ISR (TIMER1_COMPB_vect) {
}

//Service routine for UART receive vector
ISR(USART1_RX_vect){
	UARTBuffer[uart_cnt_rx] = UDR1;
	flag_uart_rx = 1; 
	evaluate_recieve();
 }


 // ================================================
// Functions
// ================================================
void setup(){
		
	//UART
	init_uart_interrupt1(UBBR_D);

	//Timers
	init_timer1();
	
	//ADC
	init_adc(1);
	startADCSampling(ADC_CHANNEL);

	//Interrupt
	sei();
	
	//OLED-display
	_i2c_address = 0X78;
	I2C_Init();
	InitializeDisplay();
	print_fonts();
	clear_display();
}

//Funktion som returnerer tilstande p� baggrund af den l�ste Type modtaget i telemetry. 
enum tilstande handle_type(char input){
	if(uart_type==0x01){
		return set_gen;
	}
	if(uart_type==0x02){
		return set_sample;
	}
	if(uart_type==0x03){
		return BodePlot;
	}
	else return scope;
}


void debug_print(char input, int value){
	char temp[100]={0};
	sprintf(temp,"%u",input);
	sendStrXY(temp,value,13);
}


//Funktion, som skelner mellem tastetryk i generator-fanen.
//BTN-byte og SW-byte gemmes i hver sin variabel. 
void handle_generator(){
	BTN = data[0];
	SW = data[1];

//Tjek v�rdien af BTN	
	switch(BTN)
	{

//ENTER: konstru�r en SPI-datapakke med det tilsvarende dataindhold.
//Ligeledes opdateres telecommand-pakken.
		case ENTER: 
			if (param == shape_s){
				telecommand[1] = SW;
				spi_package[0]=4;
				spi_package[1]=SW;	
			}
			
			if (param == freq_s){
				telecommand[3] = SW;	
				spi_package[0]=7;
				spi_package[1]=SW;
				
			}
			if (param == amplitude_s){
				telecommand[2] = SW;
				spi_package[0]=5;
				spi_package[1]=SW;
				
				
			}
		break;

//SELECT: Tilstandsloop, som gemmer v�rdien af den nuv�rende valgte parameter (amplitude, frekvens eller shape). 
//Opdater telecommandpakken med den tilsvarende v�rdi. 
		case SELECT:
			switch(param){
				case shape_s:
				
				telecommand[0]=1;
				//transmit Generator DATA UART	
				param = amplitude_s;
				break;
				
				case amplitude_s:
				telecommand[0]=2;
				//transmit_generator
				param = freq_s;
				break;
				
				case freq_s:
				telecommand[0] = 0;
				//transmit_generator
				param = shape_s;
				break;
			}
			break;

//Run/Stop: Toggle stop-char mellem de to start/stop v�rdier. Opdat�r SPI-pakken med tilh�rende v�rdi.		
		case START_STOP: //run/stop
			//toggle bit-0;
			TOGGLEBIT(stop,0);
// 			if(stop == 0x03){
// 				stop = 0x02;
// 			}
			spi_package[0] = stop;
			spi_package[1] = 0;
			//send stop-byte on SPI
			break;

//RESET: Toggle reset-byte og opdater dette i spi-package. 		
		case RESET:
			
			spi_package[0] = RESET_SPI;
			spi_package[1] = 0;
			//send reset_byte + 0data SPI;
			

			putCharUSART(0xff);
			break;
			
	}
}

//Tilstandsmaskine, som genneml�ber datapakkens bestandele. 
void evaluate_recieve(){
	switch(state){
		
		//Tjek om f�rste karakter er 0x55 og skift tilstand hvis sand. 
		case sync1:
		if(UARTBuffer[uart_cnt_rx++] == 0x55){
			state = sync2;
		}
		break;
		
		//Tjek om anden karakter er 0xAA og skift tilstand hvis sand. Ellers skift til tilstand Sync 1 igen.
		case sync2:
		if(UARTBuffer[uart_cnt_rx++]==0xAA){
			state = Length;
			}
		else{
			state = sync1;
		}
		break;
		
		
		//L�s l�ngden af den modtagne pakke (byte1)
		case Length:
		Len = (UARTBuffer[uart_cnt_rx++]<<8);
		state = Length2;
		break;
		
		//L�s l�ngden af den modtagne pakke (byte2)
		case Length2:
		Len = Len + (UARTBuffer[uart_cnt_rx++]);
		compare = Len-2;
		state = Type;			
		break;
		
		//L�s type-byten og gem den i en char. 
		case Type:
		uart_type = UARTBuffer[uart_cnt_rx++];
		state = ReadData;
		break;
		
		//L�s data, hvis der findes databytes i pakken og gem det i data[]  IF ELSE
		case ReadData:
		if(Len>7){
			if(uart_cnt_rx < (compare)){
				data[uart_cnt_rx-5]=UARTBuffer[uart_cnt_rx];
				uart_cnt_rx++;
//				break;
				}
				
		//Hvis hele datapakken er l�st og gemt skiftes tilstand. 	
			if(uart_cnt_rx==(compare)){
				state = CS1;
				uart_cnt_rx++;
			}
		}
		else state = CS1;
		break;
		
		//L�s f�rste checksum-byte
		case CS1:
		checksum_val = (UARTBuffer[uart_cnt_rx++]<<8);
		state = CS2;
		break;
		
		//L�s anden checksum-byte og kontroller om den nye int checksum_val == 0x000
		case CS2:
		checksum_val = checksum_val + (UARTBuffer[uart_cnt_rx++]);
		if(checksum_val==0x0000){
			rec_complete=1;	
			uart_cnt_rx=0;
			checksum_flag=0;
			Len=0;
			state = sync1;
			}	
		else{
			checksum_flag=1;
			uart_cnt_rx=0;
			state = sync1;
			Len=0;
		}		
		break;
	}		
}



// ================================================
// Utils
// ================================================

unsigned int calcCheckSum(){
	return 0x0000; 
}


void debug_print_int(int input){
	if(DEVEL){
		char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, 0,0);
	}
}	


// ================================================
// ADC
// ================================================


//Calculate and set compare match value for ADC Auto Trigger Source based on target ADC sample rate value.
void setSampleRate(unsigned int sampleRate){
	int compareValue = (F_CPU/(2*sampleRate))/ADC_TRIG_SRC_PS-1;
	OCR1A = compareValue;
	OCR1B = compareValue;
	//debug_print_int(OCR1A);
}





// ================================================
// Serial
// ================================================

void transmitUARTPackage(char * data, unsigned char type, unsigned int dataSize){
		
		//Construct package		
		sampleBuffer[uart_user][0] = 0x55;
		sampleBuffer[uart_user][1] = 0xAA;
		sampleBuffer[uart_user][2] = (dataSize+PADDING_SIZE) >> 8;
		sampleBuffer[uart_user][3] = (dataSize+PADDING_SIZE);
		sampleBuffer[uart_user][4] = type;
		
		//for(int i = HEADER_SIZE; i < HEADER_SIZE+dataSize; i++){
			//UARToutputBuffer[i] = data[i-HEADER_SIZE];
		//}
		
		
		//int checksum = calcCheckSum();
				//UARToutputBuffer[505] = 0x00;
				//UARToutputBuffer[506] = 0x00;
		//UARToutputBuffer[HEADER_SIZE+dataSize] = 0x00;//checksum << 8;
		//UARToutputBuffer[HEADER_SIZE+dataSize+1] =0x00;// checksum; 
		sampleBuffer[uart_user][HEADER_SIZE+dataSize] = 0x00;//checksum << 8;
		sampleBuffer[uart_user][HEADER_SIZE+dataSize+1] = 0x00; //checksum;
		//Start transmission by sending first byte, then enable transmit interrupt
		//UDR1 = sampleBuffer[uart_user][0];//UARToutputBuffer[0];
		//SETBIT(UCSR1B, TXCIE1);
			
		//UDR1 = UARToutputBuffer[uart_cnt_tx++];
		for(int i = 0; i < recordLength+PADDING_SIZE; i++){
			putCharUSART(sampleBuffer[uart_user][i]);
		}
		
}
*/