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
int Amplitude[2] = {0};
int Phase[2] = {0};
char active_write = 0;
char active_read = 1;
char AmpTrig[NUM_SAMPLES]={0};
char PhaseTrig[NUM_SAMPLES]={0};
char trig_count = 0;
char DFT_ready;
enum tilstande {reset, run};
char tilstand = run;
char angle=0;
int modulus=0;
int DFT_counter = 0;
char printbuffer[4]={0};


 int main(void){
	setup();
	
//Main loop	
	while(1){
		switch(tilstand){
			
			case run:
				if(DFT_ready==1){
					
					if(DFT_counter == 2){
						
						modulus = (modulus*0.9)+(0.1*sqrt(Phase[active_read]*Phase[active_read]+Amplitude[active_read]*Amplitude[active_read]));
						debug_print_char(modulus,1,7);
						angle = (180/M_PI)*atan2(Phase[active_read],Amplitude[active_read]);
						debug_print_char(angle,2,7);
						DFT_counter = 0;
						debug_print_char(ADC_value_output,4,7);
 						formatADCSample(ADC_value,printbuffer);
						sendStrXY(printbuffer,3,7);
					}
					else{
						DFT_counter++;
					}
					DFT_ready = 0;
				}
				
			break;		
		}
	 
	}
 }
 
 
 
// Function initializations


 void setup(){
	 sei();
	 SETBIT(DDRB,6);
	 SETBIT(DDRB,5);
	 CLRBIT(PORTB,5);
	 CLRBIT(PORTB,6);
		 
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
	 
	 //if(uart_type==0x01){
		 //return set_gen;
	 //}
	 //if(uart_type==0x02){
		 //return set_sample;
	 //}
	 //if(uart_type==0x03){
		 //return BodePlot;
	 //}
	 //else return scope;
 }
 
 
 //Preload Trigonometric values into buffers:
 void init_trigonometry(){
	 for(int i=0;i<NUM_SAMPLES;i++){
		 switch(trig_count){
			 case 0:
				AmpTrig[i]=1;
				PhaseTrig[i]=0;
				trig_count++;
				break;
				
			case 1:
				AmpTrig[i]=0;
				PhaseTrig[i]=1;
				trig_count++;
				break;
			
			case 2:
				AmpTrig[i]=-1;
				PhaseTrig[i]=0;
				trig_count++;
				break;
			
			case 3:
				AmpTrig[i]=0;
				PhaseTrig[i]=-1;
				trig_count = 0;
				break;
		 }
	 }
 }

//Int to ascii conversion
int intToAscii(int number) {
	return '0' + number;
}

void debug_print_char(char input,char x, char y){
			char temp[100] = {0};
		sprintf(temp,"%u",input);
		sendStrXY(temp, x,y);
}




//Service Routines

//Service routine for Timer1 Compare B
ISR(TIMER0_COMPA_vect){
//	TOGGLEBIT(PORTB,6);
 	if(timercount != 3){
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
	
	if(buffercounter < NUM_SAMPLES){
		Amplitude[active_write] = Amplitude[active_write]+(((ADC_value*5)/255)*AmpTrig[buffercounter]);
		Phase[active_write] = (Phase[active_write]+(((ADC_value*5)/255)*PhaseTrig[buffercounter]))*(-1);
		ADC_value_output=(ADC_value*5)/255;
		buffercounter++;
	}
	else{
		Amplitude[active_read] = 0;
		Phase[active_read] = 0;
		if(active_write == 0){
			active_write = 1;
			active_read = 0;
			}
		else{
			active_write = 0;
			active_read = 1;
		}
		DFT_ready = 1;
		buffercounter = 0;
	}
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
		debug_print(uart_type,5);
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