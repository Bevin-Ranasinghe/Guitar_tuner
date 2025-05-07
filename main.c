#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define BAUD 76800  //19200
#define myUBBR0 F_CPU/16/BAUD-1
#define BUFFERSIZE 1000

volatile uint16_t tick125us = 0;
volatile uint8_t takeADC = 0;
volatile uint8_t button_pressed = 0;
volatile uint8_t recordToBuffer = 0;
volatile uint8_t zeroCrossingCount = 0;

typedef struct {
	uint8_t buffer[BUFFERSIZE];
	uint16_t head;
	uint16_t tail;
	uint16_t BufferCount;
}CircularBuffer;

volatile CircularBuffer ADCvalueBuffer;

void InitializeIO(void) {
	DDRB |= (1 << PORTB5); //PB5 as output
	DDRB &= ~(1 << PORTB7); //internal button as an input    FOR TEST PURP
	PORTB |= (1 << PORTB7); //pull-up    
}
//timer0 setup
void InitializeTimer0(void) {
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR0A |= (1 << WGM01); //CTC
	TCCR0B |= (1 << CS01);  // prescaler 8   -> 010->8, 011->64, 100->256, 101->1024
	OCR0A = 249;  //for each tick = 125us   (0.000125*16000000)/8 - 1 = 249
	TIMSK0 |= (1 << OCIE0A);  //compare A match interrupt enabled
}

void InitializeUart0(unsigned int ubrr) {//setting up UART0
	//Baud rate   //  UBRR0 = 16000000/(16*19200)-1 = 51
	UBRR0 = 0;
	UBRR0H = (unsigned char)(ubrr >> 8); //BRR Low and High
	UBRR0L = (unsigned char)ubrr;
	UCSR0B |= (1 << TXEN0); //Enable TX
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void InitializeAdc(void) {
	//setting up ADC
	ADMUX = 0; //clear everything just in case
	ADMUX |= (1 << REFS0); //V_ref as AV_cc
	//MUX is already at 000
	ADCSRA = 0; //clear everything just in case
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // | (1 << ADPS0);  // enabling ADC and setting prescaler to 64 (before was128)
	//   16MHz/128 = 125kHz
	ADCSRB = 0;
}

void InitializeBuffer(CircularBuffer *ADCvalueBuffer){
	ADCvalueBuffer->head = 0;
	ADCvalueBuffer->tail = 0;
	ADCvalueBuffer->BufferCount = 0;
}

void addToCircularBuffer(CircularBuffer *ADCvalueBuffer, uint8_t value){
	if(ADCvalueBuffer->BufferCount == BUFFERSIZE){//if buffer is full
		ADCvalueBuffer->tail = (ADCvalueBuffer->tail + 1) % BUFFERSIZE;
	}
	else{
		ADCvalueBuffer->BufferCount++; //we added 1 item to the buffer
	}
	ADCvalueBuffer->buffer[ADCvalueBuffer->head] = value;
	ADCvalueBuffer->head = (ADCvalueBuffer->head + 1) % BUFFERSIZE;
}

uint8_t averageBuffer(CircularBuffer *ADCvalueBuffer) {
	if(ADCvalueBuffer->BufferCount == 0){return 0;} //if buffer empty, return 0
	uint32_t sum = 0, temp = 0;
	uint16_t index = ADCvalueBuffer->tail;

	for(uint16_t i = 0; i < ADCvalueBuffer->BufferCount; ++i){
		temp = ADCvalueBuffer->buffer[(index+i) % BUFFERSIZE];
		sum += temp; 
	}
	return sum/(ADCvalueBuffer->BufferCount);
}

uint8_t MinMaxBuffer(CircularBuffer *ADCvalueBuffer) {
	if(ADCvalueBuffer->BufferCount == 0){return 0;} //if buffer empty, return 0
	uint32_t sum = 0, current_value;
	uint16_t index = ADCvalueBuffer->tail;
	uint8_t max_value = 0;
	uint8_t min_value = 0; 
	uint8_t last_value = 0;
	
	for(uint16_t i = 0; i < ADCvalueBuffer->BufferCount; ++i){
		current_value = ADCvalueBuffer->buffer[(index+i) % BUFFERSIZE];
		if(last_value>current_value){
			max_value = last_value;
		}
		else if(last_value<current_value){
			min_value = last_value;
		}
		
		last_value = current_value;
	}
	return (max_value - min_value);
}

uint16_t zeroCrossingBuffer(CircularBuffer *ADCvalueBuffer, uint8_t average){
	if(ADCvalueBuffer->BufferCount == 0){return 0;} //if buffer empty, return 0
	//uint8_t averageBufferValue = averageBuffer(&ADCvalueBuffer);
	uint16_t index = ADCvalueBuffer->tail;
	uint8_t value, lastState = 0;
	zeroCrossingCount = 0;
	for(uint16_t i = 0; i < ADCvalueBuffer->BufferCount; ++i){
		value = ADCvalueBuffer->buffer[(index+i) % BUFFERSIZE];
		if((value > average) && (lastState == 0)) {
			zeroCrossingCount++;
			lastState = 1;
		}
		else if((value < average) && (lastState == 1)){
			zeroCrossingCount++;
			lastState = 0;
		}
	}
	
	return (zeroCrossingCount / 2);
}

void sendBuffer(CircularBuffer *ADCvalueBuffer){
	if(ADCvalueBuffer->BufferCount == 0){
		USART0_TransmitStr("Nothing in the buffer");
	}
	else{
		uint16_t index = ADCvalueBuffer->tail;
		
		for(uint16_t i = 0; i < ADCvalueBuffer->BufferCount; ++i){
			/*if(tick125us % 2 == 0){
				
			
			}*/
			USART0_Transmit(ADCvalueBuffer->buffer[(index+i) % BUFFERSIZE]);
		}
	}
}


//sending a byte
void USART0_Transmit(unsigned char data) { //send byte
	while (!(UCSR0A & (1 << UDRE0))); //wait for empty buffer
	UDR0 = data;
}
void USART0_Transmit16bits(uint16_t bits){
	USART0_Transmit((bits >> 8) & 0xFF); // high byte
	USART0_Transmit(bits & 0xFF);
}
void USART0_TransmitStr(const char *str) { //from string, send byte by byte to function above
	while (*str) {
		USART0_Transmit(*str++);
	}
}
void USART0_TransmitInt(uint16_t value) { //to transmit uint16_t , didnt work for me, was bad  DIDNT WORK PROBABLY BECAUSE ITS LATER TURND INTO STR, AND CANNOT DRAW STR IN GRAPH!!!!!! ONLY IN TERMINAL
	char buffer[10]; // 9 digits + terminator
	itoa(value, buffer, 10); // Converts int to string, base 10
	USART0_TransmitStr(buffer);  
	USART0_TransmitStr("\r\n"); //add this line back if you want to see every packet on different line
}

uint16_t read_ADC(void) {
	ADCSRA |= (1 << ADSC); //Begin conversion
	while (ADCSRA & (1 << ADSC)); //waiting until its done
	return ADC;  //return it to place read_ADC was called from
}


volatile uint16_t bufferfull = 0;
volatile uint8_t mic_adc;
ISR(TIMER0_COMPA_vect){
	tick125us++;
	if((tick125us % 2) == 0){ //take ADC every 250us
		mic_adc = read_ADC()/4;
		//USART0_Transmit(mic_adc);
		if(!button_pressed){ //stop adding values when sending them
		addToCircularBuffer(&ADCvalueBuffer, mic_adc);
		}
	}
	
	if(tick125us >= 4000){
		tick125us = 0;
		PORTB ^= (1 << PORTB5);
	}
}

int main(void) {
	InitializeIO();
	InitializeAdc();
	InitializeTimer0();
	InitializeUart0(myUBBR0);
	InitializeBuffer(&ADCvalueBuffer);
	sei();
	uint8_t averageValue;
	uint8_t zeroCros;
	uint16_t freq;
	uint8_t currentPP;
	uint8_t Our0Crossing;
	while (1) {
		
		
		if(button_pressed){
			button_pressed = 0;
			//USART0_Transmit(250); // to see that we start to send Buffer
			//sendBuffer(&ADCvalueBuffer);
			//USART0_Transmit(200);  //to see that buffer has ended
			averageValue = averageBuffer(&ADCvalueBuffer);
			currentPP = MinMaxBuffer(&ADCvalueBuffer);
			Our0Crossing = averageValue+(currentPP*0.5);
			//USART0_Transmit(averageValue);
			//USART0_Transmit(200);ä
			zeroCros = zeroCrossingBuffer(&ADCvalueBuffer, Our0Crossing);
			freq = zeroCros/(BUFFERSIZE*0.00025);
			USART0_TransmitInt(freq);
		}
		
		if(tick125us % 1000 == 0){
			if(!(PINB & (1 << PINB7))){ //check every 125ms if button is pressed
				button_pressed = 1;
				USART0_Transmit(0);
			}
		}
			
		
	}
}
