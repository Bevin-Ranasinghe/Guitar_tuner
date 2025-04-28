#define F_CPU 160000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

//sending a byte
void USART0_Transmit(unsigned char data) {  //send byte
	while (!(UCSR0A & (1 << UDRE0)));  //wait for empty buffer
	UDR0 = data;
 }

void USART0_TransmitStr(const char* str) {	//from string, send byte by byte to function above
	while(*str) {
		USART0_Transmit(*str++);
	}
}

void USART0_TransmitInt(uint16_t value) {
	char buffer[10];   //create buffer
	sprintf(buffer, "%u\r\n", value); //int into char[] 
	USART0_TransmitStr(buffer);
}

uint16_t read_ADC(void) {
	ADCSRA |= (1 << ADSC); //Begin conversion
	while (ADCSRA & (1 << ADSC));  //waiting until its done
	return ADC;
}

int main(void) {
	//setting up ADC
	ADMUX = 0;   //clear everything just in case   
	ADMUX |= (1 << REFS0); //V_ref as AV_cc
		//MUX is already at 000
	ADCSRA = 0;  //clear everything just in case   
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // enabling ADC and setting prescaler to 128
														//   16MHz/128 = 125kHz
	ADCSRB = 0;


	//setting up UART0	
		//Baud rate   //  UBRR0 = 16000000/(16*19200)-1 = 51
	UBRR0 = 0;
	UBRR0H = (51 >> 8);   //BRR Low and High
	UBRR0L = 51;

	UCSR0B |= (1 << TXEN0); //Enable TX
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);// 8 data bits, 1 stop bit

	while(1) {
		uint16_t mic_adc = read_ADC(); //read value of mic
		USART0_TransmitInt(mic_adc);
		_delay_ms(1);//can add delay
	}
}
