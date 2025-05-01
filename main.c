#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define sizeOfAvgArr 50
#define BAUD 38400  //19200
#define myUBBR0 F_CPU/16/BAUD-1

volatile uint16_t count = 0;
volatile uint8_t NumberOfValuesinArray = 0;
volatile uint8_t *ptrToAvg;

void InitializeTimer0(void) {
    DDRB |= (1 << PORTB5); //PB5 as output
	TCCR0A = 0;
	TCCR0B = 0;
    //timer0    -   compare match
	//TCCR0A |= (1 << COM0A0);
	TCCR0A |= (1 << WGM01);
    TCCR0B |= (1 << CS01) | (1 << CS00); //CTC
    OCR0A = 24;  //100us  presc = 64
	TIMSK0 |= (1 << OCIE0A);
	sei();
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

//sending a byte
void USART0_Transmit(unsigned char data) { //send byte
    while (!(UCSR0A & (1 << UDRE0))); //wait for empty buffer
    UDR0 = data;
}

void USART0_TransmitStr(const char * str) { //from string, send byte by byte to function above
    while ( * str) {
        USART0_Transmit( * str++);
    }
}

void USART0_TransmitInt(uint16_t value) { //to transmit uint16_t , didnt work for me, was bad
    char buffer[10]; //create buffer
    sprintf(buffer, "%u\r\n", value); //int into char[]
    USART0_TransmitStr(buffer);
}

uint16_t read_ADC(void) {
    ADCSRA |= (1 << ADSC); //Begin conversion
    while (ADCSRA & (1 << ADSC)); //waiting until its done
    return ADC;
}


void shiftValueArr(uint8_t value) {
	if(NumberOfValuesinArray < 255) {
		ptrToAvg[sizeOfAvgArr-NumberOfValuesinArray] = value;
		NumberOfValuesinArray++;
	}
	else {
		memmove(*(ptrToAvg+1), *ptrToAvg, (NumberOfValuesinArray-1)*sizeof(uint8_t));  //move mem by 1, but codies 1 byte less, so we could shift this shit
	}
}

void calcArray(void) {
	uint8_t temp1, temp2;
	for(uint8_t i = 0; i < (NumberOfValuesinArray-1); ++i) {
		temp1 = *(ptrToAvg+i);
		temp2 = *(ptrToAvg+i+1);
		if(temp1 > temp2) {
			
		}
	}
}


ISR(TIMER0_COMPA_vect){
	count++;
		uint8_t mic_adc = (read_ADC()) / 4; //read value of mic  10bits to 8bits
		//shiftValueArr(mic_adc);
		if(mic_adc > 25){
			USART0_Transmit(mic_adc);
			//shiftValueArr(mic_adc);
			
		}
		else{
			USART0_Transmit(0);
		}

	//if((count % 10) == 0){	}
	
	if(count >= 5000) {  //every 500ms toggle PB5
		count = 0;
		PORTB ^= (1 << PORTB5);
	}
}


int main(void) {
   	InitializeAdc();
	InitializeTimer0();
	InitializeUart0(myUBBR0);
	
	ptrToAvg = (uint8_t) calloc(sizeOfAvgArr, sizeof(uint8_t));
	if(ptrToAvg == NULL) {
		return 0;
	}

    while (1) {

       
    }
}