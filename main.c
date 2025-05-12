#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>


#define BAUD 9600
#define myUBBR0 F_CPU/16/BAUD-1
#define BUFFERSIZE_16_BIT 500

volatile uint8_t AMPLITUDE_DETECTION_THRESHOLD = 30;
//#define AMPLITUDE_DETECTION_THRESHOLD 30
#define FREQUENCY_DETECTION_THRESHOLD 5
#define POTENTIOMETER 1
#define SOUND_SENSOR 0

#define BUTTON_PIN     PB7
#define LED_PIN        PB5
#define DEBOUNCE_DELAY 50

typedef struct {
    float freq;
    const char *note;
} GuitarNote;

GuitarNote standardNotes[] = {
    {82.41, "E2"},
    {110.00, "A2"},
    {146.83, "D3"},
    {196.00, "G3"},
    {246.94, "B3"},
    {329.63, "E4"}
};

volatile bool toggle_adc_channel = false;
volatile bool takeADC = false, soundDetected = false, startRecording = false;
volatile uint16_t tickOf125us = 0;
volatile uint16_t previousAmplitude = 0, currentAmplitude = 0;
volatile bool button_pressed = false;

//circularBuffer
typedef struct {
	uint16_t buffer[BUFFERSIZE_16_BIT];
	uint16_t head;
	uint16_t tail;
	uint16_t BufferCount;
}CircularBuffer16bit;

volatile CircularBuffer16bit ADC_Buffer_16_Bit;

//initialize everything here
void InitializeIO(void) {
	DDRB |= (1 << LED_PIN); // output
	
	DDRB &= ~(1 << BUTTON_PIN); //internal button as an input    FOR TEST PURP
	PORTB |= (1 << BUTTON_PIN); //pull-up

	// Enable Pin Change Interrupt for PB7
	PCICR |= (1 << PCIE0);        // Enable Pin Change Interrupt group 0 (PCINT[7:0])
	PCMSK0 |= (1 << PCINT7);      // Enable interrupt for PCINT7 (PB7)

	sei();
}
void InitializeTimer0(void) {
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR0A |= (1 << WGM01); //CTC
	TCCR0B |= (1 << CS01);  // prescaler 8   -> 010->8, 011->64, 100->256, 101->1024
	OCR0A = 249;  //for each tick = 125us   (0.000125*16000000)/8 - 1 = 249
	TIMSK0 |= (1 << OCIE0A);  //compare A match interrupt enabled
}
void InitializeADC(void) {
	//setting up ADC
	ADMUX = 0; //clear everything just in case
	ADMUX |= (1 << REFS0); //V_ref as AV_cc
	//MUX is already at 000
	ADCSRA = 0; //clear everything just in case
	ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // | (1 << ADPS0);  // enabling ADC and setting prescaler to 64 (before was128)
	//   16MHz/128 = 125kHz
	ADCSRB = 0;
}
void InitializeUart0(unsigned int ubrr) {//setting up UART0
	//Baud rate   //  UBRR0 = 16000000/(16*19200)-1 = 51
	UBRR0 = 0;
	UBRR0H = (unsigned char)(ubrr >> 8); //BRR Low and High
	UBRR0L = (unsigned char)ubrr;
	UCSR0B |= (1 << TXEN0); //Enable TX
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}
void InitializeBuffer(CircularBuffer16bit *ADCvalueBuffer){
	ADCvalueBuffer->head = 0;
	ADCvalueBuffer->tail = 0;
	ADCvalueBuffer->BufferCount = 0;
}

//read ADC
uint16_t read_ADC(uint8_t channel) {
	ADMUX = (ADMUX & 0xF8) | (channel & 0x07);
	ADCSRA |= (1 << ADSC); //Begin conversion
	while (ADCSRA & (1 << ADSC)); //waiting until its done
	return ADC;  //return it to place read_ADC was called from
}

//Buffer functions
void addTo16BitCircularBuffer(CircularBuffer16bit *AdcBuffer16Bit, uint16_t value){
	if(AdcBuffer16Bit->BufferCount == BUFFERSIZE_16_BIT){//if buffer is full
		AdcBuffer16Bit->tail = (AdcBuffer16Bit->tail + 1) % BUFFERSIZE_16_BIT;
	}
	else{
		AdcBuffer16Bit->BufferCount++; //we added 1 item to the buffer
	}
	AdcBuffer16Bit->buffer[AdcBuffer16Bit->head] = value;
	AdcBuffer16Bit->head = (AdcBuffer16Bit->head + 1) % BUFFERSIZE_16_BIT;
}
uint16_t averageOfTop2ExcludingMax(CircularBuffer16bit *Bufferr) {
	if (Bufferr->BufferCount < 3) return 0;  // Need at least 3 values

	uint16_t max1 = 0, max2 = 0, max3 = 0;
	uint16_t index = Bufferr->tail;

	for (uint16_t i = 0; i < Bufferr->BufferCount; ++i) {
		uint16_t val = Bufferr->buffer[(index + i) % BUFFERSIZE_16_BIT];

		if (val > max1) {
			max3 = max2;
			max2 = max1;
			max1 = val;
			} else if (val > max2) {
			max3 = max2;
			max2 = val;
			} else if (val > max3) {
			max3 = val;
		}
	}
	return (max2 + max3) / 2;
}
uint16_t zeroCrossing16BitBuffer(CircularBuffer16bit *AdcBuffer16Bit, uint16_t average){
	if(AdcBuffer16Bit->BufferCount == 0){return 0;} //if buffer empty, return 0
	//uint8_t averageBufferValue = averageBuffer(&ADCvalueBuffer);
	uint16_t index = AdcBuffer16Bit->tail;
	uint16_t value, lastState = 0;
	uint16_t zero16CrossingCount = 0;
	for(uint16_t i = 0; i < AdcBuffer16Bit->BufferCount; ++i){
		value = AdcBuffer16Bit->buffer[(index+i) % BUFFERSIZE_16_BIT];
		if((value > average) && (lastState == 0)) {
			zero16CrossingCount++;
			lastState = 1;
		}
		else if((value < average) && (lastState == 1)){
			zero16CrossingCount++;
			lastState = 0;
		}
	}
	return floor(zero16CrossingCount/2);
}
uint16_t averageAmplitude(CircularBuffer16bit *Buffer) {
	if (Buffer->BufferCount == 0) return 0;
	uint32_t sum = 0;
	uint16_t index = Buffer->tail, temp = 0, avg = 0, count = 0;
	for (uint16_t i = 0; i < Buffer->BufferCount; ++i) {
		sum += Buffer->buffer[(index + i) % BUFFERSIZE_16_BIT];
	}
	avg = sum/Buffer->BufferCount;
	sum = 0;
	for (uint16_t i = 0; i < Buffer->BufferCount; ++i) {
		temp = Buffer->buffer[(index + i) % BUFFERSIZE_16_BIT];
		if(temp > avg){
			sum += temp;
			count++;
		}
		
	}
	
	if (count == 0) return 0; // lets not divide by 0

	return sum/count;   //  / 65536; //turn 32bit to 8bit
}


//USART functions
void USART0_Transmit(unsigned char data) { //send byte
	while (!(UCSR0A & (1 << UDRE0))); //wait for empty buffer
	UDR0 = data;
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

void fillInTheBuffer(){
	while(!(ADC_Buffer_16_Bit.BufferCount == BUFFERSIZE_16_BIT)){
		if(takeADC){
			uint16_t mic_adc_16_bit = read_ADC(SOUND_SENSOR);
			addTo16BitCircularBuffer(&ADC_Buffer_16_Bit, mic_adc_16_bit);
			//USART0_TransmitInt(mic_adc_16_bit);
			takeADC = false;
		}
	}
}
void detectSound(){
	currentAmplitude = averageAmplitude(&ADC_Buffer_16_Bit);
	if((currentAmplitude > (previousAmplitude+AMPLITUDE_DETECTION_THRESHOLD)) && (previousAmplitude > 0)){
		tickOf125us = 0;
		soundDetected = true;
		PORTB |= (1 << PORTB5);
	}
	previousAmplitude = currentAmplitude;
}
void calculateFrequencyAndNote(){
	cli();
	uint16_t almostMaxValue = averageOfTop2ExcludingMax(&ADC_Buffer_16_Bit);
	uint16_t Our0Crossing2avg = zeroCrossing16BitBuffer(&ADC_Buffer_16_Bit, almostMaxValue-FREQUENCY_DETECTION_THRESHOLD);
	uint16_t Freq = Our0Crossing2avg/(ADC_Buffer_16_Bit.BufferCount*0.00025);
	sei();

const char *closestNote = "Unknown";
    float minDiff = 9999;
    for (uint8_t i = 0; i < sizeof(standardNotes)/sizeof(GuitarNote); i++) {
        float diff = fabs(freq - standardNotes[i].freq);
        if (diff < minDiff) {
            minDiff = diff;
            closestNote = standardNotes[i].note;
        }
    }

    char noteDisplay[16];
    snprintf(noteDisplay, sizeof(noteDisplay), "%s (%.1fHz)", closestNote, freq);
    LCD_clear();
    LCD_print(noteDisplay);
}
	USART0_TransmitStr("So called freq: ");
	USART0_TransmitInt(Freq);
}

uint8_t scale_input_threshold(uint16_t input) {
	// Define known input-output points
	float x1 = 80.0f, y1 = 30.0f;
	float x2 = 300.0f, y2 = 10.0f;

	// Compute slope (m) and intercept (b) for y = mx + b
	float slope = (y2 - y1) / (x2 - x1); // = (10 - 30) / (300 - 80)
	float intercept = y1 - slope * x1;

	// Apply scaling
	float output = slope * input + intercept;

	// Clamp to 0–255
	if (output < 0) output = 0;
	if (output > 255) output = 255;

	return (uint8_t)(output + 0.5f); // Round to nearest integer
}

ISR(TIMER0_COMPA_vect){
	tickOf125us++;
	if((tickOf125us % 2) == 0){
		takeADC = true;
	}
	if(tickOf125us > 4000 && soundDetected){
		startRecording = true;
	}
	if(tickOf125us > 8000){
		PORTB &= ~(1 << PORTB5);
		tickOf125us = 0;
		soundDetected = false;
		startRecording = false;
	}
}
ISR(PCINT0_vect) {
	static uint8_t last_state = 1;  // Assumes pull-up (idle high)
	uint8_t current_state = (PINB & (1 << BUTTON_PIN)) ? 1 : 0;

	if (current_state != last_state) {
		_delay_ms(DEBOUNCE_DELAY);  // Debounce delay
		current_state = (PINB & (1 << BUTTON_PIN)) ? 1 : 0;

		if (current_state != last_state) {
			last_state = current_state;

			if (current_state == 0) {  // Falling edge (button press)
				button_pressed = true;
			}
		}
	}
}

int main(void){
	InitializeIO();
	InitializeADC();
	InitializeTimer0();
	InitializeUart0(myUBBR0);
	sei();
	while(1){
		
		if (button_pressed) {
			PORTB ^= (1 << LED_PIN);  // Toggle LED on PB5
			toggle_adc_channel = !toggle_adc_channel; //toggle
			button_pressed = false;
		}
		if(toggle_adc_channel){
			InitializeBuffer(&ADC_Buffer_16_Bit); //reset buffer basically..
			fillInTheBuffer();
			if(!soundDetected) {detectSound();}
			else if (startRecording){calculateFrequencyAndNote();}
		}
		else{
			uint16_t potent = read_ADC(POTENTIOMETER);
			USART0_TransmitInt(potent);
			AMPLITUDE_DETECTION_THRESHOLD = scale_input_threshold(potent);
			
		}
		
		


	}
}
