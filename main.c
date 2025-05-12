#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>

#define LCD_ADDR        0x3E
#define SCL_CLOCK 100000L // 100kHz standard I2C clock speed
#define BAUD 9600
#define myUBBR0 F_CPU/16/BAUD-1
#define BUFFERSIZE_16_BIT 500

volatile uint8_t AMPLITUDE_DETECTION_THRESHOLD = 30;
//#define AMPLITUDE_DETECTION_THRESHOLD 30
volatile uint16_t potent = 0;
#define FREQUENCY_DETECTION_THRESHOLD 5
#define POTENTIOMETER 1
#define SOUND_SENSOR 0
// LCD Commands
#define LCD_CMD         0x80
#define LCD_DATA        0x40
#define LCD_CLEAR       0x01
#define LCD_LINE1       0x80
#define LCD_LINE2       0xC0


#define BUTTON_PIN     PB7
#define LED_PIN        PB5
#define DEBOUNCE_DELAY 50

// ================= I2C Interface =================
void TWI_init(void) {
	TWSR0 = 0x00; // Prescaler = 1
	TWBR0 = ((F_CPU / SCL_CLOCK) - 16) / 2; // Bit rate register
}

void TWI_start(void) {
	TWCR0 = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT); // Send START
	while (!(TWCR0 & (1 << TWINT)));
}

void TWI_stop(void) {
	TWCR0 = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT); // Send STOP
	_delay_ms(1);
}

void TWI_write(uint8_t data) {
	TWDR0 = data;
	TWCR0 = (1 << TWEN) | (1 << TWINT); // Start transmission
	while (!(TWCR0 & (1 << TWINT)));
}


// ================= LCD Functions =================
void LCD_sendCommand(uint8_t cmd) {
	TWI_start();
	TWI_write(LCD_ADDR << 1); // Write address
	TWI_write(0x80);          // Co = 1, RS = 0 (Command Mode)
	TWI_write(cmd);
	TWI_stop();
}

void LCD_sendData(uint8_t data) {
	TWI_start();
	TWI_write(LCD_ADDR << 1); // Write address
	TWI_write(0x40);          // Co = 0, RS = 1 (Data Mode)
	TWI_write(data);
	TWI_stop();
}

void LCD_init(void) {
	_delay_ms(50); // Wait for LCD power up
	LCD_sendCommand(0x38); // 8-bit, 2 line, normal font
	LCD_sendCommand(0x39); // Function set
	LCD_sendCommand(0x14); // Internal OSC freq
	LCD_sendCommand(0x70); // Contrast set
	LCD_sendCommand(0x56); // Power/icon/contrast control
	LCD_sendCommand(0x6C); // Follower control
	_delay_ms(200);
	LCD_sendCommand(0x38); // Function set
	LCD_sendCommand(0x0C); // Display ON
	LCD_sendCommand(0x01); // Clear display
	_delay_ms(2);
}

void LCD_setCursor(uint8_t col, uint8_t row) {
	uint8_t address = (row == 0) ? col : (0x40 + col);
	LCD_sendCommand(0x80 | address);
}

void LCD_print(const char *str) {
	while (*str) {
		LCD_sendData(*str++);
	}
}

//end of LCD functions

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
void calculateFrequency(){
	cli();
	uint16_t almostMaxValue = averageOfTop2ExcludingMax(&ADC_Buffer_16_Bit);
	uint16_t Our0Crossing2avg = zeroCrossing16BitBuffer(&ADC_Buffer_16_Bit, almostMaxValue-FREQUENCY_DETECTION_THRESHOLD);
	uint16_t Freq = Our0Crossing2avg/(ADC_Buffer_16_Bit.BufferCount*0.00025);
	sei();
	USART0_TransmitStr("So called freq: ");
	USART0_TransmitInt(Freq);
	writeToLCD(Freq);
}

void writeToLCD(uint16_t freq)
{    
	char buffer[16];
	 LCD_sendCommand(0x01); // Clear display
	 _delay_ms(10);
	 
	 //LCD_print("potent");
	 
	 //Locate string
// 	 if(potent >= 77 && potent <= 87)
// 	 {
// 		 LCD_setCursor(0, 0);
// 		 LCD_print("E2: 82 Hz");
// 	 }
// 	 	 else if(potent >= 110 && potent <= 114)
// 	 	 {
// 		 	 LCD_setCursor(0, 0);
// 		 	 LCD_print("A2: 112 Hz");
// 	 	 }
// 		  	 	 else if(potent >= 141 && potent <= 151)
// 		  	 	 {
// 			  	 	 LCD_setCursor(0, 0);
// 			  	 	 LCD_print("D3: 146 Hz");
// 		  	 	 }
// 						 	 else if(potent >= 191 && potent <= 201)
// 						 	 {
// 							 	 LCD_setCursor(0, 0);
// 							 	 LCD_print("G3: 196 Hz");
// 						 	 }
// 							  	 	 else if(potent >= 241 && potent <= 251)
// 							  	 	 {
// 								  	 	 LCD_setCursor(0, 0);
// 								  	 	 LCD_print("B3: 246 Hz");
// 							  	 	 }
// 											 	 else if(potent >= 325 && potent <= 335)
// 											 	 {
// 												 	 LCD_setCursor(0, 0);
// 												 	 LCD_print("E4: 330 Hz");
// 											 	 }
// 	 else
// 	 {
// 		 sprintf(buffer, "Move Potent: %u Hz", potent);
// 		 LCD_setCursor(0, 0);
// 		 LCD_print(buffer);
// 	 }
// 	 
typedef struct {
	uint16_t min;
	uint16_t max;
	const char* note;
} NoteRange;

NoteRange notes[] = {
	{77,  87,  "E2: 82 Hz"},
	{110, 114, "A2: 112 Hz"},
	{141, 151, "D3: 146 Hz"},
	{191, 201, "G3: 196 Hz"},
	{241, 251, "B3: 246 Hz"},
	{325, 335, "E4: 330 Hz"}
};

uint8_t matched = 0;
for (uint8_t i = 0; i < sizeof(notes)/sizeof(notes[0]); i++) {
	if (potent >= notes[i].min && potent <= notes[i].max) {
		LCD_setCursor(0, 0);
		LCD_print(notes[i].note);
		matched = 1;
		break;
	}
}

if (!matched) {
	sprintf(buffer, "Move Potent: %u Hz", potent);
	LCD_setCursor(0, 0);
	LCD_print(buffer);
}

	 sprintf(buffer, "Now Playing:%u Hz", freq);
		 LCD_setCursor(0, 1);
		 LCD_print(buffer);
	 //sprintf(freq, "%u", freq);
	 //LCD_setCursor(0, 1);
	 //LCD_print(freq);
	 //LCD_print(" Hz");

	 
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

	// Clamp to 0?255
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
	TWI_init();
	LCD_init();
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
			else if (startRecording){calculateFrequency();}
		}
		else{
			potent = read_ADC(POTENTIOMETER);
			USART0_TransmitInt(potent);
			writeToLCD(0);
			AMPLITUDE_DETECTION_THRESHOLD = scale_input_threshold(potent);
			
			
		}
		
		


	}
}
