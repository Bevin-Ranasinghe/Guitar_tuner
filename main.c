#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// ==== UART SETUP ====

// Initialize UART0
void uart_init(unsigned int baud) {
	unsigned int ubrr = (F_CPU / 16 / baud) - 1;
	UBRR0H = (ubrr >> 8);
	UBRR0L = ubrr;
	UCSR0B = (1 << TXEN0);   // Enable transmitter
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
}

// Send one byte
void uart_send(char data) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

// Send a string
void uart_send_string(const char* str) {
	while (*str) {
		uart_send(*str++);
	}
}

// Send integer as string
void uart_send_int(uint16_t value) {
	char buffer[10];
	sprintf(buffer, "%u\r\n", value);
	uart_send_string(buffer);
}

// ==== ADC SETUP ====

// Initialize ADC0 (PC0)
void adc_init(void) {
	ADMUX = (1 << REFS0); // AVcc as reference voltage, select ADC0 (PC0)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, prescaler = 64
}

// Read ADC value
uint16_t adc_read(void) {
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait until conversion complete
	return ADC;
}

// ==== MAIN ====

int main(void) {
	uart_init(9600); // Initialize UART at 9600 baud
	adc_init();      // Initialize ADC

	while (1) {
		uint16_t mic_value = adc_read();  // Read microphone analog value
		uart_send_int(mic_value);          // Send value over UART
		_delay_ms(100);                   // Small delay (adjust if needed)
	}
}
