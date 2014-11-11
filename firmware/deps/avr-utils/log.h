#include <string.h>
#include <stdio.h>
#include "ringbuffer.h"

#define clear_bit(v, bit) v &= ~(1 << bit)
#define set_bit(v, bit)   v |=  (1 << bit)

ringbuffer logger_buffer;
uint8_t logger_buffer_data[128];

ISR(USART_UDRE_vect) {
	if (logger_buffer.size) {
		UDR0 = ringbuffer_get(&logger_buffer);
	} else {
		clear_bit(UCSR0B, UDRIE0);
	}
}

inline void logger_putchar(char c) {
	if (bit_is_set(UCSR0A, UDRE0)) {
		UDR0 = c;
	} else {
		ringbuffer_put(&logger_buffer, c);
		set_bit(UCSR0B, UDRIE0);
	}
}

inline void logger(char* string) {
	unsigned int len = strlen(string);
	unsigned int i;

	for (i = 0; i < len; i++) {
		logger_putchar(string[i]);
	}

	logger_putchar('\r');
	logger_putchar('\n');
}

inline void logger_init(unsigned long baudrate) {
	unsigned int d = ((F_CPU + (baudrate * 8L)) / (baudrate * 16L) - 1);
	UBRR0L = d;
	UBRR0H = d >> 8;

	UCSR0B =
		(0<<RXCIE0) | // RX Complete Interrupt Enable
		(0<<TXCIE0) | // TX Complete Interrupt Enable
		(0<<UDRIE0) | // USART Data Register Empty Interrupt Enable
		(1<<RXEN0)  | // Receiver Enable
		(1<<TXEN0)  | // Transmitter Enable
		(0<<UCSZ02) | // Character Size
		(0<<RXB80)  | // Receive Data Bit 8
		(0<<TXB80)  ; // Transmit Data Bit 8

	UCSR0C =
		(0<<UMSEL01)|(UMSEL00) | // USART Mode Select: 0=Asynchronous Operation, 1=Synchronous Operation
		(0<<UPM01)|(0<<UPM00)   | // Parity Mode
		(0<<USBS0)              | // Stop Bit Select
		(1<<UCSZ01)|(1<<UCSZ00) | // Character Size (with UCSRB)
		(0<<UCPOL0)             ; // Clock Polarity

	ringbuffer_init(&logger_buffer, logger_buffer_data, 128);

	static FILE mystdout = FDEV_SETUP_STREAM( (int(*)(char, FILE *))logger_putchar, NULL, _FDEV_SETUP_WRITE);
	stdin = stdout = &mystdout;
}
