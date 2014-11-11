#define SERIAL_ERROR_BUFFER_OVERFLOW -2

ringbuffer* serial_read_buffer;
ringbuffer* serial_write_buffer;
int serial_last_read_error = 0;

void serial_init(unsigned short baudrate, ringbuffer* write_buffer, ringbuffer* read_buffer) {
	unsigned int d = ((F_CPU + (baudrate * 8L)) / (baudrate * 16L) - 1);
	UBRR0L = d;
	UBRR0H = d >> 8;

	UCSR0B =
		(1<<RXCIE0) | // RX Complete Interrupt Enable
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

	serial_read_buffer = read_buffer;
	serial_write_buffer = write_buffer;
}

int serial_putc(char byte) {
	if (serial_write_buffer->capacity < serial_write_buffer->size + 1) {
		return SERIAL_ERROR_BUFFER_OVERFLOW;
	}

	if (bit_is_clear(UCSR0B, UDRIE0)) {
		UDR0 = byte;
		// Enable Interrupt
		UCSR0B |= (1 << UDRIE0);
	} else {
		ringbuffer_put(serial_write_buffer, byte);
	}

	return 0;
}

int serial_write(char* buf, int len) {
	int i;

	for (i = 0; i < len; i++) {
		serial_putc(buf[i]);
	}

	return 0;
}

int serial_puts(char* str) {
	int ret;
	int len = strlen(str);
	char terminate[2];
	terminate[0] = '\r';
	terminate[1] = '\n';
	ret = serial_write((char*)str, len);
	if (ret) return ret;
	return serial_write(terminate, 2);
}

int serial_gets_async(char terminator, char* buf, int limit) {
	int i;
	int len = 0;
	for (i = 0; i < serial_read_buffer->size; i++) {
		char byte = ringbuffer_get_nth(serial_read_buffer, i);

		if (byte == terminator) {
			len = i + 1;
			break;
		}
	}

	if (len) {
		for (i = 0; i < len; i++) {
			char byte = ringbuffer_get(serial_read_buffer);
			if (i < limit) {
				buf[i] = byte;
			}
		}
		if (len < limit) {
			buf[i] = 0;
		} else {
			buf[limit-1] = 0;
		}
	}

	return len;
}

void serial_sync() {
	while (serial_write_buffer->size) ;
}

ISR(USART_RX_vect) {
	char byte = UDR0;
	if (serial_read_buffer->capacity < serial_read_buffer->size + 1) {
		serial_last_read_error = SERIAL_ERROR_BUFFER_OVERFLOW;
	}

	ringbuffer_put(serial_read_buffer, byte);
}

ISR(USART_UDRE_vect) {
	if (serial_write_buffer->size) {
		UDR0 = ringbuffer_get(serial_write_buffer);
	} else {
		UCSR0B &= ~(1 << UDRIE0);
	}
}

ISR(USART_TX_vect) {
}

/**
 * char write_data[32];
 * char read_data[32];
 * ringbuffer write_ring;
 * ringbuffer read_ring;
 *
 * ringbuffer_init(&write_ring, write_data, 32);
 * ringbuffer_init(&read_ring, read_data, 32);
 *
 * serial_init(9600, &write_ring, &read_ring);
 *
 * serial_write("TEST", 4);
 *
 *
 */
