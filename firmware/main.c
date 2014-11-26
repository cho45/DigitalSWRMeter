#include <stdio.h>
#include <math.h>
#include <string.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "avr-utils/ringbuffer.h"
#include "serial.h"
#include "avr-utils/i2c.h"

#define clear_bit(v, bit) v &= ~(1 << bit)
#define set_bit(v, bit)   v |=  (1 << bit)
#define nop() asm volatile ("nop")

#define INPUT_NOR PB1
#define INPUT_REV PB0

#define ADC_FWD 0
#define ADC_REF 1
#define ADC_VCC 2.465
#define ADC_RES (ADC_VCC / 1024.0)

#define CALIBRATION_FWD_A 1.0738396624472575
#define CALIBRATION_FWD_B 0.044856540084388184
#define CALIBRATION_REF_A 1.0852878464818763
#define CALIBRATION_REF_B 0.042818763326226014

#define DISPLAY_ADDRESS 0b1010000

struct {
	float v_fwd, v_ref;
	float p_fwd, p_ref;
	float swr;
} max;

struct {
	uint16_t msec;
	uint8_t sec;
} timer = { 0, 1 };

struct {
	uint8_t request_redraw;
	uint8_t peak_detected;
} flag = {
	1
};

enum {
	MODE_NORMAL,
	MODE_CALIB
} mode = MODE_NORMAL;

uint8_t write_data[32];
uint8_t read_data[32];
ringbuffer write_ring;
ringbuffer read_ring;

int display_write_instruction(uint8_t instruction) {
	uint8_t ret = 0;
	uint8_t data[2];
	i2c_master_init(DISPLAY_ADDRESS);
	data[0] = 0b00000000;
	data[1] = instruction;
	ret = i2c_master_write(data, 2);
	if (ret) goto end;
end:
	i2c_master_stop();
	return ret;
}

int display_write_data (int line, char* string) {
	uint8_t ret = 0;
	uint16_t len = strlen(string);
	uint8_t i;
	uint8_t data[2];

	i2c_master_init(DISPLAY_ADDRESS);
	if (line != 2) {
		display_write_instruction(0b10000000); // set ddram address to line 1
	} else {
		display_write_instruction(0b11000000); // set ddram address to line 2
	}
	_delay_us(60);

	i2c_master_init(DISPLAY_ADDRESS);
	for (i = 0; i < 16; i++) {
		data[0] = 0b10000000;
		data[1] = i < len ? string[i] : 0x20;
		ret = i2c_master_write(data, 2);
		if (ret) goto end;
		_delay_us(60);
	}
	i2c_master_stop();
end:
	i2c_master_stop();
	return ret;
}


void do_adc (float* res_fwd, float* res_ref) {
	uint8_t i;
	uint16_t res;

	uint16_t fwd = 0;
	uint16_t ref = 0;

	ADCSRA =
		(1<<ADEN)  | // Enable (Turn on ADC)
		(0<<ADATE) | // Auto Trigger Enable
		(0<<ADIF)  |
		(1<<ADIE)  | // Interrupt Enable
		0b110      ; // Prescale 64

	ADCSRB = 0;

#define ADC_LOOPCOUNT 100

	for (i = 0; i < ADC_LOOPCOUNT; i++) {
		res = 0;
		ADMUX =
			(0<<REFS1) | (1<<REFS0) | // Use VCC
			(0<<ADLAR) | // Right Adjust
			(i % 2 == 0 ? ADC_FWD : ADC_REF);

		// START
		set_bit(ADCSRA, ADSC); 
		// set_sleep_mode(SLEEP_MODE_ADC);
		while (bit_is_set(ADCSRA, ADSC)) nop(); // sleep_mode();

		// Read ADCL first, it locks ADCH register until ADCH is read.
		res = (ADCL) | ((ADCH) << 8);

		if (i % 2 == 0) {
			fwd += res;
		} else {
			ref += res;
		}
	}

	// Disable ADC
	ADCSRA = 0;

	*res_fwd = (float)fwd / (ADC_LOOPCOUNT / 2);
	*res_ref = (float)ref / (ADC_LOOPCOUNT / 2);
}

ISR(ADC_vect) {
}

ISR(TIMER0_COMPA_vect) {
	timer.msec++;

	if (timer.msec >= 1000) {
		timer.msec = 0;
		timer.sec++;
	}

	if (timer.msec % 100 == 0) {
		flag.request_redraw++;

		if (flag.peak_detected) {
			flag.peak_detected = 0;
		} else {
			max.v_fwd = 0; max.v_ref = 0;
			max.p_fwd = 0; max.p_ref = 0;
			max.swr = 0;
		}
	}
}

static inline void setup_io () {
	wdt_enable(WDTO_1S);

	/**
	 * Data Direction Register: 0=input, 1=output
	 * 必要なポートだけインプットポートにする。
	 */
	DDRB  = 0b11111100;
	DDRC  = 0b11111111;
	DDRD  = 0b11111111;

	PORTB = 0b00000011;
	PORTC = 0b00000000;
	PORTD = 0b00000000;

	// Diable Input Digital Register
	DIDR0 = (1<<ADC1D) | (1<<ADC0D);

	// 1ms Clock 
	uint8_t TIMER0_MODE = 2; // CTC
	TCCR0A = (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (((TIMER0_MODE>>1)&1)<<WGM01) | ( (TIMER0_MODE&1)<<WGM00);
	TCCR0B = (0<<FOC0A) | (0<<FOC0B) | ((TIMER0_MODE&0b100)<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = (0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);
	OCR0A  = 125;

//	uint16_t TIMER1_MODE = 4;
//	TCCR1A = (0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (((TIMER1_MODE>>1)&1)<<WGM11) | ((TIMER1_MODE&1)<<WGM10);
//	TCCR1B = (((TIMER1_MODE>>3)&1)<<WGM13) | (((TIMER1_MODE>>2)&1)<<WGM12) | (0<<CS12) | (1<<CS11) | (1<<CS10);
//	ICR1   = 0;
//	OCR1A  = 0;
//	TIMSK1 = (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);

	ringbuffer_init(&write_ring, write_data, 32);
	ringbuffer_init(&read_ring, read_data, 32);

	serial_init(9600, &write_ring, &read_ring);
	i2c_set_bitrate(10);

	sei();
	serial_puts("Initialized");
}

int main(void) {
	setup_io();

	display_write_instruction(0b00111100); // function set
	_delay_us(60);
	display_write_instruction(0b00001111); // display on off control
	_delay_us(60);
	display_write_instruction(0b00000001); // clear display
	_delay_ms(3);
	display_write_data(1, "Initialized.....");
	display_write_data(2, "Waiting.........");

	float v_fwd = 0, v_ref = 0;
	float p_fwd = 0, p_ref = 0;
	float reflection_coefficient = 0, swr = 0;
	char buf[32];
	for (;;) {
		wdt_reset();

		do_adc(&v_fwd, &v_ref);
		v_fwd = v_fwd * ADC_RES;
		v_ref = v_ref * ADC_RES;

		// Auto detect direction
		if (v_fwd < v_ref) {
			swr = v_fwd; // using swr as temporary value
			v_fwd = v_ref;
			v_ref = swr;
		}

		if (mode != MODE_CALIB) {
			v_fwd = v_fwd * CALIBRATION_FWD_A + CALIBRATION_FWD_B;
			v_ref = v_ref * CALIBRATION_REF_A + CALIBRATION_REF_B;
		}

		if (max.v_fwd < v_fwd) {
			p_fwd = 9 * v_fwd * v_fwd;
			p_ref = 9 * v_ref * v_ref;
			reflection_coefficient = v_ref / v_fwd;
			swr = (1 + reflection_coefficient) / (1 - reflection_coefficient);
			if (swr < 0) swr = 0/1;

			if (!isnan(swr) && p_fwd > 0.1) {
				max.v_fwd = v_fwd;
				max.v_ref = v_ref;
				max.p_fwd = p_fwd;
				max.p_ref = p_ref;
				max.swr = swr;

				flag.peak_detected++;
			}
		}

		if (flag.request_redraw) { flag.request_redraw = 0;
			if (mode == MODE_CALIB) {
				sprintf(buf, "FWD: %.1fmV", max.v_fwd * 1000);
				display_write_data(1, buf);
				sprintf(buf, "REF: %.1fmV", max.v_ref * 1000);
				display_write_data(2, buf);
				sprintf(buf, "%.1fmV %.1fmV", max.v_fwd * 1000, max.v_ref * 1000);
				serial_puts(buf);
			} else {
				sprintf(buf, "% .1fW / % .1fW", max.p_fwd, max.p_ref);
				display_write_data(1, buf);
				sprintf(buf, "SWR: %.2f", max.swr);
				display_write_data(2, buf);
			}
		}

		if (serial_gets_async(13, buf, 32)) {
			if (strncmp(buf, "MAX", 3) == 0) {
				sprintf(buf, "%.3fW %.3fW %.3f", max.p_fwd, max.p_ref, max.swr);
				serial_puts(buf);
			} else
			if (strncmp(buf, "NOW", 3) == 0) {
				sprintf(buf, "%.3fW %.3fW %.3f", p_fwd, p_ref, swr);
				serial_puts(buf);
			} else
			if (strncmp(buf, "CAL", 3) == 0) {
				if (mode == MODE_CALIB) {
					mode = MODE_NORMAL;
				} else {
					mode = MODE_CALIB;
				}
				serial_puts("CAL");
			} else {
				serial_puts("?");
			}
		}

//		sprintf(buf, "TEST: %d; ERR: %d", read_ring.size, serial_last_read_error);
//		serial_puts(buf);
//		_delay_ms(10);
	}
}
