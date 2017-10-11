// ***********************************************************
// Project:
// Author:
// Module description:
// ***********************************************************

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#include <avr\io.h>              // Most basic include files
#include <avr\interrupt.h>       // Add the necessary ones
#include <avr\signal.h>          // here
#include <avr\wdt.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "i2c_master.h"

#define 	MLX90614_WRITE	(0x5A<<1)
#define 	MLX90614_READ	(MLX90614_WRITE+1)
#define	ADR_T_A			0x06
#define	ADR_T_OBJ1		0x07
#define	ADR_T_OBJ2		0x08


#define __STDIO_FDEVOPEN_COMPAT_12						
// Buffer sizes must be 2^n
//
#define TBUFSIZE	32
#define RBUFSIZE	32

#define TMASK		(TBUFSIZE-1)
#define RMASK		(RBUFSIZE-1)

#define FLASH_LED		PC3
#define STATUS_LED1	PD4
#define STATUS_LED2	PD3
#define RELAIS			PB7
#define BUZZER			PB6
#define SWITCH			PD2

#define RELAIS_ON			PORTB |= (1<<RELAIS)
#define RELAIS_OFF		PORTB &=~(1<<RELAIS)

#define BUZZER_ON			PORTB |= (1<<BUZZER)
#define BUZZER_OFF		PORTB &=~(1<<BUZZER)

#define STATUS_LED1_ON	PORTD |= (1<<STATUS_LED1)
#define STATUS_LED1_OFF	PORTD &=~(1<<STATUS_LED1)

#define FLASH_LED_ON		PORTC |= (1<<FLASH_LED)
#define FLASH_LED_OFF	PORTC &=~(1<<FLASH_LED)

#define STATUS_LED2_ON	PORTD |= (1<<STATUS_LED2)
#define STATUS_LED2_OFF	PORTD &=~(1<<STATUS_LED2)

#define OFF_COUNTER		15

#define	MODE_OFF			0
#define	MODE_ON			1
#define	MODE_TEMP_PROT	3

// Define here the global static variables
//
volatile unsigned char tbuf[TBUFSIZE];  // TX buffer
volatile unsigned char rbuf[RBUFSIZE];  // RX buffer

volatile unsigned char t_in;            // TX buffer in index
volatile unsigned char t_out;           // TX buffer out index

volatile unsigned char r_in;            // RX buffer in index
volatile unsigned char r_out;           // RX buffer out index

uint8_t	interval;
int16_t t_array[6];
uint8_t	off_counter = 0;
uint8_t	mode;
int16_t	slope2;

uint16_t	t_la_threshold_up 	=  300;
uint16_t	t_abs_threshold_up 	=  270;
uint16_t	t_la_threshold_down 	=  250;
uint16_t	t_abs_threshold_down	=  250;



// Clock Timer
SIGNAL(SIG_OVERFLOW2) {
	static uint8_t c = 0;
	interval++;
}

SIGNAL(SIG_OVERFLOW0) {
	static uint8_t	c = 0;
	uint8_t slow=0;
	//if (off_counter < OFF_COUNTER-2) slow=1;
	c++;
	if (mode==MODE_TEMP_PROT) {
		if(c>(7<<slow)) {
			FLASH_LED_ON;
		}
	}
	if(c>(15<<slow)) {
		c=0;
		FLASH_LED_OFF;
	}
}


SIGNAL(SIG_INTERRUPT0) {
	uint8_t i;
	uint8_t c=0;
	EIMSK = 0;
	sei();
	
	for(i=0;i<250;i++) if((PIND & (1<<SWITCH))) c++;
	//printf("INT0 %i\n", c);

	if(c < 40) {
		switch(mode) {
		case MODE_OFF:
			if(off_counter) mode = MODE_TEMP_PROT;
			else mode = MODE_ON;
			break;
		case MODE_ON:
			mode = MODE_OFF;
			break;
		case MODE_TEMP_PROT:
		default:
			mode = MODE_OFF;
		}
	}
	EIMSK = (1<<INT0);
}


SIGNAL(SIG_USART_RECV) {
//******************
// RX interrupt handler
//
	char c;	
	c = UDR0;							// Get received char
	rbuf[r_in & RMASK] = c;
	r_in++;
}

SIGNAL(SIG_USART_DATA) {
//*******************
// Data register empty interrupt handler.
// Indicates that next char can be transmitted
//
	if(t_in != t_out) {
		UDR0 = tbuf[t_out & TMASK];
		t_out++;	
	}
	else {
		UCSR0B &= ~(1<<UDRIE0);
	}
}

char tbuflen(void) {
//****************
// Retrieve pending chars in TX buffer
//
	return(t_in - t_out);
}

int UART_putchar(char c, FILE *stream) {
//*********************
// Fills the transmit buffer, if it is full wait
//
	while((TBUFSIZE - tbuflen()) <= 2);  // Wait...
	
	// Add data to the transmit buffer, enable TXCIE
	//
	tbuf[t_in & TMASK] = c;
	t_in++;	
	UCSR0B |= (1<<UDRIE0);			// Enable UDR empty interrupt	
	return(0);
}

char rbuflen(void) {
// ***************
// Retrive pending chars in RX buffer
//
	return(r_in - r_out);
}

int UART_getchar(FILE *stream) {
//*******************
// Retieves character from UART. This function is to be passed
// to fdevopen
//
	unsigned char c;
	while(rbuflen() == 0);	  // Wait...
	c = rbuf[r_out & RMASK];
	r_out++;	
	return(c);
}

void UART_first_init(void) {
//***********************
// The function fdevopen(..) must contain as parameters the
// corresponding  ..putchar() and  ..getchar() functions, defined before.
//
	UBRR0 = 12;										 // 4800 BPS
	
	UCSR0B = (1<<RXCIE0)|(1<<TXEN0)|(1<<RXEN0);	 // 8 Databits, receive and transmit enabled, receive and transmit complete interrupt enabled
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	fdevopen(UART_putchar, UART_getchar);
	sei();											 // Global interrupt enable
}


void print_array(){
	uint8_t i;
  	printf("Array:");
	for(i=0;i<6;i++) {
   	printf(" %i", t_array[i]);
	}
  	printf("\n");
}


void add_value(uint16_t value) {
	uint8_t i;
	if(t_array[0]==0) {
	t_array[0]=t_array[1]=t_array[2]=t_array[3]=t_array[4]=t_array[5]=value;
	}
	else {
		for(i=0;i<=6;i++) {
			t_array[i]=t_array[i+1];
		}
		t_array[5]=value;
		slope2 = (10*(t_array[5]-t_array[4]) + 19*slope2) / 20;
	}
}


int16_t get_slope() {
	int16_t s1, s2, s3;

	s1 = t_array[5] - t_array[0];
	s2 = t_array[4] - t_array[1];
	s3 = t_array[3] - t_array[2];
	
	return ((3*s1+5*s2+15*s3)/9);
}


int16_t lookahead_temp(int8_t slope, uint8_t steps) {
	int16_t max = 0;
	uint16_t av;
	uint8_t i;
//	for(i=0;i<6;i++) {
//		if(t_array[i]>max) max=t_array[i];
//	}
	av = (t_array[5]+t_array[4])>>1;
	return (av + steps*(int16_t)slope);
}



uint16_t get_temperature(uint8_t adr) {
	uint16_t raw;
	uint8_t 	ret;
	uint8_t 	lo, hi, pec;
	uint8_t	pec_read[6];

	i2c_start(MLX90614_WRITE);
	i2c_write(adr);
	
	ret = i2c_rep_start(MLX90614_READ);
	if(ret) {
		i2c_rep_start(MLX90614_READ);
   }

	lo = i2c_read_ack();
	hi = i2c_read_ack();
	raw = (uint16_t)(hi<<8)+lo;
	//printf("0x%04x\n", raw);
	pec = i2c_read_ack();
	
	i2c_stop();
	if(raw & 0x8000) return 0;
	
	return (raw / 5 - 2731);
}

void set_relais(on) {
	static uint8_t last = 0;
	if(on) {
		if(on != last) printf(">>> Relais ON\n");
		RELAIS_ON;
	}
	else {
		if(on != last) printf(">>> Relais OFF\n");
		RELAIS_OFF;
	}
	last = on;
}

// It is recommended to use this coding style to
// follow better the mixed C-assembly code in the
// Program Memory window
//
void my_function(void) {  // Put the open brace '{' here

   asm("nop");          // Inline assembly example
}




// ***********************************************************
// Main program
//
int main(void) {

	DDRB = 0x00 | (1<<RELAIS) | (1<<BUZZER);
	DDRC = 0x00 | (1<<FLASH_LED);
	DDRD = 0x00 | (1<<STATUS_LED1) | (1<<STATUS_LED2);

	PORTB = ~((1<<RELAIS) | (1<<BUZZER));
	PORTC = ~(1<<FLASH_LED);
	PORTD = ~((1<<STATUS_LED1) | (1<<STATUS_LED2) | (1<<FLASH_LED));

	PRR != ~(1<<PRTWI);

	wdt_reset();
	wdt_enable(WDTO_4S);
	
	UART_first_init();
	i2c_init();
	
	interval=0;
	// Timer 2 config (RTC)
   TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);	// clk/256
   TIMSK2 = (1<<TOIE2);

	TCCR0A = 0;
	TCCR0B = (0<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
	
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);
	

	STATUS_LED1_ON;
	STATUS_LED2_OFF;
	printf("\nStart\n");
	set_relais(0);
	mode = MODE_OFF;
	
	uint16_t temp, temp_sum;
	int16_t	lookahead;
	int16_t 	slope;
	int16_t	slope_raw;
	uint8_t	count=0;
	uint8_t  last_interval = 0xff;
	
	temp_sum = 0;
	
	sei();

   while(1) {             // Infinite loop; define here the
   	if(!(interval < 16)) {
   		wdt_reset();
   		interval=0;
   		count=0;
   		//printf("Messen\n");
   		// Temperatur einlesen
//	      temp = get_temperature(ADR_T_OBJ1);
			temp = temp_sum >> 4;
			temp_sum = 0;
	      //printf("Temp: %i\n", temp);
	      if(temp==0) {
	      // error!
	      	printf("Error Temp=0");
	      }
	      else {
   	   	printf("Temp: %i, ", temp);
   	   	// Temperaturverlauf auswerten
   	   	add_value(temp);
   	   	//print_array();
   	   	slope_raw = get_slope();
				//if(slope_raw<0) slope_raw = 0;
   	   	slope = (15*slope + 10*slope_raw)/16;
   	   	printf("slope_raw: %i, slope: %i, slope2: %i, ", slope_raw, slope, slope2);
				lookahead=lookahead_temp(slope, 5);   	   	
   	   	//printf("Prognose: %i\n", lookahead);
   	   	printf("Ambient: %i\n", get_temperature(ADR_T_A));
				
   	   	// Relais sperren, wenn ein Wert über dem Grenzwert
	   	   //if((lookahead > t_la_threshold_up) | (temp > t_abs_threshold_up)) off_counter = OFF_COUNTER+1;
	   	
	   		if(temp > 480) {
	   			if(slope > 30) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 48, ");
	   			}
	   		}
	   		else if(temp > 450) {
	   			if(slope > 50) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 45, ");
	   			}
	   		}
	   		else if(temp > 400) {
	   			if(slope > 60) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 40, ", slope);
	   			}
	   		}
	   		else if(temp > 350) {
	   			if(slope > 80) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 35, ");
               }
	   		}
	   		else if(temp > 300) {
	   			if(slope > 120) {
	   				off_counter = OFF_COUNTER+1;
		   			printf("Temperature Protect Rule 30, ");
		   		}
	   		}
	   		else {
	   			if(slope > 160) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect General Rule, ");
	   			}
	   		}

   	   }
   		if(off_counter) {
  				off_counter--;
  				if(mode == MODE_ON) mode = MODE_TEMP_PROT;
  				//RELAIS_OFF;
				//STATUS_LED1_OFF;
				//STATUS_LED2_ON;
				printf("Counter: %i; \n", off_counter);
   		}
   		else {
	   	   if((lookahead < t_la_threshold_down) & (temp < t_abs_threshold_down)) {
	   	   	// Relais wieder einschalten, wenn alle Temperaturen unter der unteren Schwelle
	   	   	//if(mode == MODE_TEMP_PROT) mode = MODE_ON;
					//printf("ON\n");
					if(mode == MODE_TEMP_PROT) mode = MODE_OFF;
	   		}
   		}
		}
		else if(interval != last_interval) {
   		last_interval = interval;
   		uint16_t temp;
   		if(count<16) {
   			temp = get_temperature(ADR_T_OBJ1);
	   		count++;
   			// Messwerte für den Mittelwert aufsummieren
   			temp_sum += temp;
   			//printf("Raw: %i\n", temp);
   		}
   	}

		
		switch(mode) {
		case MODE_OFF:
			set_relais(0);
			STATUS_LED1_OFF;
			STATUS_LED2_ON;
			break;
		case MODE_ON:
			set_relais(1);
			STATUS_LED1_ON;
			STATUS_LED2_OFF;
			break;
		case MODE_TEMP_PROT:
			set_relais(0);
			STATUS_LED1_OFF;
			STATUS_LED2_ON;
			break;
		default:
			mode = MODE_OFF;
		}		
   }
}



