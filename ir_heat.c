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

#include "i2c_mstr.h"
#include "ir_heat.h"

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
#define RELAIS			PB6
#define BUZZER			PB7
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

#define OFF_COUNTER		5

#define TIMER1_STOP		TCCR1B = 0
#define TIMER1_RUN		TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10)

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

int8_t	interval = -12;
int16_t 	t_array[6];
uint8_t	off_counter = 0;
uint8_t	mode;
int16_t	slope2;

uint16_t	t_la_threshold_up 	=  300;
uint16_t	t_abs_threshold_up 	=  270;
uint16_t	t_la_threshold_down 	=  250;
uint16_t	t_abs_threshold_down	=  250;



// Clock Timer
SIGNAL(SIG_OVERFLOW2) {
	interval++;
}

SIGNAL(SIG_OVERFLOW1) {
	TIMER1_STOP;
	TCNT1H = 0;
	TCNT1L = 0;
}

SIGNAL(SIG_OVERFLOW0) {
	static uint8_t	c = 0;
	uint8_t slow=0;
	//if (off_counter < OFF_COUNTER-2) slow=1;
	c++;
	if (mode==MODE_TEMP_PROT) {
		if(c > (6<<slow)) {
			FLASH_LED_ON;
		}
	}
	if(c > (10<<slow)) {
		c=0;
		FLASH_LED_OFF;
	}
}


SIGNAL(SIG_INTERRUPT0) {
	static uint8_t running = 0;
	static uint8_t last_interval = 0xff;
	
	if(running | TCNT1H | TCNT1L){
		printf("X");
		return;
	}
	running = 1;
	
	uint16_t i;
	uint8_t c=0;
	EIMSK = 0;
	sei();
	printf("In");
	for(i=0;i<2000;i++) if((PIND & (1<<SWITCH))) c++;
	printf(" %i ", c);

	if(c < 100) {
		TCNT1L = 1;
		TIMER1_RUN;
		switch(mode) {
		case MODE_OFF:
			if(off_counter) mode = MODE_TEMP_PROT;
			else mode = MODE_ON;
			break;
		case MODE_ON:
		case MODE_TEMP_PROT:
		default:
			mode = MODE_OFF;
		}
	}
	printf("Out\n");
	EIFR = (1<<INTF0);
	EIMSK = (1<<INT0);
	running = 0;
	printf("Exit\n");
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
		for(i=0;i<5;i++) {
			t_array[i]=t_array[i+1];
		}
		t_array[5] = value;
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
	uint16_t av;
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


void set_relais(uint8_t on) {
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




// ***********************************************************
// Main program
//
int main(void) {
   // Ausgänge definieren
	DDRB = 0x00 | (1<<RELAIS) | (1<<BUZZER);
	DDRC = 0x00 | (1<<FLASH_LED);
	DDRD = 0x00 | (1<<STATUS_LED1) | (1<<STATUS_LED2);

	// Ausgänge ausschalten
	PORTB = ~((1<<RELAIS) | (1<<BUZZER));
	PORTC = ~(1<<FLASH_LED);
	PORTD = ~((1<<STATUS_LED1) | (1<<STATUS_LED2) | (1<<FLASH_LED));

	// TWI aus Energiesparmode rausnehmen
	PRR != ~(1<<PRTWI);

	// Whatchdog initialisieren
	wdt_reset();
	wdt_enable(WDTO_8S);
	
	// UART initialisieren
	UART_first_init();
	i2c_init();
	
	interval=0;
	// Timer 2 initialisieren (RTC)
   TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);	// clk/256
   TIMSK2 = (1<<TOIE2);

 	// Timer 0 initialisieren (Blinken)
	TCCR0A = 0;
	TCCR0B = (0<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
	
	
	TCCR1A = 0;
	TIMER1_STOP;
	TCCR1C = 0;
	TIMSK1 = (1<<TOIE1);
	
	// Interrupt für Taster initialisieren
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);
	

	printf("\nStart\n");
	STATUS_LED1_ON;		// grüne LED ein
	STATUS_LED2_OFF;		// rote LED aus
	set_relais(0);			// Relais aus
	mode = MODE_OFF;
	
	int16_t temp, temp_sum;
//	int16_t	lookahead;
	int16_t 	slope;
	int16_t	slope_raw;
	uint8_t	count=0;
	uint8_t  last_interval = 0xff;
	uint8_t	startup = 3;
	
	temp_sum = 0;
	
	// Interrupts aktivieren
	sei();

   while(1) {
   	if(!(interval < 16)) { 					// Alle 16x (alle 4 Sekunden) Temperatur checken
   		wdt_reset();                  	// Whatchdog zurücksetzen
   		interval=0;
   		count=0;

			temp = temp_sum >> 4;				// Mittelwert der 16 Messungen ermitteln
			temp_sum = 0;
	      //printf("Temp: %i\n", temp);
	      if(temp==0) {
	      	// error!
	      	printf("Error Temp=0");
	      }
	      else {
				if (startup>0) {
					// wird nur beim ersten Durchlauf, direkt nach Startup ausgeführt
					printf("Startup %i ", startup);
					startup--;
					temp = get_temperature(ADR_T_OBJ1);
					slope = 0;
				}
   	   	printf("Temp: %i, ", temp);
   	   	add_value(temp);									// Neue Temperatur zu Array hinzufügen
   	   	//print_array();
   	   	slope_raw = get_slope();						// Aktuelle Steigung ermitteln
				//if(slope_raw<0) slope_raw = 0;
   	   	slope = (15*slope + 10*slope_raw)/16;		// Steigung wird mit einer Dämpfung von 16 gedämpft
   	   	printf("slope_raw: %i, slope: %i ", slope_raw, slope);
				//lookahead=lookahead_temp(slope, 5);   	   	
   	   	//printf("Prognose: %i\n", lookahead);
   	   	printf("Ambient: %i\n", get_temperature(ADR_T_A));
				
				// Je nach aktueller Temperatur und Steigung in Temperaturschutz gehen 	   	
	   		if(temp > 480) {
	   			// Temperatur > 48°C
	   			if(slope > 30) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 48, ");
	   			}
	   		}
	   		else if(temp > 450) {
	   			// Temperatur > 45°C
	   			if(slope > 50) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 45, ");
	   			}
	   		}
	   		else if(temp > 400) {
	   			// Temperatur > 40°C
	   			if(slope > 60) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 40, ");
	   			}
	   		}
	   		else if(temp > 350) {
	   			// Temperatur > 35°C
	   			if(slope > 80) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect Rule 35, ");
               }
	   		}
	   		else if(temp > 300) {
	   			// Temperatur > 30°C
	   			if(slope > 120) {
	   				off_counter = OFF_COUNTER+1;
		   			printf("Temperature Protect Rule 30, ");
		   		}
	   		}
	   		else {
	   			// Ansonsten
	   			if(slope > 160) {
	   				off_counter = OFF_COUNTER+1;
	   				printf("Temperature Protect General Rule, ");
	   			}
	   		}

   	   }
   		if(off_counter) {
   			// Protection Counter läuft
  				off_counter--;
  				if(mode == MODE_ON) mode = MODE_TEMP_PROT;
				printf("Counter: %i; \n", off_counter);
   		}
   		else {
   			if(mode == MODE_TEMP_PROT) mode = MODE_OFF;
   		}
		}
		else if(interval != last_interval) {
			// In jedem Interval 1x die Temperatur abrufen
			// und für den Mittelwert aufsummieren
   		last_interval = interval;
   		//uint16_t temp;
   		if(count<16) {
   			//temp = get_temperature(ADR_T_OBJ1);
	   		count++;
   			// Messwerte für den Mittelwert aufsummieren
   			temp_sum += get_temperature(ADR_T_OBJ1);
   			//printf("Raw: %i\n", temp);
   		}
   	}

		// Je nach Mode Relais und LEDs setzen
		switch(mode) {
		case MODE_OFF:
			set_relais(0);
			STATUS_LED1_ON;      // Grün
			STATUS_LED2_OFF;
			break;
		case MODE_ON:
			set_relais(1);
			STATUS_LED1_ON;     // Orange
			STATUS_LED2_ON;
			break;
		case MODE_TEMP_PROT:
			set_relais(0);
			STATUS_LED1_OFF;
			STATUS_LED2_ON;      // Rot
			slope = 0;
			break;
		default:
			mode = MODE_OFF;
		}		
   }
}




