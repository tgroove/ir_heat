// ***********************************************************
// Project:
// Author:
// Module description:
// ***********************************************************

#ifndef F_CPU
#define F_CPU 2000000UL
#endif

#include <avr\io.h>              // Most basic include files
#include <avr\interrupt.h>       // Add the necessary ones
//#include <avr\signal.h>          // here
#include <avr\wdt.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/power.h>
#include	<util/delay.h>

#include "i2c_mast.h"
#include "ir_heat.h"
#include "VL53L0X.h"

#define 	MLX90614_WRITE	(0x5A<<1)
#define 	MLX90614_READ	(MLX90614_WRITE+1)
#define	ADR_T_A			0x06
#define	ADR_T_OBJ1		0x07
#define	ADR_T_OBJ2		0x08
#define	DEFAULT_TEMP	150


#define __STDIO_FDEVOPEN_COMPAT_12						
// Buffer sizes must be 2^n
//
#define TBUFSIZE			32
#define RBUFSIZE			32

#define TMASK				(TBUFSIZE-1)
#define RMASK				(RBUFSIZE-1)

#define FLASH_LED			PC3
#define STATUS_LED1		PD4
#define STATUS_LED2		PD3
#define RELAIS				PB6
#define BUZZER				PB7
#define SWITCH				PD2

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

#define OFF_COUNTER		2

#define TIMER1_STOP		TCCR1B = 0
#define TIMER1_RUN		TCCR1B = (0<<CS12) | (1<<CS11) | (0<<CS10)

#define	MODE_OFF				0
#define	MODE_ON				1
#define	MODE_ON_NO_PROT	2
#define	MODE_TEMP_PROT		3

#define	BEEP_SHORT			1
#define	BEEP_2SHORT			2
#define	BEEP_LONG			3
#define	BEEP_XLONG			4
#define	BEEP_2LONG			5
#define	BEEP_SHORT_LONG	6

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
uint8_t	flash_button = 0;
uint8_t	flash_LED = 0;

/*
uint16_t	t_la_threshold_up 	=  300;
uint16_t	t_abs_threshold_up 	=  270;
uint16_t	t_la_threshold_down 	=  250;
uint16_t	t_abs_threshold_down	=  250;
*/

volatile uint32_t g_Millis=0;

// Return ellapsed time since startup in [ms]
uint32_t millis(){
    uint32_t m;
    uint8_t oldSREG = SREG;
     // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    cli();
    m = g_Millis;
    SREG = oldSREG;
    return m;
}

// Clock Timer
SIGNAL(SIG_OVERFLOW2) {
	interval++;
	//*************************
	static uint8_t usFract=0;
	g_Millis += MILLIS_INC;
//	usFract  += MILLIS_INC_FRACT>>3;// 680 / 8 =  85.0
//	if( usFract >= 1000>>3 ){		//1000 / 8 = 125.0
//		usFract -= 1000>>3;			//Fractional part added up to 1 ms
//		g_Millis++;
//	}
	//**************************
}

SIGNAL(SIG_OVERFLOW1) {
	TIMER1_STOP;
	TCNT1H = 0;
	TCNT1L = 0;
}


// LED Flasher
SIGNAL(SIG_OVERFLOW0) {
	static uint8_t	c1 = 0;
	static uint8_t c2 = 0;
	uint8_t slow=0;
	c1++;
	c2++;
	
	// Tasten LED
	if (mode==MODE_TEMP_PROT) {
		if(c1 > (6<<slow)) {
			FLASH_LED_ON;
		}
	}
	if(c1 > (10<<slow)) {
		c1 = 0;
		FLASH_LED_OFF;
	}
	
	// Status LED
	if (mode==MODE_ON_NO_PROT) {
		if(c2 > 120) {
			STATUS_LED1_OFF;	// rot
		}
	}
	if(c2 > 135) {
		c2 = 0;
		STATUS_LED1_ON; 		// orange
	}	
}



//*******************************************
//
// Taster IQR und Entprellung
//
SIGNAL(SIG_INTERRUPT0) {
	static uint8_t running = 0;
	
	if(running | TCNT1H | TCNT1L){
//		printf("X");
		return;
	}
	running = 1;
	wdt_reset();
	
	uint16_t i;
	uint16_t c = 0;
	EIMSK = 0;
	sei();
	//printf("In");
	for(i=0;i<1000;i++) if((PIND & (1<<SWITCH))) c++;
	//printf(" %i ", c);

	if(c < 200) {
		TCNT1L = 1;
		TIMER1_RUN;
		switch(mode) {
		case MODE_OFF:
			mode = MODE_ON;
			set_relais(1);
			STATUS_LED1_ON;			// orange
			STATUS_LED2_ON;
			c = 0;
			while((!(PIND & (1<<SWITCH))) && (c < 300)) {
				c++;
				_delay_ms (10);
			}
			//printf("c: %i", c);

			if(c < 300) {
				// normal einnschalten
				mode = MODE_ON;
			}
			else {
				// einschalten, aber ohne Hitzeschutz
				mode = MODE_ON_NO_PROT;
				printf("Temperature Protection Off!\n");
				STATUS_LED1_OFF;		// rot
				STATUS_LED2_ON;
				beep(BEEP_SHORT_LONG);
			}
			break;
		case MODE_ON:
		case MODE_ON_NO_PROT:
		case MODE_TEMP_PROT:
		default:
			printf("\nxXx\n");
			mode = MODE_OFF;
		}
	}
//	printf("Out\n");
	EIFR 		= (1<<INTF0);
	EIMSK 	= (1<<INT0);
	running 	= 0;
	//printf("Exit\n");
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
	UBRR0 = 12;										 		// 4800 BPS
	
	//UCSR0A |= (1<<U2X0);
	UCSR0B = (1<<RXCIE0)|(1<<TXEN0)|(1<<RXEN0);	// 8 Databits, receive and transmit enabled, receive and transmit complete interrupt enabled
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	fdevopen(UART_putchar, UART_getchar);
	sei();											 		// Global interrupt enable
}




int16_t exp_slope(int16_t temp) {
	return 10 * (-temp/16 + 32) + 15;
}


int16_t get_slope2() {
	static int16_t last_slope = 0;
//	last_slope = (200*(t_array[5]-t_array[2]) / 16 +  2*last_slope) / 3;
	last_slope = (16*(t_array[5]-t_array[2]) +  2*last_slope) / 3;
	return last_slope;
}




//*********************************************
//
// Gibt den Temperatur Ringspeicher
// �ber den UART aus
//
void print_array(){
	uint8_t i;
  	printf("Array:");
	for(i=0;i<6;i++) {
   	printf(" %i", t_array[i]);
	}
  	printf("\n");
}



//*********************************************
//
// F�gt einen Me�wert zum Ringspeicher hinzu
// Der Ringspeicher enth�lt die letzten 6 Werte
// t_array[5] ist der neuste Wert
//
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



//********************************************
//
// Gibt die gemittelte Steigung
// �ber die letzten 4 Sekunden zur�ck
//
int16_t get_slope() {
	int16_t s1, s2, s3;

	s1 = t_array[5] - t_array[0];
	s2 = t_array[4] - t_array[1];
	s3 = t_array[3] - t_array[2];
	
	return ((3*s1+5*s2+15*s3)/9);
}



//********************************************
//
// Gibt die aktuelle Steigung der Temperatur zur�ck
// in 0.1�C in 4s
//
int16_t	get_last_slope() {
    return (t_array[5] - t_array[4]);
}



//********************************************
//
// Liest die vom MLX90614 gemessene Temperatur aus
// R�ckgabe in 0.1�C, also 215 = 21.5�C
//
uint16_t get_temperature(uint8_t adr) {
	uint16_t raw;
	uint8_t 	ret;
	uint8_t 	lo, hi, pec;
	uint8_t	pec_read[6];

	if(i2c_start(MLX90614_WRITE)) return DEFAULT_TEMP;
	if(i2c_write(adr)) return DEFAULT_TEMP;
	
	ret = i2c_rep_start(MLX90614_READ);
	if(ret) {
		i2c_rep_start(MLX90614_READ);
   }

	lo = i2c_readAck();
	hi = i2c_readAck();
	raw = (uint16_t)(hi<<8)+lo;
	pec = i2c_readAck();
	
	i2c_stop();
	
	if(raw & 0x8000) return DEFAULT_TEMP;
	
	return (raw / 5 - 2731); 					// 1 = 0.1�C
}



void _beep(uint16_t duration_ms){
	uint16_t i;
	BUZZER_ON;
	for(i=0;i<(duration_ms/20);i++) _delay_ms(20);
	BUZZER_OFF;
}


void	beep(uint8_t type){
	cli();
	wdt_reset();
	switch(type){
	case BEEP_SHORT:
		_beep(120);
		break;
	case BEEP_LONG:
		_beep(200);
		break;
	case BEEP_XLONG:
		_beep(850);
		break;
	case BEEP_2SHORT:
		_beep(80);
		_delay_ms(80);
		_beep(80);
		break;
	case BEEP_SHORT_LONG:
		_beep(100);
		_delay_ms(180);
		_beep(350);		
	}
	sei();	
	printf("Beep\n");
}



//***************************************************
//
// Relais Ein- und Ausschalen
//
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
   // Ausg�nge definieren
	DDRB = 0x00 | (1<<RELAIS) | (1<<BUZZER);
	DDRC = 0x00 | (1<<FLASH_LED);
	DDRD = 0x00 | (1<<STATUS_LED1) | (1<<STATUS_LED2) | (1<<FLASH_LED);

	// Ausg�nge ausschalten
	PORTB = ~((1<<RELAIS) | (1<<BUZZER));
	PORTC = ~(1<<FLASH_LED);
	PORTD = ~((1<<STATUS_LED1) | (1<<STATUS_LED2) | (1<<FLASH_LED));

	// TWI aus Energiesparmode rausnehmen
	PRR != ~(1<<PRTWI);

	// Whatchdog initialisieren
	wdt_reset();
	wdt_enable(WDTO_8S);
	
	// Set clock divider to 4 => 2MHz
	clock_prescale_set(clock_div_4);
	
	// UART initialisieren
	UART_first_init();
	i2c_init();
	//initMillis();
	
	interval=0;
	
	// Timer 2 initialisieren (RTC)
   TCCR2B = (1<<CS22) | (1<<CS21) | (1<<CS20);	// clk/256
   TIMSK2 = (1<<TOIE2);

 	// Timer 0 initialisieren (Blinken)
	TCCR0A = 0;
	TCCR0B = (0<<CS02) | (1<<CS01) | (1<<CS00);
	TIMSK0 = (1<<TOIE0);
	
	// Timer 1 initialisieren (Entprellen?)
	TCCR1A = 0;
	TIMER1_STOP;
	TCCR1C = 0;
	TIMSK1 = (1<<TOIE1);
	
	// Interrupt f�r Taster initialisieren
	EICRA = (1<<ISC01);
	EIMSK = (1<<INT0);
	

	printf("\n\nStart\n\n");
	STATUS_LED1_ON;		// gr�ne LED ein
	STATUS_LED2_OFF;		// rote LED aus
	set_relais(0);			// Relais aus
	mode = MODE_OFF;
	
	initVL53L0X(1);	
	// lower the return signal rate limit (default is 0.25 MCPS)	
	setSignalRateLimit(0.1);	
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)	
	//setVcselPulsePeriod(VcselPeriodPreRange, 18);	
	//setVcselPulsePeriod(VcselPeriodFinalRange, 14);	
	//setMeasurementTimingBudget( 100 * 1000UL );		
	// integrate over 500 ms per measurement
	
//	int16_t temp, temp_sum, temp_a;
//	int16_t	lookahead;
//	int16_t 	slope, max_slope;
//	int16_t	slope_raw;
	int16_t 	temp, temp_sum = 0;
	int16_t	slope_raw, slope = 0, temp_a, max_slope;
	uint8_t	count=0;
	uint8_t  last_interval = 0xff;
	uint8_t	startup = 3;
	uint8_t	on_counter = 0;
	int8_t	beep_counter = 0;
	int16_t	factor;
	int16_t	integral = 0;
	int8_t	fx = 0;
	
	int16_t	slope_std = 0;
	int16_t	slope_real = 0;
	
	statInfo_t xTraStats;
	uint16_t	dist;
		
	// Interrupts aktivieren
	sei();

   while(1) {
   	if(!(interval < 16)) { 					// Alle 16x (alle 4 Sekunden) Temperatur checken
   		wdt_reset();                  	// Whatchdog zur�cksetzen

			temp = temp_sum / count;				// Mittelwert der 16 Messungen ermitteln

   		interval=0;
   		count=0;
			temp_sum = 0;
	      //printf("Temp: %i\n", temp);
	      if(temp==0) {
	      	// error!
	      	printf("Error Temp=0");
	      }
	      else {
				if (startup>0) {
					// wird nur beim ersten Durchlauf, direkt nach Startup ausgef�hrt
					printf("Startup %i ", startup);
					startup--;
					temp = get_temperature(ADR_T_OBJ1);
					slope = 0;
				}
   	   	printf("Temp: %i, ", temp);
   	   	add_value(temp);									// Neue Temperatur zu Array hinzuf�gen
   	   	slope_raw = get_slope();						// Aktuelle Steigung ermitteln
   	   	if (slope_raw < -100) slope_raw = -100;
   	   	
   	   	factor = (temp - 620) / -25;					// Fakort ermitteln
   	   	temp_a = get_temperature(ADR_T_A);
   	   	//factor = (temp - 900 + 2*temp_a ) / -25;					// Fakort ermitteln
   	   	if(factor < 0) factor = 0;
   	   	   	   	
				if(slope_raw > factor) { 						// "Steigungsintegral"
					integral = 8*(integral + slope_raw) / factor;
					if(integral > 2000) integral = 2000;
				}
				else {
					integral = integral / 4;
				}
				
//				slope = (31*slope + 10*slope_raw)/32; 		// Steigung d�mpfen

				
				if(slope_raw < -10) slope_raw = -10;
				if(slope_raw<0) {									// Fallende Temperaturen werden st�rker gewichtet
	   	   	slope = (3*slope + 10*slope_raw)/4;		// Negative Steigung wird mit einer D�mpfung von 8 ged�mpft
				}
				else {
	   	   	slope = (31*slope + 10*slope_raw)/32;	// Positive Steigung wird mit einer D�mpfung von 16 ged�mpft
	   	   }
	   	

	   		slope_real = get_slope2();

				//printf("bc: %i,sr: %i", beep_counter, slope_real);
				if(beep_counter > 0) {
					beep_counter++;
					if((slope_real < 0)) {
						fx = fx+20;
						beep_counter = -10;
					}
				}
				
				if(fx > 60) fx= 60;

	   		slope_std = exp_slope(temp) + fx;
	   	
/*
   	   	printf("slope_raw: %i, slope: %i ", slope_raw, slope);
     	   	printf("Ambient: %i\n", get_temperature(ADR_T_A));
*/				

				// temp_a 150 -> 45
				// temp_a 100 -> 60
//   	   	max_slope = temp_a * -1.3 + 240;
//   	   	max_slope = max_slope * (600-temp)/50;
				max_slope = (float)temp * -0.8 + 360;

//   	   	printf("sl_raw: %i, sl: %i, s_max: %i, f: %i, int: %i t_a: %i\n", slope_raw, slope, max_slope, factor, integral, temp_a);
   	   	printf("exp_s: %i, s2: %i, bc: %i, fx: %i\n", slope_std, slope_real, beep_counter, fx);

//				if((slope > max_slope) || (integral > 500)) {
				if(slope_real > slope_std) {
					on_counter++;
		   		printf("On-Counter: %i; \n", on_counter);
	   			if(mode == MODE_ON_NO_PROT){
						//on_counter++;
	   				if(on_counter==3){
	   					beep(BEEP_SHORT);
	   					beep_counter=1;
	   					on_counter = 0;
	   				}
  					}
   				else {
   					if(get_last_slope() > 0) {
							//on_counter++;
			   			if((on_counter > 11) || (temp > 520)) {
   							off_counter = OFF_COUNTER+1;
   							on_counter = 0;
   							if(mode == MODE_ON) {
	   							beep(BEEP_XLONG);
   								if(temp<500) {
				   					beep_counter=1;
   									//fx = fx+20;
   								}
   							}
		   				}
		   				else {
		   					if( ((on_counter > 4) && ((on_counter % 3) == 0)) || (temp > 500) ) {
		   						beep(BEEP_LONG);
		   						beep_counter = 1;
		   					}
   						}
   					}
   					else {
   						if(slope_raw<0) on_counter=0;
   					}
   				}
				}			
				else {
					on_counter = 0;
				}					
   	   }

   		if(off_counter) {
   			// Protection Counter l�uft
  				off_counter--;
  				if(mode == MODE_ON) mode = MODE_TEMP_PROT;
				printf("Off-Counter: %i; \n", off_counter);
   		}
   		else {
   			if(mode == MODE_TEMP_PROT) {
   				slope = 0;
   				integral = 0;
   				mode = MODE_OFF;
   			}
   		}
   		
   			readRangeSingleMillimeters( &xTraStats );	// blocks until measurement is finished
//				dist = readRangeSingleMillimeters( 0 );	// blocks until measurement is finished
//				printf("Dist: %i mm\n", dist);

				printf("Status     %x\n", xTraStats.rangeStatus);
				printf("Dist:      %u mm\n", xTraStats.rawDistance);
				printf("SigCount:  %u MCPS\n", xTraStats.signalCnt);
				printf("AmbiCount: %u MCPS\n", xTraStats.ambientCnt);
				printf("SpadCount: %u\n", xTraStats.spadCnt);
				if ( timeoutOccurred() ) {
					printf("!!! Timeout !!!\n");
				}
   		
		}
		else if(interval != last_interval) {
			// In jedem Interval 1x die Temperatur abrufen
			// und f�r den Mittelwert aufsummieren
   		last_interval = interval;
    		if(count<16) {
	   		count++;
   			// Messwerte f�r den Mittelwert aufsummieren
   			temp_sum += get_temperature(ADR_T_OBJ1);
   			
   		}
   		//_delay_ms(100);
			//printf("Test: %X\n", readReg(0xC1));
			//_delay_ms(100);
   	}

		// Je nach Mode Relais und LEDs setzen
		switch(mode) {
		case MODE_OFF:
			set_relais(0);
			STATUS_LED1_ON;      // Gr�n
			STATUS_LED2_OFF;
			off_counter = 0;
			on_counter = 0;
			beep_counter = 0;
			break;
		case MODE_ON:
			STATUS_LED1_ON;      // Gr�n
		case MODE_ON_NO_PROT:
			set_relais(1);
			STATUS_LED2_ON;      // Rot
			break;
		case MODE_TEMP_PROT:
			set_relais(0);
			STATUS_LED1_OFF;
			STATUS_LED2_ON;      // Rot
			slope = 0;
			integral = 0;
			break;
		default:
			mode = MODE_OFF;
		}
//		readRangeSingleMillimeters( &xTraStats );	// blocks until measurement is finished
/*
		printf("Status %x\n", xTraStats.rangeStatus);
		printf("Dist: %i mm\n", xTraStats.rawDistance);
		printf("SigCount: %i MCPS", xTraStats.signalCnt);
		printf("AmbiCount: %i MCPS", xTraStats.ambientCnt);
		printf("SpadCount: %i", xTraStats.spadCnt);
		if ( timeoutOccurred() ) {
			printf("!!! Timeout !!!\n");
		}
		
/*		
	  	debug_str(   "\n\nstatus  = ");		debug_hex( xTraStats.rangeStatus, 1 );	
	  	debug_str(     "\ndist    = ");		debug_dec( xTraStats.rawDistance );	 	
	  	debug_str(  " mm\nsignCnt = ");		debug_dec_fix( xTraStats.signalCnt,  7 );		
	  	debug_str(" MCPS\nambiCnt = ");		debug_dec_fix( xTraStats.ambientCnt, 7 );		
	  	debug_str(" MCPS\nspadCnt = ");		debug_dec_fix( xTraStats.spadCnt,    8 );		
	  	if ( timeoutOccurred() ) {			debug_str(" !!! Timeout !!! \n");		}		
*/		
		_delay_ms(100);		
   }
}




