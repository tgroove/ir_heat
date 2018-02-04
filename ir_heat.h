#ifndef IR_HEAT_H
#define IR_HEAT_H

#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (F_CPU / 1000L) )
#define T0_PRESCALE 1024
#define T0_RELOAD    244/2
//define MICROSECONDS_PER_TIMER0_OVERFLOW ( clockCyclesToMicroseconds(T0_PRESCALE*(T0_RELOAD+1L)) )
// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC       (131)
//define MILLIS_INC       (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)
// the fractional number of microseconds per timer0 overflow.
//define MILLIS_INC_FRACT (MICROSECONDS_PER_TIMER0_OVERFLOW % 1000)

char 		tbuflen(void);
int 		UART_putchar(char c, FILE *stream);
char 		rbuflen(void);
int 		UART_getchar(FILE *stream);
void		UART_first_init(void);
void 		print_array(void);
void 		add_value(uint16_t value);
int16_t 	get_slope(void);
int16_t 	lookahead_temp(int8_t slope, uint8_t steps);
uint16_t get_temperature(uint8_t adr);
void 		set_relais(uint8_t on);
int16_t	get_last_slope(void);
void 		_beep(uint16_t duration_ms);
void		beep(uint8_t type);
int 		main(void);
int16_t	exp_slope(int16_t temp);
int16_t 	get_slope2(void);

#endif // IR_HEAT_H




