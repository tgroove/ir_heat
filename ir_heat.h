#ifndef IR_HEAT_H
#define IR_HEAT_H

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
int 		main(void);

#endif // IR_HEAT_H
