#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdio.h>

#define LEFT_GROUND PB0
#define RIGHT_GROUND PB2

#define VOLTAGE_LEVEL 12.0f

#define F PA4
#define A PA6
#define B PA7
#define C PA2
#define D PA1
#define E PA0
#define G PA5
#define DOT PA3

#define PWM_DDR DDRB
#define PWM_PORT PORTB
#define PWM_PIN PB5

#define ADC_PIN 7

uint8_t DICT[10] = 
{	
	~(1 << G), // 0
	(1 << B) | (1 << C), // 1
	~((1 << C) | (1 << F)), // 2
	~((1 << E) | (1 << F)), // 3
	~((1 << A) | (1 << D) | (1 << E)), // 4
	~((1 << B) | (1 << E)), // 5
	~((1 << B)), // 6
	((1 << A) | (1 << B) | (1 << C)), // 7
	0xFF, // 8
	~(1 << E)// 9
};

volatile float VOLTAGE = 0;
volatile uint8_t VOLTAGE_INT = 0;
volatile uint8_t CURRENT_TIMER_COUNTER = 0;

volatile uint32_t ADC_VALUE = 0;
volatile uint8_t LEFT_VAL = 0, RIGHT_VAL = 0, CURRENT = 0;

ISR(TIMER0_OVF_vect) // 122 Hz interrupt frequency
{	
	if(CURRENT)
	{
		PORTB &= ~(1 << LEFT_GROUND);
		PORTB |= (1 << RIGHT_GROUND);
		PORTA = DICT[LEFT_VAL];
		if(VOLTAGE < 10)
		{
			PORTA |= (1 << DOT); // TURN DOT ON
		}
		else PORTA &= ~(1 << DOT); // TURN DOT OFF
		CURRENT = 0;
	}
	else
	{
		PORTB &= ~(1 << RIGHT_GROUND);
		PORTB |= (1 << LEFT_GROUND);
		PORTA = DICT[RIGHT_VAL];
		PORTA &= ~(1 << DOT); // TURN DOT OFF
		CURRENT = 1;
	}
}

//~ ISR(TIMER0_OVF_vect) // 323 Hz interrupt
//~ ISR(TIMER0_COMPA_vect)
//~ {
	//~ if(CURRENT_TIMER_COUNTER < VOLTAGE_INT) PWM_PORT |= (1 << PWM_PIN);
	//~ else PWM_PORT &= ~(1 << PWM_PIN);
	//~ 
	//~ CURRENT_TIMER_COUNTER = (CURRENT_TIMER_COUNTER + 1) % 100;
//~ }

uint16_t adc_read(uint8_t ch)
{
	ADMUX = (ADMUX & 0b1110000) | (ch & 0x1F); 
	
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	
	return (ADC);
}

void setupADC(void)
{
	ADCSRA |= (1 << ADEN);
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void setupDisplay(void)
{
	DDRB |= (1 << LEFT_GROUND) | (1 << RIGHT_GROUND);
	DDRA = 0xFF;
	
	TCCR0B |= (1 << CS02);

	TIMSK |= (1 << TOIE0);
}

void setupPWM(void)
{
	PWM_DDR |= (1 << PWM_PIN);
	
	TCCR1B |= (1 << CS12) | (1 << CS11) | (1 << CS10); // 64 prescaler
	TCCR1C |= (1 << COM1D0) | (1 << COM1D1) | (1 << PWM1D); // Toggle on compare match

	//~ TC1H = 0x03;
	OCR1C = 0xFF;
}

int main(void)
{
	setupDisplay();
	setupADC();
	setupPWM();
	
	sei();
	
	uint8_t i;
	char buffer[10];
	
	while(1)
	{
		for(i = 0; i < 100; i++)
		{
			ADC_VALUE += adc_read(ADC_PIN);
			_delay_ms(2);
		}
		
		ADC_VALUE = ADC_VALUE / i; // mean
		
		OCR1D = ADC_VALUE / 4;
		
		VOLTAGE = ADC_VALUE / ((float) 102);
		VOLTAGE_INT = 5 * (0.8 * VOLTAGE);
			
		sprintf(buffer, "%.1f", VOLTAGE);
			
		LEFT_VAL = (uint8_t) (buffer[0] - '0');
		if(buffer[1] == '.') RIGHT_VAL = buffer[2] - '0';
		else RIGHT_VAL = buffer[1] - '0';
		
		
		ADC_VALUE = 0;
		
		//~ 
		//~ for(i = 0; i < 10; i++)
		//~ {
			//~ LEFT_VAL = i;
			//~ for(uint8_t j = 0; j < 10; j++)
			//~ {
				//~ RIGHT_VAL = j;
				//~ OCR1D = i*10 + j;
				//~ _delay_ms(100);
			//~ }
		//~ }
		
		
		
		//~ 
		
		//~ 
	}
	
	return 1;
}
